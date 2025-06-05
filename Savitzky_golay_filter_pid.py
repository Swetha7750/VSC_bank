import asyncio
import xml.etree.ElementTree as ET
import pkg_resources
import struct
import qtm
import math
import socket
import json
import time
import numpy as np
from pymavlink import mavutil
import logging
from scipy.spatial.transform import Rotation as R 
from scipy.signal import savgol_filter
import csv
import signal
import sys
from collections import deque
# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class QTMTracker:
    def __init__(self, qtm_ip="192.168.252.1", wanted_body="drone_body"):
        self.wanted_body = wanted_body
        self.qtm_ip = qtm_ip
        self.QTM_file = pkg_resources.resource_filename("qtm", "data/demo.qtm")
        self.connection = None
        self.realtime = True
        self.body_index = {}
        self.position = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.rotation = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
        self.last_valid_position = None
        self.data_available = False
        self.rot_matrix = None  # Initialize rot_matrix

    async def setup_connection(self):
        try:
            self.connection = await qtm.connect(self.qtm_ip)
            if self.connection is None:
                logger.error("Failed to connect to QTM")
                return False
            logger.info(f"Connected to QTM at {self.qtm_ip}")
            return True
        except Exception as e:
            logger.error(f"Error connecting to QTM: {e}")
            return False

    def create_body_index(self, xml_string):  
        try:
            xml = ET.fromstring(xml_string)
            body_to_index = {}
            for index, body in enumerate(xml.findall("*/Body/Name")):
                body_to_index[body.text.strip()] = index
            logger.info(f"Found bodies: {list(body_to_index.keys())}")
            
            return body_to_index
        except Exception as e:
            logger.error(f"Error parsing XML: {e}")
            return {}
    async def start_streaming(self):
        if not await self.setup_connection():
            return False
        try:
            async with qtm.TakeControl(self.connection, "password"):
                if self.realtime:
                    await self.connection.new()
                else:
                    await self.connection.load(self.QTM_file)
                    await self.connection.start(rtfromfile=True)

            xml_string = await self.connection.get_parameters(parameters=["6d"])
            self.body_index = self.create_body_index(xml_string)    
            if self.wanted_body not in self.body_index:
                logger.error(f"Body '{self.wanted_body}' not found in QTM data")
                return False
            await self.connection.stream_frames(components=["6d"], on_packet=self.on_packet)
            return True
        except Exception as e:
            logger.error(f"Error starting QTM streaming: {e}")
            return False

    def on_packet(self, packet):
        
        try:
            info, bodies = packet.get_6d()
            if self.wanted_body in self.body_index:
                wanted_index = self.body_index[self.wanted_body]
                
                if wanted_index < len(bodies):
                    position, rotation = bodies[wanted_index]
                    
                    # Check for valid position data
                    if not (math.isnan(position.x) or math.isnan(position.y) or math.isnan(position.z)):
                        # Convert from mm to meters
                        self.position = {
                            'x': round(position.x / 1000.0, 5),
                            'y': round(position.y / 1000.0, 5),
                            'z': round(position.z / 1000.0, 5)
                        }
                        
                        # Convert rotation matrix to Euler angles
                        if rotation:
                            self.rot_matrix = np.array(rotation).reshape(3, 3, order='F')
                            
                        
                        self.last_valid_position = time.time()
                        self.data_available = True
                        
        except Exception as e:
            logger.error(f"Error processing packet: {e}")

    def rotation_matrix_to_euler_xyz(self, rot_matrix):

        rot_matrix = rot_matrix
        r = R.from_matrix(rot_matrix)
        euler_angles = r.as_euler('xyz', degrees=True)
        self.roll, self.pitch, self.yaw = euler_angles  
        #print(euler_angles)
        return euler_angles


    def get_position(self):
        return self.position.copy() if self.data_available else None

    def get_rotation(self):
        return self.rotation.copy() if self.data_available else None

    async def stop_streaming(self):
        try:
            if self.connection is not None:
                await self.connection.disconnect()
        except Exception as e:
            logging.error(f"Error while stopping QTM stream: {e}")



class GPSConverter:
    def __init__(self, qtm_tracker, reference_lat=50.764905, reference_lon=6.07875, udp_port=25100):
        self.qtm_tracker = qtm_tracker
        self.reference_lat = reference_lat  
        self.reference_lon = reference_lon  
        self.reference_lat_rad = math.radians(reference_lat)
        self.RADIUS_OF_EARTH = 6371000.0 
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_address = ("127.0.0.1", udp_port)
        logger.info(f"GPS reference point: {reference_lat}, {reference_lon}")

    def local_to_gps(self, x_local, y_local):
        # Convert local NED coordinates to lat/lon
        lat = self.reference_lat + (y_local / self.RADIUS_OF_EARTH) * (180 / math.pi)
        lon = self.reference_lon + (x_local / (self.RADIUS_OF_EARTH * math.cos(self.reference_lat_rad))) * (180 / math.pi)
        return round(lat, 7), round(lon, 7)

    def create_gps_message(self, position):
        if not position:
            return None  
        lat, lon = self.local_to_gps(position['x'], position['y'])
        gps_data = {
            'time_usec': int(time.time() * 1000000),  # Current time in microseconds
            'gps_id': 0,                              # GPS ID
            'ignore_flags': 8,                        # Flags for ignored fields
            'time_week_ms': int((time.time() % 604800) * 1000),  # GPS time (ms from start of week)
            'time_week': int(time.time() / 604800),   # GPS week number
            'fix_type': 3,                            # 3D fix
            'lat': int(lat * 1e7),                    # Latitude (degrees * 1E7)
            'lon': int(lon * 1e7),                    # Longitude (degrees * 1E7)
            'alt': int(position['z'] * 1000),         # Altitude (mm above reference)
            'hdop': 1,                              # Horizontal dilution of precision (cm)
            'vdop': 1,                              # Vertical dilution of precision (cm)
            'vn': 0,                                  # Velocity north (cm/s)
            've': 0,                                  # Velocity east (cm/s)
            'vd': 0,                                  # Velocity down (cm/s)
            'speed_accuracy': 0.5,                     # Speed accuracy (cm/s)
            'horiz_accuracy': 1.0,                    # Horizontal accuracy (cm)
            'vert_accuracy': 1.5,                     # Vertical accuracy (cm)
            'satellites_visible': 9                  # Number of visible satellites
        }
        
        return gps_data

    def send_gps_data(self, gps_data):
        try:
            json_data = json.dumps(gps_data)
            self.socket.sendto(json_data.encode(), self.udp_address)
            logger.debug(f"Sent GPS data: Lat={gps_data['lat']/1e7:.6f}, Lon={gps_data['lon']/1e7:.6f}, Alt={gps_data['alt']/1000:.2f}m")
        except Exception as e:
            logger.error(f"Error sending GPS data: {e}")

    def process_and_send(self):
        position = self.qtm_tracker.get_position()
        if position:
            gps_data = self.create_gps_message(position)
            if gps_data:
                self.send_gps_data(gps_data)
                return True
        return False
    
#print(euler_angles)

class PIDController:
    def __init__(self, qtm_tracker):
        self.qtm_tracker = qtm_tracker

        # PID parameters
        self.kp = 0.8
        self.ki = 0.00
        self.kd = 0.5
        self.output_limits = (-0.0872, 0.0872) # degrees = 5
        self.integral_limit = 4

        # PID states
        self.integral_x = 0.0
        self.integral_y = 0.0
        self.last_error_x = 0.0
        self.last_error_y = 0.0
        self.last_time = time.time()
        self.first_call = True

        # Target position
        self.target_x = 0.0
        self.target_y = 0.0

        # Trajectory states
        self.captured = False
        self.released = False
        self.capture_x = 0.0
        self.capture_y = 0.0

        # Actuator limits
        self.operational_limit = 0.0872665  # radians
        self.min_angle = -0.523599
        self.max_angle = 0.523599
        self.min_pwm = 1000
        self.max_pwm = 2000

        self.pwm_x = 1500
        self.pwm_y = 1500

        # Initialize body frame variables
        self.measured_x_body = 0.0
        self.measured_y_body = 0.0
        self.command_x_body = 0.0
        self.command_y_body = 0.0
        self.rotation = [0.0, 0.0, 0.0]  # Initialize rotation
        self.x_buffer = deque(maxlen=50)
        self.y_buffer = deque(maxlen=50)
        self.non_f_y =self.non_f_y= 0

        

    def X_global_to_body_frame(self,xx,yy, psi): # the function that does the global to body transformation
        self.x_body = ((math.cos(psi)*xx)+(math.sin(psi)*yy))
        return self.x_body
        
    def y_global_to_body_frame(self,xx,yy, psi):
        self.y_body = ((-(math.sin(psi)*xx))+((math.cos(psi)*yy)))
        return self.y_body

    def set_target(self, x, y):
        self.target_x = x
        self.target_y = y

        #print(f"Set Target: X:{self.target_x}, Y:{self.target_y}")

    def compute_pid(self, error_x, error_y, dt):
        self.x_buffer.append(error_x)
        self.y_buffer.append(error_y)

        try:
            if len(self.x_buffer) >= 5:
                # Apply Savitzky-Golay filter to derivative (1st order)
                dx_series = list(self.x_buffer)
                dy_series = list(self.y_buffer)

                self.derivative_x = savgol_filter(dx_series, window_length=11, polyorder=1, deriv=1, delta=dt)[-1]
                self.derivative_y = savgol_filter(dy_series, window_length=11, polyorder=1, deriv=1, delta=dt)[-1]
            else:
                # Fall back to simple difference
                self.derivative_x = (error_x - self.last_error_x) / dt
                self.derivative_y = (error_y - self.last_error_y) / dt
        except Exception as e:
            print(f"SG filter error: {e}")
            self.derivative_x = (error_x - self.last_error_x) / dt
            self.derivative_y = (error_y - self.last_error_y) / dt

        self.non_f_x = (error_x - self.last_error_x) / dt
        self.non_f_y = (error_y - self.last_error_y) / dt


            # self.derivative_x = (error_x - self.last_error_x) / dt
            # self.derivative_y = (error_y - self.last_error_y) / dt

        # Integral calculation with windup prevention
        self.integral_x += error_x * dt
        self.integral_y += error_y * dt
        self.integral_x = max(min(self.integral_x, self.integral_limit), -self.integral_limit)
        self.integral_y = max(min(self.integral_y, self.integral_limit), -self.integral_limit)

        # PID output
        control_x = -(self.kp * error_x + self.ki * self.integral_x + self.kd * self.derivative_x)
        control_y = self.kp * error_y + self.ki * self.integral_y + self.kd * self.derivative_y

        # Clamp output
        control_x = max(min(control_x, self.output_limits[1]), self.output_limits[0])
        control_y = max(min(control_y, self.output_limits[1]), self.output_limits[0])

        # Update last error and time
        self.last_error_x = error_x
        self.last_error_y = error_y
        #print(self.last_error_x, self.last_error_y)

        return control_x, control_y

    def angle_to_pwm(self, cmd_angle):
        limited_angle = max(min(cmd_angle, self.operational_limit), -self.operational_limit)
        pwm_min_limit = np.interp(-self.operational_limit, [self.min_angle, self.max_angle], [self.min_pwm, self.max_pwm])
        pwm_max_limit = np.interp(self.operational_limit, [self.min_angle, self.max_angle], [self.min_pwm, self.max_pwm])
        pwm_val = int(np.interp(limited_angle, [-self.operational_limit, self.operational_limit], [pwm_min_limit, pwm_max_limit]))
        return pwm_val

    def reset(self):
        self.integral_x = self.integral_y = 0.0
        self.last_error_x = self.last_error_y = 0.0
        self.last_time = time.time()
        self.first_call = True
        self.captured = False
        self.released = False

    def control_loop(self, database=None):
        
        
        start_time = time.time()
        while time.time() - start_time < 300:  # 5 minutes
            position = self.qtm_tracker.get_position()
            # if position is None :
            #     time.sleep(0.1)
            #     continue

            # if self.qtm_tracker.rot_matrix is None:
            #     # Skip this iteration if rotation data is missing
            #     time.sleep(0.1)
            #     continue
            self.rotation = self.qtm_tracker.rotation_matrix_to_euler_xyz(self.qtm_tracker.rot_matrix)
            yaw_rad = math.radians(self.rotation[2])  # assuming rotation[2] is yaw angle
            self.measured_x_body = self.X_global_to_body_frame(position['x'], position['y'], yaw_rad)
            self.measured_y_body = self.y_global_to_body_frame(position['x'], position['y'], yaw_rad)
            self.command_x_body = self.X_global_to_body_frame(self.target_x, self.target_y, yaw_rad)
            self.command_y_body = self.y_global_to_body_frame(self.target_x, self.target_y, yaw_rad)
            #print("Init While Loop")
            current_time = time.time()
            dt = 0.02
            # if dt <= 0.0:derivative_x
            elapsed_time = current_time - start_time
            # Release phase
            if self.captured and not self.released and elapsed_time >= 31:
                self.set_target(0.0, 0.0)
                self.released = True
                #logger.info(f"Target reset to origin at t={elapsed_time:.1f}s")

            error_x = self.command_x_body - self.measured_x_body
            error_y = self.command_y_body - self.measured_y_body
            # error_x = self.target_x - position['x']
            # error_y = self.target_y - position['y']
            control_x, control_y = self.compute_pid(error_x, error_y, dt)
            self.pwm_x = self.angle_to_pwm(-control_x)
            self.pwm_y = self.angle_to_pwm(control_y)
            # Store data if database is provided
            if database:
                database.append_values( elapsed_time, position, error_x, error_y, control_x, control_y, self.kp, self.ki, self.kd, self.integral_x, self.integral_y, self.derivative_x, self.derivative_y)

            logger.info(
                f"t={elapsed_time:.1f}s | "
                f"Pos: X={position['x']:.3f}, Y={position['y']:.3f}, Z={position['z']:.3f} | "
                f"Rot: Roll={(self.rotation[0]):.2f} | "
                f"Pitch={(self.rotation[1]):.2f}|  "
                f"Yaw={(self.rotation[2]):.2f}| "
                f"Target: X={self.target_x:.3f}, Y={self.target_y:.3f} | "
                f"Error: X={error_x:.3f}, Y={error_y:.3f} | "
                f"Command: theta_command={control_x:.3f}, pitch_cmd={control_y:.3f} | "
                f"PWMtheta={self.pwm_x}, pwmphi={self.pwm_y}|"
                
            )
            self.last_time = current_time
            time.sleep(dt)
        return self.pwm_x, self.pwm_y
class mav_work:
    def __init__(self, PIDController):
        self.MAVLINK_CONNECTION = "udp:127.0.0.1:14550" 
        self.vehicle = mavutil.mavlink_connection(self.MAVLINK_CONNECTION)
        self.armed =False
        self.pid_controller =PIDController

    def connect_to_mavlink(self):
        try:
            self.vehicle = mavutil.mavlink_connection(self.MAVLINK_CONNECTION)
            self.vehicle.wait_heartbeat()
            print("Connected to vehicle!")
        except Exception as e:
            print(f"Failed to connect to vehicle: {e}")
            raise

    def arm_vehicle(self):
        
        self.vehicle.mav.command_long_send(
        self.vehicle.target_system, self.vehicle.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
        msg = self.vehicle.recv_match(type='COMMAND_ACK', blocking=True)
        if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            self.armed = True
            print("Vehicle armed!")
        else:
            print("Failed to arm vehicle!")    
    def diarm_vehicle(self):
        
        self.vehicle.mav.command_long_send(
        self.vehicle.target_system, self.vehicle.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0)
        msg = self.vehicle.recv_match(type='COMMAND_ACK', blocking=True)
        if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            self.armed = False
            print("Vehicle disarmed!")
        else:
            print("Failed to disarm vehicle!")
          

    def rc_override(self):
        rc_channel_values = [65535 for _ in range(18)]
        rc_channel_values[1] =  self.pid_controller.pwm_y
        rc_channel_values[2] = self.pid_controller.pwm_x
        self.vehicle.mav.rc_channels_override_send(
            self.vehicle.target_system,                
            self.vehicle.target_component,             
            *rc_channel_values)
        return 
    


class data_base:
    def __init__(self, pid_controller):
        self.pid_controller = pid_controller
        self.time_data = []
        self.x_cmd = []
        self.y_cmd = []
        self.x_cmd_body = []
        self.y_cmd_body = []
        self.x_data = []
        self.y_data = []
        self.x_data_body = []
        self.y_data_body = []
        self.theta_cmd = []
        self.phi_cmd = []
        self.theta_cmd_degree = []
        self.phi_cmd_degree = []
        self.theta_cmd_radians = []
        self.phi_cmd_radians = []
        self.pwm_theta = []
        self.pwm_phi = []
        self.roll_degree = []
        self.pitch_degree = []
        self.yaw_degree = []
        self.roll_radians = []
        self.pitch_radians = []
        self.yaw_radians = []
        self.x_error =[]
        self.x_integral =[]
        self.y_integral=[]
        self.x_error=[]
        self.y_error=[]
        self.kp_x=[]
        self.ki_x=[]
        self.kd_x=[]
        self.kp_y=[]
        self.ki_y=[]
        self.kd_y=[]
        self.not_filt_x=[]
    def append_values(self, elapsed_time, position, error_x, error_y, control_x, control_y, kp, ki, kd, integral_x, integral_y, derivative_x, derivative_y):
     
        """Append current values to all data lists"""
        # Get current position from QTM tracker
        position = self.pid_controller.qtm_tracker.get_position()
        
        # Time data
        self.time_data.append(elapsed_time)
        
        # Command positions (global frame)
        self.x_cmd.append(self.pid_controller.target_x)
        self.y_cmd.append(self.pid_controller.target_y)
        
        # Command positions (body frame)
        self.x_cmd_body.append(self.pid_controller.command_x_body)
        self.y_cmd_body.append(self.pid_controller.command_y_body)
        
        # Actual positions (global frame)
        self.x_data.append(position['x'] if position else 0.0)
        self.y_data.append(position['y'] if position else 0.0)
        
        
        # Actual positions (body frame)
        self.x_data_body.append(self.pid_controller.measured_x_body)
        self.y_data_body.append(self.pid_controller.measured_y_body)
        
        # Control commands - calculate them here to avoid double computation
        error_x = self.pid_controller.command_x_body - self.pid_controller.measured_x_body
        error_y = self.pid_controller.command_y_body - self.pid_controller.measured_y_body
        
        # Get the current control outputs (these should be calculated before calling this method)
        current_time = time.time()
        #dt = 0.02
        
        # Store the current PID outputs that were calculated in the control loop
        # control_x = self.pid_controller.kp * error_x + self.pid_controller.ki * self.pid_controller.integral_x + self.pid_controller.kd * (error_x - self.pid_controller.last_error_x) / dt if dt > 0 else 0
        # control_y = self.pid_controller.kp * error_y + self.pid_controller.ki * self.pid_controller.integral_y + self.pid_controller.kd * (error_y - self.pid_controller.last_error_y) / dt if dt > 0 else 0
        # control_x, control_y = self.pid_controller.compute_pid(error_x, error_y, dt) 
        # # Clamp outputs
        # control_x = max(min(control_x, self.pid_controller.output_limits[1]), self.pid_controller.output_limits[0])
        # control_y = max(min(control_y, self.pid_controller.output_limits[1]), self.pid_controller.output_limits[0])
        self.integral_x =self
        self.x_integral.append(self.pid_controller.integral_x)
        self.y_integral.append(self.pid_controller.integral_y)
        # self.x_kp.append(self.pid_controller.kp*error_x)
        # self.y_kp.append(self.pid_controller.kp*error_y)
        self.x_error.append(error_x)
        self.y_error.append(error_y)
        
        # Control commands in radians and degrees
        self.theta_cmd_radians.append(control_x)
        self.phi_cmd_radians.append(control_y)
        self.theta_cmd_degree.append(math.degrees(control_x))
        self.phi_cmd_degree.append(math.degrees(control_y))
        
        # For backwards compatibility (these are the same as above)
        self.theta_cmd.append(control_x)
        self.phi_cmd.append(control_y)
        
        # PWM values
        self.pwm_theta.append(self.pid_controller.pwm_x)
        self.pwm_phi.append(self.pid_controller.pwm_y)

        self.kp_x.append(kp*error_x)
        self.ki_x.append(ki*integral_x)
        self.kd_x.append(kd*derivative_x)

        self.kp_y.append(kp*error_y)
        self.ki_y.append(ki*integral_y)
        self.kd_y.append(kd*derivative_y)
        self.not_filt_x.append(kp*self.pid_controller.non_f_x)

        # Rotation data (from QTM tracker)
        if hasattr(self.pid_controller, 'rotation') and self.pid_controller.rotation is not None:
            roll_rad = math.radians(self.pid_controller.rotation[0])
            pitch_rad = math.radians(self.pid_controller.rotation[1])
            yaw_rad = math.radians(self.pid_controller.rotation[2])
            
            self.roll_radians.append(roll_rad)
            self.pitch_radians.append(pitch_rad)
            self.yaw_radians.append(yaw_rad)
            
            self.roll_degree.append(self.pid_controller.rotation[0])
            self.pitch_degree.append(self.pid_controller.rotation[1])
            self.yaw_degree.append(self.pid_controller.rotation[2])
        else:
            # Default values if rotation data is not available x_data_body
            self.yaw_degree.append(0.0)
   

    def save_data(self):
        print("Saving data to CSV file...")
        filename = f'trajectory_data_nine.csv'
        try:
            with open(filename, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                # Define column headers
                writer.writerow([
                    'Time',
                    'x_cmd', 'y_cmd',
                    'x_cmd_body', 'y_cmd_body',
                    'x_data', 'y_data',
                    'x_data_body', 'y_data_body',
                    'theta_cmd_radians', 'theta_cmd_degrees',
                    'phi_cmd_radians', 'phi_cmd_degrees',
                    'pwm_theta', 'pwm_phi',
                    'roll_radians', 'roll_degree',
                    'pitch_radians', 'pitch_degree',
                    'yaw_radians', 'yaw_degree',
                    'x_integral', 'y_intergal'
                    'x_error', 'y_error',
                    'kp_x', 'kp_y',
                   ' ki_x', 'ki_y', 
                   'kd_x',  'kd_y', 
                   'not_filt_x'              
                    
                ])
                
                # Zip all data together for writing
                rows = zip(
                    self.time_data,
                    self.x_cmd, self.y_cmd,
                    self.x_cmd_body, self.y_cmd_body,
                    self.x_data, self.y_data,
                    self.x_data_body, self.y_data_body,
                    self.theta_cmd_radians, self.theta_cmd_degree,
                    self.phi_cmd_radians, self.phi_cmd_degree,
                    self.pwm_theta, self.pwm_phi,
                    self.roll_radians, self.roll_degree,
                    self.pitch_radians, self.pitch_degree,
                    self.yaw_radians, self.yaw_degree,
                    self.x_integral, self.y_integral,
                    self.x_error, self.y_error,
                    self.kp_x, self.kp_y,
                    self.ki_x, self.ki_y, 
                    self.kd_x, self.kd_y,
                    self.not_filt_x
                    
                )
                
                # Write all rows
                writer.writerows(rows)
            print(f"Data saved to {filename} with {len(self.time_data)} records.")
        except Exception as e:
            print(f"Error saving data: {e}")


# Global variable to store database reference for signal handler
global_database = None

def signal_handler(sig, frame):
    """Handle Ctrl+C signal"""
    print("\nCtrl+C pressed, saving data...")
    if global_database:
        global_database.save_data()
        print("Data saved. Exiting program.")
    else:
        print("No data to save.")
    sys.exit(0)


async def main():
    global global_database
    
    # Initialize QTM tracker
    qtm_tracker = QTMTracker(qtm_ip="192.168.252.1", wanted_body="drone_body")
    # Initialize GPS converter
    gps_converter = GPSConverter(qtm_tracker)
    # Initialize PID controller
    pid_ctrl = PIDController(qtm_tracker)
    # Initialize data logger
    database = data_base(pid_ctrl)
    # Store database reference globally for signal handler
    global_database = database
    # Initialize MAVLink handler
    mav_handler = mav_work(pid_ctrl)
    
    # Register signal handler for Ctrl+C
    signal.signal(signal.SIGINT, signal_handler)
    
    try:
        # Step 1: Connect to QTM and start streaming
        if not await qtm_tracker.start_streaming():
            logger.error("Failed to start QTM streaming")
            return
        logger.info("QTM streaming started")

        
        # Step 2: Connect to MAVLink
        mav_handler.connect_to_mavlink()
        
        # Step 3: Wait for initial QTM data
        await asyncio.sleep(2)
        #time.sleep(5)        
        # Step 4: Arm the vehicle
        mav_handler.arm_vehicle()
        
        # Step 5: Start control loop in background thread, passing the database
        control_task = asyncio.create_task(
            asyncio.to_thread(pid_ctrl.control_loop, database)
        )
        
        # Step 6: Loop to send GPS and RC overrides
        start_time = time.time()
        try:
            while time.time() - start_time < 300:
                gps_converter.process_and_send()
                mav_handler.rc_override()
                await asyncio.sleep(0.04)
        except asyncio.CancelledError:
            logger.info("Main loop cancelled.")

        # Step 7: Disarm the vehicle
        mav_handler.diarm_vehicle()

        # Step 8: Wait for control loop to finish
        await control_task
        
        # Step 9: Save collected data
        database.save_data()

    except KeyboardInterrupt:
        logger.info("Interrupted by user, shutting down...")
        # Save data even if interrupted
        database.save_data()
    except Exception as e:
        logger.error(f"Unexpected error: {e}")
        # Save data even if there's an error
        database.save_data()
    finally:
        await qtm_tracker.stop_streaming()
        logger.info("Shutdown complete")

if __name__ == "__main__":
    asyncio.run(main())
