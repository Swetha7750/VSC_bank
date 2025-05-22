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

    async def setup_connection(self):
        """Establish connection to QTM system"""
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
        """Create mapping from body names to indices"""
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
        """Initialize QTM streaming"""
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
        """Process incoming QTM packet"""
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
                            rot_matrix = np.array(rotation).reshape(3, 3)
                            self.rotation = self.rotation_matrix_to_euler_xyz(rot_matrix)
                        
                        self.last_valid_position = time.time()
                        self.data_available = True
                        
        except Exception as e:
            logger.error(f"Error processing packet: {e}")

    def rotation_matrix_to_euler_xyz(self, rot_matrix):
        """Convert rotation matrix to Euler angles (XYZ order)"""
        try:
            
            pitch = math.asin(-rot_matrix[2, 0])  # -r13
            
            if abs(rot_matrix[2, 0]) < 0.99999:  # Not at pitch = ±90°
                roll = math.atan2(rot_matrix[2, 1], rot_matrix[2, 2])  # r23/r33
                yaw = math.atan2(rot_matrix[1, 0], rot_matrix[0, 0])   # r12/r11
            else:
                # Gimbal lock case
                roll = 0  # assume roll = 0
                yaw = math.atan2(-rot_matrix[0, 1], rot_matrix[1, 1])  # -r21/r22

            return {
                'roll': roll,
                'pitch': pitch,
                'yaw': yaw
            }
        except Exception as e:
            logger.error(f"Error converting rotation matrix: {e}")
            return {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}

    def get_position(self):
        """Get current position"""
        return self.position.copy() if self.data_available else None

    def get_rotation(self):
        """Get current rotation"""
        return self.rotation.copy() if self.data_available else None

    async def stop_streaming(self):
        """Stop QTM streaming"""
        if self.connection:
            await self.connection.stream_frames_stop()
            await self.connection.disconnect()


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
        """Convert local coordinates to GPS coordinates"""
        # Convert local NED coordinates to lat/lon
        lat = self.reference_lat + (y_local / self.RADIUS_OF_EARTH) * (180 / math.pi)
        lon = self.reference_lon + (x_local / (self.RADIUS_OF_EARTH * math.cos(self.reference_lat_rad))) * (180 / math.pi)
        
        return round(lat, 7), round(lon, 7)

    def create_gps_message(self, position):
        """Create GPS injection message"""
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
            'satellites_visible': 12                  # Number of visible satellites
        }
        
        return gps_data

    def send_gps_data(self, gps_data):
        """Send GPS data via UDP"""
        try:
            json_data = json.dumps(gps_data)
            self.socket.sendto(json_data.encode(), self.udp_address)
            logger.debug(f"Sent GPS data: Lat={gps_data['lat']/1e7:.6f}, Lon={gps_data['lon']/1e7:.6f}, Alt={gps_data['alt']/1000:.2f}m")
        except Exception as e:
            logger.error(f"Error sending GPS data: {e}")

    def process_and_send(self):
        """Process QTM data and send GPS message"""
        position = self.qtm_tracker.get_position()
        if position:
            gps_data = self.create_gps_message(position)
            if gps_data:
                self.send_gps_data(gps_data)
                return True
        return False
    

class position_controller:
    def __init__(self, kp, ki, kd, output_limits, integral_limit=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limits = output_limits
        self.integral_limit = integral_limit
        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = time.time()
        self.first_call = True

    def compute(self, error: float) -> float:
        current_time = time.time()
        dt = current_time - self.last_time
        
        if dt > 0:
            # Calculate integral term
            self.integral += error * dt
            
            # Apply integral limits to prevent windup
            if self.integral_limit is not None:
                self.integral = max(min(self.integral, self.integral_limit), -self.integral_limit)
            
            # Calculate derivative term (avoid spike on first call)
            if self.first_call:
                derivative = 0.0
                self.first_call = False
            else:
                derivative = (error - self.last_error) / dt
            
            # Compute PID output
            output = (self.kp * error + 
                     self.ki * self.integral + 
                     self.kd * derivative)
            
            # Apply output limits
            output = max(min(output, self.output_limits[1]), self.output_limits[0])
            
            # Update state
            self.last_error = error
            self.last_time = current_time
            
            return output
        return 0.0

    def reset(self):
        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = time.time()
        self.first_call = True


class pid_controller:
    def __init__(self, qtm_tracker):
        self.qtm_tracker = qtm_tracker
        self.x_pid = position_controller(kp=0.01, ki=0.0, kd=0.05, 
                                       output_limits=(-5, 5), integral_limit=10.0)
        self.y_pid = position_controller(kp=0.01, ki=0.0, kd=0.05, 
                                       output_limits=(-5, 5), integral_limit=10.0)
        
        # Target position
        self.target_x = 0.0
        self.target_y = 0.0
        
        # Trajectory control variables
        self.captured = False
        self.released = False
        self.capture_x = 0.0
        self.capture_y = 0.0
        self.capture_time = 0.0

    def set_target(self, x: float, y: float):
        """Set the target position"""
        self.target_x = x
        self.target_y = y

    def compute_control(self):
        """Compute control outputs based on current position from QTM"""
        position = self.qtm_tracker.get_position()
        
        if position is not None:
            current_x = position['x']
            current_y = position['y']
            
            # Calculate errors
            error_x = self.target_x - current_x
            error_y = self.target_y - current_y
            
            # Compute control outputs
            control_x = self.x_pid.compute(error_x)
            control_y = self.y_pid.compute(error_y)
            
            return control_x, control_y, error_x, error_y
        
        return 0.0, 0.0, 0.0, 0.0

    def target_trajectory(self):
        """Execute the trajectory capture and release sequence"""
        start_time = time.time()
        
        while (time.time() - start_time) < 300:  # Run for 5 minutes
            current_time = time.time() - start_time
            position = self.qtm_tracker.get_position()
            
            if position is None:
                continue
                
            # Capture phase: Lock the command values at t=30
            if not self.captured and current_time >= 30:
                self.capture_x = position['x']
                self.capture_y = position['y']
                self.capture_time = current_time
                self.captured = True
                self.set_target(self.capture_x, self.capture_y)
                logger.info(f"Position captured at t={current_time:.1f}s: X={self.capture_x:.3f}m, Y={self.capture_y:.3f}m")
            
            # Release phase: Reset to origin after t=60
            elif self.captured and not self.released and current_time >= 60:
                self.set_target(0.0, 0.0)
                self.released = True
                logger.info(f"Target reset to origin at t={current_time:.1f}s")
            
            # Compute control outputs
            control_x, control_y, error_x, error_y = self.compute_control()
            
            # Log current status
            if current_time % 5 < 0.1:  # Log every 5 seconds
                logger.info(f"t={current_time:.1f}s | Pos: X={position['x']:.3f}m Y={position['y']:.3f}m | "
                           f"Target: X={self.target_x:.3f}m Y={self.target_y:.3f}m | "
                           f"Error: X={error_x:.3f}m Y={error_y:.3f}m | "
                           f"Control: X={control_x:.3f} Y={control_y:.3f}")
            
            time.sleep(0.1)  # 10 Hz control loop

    def reset_controllers(self):
        """Reset both PID controllers"""
        self.x_pid.reset()
        self.y_pid.reset()
        self.captured = False
        self.released = False

    def tune_parameters(self, axis: str, kp: float = None, ki: float = None, kd: float = None):
        """Tune PID parameters for a specific axis"""
        controller = self.x_pid if axis.lower() == 'x' else self.y_pid
        
        if kp is not None:
            controller.kp = kp
        if ki is not None:
            controller.ki = ki
        if kd is not None:
            controller.kd = kd
        
        logger.info(f"Tuned {axis.upper()} axis: Kp={controller.kp}, Ki={controller.ki}, Kd={controller.kd}")


async def main():
    # Initialize QTM tracker
    qtm_tracker = QTMTracker(qtm_ip="192.168.252.1", wanted_body="drone_body")
    
    # Initialize GPS converter
    gps_converter = GPSConverter(qtm_tracker, reference_lat=50.764905, reference_lon=6.07875)
    
    # Initialize PID controller
    pid_ctrl = pid_controller(qtm_tracker)
    
    try:
        # Start QTM streaming
        if not await qtm_tracker.start_streaming():
            logger.error("Failed to start QTM streaming")
            return
        
        logger.info("QTM streaming started successfully")
        
        # Wait for initial data
        await asyncio.sleep(2)
        
        # Start trajectory control in a separate task
        trajectory_task = asyncio.create_task(
            asyncio.to_thread(pid_ctrl.target_trajectory)
        )
        
        # Main processing loop for GPS data
        while not trajectory_task.done():
            try:
                # Process and send GPS data
                if gps_converter.process_and_send():
                    # Print current position for monitoring
                    position = qtm_tracker.get_position()
                    rotation = qtm_tracker.get_rotation()
                    
                    if position and rotation:
                        lat, lon = gps_converter.local_to_gps(position['x'], position['y'])
                        print(f"Pos: X={position['x']:.3f}m Y={position['y']:.3f}m Z={position['z']:.3f}m | "
                              f"GPS: {lat:.6f}, {lon:.6f} | "
                              f"Rot: R={math.degrees(rotation['roll']):.1f}° "
                              f"P={math.degrees(rotation['pitch']):.1f}° "
                              f"Y={math.degrees(rotation['yaw']):.1f}°")
                
                await asyncio.sleep(0.1)  # 10 Hz update rate
                
            except KeyboardInterrupt:
                logger.info("Shutting down...")
                trajectory_task.cancel()
                break
            except Exception as e:
                logger.error(f"Error in main loop: {e}")
                await asyncio.sleep(1)
        
        # Wait for trajectory task to complete
        try:
            await trajectory_task
        except asyncio.CancelledError:
            logger.info("Trajectory control cancelled")
    
    finally:
        # Clean shutdown
        await qtm_tracker.stop_streaming()
        logger.info("Shutdown complete")


if __name__ == "__main__":
    asyncio.run(main())
