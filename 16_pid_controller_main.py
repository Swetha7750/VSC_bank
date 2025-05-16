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
import csv
from pymavlink import mavutil
import pandas as pd
import signal 
from scipy.spatial.transform import Rotation as R
# Definitions
wanted_body = "drone_body"
# wanted_body = "testboxx"
file_path = 'flight_2_catch.csv'

# Define the UDP address and port
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # IPv4, UDP
out_addr = ("127.0.0.1", 25100)  # Replace port number if necessary
RADIUS_OF_EARTH = 6371000.0  # Radius of Earth in meters
orthometric_height = 18.893  # Orthometric height in meters

# MAVLink connection
MAVLINK_CONNECTION = "udp:127.0.0.1:14550"  # Replace with your MAVLink connection string

# QTM file path
QTM_FILE = pkg_resources.resource_filename("qtm", "data/demo.qtm")

# Global variables for MAVLink connection and vehicle state
vehicle = None
armed = False

# Constants for GPS calculations
LATITUDE_0 = 50.764905  # Reference latitude in degrees
LONGITUDE_0 = 6.07875  # Reference longitude in degrees
LATITUDE_0_RAD = math.radians(LATITUDE_0)  # Reference latitude in radians

# PID parameters
X_kp, x_ki, x_kd = 0.25, 0.03, -0.3
y_kp, y_ki, y_kd = 0.07, 0.0, -0.13
ts = 0.02  # time step = 50 Hz

# Global variables for PID control
XX, YY = 0.0, 0.0
x_integral = 0
y_integral = 0
roll_degrees = 0

# Data tracking
time_data = []
x_cmd_data = []
XX_data = []
y_cmd_data = []
yy_data = []
theta_cmd_radians_data =[] #pitch command values in radians
theta_cmd_degree_data =[]
theta_radians =[] #pitch 
theta_degrees_data =[]


phi_cmd_radians_data=[]
phi_cmd_degreee_data =[]


pwmtheta_data =[]
pwmphi_data =[]
phi_cmd_radians_data =[]

phi_cmd_degrees_data =[]
phi_radians_data =[]
phi_degrees_data =[]

yaw_angle_data =[]
# Capture state variables
captured = False
released = False

def signal_handler(sig, frame):
    print("Ctrl+C pressed, saving data...")
    save_data()
    print("Data saved. Exiting program.")
    exit(0)

def save_data():
    print("Saving data to CSV file...")
    with open('trajectory_data.csv', 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        # Define column headers
        writer.writerow([
            'Time', 'x_cmd', 'XX', 'y_cmd', 'YY',
            'theta_cmd_radians', 'theta_cmd_degrees', 'theta_radians', 'theta_degrees',
            'phi_cmd_radians', 'phi_cmd_degrees', 'phi_radians', 'phi_degrees',
            'pwm_theta', 'pwm_phi', 'yaw_angle'
        ])
        
        # Zip all data together for writing
        rows = zip(
            time_data, x_cmd_data, XX_data, y_cmd_data, yy_data,
            theta_cmd_radians_data, theta_cmd_degree_data, theta_radians, theta_degrees_data,
            phi_cmd_radians_data, phi_cmd_degrees_data, phi_radians_data, phi_degrees_data,
            pwmtheta_data, pwmphi_data, yaw_angle_data
        )
        
        # Write all rows
        writer.writerows(rows)
    print(f"Data saved to trajectory_data.csv with {len(time_data)} records.")

def create_body_index(xml_string):
    """ Extract a name to index dictionary from 6dof settings xml """
    xml = ET.fromstring(xml_string)
    body_to_index = {}
    for index, body in enumerate(xml.findall("*/Body/Name")):
        body_to_index[body.text.strip()] = index
    return body_to_index

async def connect_mavlink():
    """ Connect to the vehicle using MAVLink """
    global vehicle
    try:
        vehicle = mavutil.mavlink_connection(MAVLINK_CONNECTION)
        vehicle.wait_heartbeat()
        print("Connected to vehicle!")
    except Exception as e:
        print(f"Failed to connect to vehicle: {e}")
        raise

async def arm_vehicle():
    """ Arm the vehicle """
    global armed
    vehicle.mav.command_long_send(
        vehicle.target_system, vehicle.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
    msg = vehicle.recv_match(type='COMMAND_ACK', blocking=True)
    if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
        armed = True
        print("Vehicle armed!")
    else:
        print("Failed to arm vehicle!")

async def disarm_vehicle():
    """ Disarm the vehicle """
    global armed
    vehicle.mav.command_long_send(
        vehicle.target_system, vehicle.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0)
    msg = vehicle.recv_match(type='COMMAND_ACK', blocking=True)
    if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
        armed = False
        print("Vehicle disarmed!")
    else:
        print("Failed to disarm vehicle!")

async def set_rc_channel_pwm(channel_id, pwm):
    if channel_id < 1 or channel_id >18:
        print('sorry the channel doesnt exist')
        return
    rc_channel_values = [65535 for _ in range(18)]
    rc_channel_values[channel_id - 1] = pwm
    vehicle.mav.rc_channels_override_send(
        vehicle.target_system,                # target_system
        vehicle.target_component,             # target_component
        *rc_channel_values)                  # RC channel list, in microseconds.

async def main():
    global armed, body_index, XX, YY, roll_degrees
    global x_integral, y_integral, captured, released

    if wanted_body is None:
        print("Define 'wanted_body' first")
        return

    # Connect to QTM
    try:
        connection = await qtm.connect("192.168.252.1")
        if connection is None:
            print("Failed to connect to QTM")
            return
    except Exception as e:
        print(f"Failed to connect to QTM: {e}")
        return

    # Connect to the vehicle
    await connect_mavlink()

    # Take control of QTM
    async with qtm.TakeControl(connection, "password"):
        realtime = True
        if realtime:
            await connection.new()
        else:
            await connection.load(QTM_FILE)
            await connection.start(rtfromfile=False)
            
        # await connection.start()
        # await connection.await_event(qtm.QRTEvent.EventCaptureStarted, timeout=10)
        # Get 6dof settings from QTM
        xml_string = await connection.get_parameters(parameters=["6d"])
        body_index = create_body_index(xml_string)

        def on_packet(packet):
            global XX, YY, roll_degrees, roll, pitch, yaw
            
            info, bodies = packet.get_6d()
            if wanted_body in body_index:
                wanted_index = body_index[wanted_body]
                position, rotation = bodies[wanted_index]
                rotation_matrix = np.array(rotation).reshape(3, 3)
                r = R.from_matrix(rotation_matrix.T)
                euler_angles = r.as_euler('xyz', degrees=True)
                roll, pitch, yaw = euler_angles  # assuming XYZ order
                
                if math.isnan(bodies[wanted_index][0].x): 
                    print("Invalid position data received.")
                    return

                position, rotation = bodies[wanted_index]
                XX = (round(position[0], 5)) / 1000
                YY = (round(position[1], 5)) / 1000
                ZZ = round(position[2], 3) / 1000
                Z_alt = 184 - ZZ

                # Calculate roll angle from rotation matrix
                

                Latitude = LATITUDE_0 + (XX / RADIUS_OF_EARTH) * (180 / math.pi)
                Longitude = LONGITUDE_0 + (YY / (RADIUS_OF_EARTH * math.cos(LATITUDE_0_RAD))) * (180 / math.pi)
                rounded_lat = round(Latitude, 7)
                rounded_long = round(Longitude, 7)

                data = {
                    'time_usec': int(time.time() * 1e6),  # Use current time
                    'gps_id': 0,
                    'ignore_flags': 8,
                    'time_week_ms': 0,
                    'time_week': 0,
                    'fix_type': 3,
                    'lat': int(rounded_lat * 1e7),
                    'lon': int(rounded_long * 1e7),
                    'alt': Z_alt,
                    'hdop': 1,
                    'vdop': 1,
                    'vn': 0,
                    've': 0,
                    'vd': 0,
                    'speed_accuracy': 0,
                    'horiz_accuracy': 0,
                    'vert_accuracy': 0,
                    'satellites_visible': 9,
                    'yaw': None,
                }

                out_data = json.dumps(data)
                s.sendto(out_data.encode(), out_addr)

        # Start streaming frames
        try:
            await connection.stream_frames(components=["6d"], on_packet=on_packet)
            await arm_vehicle()

            # === STRAIGHT LINE PID CONTROLLER ===
            x_cmd = 0.0
            y_cmd = 0.0
            capture_time = 0.0
            XX_prev = XX
            YY_prev = YY

            duration = 300.0  # Total run time in seconds
            print("Starting PID trajectory control...")
            start_time = time.time()

            while time.time() - start_time < duration:
                current_time = time.time() - start_time

                # Lock the command values at t=30
                if not captured and current_time >= 30:
                    x_cmd = XX
                    y_cmd = YY
                    capture_time = current_time
                    captured = True
                    
                # Reset to 0 after 60 seconds
                elif captured and not released and current_time >= capture_time + 60:
                    x_cmd = 0
                    y_cmd = 0
                    released = True
                
                # --- X Axis ---
                X_error = x_cmd - XX
                x_integral += X_error * ts
                x_integral = max(min(x_integral, 2), -2)  # Anti-windup
                X_dot = (XX - XX_prev) / ts
                xp = X_kp * X_error
                xi = x_ki * x_integral
                xd = x_kd * X_dot
                XX_prev = XX  # previous position 

                max_theta = 5
                min_theta = -5
                theta = -(xp + xi + xd)
                theta = max(min(theta, max_theta), min_theta)
                theta_cmd = theta * (180 / math.pi)

                # --- Y Axis ---
                Y_error = y_cmd - YY
                y_integral += Y_error * ts
                y_integral = max(min(y_integral, 2), -2)  # Anti-windup
                Y_dot = (YY - YY_prev) / ts
                yp = y_kp * Y_error
                yi = y_ki * y_integral
                yd = y_kd * Y_dot
                YY_prev = YY

                phi = (yp + yi + yd)
                phi = max(min(phi, 5), -5)
                phi_cmd = phi * (180 / math.pi)
                
                # Convert angles to PWM values
                min_angle = -30
                max_angle = 30
                min_pwm = 1000
                max_pwm = 2000
                

                # For theta
                theta_cmd = max(min(theta_cmd, max_angle), min_angle)
                normalized = (theta_cmd - min_angle) / (max_angle - min_angle)
                pwm_theta = int(min_pwm + normalized * (max_pwm - min_pwm))
                pwm_theta = max(min(pwm_theta, max_pwm), min_pwm)

                # For phi
                phi_cmd = max(min(phi_cmd, max_angle), min_angle)
                normalized = (phi_cmd - min_angle) / (max_angle - min_angle)
                pwm_phi = int(min_pwm + normalized * (max_pwm - min_pwm))
                pwm_phi = max(min(pwm_phi, max_pwm), min_pwm)

                # Store data for logging
                time_data.append(current_time)
                x_cmd_data.append(x_cmd)
                XX_data.append(XX)
                y_cmd_data.append(y_cmd)
                yy_data.append(YY)
                theta_cmd_radians_data.append(theta)
                
                theta_cmd_degree_data.append(theta_cmd)
                pitch_rad = (pitch*(math.pi/180))
                theta_radians.append(pitch_rad)
                theta_degrees_data.append(pitch)

                phi_cmd_radians_data.append(phi)

                phi_cmd_degrees_data.append(phi_cmd)
                roll_radians = (roll*(math.pi/180))
                phi_radians_data.append(roll_radians)
                phi_degrees_data.append(roll)


                pwmtheta_data.append(pwm_theta)
                pwmphi_data.append(pwm_phi)
                
                yaw_angle_data.append(yaw)
        
                # Send commands to vehicle
                await set_rc_channel_pwm(2, pwm_phi)
                await set_rc_channel_pwm(3, pwm_theta)
                print(f"t={current_time:.2f}s, x_cmd={x_cmd:.2f}, y_cmd={y_cmd:.2f}, XX:{XX:.2f}, YY:{YY:.2f}, x_error:{X_error:.2f}, y_error:{Y_error:.2f}")
                print(f"  -> theta_cmd: {theta:.3f}, phi_cmd: {phi:.3f}, pwm_theta:{pwm_theta}, pwm_phi:{pwm_phi}, theta_degree:{theta_cmd:.2f}, phi_degree:{phi_cmd:.2f}")# roll_angle:{euler_angles:2f}")# roll{roll_degrees:.2f}")
                print(f"  -> roll_angle: {roll:.2f}, pitch_angle: {pitch:.2f}, yaw_angle: {yaw:.2f}")

                await asyncio.sleep(ts)  # Run at 50Hz

            # Disarm the vehicle after trajectory is complete
            await disarm_vehicle()
            
            # Save data to CSV
            with open('trajectory_data.csv', 'w', newline='') as csvfile:
                import csv
                writer = csv.writer(csvfile)
                writer.writerow(['Time', 'x_cmd', 'XX', 'y_cmd', 'YY'])  # Column headers
                for t, xc, x, yc, y in zip(time_data, x_cmd_data, XX_data, y_cmd_data, yy_data):
                    writer.writerow([t, xc, x, yc, y])
            print("Data saved to trajectory_data.csv")

        except asyncio.CancelledError:
            print("QTM task was cancelled")
        finally:
            print("Cleaning up resources...")
            await connection.stop()

            result = await connection.close()
            if result == b"Closing connection":
                # await connection.await_event(qtm.QRTEvent.EventConnectionClosed, timeout=10)
                # await connection.await_event(qtm.QRTEvent.EventCaptureStopped, timeout=10)
                await connection.save("captured_data_1.qtm")
                print("QTM closing done")

def run_until_keyboard_interrupt():
    global main_task, loop
    signal.signal(signal.SIGINT, signal_handler)
    loop = asyncio.get_event_loop()
    task = loop.create_task(main())
    try:
        loop.run_until_complete(task)
    except KeyboardInterrupt:   
        print("KeyboardInterrupt received. Stopping QTM loop...")
        task.cancel()
        loop.run_until_complete(task)
        
        # Save data to CSV when interrupted
        # with open('trajectory_data.csv', 'w', newline='') as csvfile:
        #     import csv
        #     writer = csv.writer(csvfile)
        #     writer.writerow(['Time', 'x_cmd', 'XX', 'y_cmd', 'YY'])  # Column headers
        #     for t, xc, x, yc, y in zip(time_data, x_cmd_data, XX_data, y_cmd_data, yy_data):
        #         writer.writerow([t, xc, x, yc, y])
        
       
    finally:
        loop.close()
        print("Event loop closed.")

if __name__ == "__main__":
    run_until_keyboard_interrupt()
