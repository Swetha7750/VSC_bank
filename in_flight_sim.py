#!/usr/bin/env python3
"""
Drone PID Controller for Position Control
----------------------------------------
This script connects to QTM motion capture system and a drone via MAVLink
to implement position control using PID controllers.
"""

import asyncio
import csv
import json
import math
import signal
import socket
import time
import xml.etree.ElementTree as ET

import numpy as np
import pkg_resources
import qtm
from pymavlink import mavutil
from scipy.spatial.transform import Rotation as R

# ===================== CONFIGURATION PARAMETERS =====================
# System Configuration
WANTED_BODY = "drone_body"
QTM_SERVER = "192.168.252.1"
QTM_PASSWORD = "password"
QTM_FILE = pkg_resources.resource_filename("qtm", "data/demo.qtm")

# MAVLink Configuration
MAVLINK_CONNECTION = "udp:127.0.0.1:14550"  # MAVLink connection string

# UDP GPS Output Configuration
UDP_IP = "127.0.0.1"
UDP_PORT = 25100
RADIUS_OF_EARTH = 6371000.0  # Radius of Earth in meters
ORTHOMETRIC_HEIGHT = 18.893  # Orthometric height in meters

# Reference GPS location
LATITUDE_0 = 50.764905  # Reference latitude in degrees
LONGITUDE_0 = 6.07875   # Reference longitude in degrees
LATITUDE_0_RAD = math.radians(LATITUDE_0)  # Reference latitude in radians

# PID Parameters
X_KP, X_KI, X_KD = 0.25, 0.03, -0.3  # X-axis PID gains
Y_KP, Y_KI, Y_KD = 0.07, 0.0, -0.13  # Y-axis PID gains
TS = 0.02  # Control loop time step (50 Hz)

# PWM Output Limits
MIN_ANGLE = -30
MAX_ANGLE = 30
MIN_PWM = 1000
MAX_PWM = 2000

# Test Duration
TEST_DURATION = 300.0  # Total run time in seconds
CAPTURE_TIME = 30.0    # Time to capture current position
HOLD_TIME = 60.0       # Time to hold captured position

# ===================== GLOBAL VARIABLES =====================
# Global state variables
vehicle = None
armed = False
captured = False
released = False
body_index = {}

# Position and control variables
XX, YY = 0.0, 0.0
roll, pitch, yaw = 0.0, 0.0, 0.0
x_integral, y_integral = 0.0, 0.0

# Data tracking arrays for logging
time_data = []
x_cmd_data = []
XX_data = []
y_cmd_data = []
yy_data = []
theta_cmd_radians_data = []
theta_cmd_degree_data = []
theta_radians = []
theta_degrees_data = []
phi_cmd_radians_data = []
phi_cmd_degrees_data = []
phi_radians_data = []
phi_degrees_data = []
pwmtheta_data = []
pwmphi_data = []
yaw_angle_data = []

# Create UDP socket for position data
udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # IPv4, UDP

# ===================== HELPER FUNCTIONS =====================
def save_data():
    """Save all collected data to CSV file"""
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
    """Extract a name to index dictionary from 6DOF settings XML"""
    xml = ET.fromstring(xml_string)
    body_to_index = {}
    for index, body in enumerate(xml.findall("*/Body/Name")):
        body_to_index[body.text.strip()] = index
    return body_to_index

def angle_to_pwm(angle):
    """Convert angle in degrees to PWM value"""
    # Constrain angle to limits
    angle = max(min(angle, MAX_ANGLE), MIN_ANGLE)
    # Normalize to 0-1 range
    normalized = (angle - MIN_ANGLE) / (MAX_ANGLE - MIN_ANGLE)
    # Convert to PWM range
    pwm = int(MIN_PWM + normalized * (MAX_PWM - MIN_PWM))
    # Ensure within limits
    return max(min(pwm, MAX_PWM), MIN_PWM)

def signal_handler(sig, frame):
    """Handle Ctrl+C interruption"""
    print("Ctrl+C pressed, saving data...")
    save_data()
    print("Data saved. Exiting program.")
    exit(0)

# ===================== MAVLINK FUNCTIONS =====================
async def connect_mavlink():
    """Connect to the vehicle using MAVLink"""
    global vehicle
    try:
        vehicle = mavutil.mavlink_connection(MAVLINK_CONNECTION)
        vehicle.wait_heartbeat()
        print("Connected to vehicle!")
    except Exception as e:
        print(f"Failed to connect to vehicle: {e}")
        raise

async def arm_vehicle():
    """Arm the vehicle"""
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
    """Disarm the vehicle"""
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

async def send_rc_channels(pwm_phi, pwm_theta):
    """Send RC channel override commands to the vehicle"""
    rc_channel_values = [65535 for _ in range(18)]
    rc_channel_values[1] = pwm_phi     # Roll control
    rc_channel_values[2] = pwm_theta   # Pitch control
    vehicle.mav.rc_channels_override_send(
        vehicle.target_system,
        vehicle.target_component,
        *rc_channel_values
    )

# ===================== QTM PACKET HANDLING =====================
def on_qtm_packet(packet):
    """Process QTM motion capture data packet"""
    global XX, YY, roll, pitch, yaw
    
    info, bodies = packet.get_6d()
    if WANTED_BODY in body_index:
        wanted_index = body_index[WANTED_BODY]
        
        # Check if we have valid data
        if math.isnan(bodies[wanted_index][0].x): 
            print("Invalid position data received.")
            return

        # Extract position data and convert to meters
        position, rotation = bodies[wanted_index]
        XX = (round(position[0], 5)) / 1000  # Convert mm to m
        YY = (round(position[1], 5)) / 1000  # Convert mm to m
        ZZ = round(position[2], 3) / 1000    # Convert mm to m
        Z_alt = 184 - ZZ  # Altitude calculation
        
        # Calculate orientation using rotation matrix
        rotation_matrix = np.array(rotation).reshape(3, 3)
        r = R.from_matrix(rotation_matrix.T)
        euler_angles = r.as_euler('xyz', degrees=True)
        roll, pitch, yaw = euler_angles  # XYZ Euler angles
        
        # Convert local coordinates to GPS coordinates
        Latitude = LATITUDE_0 + (XX / RADIUS_OF_EARTH) * (180 / math.pi)
        Longitude = LONGITUDE_0 + (YY / (RADIUS_OF_EARTH * math.cos(LATITUDE_0_RAD))) * (180 / math.pi)
        rounded_lat = round(Latitude, 7)
        rounded_long = round(Longitude, 7)
        
        # Prepare GPS data for UDP transmission
        data = {
            'time_usec': int(time.time() * 1e6),
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
        
        # Send GPS data via UDP
        out_data = json.dumps(data)
        udp_socket.sendto(out_data.encode(), (UDP_IP, UDP_PORT))

# ===================== MAIN CONTROLLER FUNCTION =====================
async def run_pid_controller(connection):
    """Run the PID controller for position control"""
    global x_integral, y_integral, captured, released
    
    await arm_vehicle()
    
    # Initialize PID controller variables
    x_cmd = 0.0
    y_cmd = 0.0
    capture_time = 0.0
    XX_prev = XX
    YY_prev = YY
    
    print("Starting PID trajectory control...")
    start_time = time.time()
    
    # Main control loop
    while (time.time() - start_time) < TEST_DURATION:
        current_time = time.time() - start_time
        
        # Update command targets based on test phase
        if not captured and current_time >= CAPTURE_TIME:
            # Phase 1: Capture current position
            x_cmd = XX
            y_cmd = YY
            capture_time = current_time
            captured = True
            print(f"Position captured at t={current_time:.2f}s: ({x_cmd:.2f}, {y_cmd:.2f})")
            
        elif captured and not released and current_time >= capture_time + HOLD_TIME:
            # Phase 2: Return to origin
            x_cmd = 0.0
            y_cmd = 0.0
            released = True
            print(f"Returning to origin at t={current_time:.2f}s")
        
        # === X-axis PID control ===
        X_error = x_cmd - XX
        x_integral += X_error * TS
        x_integral = max(min(x_integral, 2), -2)  # Anti-windup
        X_dot = (XX - XX_prev) / TS
        
        # PID terms
        xp = X_KP * X_error
        xi = X_KI * x_integral
        xd = X_KD * X_dot
        XX_prev = XX  # Store for next iteration
        
        # Calculate control output and convert to angle
        theta = -(xp + xi + xd)
        theta = max(min(theta, 5), -5)  # Limit control output
        theta_cmd = -(theta * (180 / math.pi))  # Convert to degrees
        
        # === Y-axis PID control ===
        Y_error = y_cmd - YY
        y_integral += Y_error * TS
        y_integral = max(min(y_integral, 2), -2)  # Anti-windup
        Y_dot = (YY - YY_prev) / TS
        
        # PID terms
        yp = Y_KP * Y_error
        yi = Y_KI * y_integral
        yd = Y_KD * Y_dot
        YY_prev = YY  # Store for next iteration
        
        # Calculate control output and convert to angle
        phi = (yp + yi + yd)
        phi = max(min(phi, 5), -5)  # Limit control output
        phi_cmd = phi * (180 / math.pi)  # Convert to degrees
        
        # Convert angles to PWM values
        pwm_theta = angle_to_pwm(theta_cmd)
        pwm_phi = angle_to_pwm(phi_cmd)
        
        # Send control commands to vehicle
        await send_rc_channels(pwm_phi, pwm_theta)
        
        # Store data for logging
        time_data.append(current_time)
        x_cmd_data.append(x_cmd)
        XX_data.append(XX)
        y_cmd_data.append(y_cmd)
        yy_data.append(YY)
        
        theta_cmd_radians_data.append(theta)
        theta_cmd_degree_data.append(theta_cmd)
        pitch_rad = pitch * (math.pi/180)
        theta_radians.append(pitch_rad)
        theta_degrees_data.append(pitch)
        
        phi_cmd_radians_data.append(phi)
        phi_cmd_degrees_data.append(phi_cmd)
        roll_radians = roll * (math.pi/180)
        phi_radians_data.append(roll_radians)
        phi_degrees_data.append(roll)
        
        pwmtheta_data.append(pwm_theta)
        pwmphi_data.append(pwm_phi)
        yaw_angle_data.append(yaw)
        
        # Print status information
        print(f"t={current_time:.2f}s, x_cmd={x_cmd:.2f}, y_cmd={y_cmd:.2f}, XX:{XX:.2f}, YY:{YY:.2f}, x_error:{X_error:.2f}, y_error:{Y_error:.2f}")
        print(f"  -> theta_cmd: {theta:.3f}, phi_cmd: {phi:.3f}, pwm_theta:{pwm_theta}, pwm_phi:{pwm_phi}, theta_degree:{theta_cmd:.2f}, phi_degree:{phi_cmd:.2f}")
        print(f"  -> roll_angle: {roll:.2f}, pitch_angle: {pitch:.2f}, yaw_angle: {yaw:.2f}")
        
        # Wait for next control cycle
        await asyncio.sleep(TS)
    
    # Disarm the vehicle after test is complete
    await disarm_vehicle()
    
    # Save data to CSV
    save_data()

# ===================== MAIN FUNCTIONS =====================
async def main():
    """Main function to setup and run the controller"""
    global body_index
    
    if WANTED_BODY is None:
        print("Define 'WANTED_BODY' first")
        return
    
    # Connect to QTM motion capture system
    try:
        connection = await qtm.connect(QTM_SERVER)
        if connection is None:
            print("Failed to connect to QTM")
            return
    except Exception as e:
        print(f"Failed to connect to QTM: {e}")
        return
    
    # Connect to the vehicle via MAVLink
    await connect_mavlink()
    
    # Take control of QTM and start the test
    try:
        async with qtm.TakeControl(connection, QTM_PASSWORD):
            # Start a new real-time measurement
            await connection.new()
            
            # Get 6DOF settings from QTM
            xml_string = await connection.get_parameters(parameters=["6d"])
            body_index = create_body_index(xml_string)
            
            # Start streaming frames with our packet handler
            await connection.stream_frames(components=["6d"], on_packet=on_qtm_packet)
            
            # Run the PID controller
            await run_pid_controller(connection)
            
    except asyncio.CancelledError:
        print("QTM task was cancelled")
    finally:
        # Clean up resources
        print("Cleaning up resources...")
        await connection.stop()
        
        result = await connection.close()
        if result == b"Closing connection":
            await connection.save("captured_data_1.qtm")
            print("QTM closing done")

def run_until_keyboard_interrupt():
    """Run the main function until keyboard interrupt (Ctrl+C)"""
    signal.signal(signal.SIGINT, signal_handler)
    loop = asyncio.get_event_loop()
    task = loop.create_task(main())
    try:
        loop.run_until_complete(task)
    except KeyboardInterrupt:   
        print("KeyboardInterrupt received. Stopping QTM loop...")
        task.cancel()
        loop.run_until_complete(task)
    finally:
        loop.close()
        print("Event loop closed.")

# ===================== ENTRY POINT =====================
if __name__ == "__main__":
    run_until_keyboard_interrupt()