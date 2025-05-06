import asyncio
import xml.etree.ElementTree as ET
import pkg_resources
import struct
import qtm
import math
import socket
import json
import time
from pymavlink import mavutil
import pandas as pd
import os 

# Definitions
wanted_body = "drone_body"
# wanted_body = "testboxx"
file_path = 'flight_2_catch.csv'
global X_cmd_series

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


async def pid_controller(file_path):
    if not os.path.exists(file_path):
        print(f"Error: File '{file_path}' not found!")
    else:
        try:
            df = pd.read_csv(file_path, usecols=["pos/x6_x", "pos/x7_y", "pos/x8_z", "__time"])
                  
            #global X_cmd      
            # Clean data
            df = df.dropna(subset=["pos/x6_x"])  # Remove missing values
            #df = df.drop_duplicates()  # Remove duplicates
            downsampled_data = df.iloc[::25]
            
            X_cmd_series = downsampled_data["pos/x6_x"].reset_index(drop=True)
             #Y_cmd, Z_cmd
#             X_cmd = df['pos/x6_x']
#             Y_cmd = df['pos/x7_y']
#             Z_cmd =df['pos/x8_z']
            return X_cmd_series, downsampled_data
        except:
            print('Error in something')

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

# async def is_vehicle_landed():
#     """ Check if the vehicle has landed """
#     # Implement logic to check if the vehicle has landed
#     # For example, you can check the altitude or the vehicle's mode
#     return True  # Placeholder, replace with actual logic

async def main():
    global armed

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
            
            
            #await connection.await_event(qtm.QRTEvent.EventCaptureStarted, timeout=10)
            
                #print("was not able to capture")
            
        else:
            await connection.load(QTM_FILE)
            await connection.start(rtfromfile=False)
            # await connection.new()
            
            
        # await connection.start()
        # await connection.await_event(qtm.QRTEvent.EventCaptureStarted, timeout=10)
        # Get 6dof settings from QTM
        xml_string = await connection.get_parameters(parameters=["6d"])
        body_index = create_body_index(xml_string)

        def on_packet(packet):
            
            info, bodies = packet.get_6d()
            if wanted_body in body_index:
                wanted_index = body_index[wanted_body]
                position, rotation = bodies[wanted_index]
                if math.isnan(bodies[wanted_index][0].x): 
                    print("Invalid position data received.")
                    return

                position, rotation = bodies[wanted_index]
                XX = (round(position[0], 5)) / 1000
                YY = (round(position[1], 5)) / 1000
                ZZ = round(position[2], 3) / 1000
                Z_alt = 184 - ZZ

                Latitude = LATITUDE_0 + (XX / RADIUS_OF_EARTH) * (180 / math.pi)
                Longitude = LONGITUDE_0 + (YY / (RADIUS_OF_EARTH * math.cos(LATITUDE_0_RAD))) * (180 / math.pi)
                rounded_lat = round(Latitude, 7)
                rounded_long = round(Longitude, 7)
                
                #print(f"lat {Latitude:.7f}\trounded lat {rounded_lat:.7f}\tlong {Longitude:.7f}\trounded long {rounded_long:.7f} ZZ {Z_alt:.7f}")

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
            file_path = 'flight_2_catch.csv'
            X_cmd_series, _ = await pid_controller(file_path)
            # Arm the vehicle
            # await arm_vehicle()

            # Send manual control inputs for takeoff
            start_time = time.time()
            while time.time() - start_time < 6000:  
                # Gradually increase throttle from 0 to 1000 (neutral)
                current_time = time.time() - start_time

                # Default to neutral angle (0°)
                angle = 0

                # Define angle changes at specific times
                if current_time >20.0 and current_time<21.0:
                    angle = 5
                elif current_time > 24.0 and current_time<25.0:
                    angle = -5
                else:
                    angle = 0
            
                min_angle=-30
                max_angle =30
                min_pwm =1000
                max_pwm =2000

                pwm_pitch =1500
                angle= max(min(angle, max_angle), min_angle)
                normalized = (angle-min_angle)/(max_angle-min_angle)
                pwm=int(min_pwm+normalized*(max_pwm-min_pwm))
                pwm=max(min(pwm, max_pwm),min_pwm)
                
                 # --- Get X_cmd from series based on current time ---
                idx = int(current_time / 0.1)
                if idx < len(X_cmd_series):
                    X_cmd = X_cmd_series.iloc[idx]
                else:
                    X_cmd = X_cmd_series.iloc[-1]  # Use last value if time exceeds
            
                
                await set_rc_channel_pwm(2, pwm)
                await set_rc_channel_pwm(3, pwm_pitch)
                await asyncio.sleep(0.1)  # Send commands at 10Hz
                file_path = 'flight_2_catch.csv'
                await pid_controller(file_path)
                print(f"Time elapsed: {time.time() - start_time:.1f} seconds, pwm: {pwm} X_cmd:{X_cmd} ")#Y_cmd:{Y_cmd}, Z_cmd:{Z_cmd}")  # Debugging output
                await asyncio.sleep(0.1)  # Send commands at 10Hz

            
           
            await disarm_vehicle()
            await asyncio.sleep(1)  # Small delay before disarming

            # # Confirm landing
            # while not await is_vehicle_landed():
            #     print("Waiting for vehicle to land...")
            #     await asyncio.sleep(1)

            # Disarm the vehicle
            print("Disarming vehicle...")
            #await disarm_vehicle()

        except asyncio.CancelledError:
            print("QTM task was cancelled")
        finally:
            print("Cleaning up resources...")
            # await connection.stream_frames_stop()
            await connection.stop()
            # await connection.save("experimentData_001", False)
            

            result = await connection.close()
            if result == b"Closing connection":
                await connection.await_event(qtm.QRTEvent.EventConnectionClosed,timeout=10)
                # await connection.await_event(qtm.QRTEvent.EventCaptureStopped,timeout=10)
                # #await connection.await_event(qtm.QRTEvent.EventCaptureSaved)
                # await connection.save("captured_data_1.qtm")
                print("QTM closing done")

def run_until_keyboard_interrupt():
    loop = asyncio.get_event_loop()
    task = loop.create_task(main())
    try:
        loop.run_until_complete(task)
    except KeyboardInterrupt:   
        print("KeyboardInterrupt received. Stopping QTM loop...")
        task.cancel()
        loop.run_until_complete(task)
        print("All stuff done")
    finally:
        loop.close()
        print("Event loop closed.")

if __name__ == "__main__":
    run_until_keyboard_interrupt()