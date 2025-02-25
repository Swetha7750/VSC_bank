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

# Definitions
wanted_body = "drone_body"
# wanted_body = "testboxx"

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

async def manual_control_send(throttle, pitch, roll, yaw, buttons=0):
    """Send manual control inputs to the vehicle."""
    vehicle.mav.manual_control_send(
        vehicle.target_system,
        pitch,  # Pitch: -1000 to 1000 (negative values pitch down, positive values pitch up)
        roll,   # Roll: -1000 to 1000 (negative values roll left, positive values roll right)
        throttle,  # Throttle: 0 to 1000 (0 = no throttle, 1000 = full throttle)
        yaw,    # Yaw: -1000 to 1000 (negative values yaw left, positive values yaw right)
        buttons  # Buttons: Bitmask for buttons (e.g., 1 = button 1, 1 << 3 = button 4, etc.)
    )
    print(f"Manual control sent: Throttle={throttle}, Pitch={pitch}, Roll={roll}, Yaw={yaw}, Buttons={buttons}")

async def land_vehicle():
    """ Land the vehicle """
    vehicle.mav.command_long_send(
        vehicle.target_system, vehicle.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0)
    msg = vehicle.recv_match(type='COMMAND_ACK', blocking=True)
    if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
        print("Landing initiated!")
    else:
        print("Failed to initiate landing!")

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
        else:
            await connection.load(QTM_FILE)
            await connection.start(rtfromfile=True)

        # Get 6dof settings from QTM
        xml_string = await connection.get_parameters(parameters=["6d"])
        body_index = create_body_index(xml_string)

        def on_packet(packet):
            info, bodies = packet.get_6d()
            if wanted_body in body_index:
                wanted_index = body_index[wanted_body]
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

                print(f"lat {Latitude:.7f}\trounded lat {rounded_lat:.7f}\tlong {Longitude:.7f}\trounded long {rounded_long:.7f} ZZ {Z_alt:.7f}")

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
             # Start streaming frames
            await connection.stream_frames(components=["6d"], on_packet=on_packet)

            # Arm the vehicle
            await arm_vehicle()

            # Send manual control inputs for takeoff
            start_time = time.time()
            while time.time() - start_time < 10:  # 10 seconds for takeoff
                # Gradually increase throttle from 0 to 1000 (neutral)
                throttle = int(1000 * (time.time() - start_time) / 10)
                await manual_control_send(
                    throttle=throttle,  # Increase throttle
                    pitch=0,            # No pitch
                    roll=0,             # No roll
                    yaw=0               # No yaw
                )
                await asyncio.sleep(0.1)  # Send commands at 10Hz

            # Maintain altitude for 60 seconds
            print("Maintaining altitude...")
            start_time = time.time()
            while time.time() - start_time < 10:  # 60 seconds
                await manual_control_send(
                    throttle=1000,  # Neutral throttle to maintain altitude
                    pitch=0,        # Maintain level
                    roll=0,         # No roll
                    yaw=0           # No yaw
                )
                await asyncio.sleep(0.1)  # Send commands at 10Hz
                print(f"Time elapsed: {time.time() - start_time:.1f} seconds")  # Debugging output

            # Start descending: Gradually decrease throttle over 10 seconds
            print("Starting descent...")
            start_time = time.time()
            while time.time() - start_time < 10:  # 10 seconds for descent
                # Gradually decrease throttle from 1000 (neutral) to 0
                throttle = int(1000 * (1 - (time.time() - start_time) / 10))
                await manual_control_send(
                    throttle=throttle,  # Decrease throttle
                    pitch=0,            # Maintain level
                    roll=0,             # No roll
                    yaw=0               # No yaw
                )
                await asyncio.sleep(0.1)  # Send commands at 10Hz

            # Land the vehicle
            print("Landing...")
            await land_vehicle()

            # Set throttle to zero and wait for the vehicle to stabilize
            await manual_control_send(throttle=0, pitch=0, roll=0, yaw=0)
            await asyncio.sleep(1)  # Small delay before disarming

            # Confirm landing
            while not await is_vehicle_landed():
                print("Waiting for vehicle to land...")
                await asyncio.sleep(1)

            # Disarm the vehicle
            print("Disarming vehicle...")
            await disarm_vehicle()

        except asyncio.CancelledError:
            print("QTM task was cancelled")
        finally:
            print("Cleaning up resources...")
            await connection.stream_frames_stop()
            result = await connection.close()
            if result == b"Closing connection":
                await connection.await_event(qtm.QRTEvent.EventConnectionClosed)
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