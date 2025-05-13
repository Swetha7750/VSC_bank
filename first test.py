import asyncio
import xml.etree.ElementTree as ET
import pkg_resources
import struct
import qtm
import math
import socket
import json
import time
import os
import pandas as pd
import numpy as np

# === CONFIG ===
wanted_body = "drone_body"
QTM_IP = "192.168.252.1"  # Your QTM server IP
PASSWORD = "password"
RADIUS_OF_EARTH = 6371000.0
out_addr = ("127.0.0.1", 25100)
latitude_0 = 50.764905
longitude_0 = 6.07875

# PID parameters
X_kp, x_ki, x_kd = 0.25, 0.03, -0.3
y_kp, y_ki, y_kd = 0.07, 0.0, -0.13
ts = 0.02  # time step = 50 Hz

# Shared drone position (updated from QTM)
XX, YY = 0.0, 0.0

# Socket setup
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# === PID Trajectory Generator ===
async def straight_line():
    global XX, YY

    x_cmd_value = 0.1
    y_start = 0.1
    y_step = 0.1
    duration = 60.0
    frequency = 50
    dt = 1 / frequency

    t = np.arange(0, duration, dt)
    y_cmd = np.array([y_start + y_step * int(ti) for ti in t])
    x_cmd = np.full_like(t, x_cmd_value)

    prev_XX, prev_YY = XX, YY

    print("Starting PID trajectory control...")

    for i in range(len(t)):
        # --- X Axis ---
        X_error = x_cmd[i] - XX
        X_dot = (XX - prev_XX) / ts
        prev_XX = XX
        theta_cmd = -(X_kp * X_error + x_ki * X_error * ts + x_kd * X_dot)

        # --- Y Axis ---
        Y_error = y_cmd[i] - YY
        Y_dot = (YY - prev_YY) / ts
        prev_YY = YY
        phi_cmd = y_kp * Y_error + y_ki * Y_error * ts + y_kd * Y_dot

        print(f"t={t[i]:.2f}s, x_cmd={x_cmd[i]:.2f}, y_cmd={y_cmd[i]:.2f}")
        print(f"  -> theta_cmd: {theta_cmd:.3f}, phi_cmd: {phi_cmd:.3f}")
        await asyncio.sleep(dt)

# === Create mapping from body name to index ===
def create_body_index(xml_string):
    xml = ET.fromstring(xml_string)
    return {body.text.strip(): idx for idx, body in enumerate(xml.findall("*/Body/Name"))}

# === QTM Streaming Callback ===
def on_packet(packet):
    global XX, YY

    info, bodies = packet.get_6d()
    if wanted_body not in body_index:
        return

    idx = body_index[wanted_body]
    pos, _ = bodies[idx]

    if math.isnan(pos.x):
        return

    # Convert mm to meters
    XX = round(pos.x, 5) / 1000.0
    YY = round(pos.y, 5) / 1000.0
    ZZ = round(pos.z, 5)

    # Convert local to GPS
    lat_rad = math.radians(latitude_0)
    lat = latitude_0 + (YY / RADIUS_OF_EARTH) * (180 / math.pi)
    lon = longitude_0 + (XX / (RADIUS_OF_EARTH * math.cos(lat_rad))) * (180 / math.pi)

    # Send GPS-like UDP packet
    gps_data = {
        'time_usec': 0,
        'gps_id': 0,
        'ignore_flags': 8,
        'time_week_ms': 0,
        'time_week': 0,
        'fix_type': 3,
        'lat': int(lat * 1e7),
        'lon': int(lon * 1e7),
        'alt': ZZ,
        'hdop': 1,
        'vdop': 1,
        'vn': 0,
        've': 0,
        'vd': 0,
        'speed_accuracy': 0.5,
        'horiz_accuracy': 1.0,
        'vert_accuracy': 1.5,
        'satellites_visible': 9
    }

    out_data = json.dumps(gps_data)
    s.sendto(out_data.encode(), out_addr)

# === Main Asynchronous Function ===
async def main():
    global body_index

    # Connect to QTM
    connection = await qtm.connect(QTM_IP)
    if connection is None:
        print("Failed to connect to QTM.")
        return

    # Take control of QTM
    async with qtm.TakeControl(connection, PASSWORD):
        await connection.new()  # Start real-time measurement

    # Get body index mapping
    xml_string = await connection.get_parameters(parameters=["6d"])
    body_index = create_body_index(xml_string)

    # Run QTM streaming and PID control concurrently
    await asyncio.gather(
        connection.stream_frames(components=["6d"], on_packet=on_packet),
        straight_line()
    )

# === Run Script ===
if __name__ == "__main__":
    asyncio.run(main())
