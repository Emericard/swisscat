# # #! /usr/bin/env python3
# # # Copyright 2021 Samsung Research America
# # #
# # # Licensed under the Apache License, Version 2.0 (the "License");
# # # you may not use this file except in compliance with the License.
# # # You may obtain a copy of the License at
# # #
# # #     http://www.apache.org/licenses/LICENSE-2.0
# # #
# # # Unless required by applicable law or agreed to in writing, software
# # # distributed under the License is distributed on an "AS IS" BASIS,
# # # WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# # # See the License for the specific language governing permissions and
# # # limitations under the License.

# # from geometry_msgs.msg import PoseStamped
# # from submodules.socket_client import SocketClient
# # import logging
# # from tf_transformations import quaternion_from_euler
# # import rclpy
# # import configparser

import matplotlib.pyplot as plt
import datetime

# Step 1: Read the data from the log file
log_file = "/home/coderey/ros2_ws/test_file.log"
with open(log_file, "r") as file:
    lines = file.readlines()

timestamps = []
calibration_values = []
odom_values = []

# Step 2: Extract the relevant information
for line in lines:
    if "CalibratedPos" in line:
        parts = line.split(",")
        print(parts)
        timestamp = datetime.datetime.strptime(parts[0], "%H:%M:%S")
        x = float(parts[4].split(":")[1])
        y = float(parts[5].split(":")[1])
        rz = float(parts[6].split(":")[1])
        calibration_values.append((timestamp, x, y, rz))
    elif "OdomGlob" in line:
        parts = line.split(",")
        timestamp = datetime.datetime.strptime(parts[0], "%H:%M:%S")
        x = float(parts[4].split(":")[1])
        y = float(parts[5].split(":")[1])
        rz = float(parts[6].split(":")[1])
        odom_values.append((timestamp, x, y, rz))

# Step 3: Calculate the differences
x_diff = []
y_diff = []
rz_diff = []

for calibration_entry in calibration_values:
    calibration_timestamp = calibration_entry[0]
    closest_odom_entry = None
    closest_odom_timestamp = None

    for odom_entry in odom_values:
        odom_timestamp = odom_entry[0]
        if odom_timestamp <= calibration_timestamp:
            if closest_odom_timestamp is None or odom_timestamp > closest_odom_timestamp:
                closest_odom_entry = odom_entry
                closest_odom_timestamp = odom_timestamp

    if closest_odom_entry is not None:
        x_diff.append(calibration_entry[1] - closest_odom_entry[1])
        y_diff.append(calibration_entry[2] - closest_odom_entry[2])
        rz_diff.append(calibration_entry[3] - closest_odom_entry[3])

# Step 4: Plot the differences over time
timestamps = [entry[0] for entry in calibration_values]
print(x_diff)
plt.figure(figsize=(10, 6))
plt.subplot(3, 1, 1)
plt.plot(timestamps, x_diff)
plt.title("X Difference")
plt.xlabel("Time")
plt.ylabel("Difference")

plt.subplot(3, 1, 2)
plt.plot(timestamps, y_diff)
plt.title("Y Difference")
plt.xlabel("Time")
plt.ylabel("Difference")

plt.subplot(3, 1, 3)
plt.plot(timestamps, rz_diff)
plt.title("RZ Difference")
plt.xlabel("Time")
plt.ylabel("Difference")

plt.tight_layout()
plt.show()

