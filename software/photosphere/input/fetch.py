import pyzed.sl as sl
import json
import cv2
import os

# File to store JSON data
json_filename = "jsons.json"

# Initialize ZED camera
zed = sl.Camera()
init_params = sl.InitParameters()
init_params.depth_mode = sl.DEPTH_MODE.NONE  # Disable depth processing
init_params.coordinate_units = sl.UNIT.DEGREE  # IMU angles in degrees
init_params.camera_fps = 30  # Set FPS

if zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
    print("Failed to open ZED camera")
    exit(1)

# Store initial yaw to calculate relative yaw
initial_yaw = None

# Load existing JSON data if file exists
if os.path.exists(json_filename):
    with open(json_filename, "r") as file:
        try:
            json_data = json.load(file)
        except json.JSONDecodeError:
            json_data = []
else:
    json_data = []

# Grab a frame
runtime_parameters = sl.RuntimeParameters()
if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
    # Retrieve image frame
    image = sl.Mat()
    zed.retrieve_image(image, sl.VIEW.LEFT)

    # Generate image filename
    image_index = len(json_data) + 1  # Unique numbering
    filename = f"img-r1-{image_index:03d}.jpg"
    cv2.imwrite(filename, image.get_data())

    # Retrieve IMU data
    sensors_data = sl.SensorsData()
    if zed.get_sensors_data(sensors_data, sl.TIME_REFERENCE.CURRENT) == sl.ERROR_CODE.SUCCESS:
        imu_data = sensors_data.get_imu_data()
        rpy = imu_data.get_pose().get_euler_angles()  # Roll, Pitch, Yaw

        if initial_yaw is None:
            initial_yaw = rpy[2]  # Set first yaw as reference

        first_yaw = rpy[2] - initial_yaw  # How much the camera turned
        absolute_yaw = 270 + first_yaw  # World-facing yaw

        # Format data in JSON
        output_data = {
            "degrees": first_yaw,  # Relative yaw
            "ring": 1,  # Fixed ring
            "sensors": {
                "fileUri": filename,  # Image filename
                "roll_pitch_yaw": {
                    "pitch": rpy[1],
                    "roll": rpy[0],
                    "yaw": absolute_yaw  # Adjusted world yaw
                }
            }
        }

        # Append new JSON data
        json_data.append(output_data)

        # Save updated JSON list to file
        with open(json_filename, "w") as file:
            json.dump(json_data, file, indent=4)

        print(f"Saved: {filename} and updated {json_filename}")

# Close the camera
zed.close()
