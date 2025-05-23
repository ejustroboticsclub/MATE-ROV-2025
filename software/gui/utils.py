# UTILS.Py

import cv2
from PyQt5.QtCore import QThread, pyqtSignal
import paramiko
import numpy as np
import os
from multiprocessing import Process, Array
import math
from screeninfo import get_monitors
import time
from rov25.gui_backend import start_ros

# Singleton Class for ROS Interface


class ROSInterface:
    _instance = None

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(ROSInterface, cls).__new__(cls)
            cls._instance.init_ros()
        return cls._instance

    def init_ros(self):
        self.node = start_ros()


class VideoCaptureThread(QThread):
    stop_signal = pyqtSignal()

    def __init__(self):
        super().__init__()
        self.cap = None
        self.fps = 0
        self.frame_width = 0
        self.frame_height = 0
        self.fourcc = cv2.VideoWriter_fourcc(*"avc1")
        self.video_writer = None
        self.recording = False

    def run(self):
        while self.cap.isOpened() and self.recording:
            ret, frame = self.cap.read()
            if ret:
                self.video_writer.write(frame)
            else:
                break

    def start_recording(self, filename="PhotosphereTask.mp4"):
        if not self.recording:
            self.cap = cv2.VideoCapture(0)
            if not self.cap.isOpened():
                print("Error: Unable to access the camera.")
                return

            self.fps = int(self.cap.get(cv2.CAP_PROP_FPS))
            self.frame_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            self.frame_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

            self.video_writer = cv2.VideoWriter(
                filename, self.fourcc, self.fps, (self.frame_width, self.frame_height)
            )
            self.recording = True
            self.start()

    def stop_recording(self):
        if self.recording:
            self.recording = False
            self.cap.release()
            self.video_writer.release()


class CameraStreamer(QThread):
    def __init__(self, ips):
        super().__init__()
        self.ips = ips

        self.grid_width = 1920
        self.grid_height = 1080

        self.zed_cam_width = 1344
        self.zed_cam_height = 376

        self.top_cam_width = 960
        self.top_cam_height = 270

        self.bottom_cam_width = 960
        self.bottom_cam_height = 434

        self.shared_arrays = (
            [Array("B", self.top_cam_width * self.top_cam_height * 3) for _ in range(2)]
            + [
                Array("B", self.bottom_cam_width * self.bottom_cam_height * 3)
                for _ in range(2)
            ]
            + [Array("B", self.zed_cam_width * self.zed_cam_height * 3)]
        )

        self.processes = []

    def get_pipeline(self, ip_url):
        return (
            f"rtspsrc location={ip_url} latency=0 buffer-mode=auto "
            "! decodebin ! videoconvert ! appsink max-buffers=1 drop=True"
        )

    def display_camera(self, ip, index, shared_array, width, height):
        while True:
            pipeline = self.get_pipeline(ip)

            cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

            if not cap.isOpened():
                print(f"Error opening pipeline for camera {index}")
                time.sleep(5)
                continue

            while cap.isOpened():
                ret, frame = cap.read()
                if not ret:
                    print(f"Connection lost for camera {index}, retrying...")
                    cap.release()
                    break

                frame = cv2.resize(frame, (width, height))
                np_frame = np.frombuffer(
                    shared_array.get_obj(), dtype=np.uint8
                ).reshape((height, width, 3))
                np_frame[:] = frame

            time.sleep(5)

    def run(self):
        for i, ip in enumerate(self.ips):
            if i < 2:
                p = Process(
                    target=self.display_camera,
                    args=(
                        ip,
                        i,
                        self.shared_arrays[i],
                        self.top_cam_width,
                        self.top_cam_height,
                    ),
                )
            elif i < 4:
                p = Process(
                    target=self.display_camera,
                    args=(
                        ip,
                        i,
                        self.shared_arrays[i],
                        self.bottom_cam_width,
                        self.bottom_cam_height,
                    ),
                )
            else:
                p = Process(
                    target=self.display_camera,
                    args=(
                        ip,
                        i,
                        self.shared_arrays[i],
                        self.zed_cam_width,
                        self.zed_cam_height,
                    ),
                )
            p.start()
            self.processes.append(p)

        cv2.namedWindow("Camera Grid", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Camera Grid", self.grid_width, self.grid_height)

        while True:
            grid = np.zeros((self.grid_height, self.grid_width, 3), dtype=np.uint8)

            for i in range(2):
                np_frame = np.frombuffer(
                    self.shared_arrays[i].get_obj(), dtype=np.uint8
                ).reshape((self.top_cam_height, self.top_cam_width, 3))
                grid[
                    0 : self.top_cam_height,
                    i * self.top_cam_width : (i + 1) * self.top_cam_width,
                ] = np_frame

            zed_x_offset = (self.grid_width - self.zed_cam_width) // 2
            np_frame = np.frombuffer(
                self.shared_arrays[4].get_obj(), dtype=np.uint8
            ).reshape((self.zed_cam_height, self.zed_cam_width, 3))
            grid[
                self.top_cam_height : self.top_cam_height + self.zed_cam_height,
                zed_x_offset : zed_x_offset + self.zed_cam_width,
            ] = np_frame

            for i in range(2, 4):
                np_frame = np.frombuffer(
                    self.shared_arrays[i].get_obj(), dtype=np.uint8
                ).reshape((self.bottom_cam_height, self.bottom_cam_width, 3))
                grid[
                    self.top_cam_height + self.zed_cam_height : self.grid_height,
                    (i - 2) * self.bottom_cam_width : (i - 1) * self.bottom_cam_width,
                ] = np_frame

            cv2.imshow("Camera Grid", grid)

            if cv2.waitKey(1) == ord("q"):
                break

        for p in self.processes:
            p.terminate()
        cv2.destroyAllWindows()


def create_ssh_client(ip, username, password):
    return None
    client = paramiko.SSHClient()
    client.load_system_host_keys()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect(ip, username=username, password=password)
    print(f"Connected to {ip} as {username}")
    return client


def terminal_execute(client, command):
    stdin, stdout, stderr = client.exec_command(command)
    _ = [print(i.strip()) for i in stdout]
    return stdout, stderr


def send_command(client, cam_port, property, value):
    command = f"v4l2-ctl -d {cam_port[0]} -c {property}={value}"
    print(f"Executing command: {command}")
    stdin, stdout, stderr = client.exec_command(command)
    _ = [print(i.strip()) for i in stdout]
    return stdout, stderr


def reconnect_command(client, cam_port):
    command = f"sudo ffmpeg -re -f v4l2 -input_format mjpeg -video_size 1280x720 -framerate 60 -i {cam_port[0]}     -c:v libx264 -preset ultrafast -tune zerolatency -b:v 4000k     -f rtsp {cam_port[1]}"
    stdin, stdout, stderr = client.exec_command(command)
    _ = [print(i.strip()) for i in stdout]
    return stdout, stderr


def reset_cameras(client, cam_port):
    commands = [
        f"v4l2-ctl -d {cam_port[0]} -c brightness=128",
        f"v4l2-ctl -d {cam_port[0]} -c contrast=32",
        f"v4l2-ctl -d {cam_port[0]} -c backlight_compensation=0",
    ]
    for command in commands:
        stdin, stdout, stderr = client.exec_command(command)
        _ = [print(i.strip()) for i in stdout]
    return stdout, stderr


# script_path=os.path.dirname("utils.py")
script_dir = os.path.dirname(os.path.abspath(__file__))

BG_path = os.path.join(script_dir, "Visuals/Background(final)")
ROV_path = os.path.join(script_dir, "Visuals/rov(final)")
copilot_path = os.path.join(script_dir, "Visuals/copilot(final)")
pilot_path = os.path.join(script_dir, "Visuals/pilot(final)")
float_path = os.path.join(script_dir, "Visuals/Float(final)")
engineer_path = os.path.join(script_dir, "Visuals/Engineer(final)")

# function for getting the scale factor on each monitor


def get_scaled_factor():
    """
    Calculate a scale factor by comparing the current monitor's DPI
    against a reference DPI (e.g., 110 or your MacBook's measured DPI).

    :param ref_dpi: The 'reference' DPI that your UI was designed for.
    :return: A floating-point scale factor.
    """
    ref_dpi = 110
    monitors = get_monitors()

    if not monitors:
        raise RuntimeError("No monitors found using screeninfo.")

    # For a MacBook, typically you'll have just one primary monitor in the list.
    # If you have multiple monitors, you could iterate and choose the one you want.
    m = monitors[0]

    width_px = m.width
    height_px = m.height
    width_mm = m.width_mm
    height_mm = m.height_mm

    if not width_mm or not height_mm or width_mm <= 0 or height_mm <= 0:
        raise RuntimeError(
            "screeninfo did not provide valid physical dimensions (width_mm/height_mm)."
        )

    # Convert mm to inches (1 inch = 25.4 mm)
    width_in = width_mm / 25.4
    height_in = height_mm / 25.4

    # Calculate horizontal and vertical DPI; then take an average
    dpi_horizontal = width_px / width_in
    dpi_vertical = height_px / height_in
    current_dpi = (dpi_horizontal + dpi_vertical) / 2.0

    # Compare to your reference DPI
    factor = current_dpi / ref_dpi

    return factor


def scale(x):
    return int(1.35 * x * get_scaled_factor())
