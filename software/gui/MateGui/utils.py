import cv2
from PyQt5.QtCore import QThread, pyqtSignal
import paramiko
import numpy as np
import os
from multiprocessing import Process, Array
import math
from screeninfo import get_monitors


class VideoCaptureThread(QThread):
    stop_signal = pyqtSignal()

    def __init__(self):
        super().__init__()
        self.cap = None 
        self.fps = 0
        self.frame_width = 0
        self.frame_height = 0
        self.fourcc = cv2.VideoWriter_fourcc(*'avc1')  
        self.video_writer = None
        self.recording = False

    def run(self):
        while self.cap.isOpened() and self.recording:
            ret, frame = self.cap.read()
            if ret:
                self.video_writer.write(frame)
            else:
                break
            

    def start_recording(self, filename="PhotosphereTask.mov"):
        if not self.recording:
            self.cap = cv2.VideoCapture(0)
            if not self.cap.isOpened():
                print("Error: Unable to access the camera.")
                return
            
            self.fps = int(self.cap.get(cv2.CAP_PROP_FPS))
            self.frame_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            self.frame_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

            self.video_writer = cv2.VideoWriter(filename, self.fourcc, self.fps, 
                                                (self.frame_width, self.frame_height))
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

        # Define the window size for the grid
        self.grid_width = 1920
        self.grid_height = 1080

        # Define the window size for the first camera initially main camera but you change it as you wish for the station
        self.first_cam_width = 960
        self.first_cam_height = 720

        # Define the window size for the second and third cameras (Tilt and side) Rec: you can make it for Grippers
        self.stacked_cam_width = 960
        self.stacked_cam_height = 360

        # Define the window size for the other cameras (Gripper L, Gripper R, and Bottom) Rec: Tilt, Bottom and side
        self.other_cam_width = 640
        self.other_cam_height = 360

        # Create shared memory arrays for each camera
        self.shared_arrays = [
            Array('B', self.first_cam_width * self.first_cam_height * 3),
            Array('B', self.stacked_cam_width * self.stacked_cam_height * 3),
            Array('B', self.stacked_cam_width * self.stacked_cam_height * 3)
        ] + [Array('B', self.other_cam_width * self.other_cam_height * 3) for _ in range(3, len(ips))]

        self.processes = []

    def create_pipeline(self, ip):
        return (
            f"rtspsrc location={ip} latency=0 buffer-mode=auto ! "
            "rtph264depay ! h264parse ! avdec_h264 ! "
            "videoconvert ! appsink sync=false"
        )

    def display_camera(self, ip, index, shared_array, width, height):
        cap = cv2.VideoCapture(self.create_pipeline(ip), cv2.CAP_GSTREAMER)
        
        if not cap.isOpened():
            print(f"Error opening pipeline for camera {index}")
            return

        while True:
            ret, frame = cap.read()
            if not ret:
                print(f"Connection lost for camera {index}")
                break
            
            frame = cv2.resize(frame, (width, height))
            np_frame = np.frombuffer(shared_array.get_obj(), dtype=np.uint8).reshape((height, width, 3))
            np_frame[:] = frame

        cap.release()

    def run(self):
        for i, ip in enumerate(self.ips):
            if i == 0:
                p = Process(target=self.display_camera, args=(ip, i, self.shared_arrays[i], self.first_cam_width, self.first_cam_height))
            elif i in [1, 2]:
                p = Process(target=self.display_camera, args=(ip, i, self.shared_arrays[i], self.stacked_cam_width, self.stacked_cam_height))
            else:
                p = Process(target=self.display_camera, args=(ip, i, self.shared_arrays[i], self.other_cam_width, self.other_cam_height))
            p.start()
            self.processes.append(p)

        cv2.namedWindow("Camera Grid", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Camera Grid", self.grid_width, self.grid_height)

        while True:
            grid = np.zeros((self.grid_height, self.grid_width, 3), dtype=np.uint8)

            # Place the first camera in the top-right quarter
            np_frame = np.frombuffer(self.shared_arrays[0].get_obj(), dtype=np.uint8).reshape((self.first_cam_height, self.first_cam_width, 3))
            grid[0:self.first_cam_height, self.grid_width - self.first_cam_width:self.grid_width] = np_frame

            # Place the second and third cameras in the top-left quarter, stacked vertically
            for i in range(1, 3):
                np_frame = np.frombuffer(self.shared_arrays[i].get_obj(), dtype=np.uint8).reshape((self.stacked_cam_height, self.stacked_cam_width, 3))
                grid[(i - 1) * self.stacked_cam_height:i * self.stacked_cam_height, 0:self.stacked_cam_width] = np_frame

            # Place the remaining cameras in the bottom half, side by side
            for i in range(3, 6):
                np_frame = np.frombuffer(self.shared_arrays[i].get_obj(), dtype=np.uint8).reshape((self.other_cam_height, self.other_cam_width, 3))
                grid[self.first_cam_height:self.first_cam_height + self.other_cam_height, (i - 3) * self.other_cam_width:(i - 2) * self.other_cam_width] = np_frame

            cv2.imshow("Camera Grid", grid)

            if cv2.waitKey(1) == ord('q'):
                break

        for p in self.processes:
            p.terminate()
        cv2.destroyAllWindows()

def create_ssh_client(ip, username, password):
    client = paramiko.SSHClient()
    client.load_system_host_keys()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect(ip, username=username, password=password)
    return client

def send_command(client, cam_port, property, value):
    command = f"sudo v4l2-ctl -d {cam_port} -c {property}={value}"
    stdin, stdout, stderr = client.exec_command(command)
    _ = [print(i.strip()) for i in stdout]
    return stdout, stderr

def reset_cameras(client, cam_port):
    commands = [
        f"v4l2-ctl -d {cam_port} -c brightness=128",
        f"v4l2-ctl -d {cam_port} -c contrast=32",
        f"v4l2-ctl -d {cam_port} -c backlight_compensation=0"
    ]
    for command in commands:
        stdin, stdout, stderr = client.exec_command(command)
        _ = [print(i.strip()) for i in stdout]
    return stdout, stderr



#script_path=os.path.dirname("utils.py")
script_dir = os.path.dirname(os.path.abspath(__file__))

BG_path = os.path.join(script_dir,"Visuals/Background(final)")
ROV_path=os.path.join(script_dir,"Visuals/rov(final)")
copilot_path=os.path.join(script_dir,"Visuals/copilot(final)")
pilot_path=os.path.join(script_dir,"Visuals/pilot(final)")
float_path=os.path.join(script_dir,"Visuals/Float(final)")
engineer_path=os.path.join(script_dir,"Visuals/Engineer(final)")

#function for getting the scale factor on each monitor

def get_scaled_factor():
    """
    Calculate a scale factor by comparing the current monitor's DPI 
    against a reference DPI (e.g., 110 or your MacBook's measured DPI).
    
    :param ref_dpi: The 'reference' DPI that your UI was designed for.
    :return: A floating-point scale factor.
    """
    ref_dpi=110
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
    dpi_vertical   = height_px / height_in
    current_dpi    = (dpi_horizontal + dpi_vertical) / 2.0

    # Compare to your reference DPI
    factor = current_dpi / ref_dpi
    
    return factor


def scale(x):
    return int(x*get_scaled_factor())