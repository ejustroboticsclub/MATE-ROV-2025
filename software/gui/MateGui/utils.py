import cv2
from PyQt5.QtCore import QThread, pyqtSignal
import paramiko
import numpy as np
import os
from multiprocessing import Process, Array

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
    def __init__(self, ips, cols=3, rows=2, window_width=640, window_height=360):
        super().__init__()
        self.ips = ips
        self.cols = cols
        self.rows = rows
        self.window_width = window_width
        self.window_height = window_height
        self.shared_arrays = [Array('B', window_width * window_height * 3) for _ in range(len(ips))]
        self.processes = []

    def create_pipeline(self, ip):
        return (
            f"rtspsrc location={ip} latency=0 buffer-mode=auto ! "
            "rtph264depay ! h264parse ! avdec_h264 ! "
            "videoconvert ! appsink sync=false"
        )

    def display_camera(self, ip, index, shared_array):
        cap = cv2.VideoCapture(self.create_pipeline(ip), cv2.CAP_GSTREAMER)
        
        if not cap.isOpened():
            print(f"Error opening pipeline for camera {index}")
            return

        while True:
            ret, frame = cap.read()
            if not ret:
                print(f"Connection lost for camera {index}")
                break
            
            frame = cv2.resize(frame, (self.window_width, self.window_height))
            np_frame = np.frombuffer(shared_array.get_obj(), dtype=np.uint8).reshape((self.window_height, self.window_width, 3))
            np_frame[:] = frame

        cap.release()

    def run(self):
        for i, ip in enumerate(self.ips):
            p = Process(target=self.display_camera, args=(ip, i, self.shared_arrays[i]))
            p.start()
            self.processes.append(p)

        cv2.namedWindow("Camera Grid", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Camera Grid", self.window_width * self.cols, self.window_height * self.rows)

        while True:
            grid = np.zeros((self.window_height * self.rows, self.window_width * self.cols, 3), dtype=np.uint8)
            for i, shared_array in enumerate(self.shared_arrays):
                row = i // self.cols
                col = i % self.cols
                np_frame = np.frombuffer(shared_array.get_obj(), dtype=np.uint8).reshape((self.window_height, self.window_width, 3))
                grid[row * self.window_height:(row + 1) * self.window_height, col * self.window_width:(col + 1) * self.window_width] = np_frame

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

def reset_cameras(client, reset_command="source camera_reset.bash"):
    stdin, stdout, stderr = client.exec_command(reset_command)
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