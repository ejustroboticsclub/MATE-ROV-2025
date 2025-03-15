import cv2
from PyQt5.QtCore import QThread, pyqtSignal
import paramiko

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
