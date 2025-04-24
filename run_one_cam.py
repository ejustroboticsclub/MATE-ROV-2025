import cv2
import argparse
# Change to station's IP
ip_pilot = "192.168.1.110"

# Change to PI's IP
ip_rasp = "192.168.1.100"

IPS = [
    f"rtsp://{ip_rasp}:5002/unicast", # Top left (Net)
    f"rtsp://{ip_rasp}:5001/unicast", # Top right (Side)
    f"rtsp://{ip_rasp}:5003/unicast", # Bottom left (Gripper)
    f"rtsp://{ip_rasp}:5004/unicast", # Bottom right (Jelly)
    f"rtsp://{ip_pilot}:8554/videofeed",  # Middle ZED
]
        
def run_cam(ip_url):
    pipeline =  (
        f"rtspsrc location={ip_url} latency=0 buffer-mode=auto "
        "! decodebin ! videoconvert ! appsink max-buffers=1 drop=True"
    )

    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        print("Error: Unable to open video stream.")
        return
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Unable to read frame.")
            break
        # Resize the frame to fit the window
        if "videofeed" in ip_url:
            frame = cv2.resize(frame, (1344, 376))
        cv2.imshow("Camera Stream", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Run camera stream from a specified IP.")
    parser.add_argument(
        "--cam_index", type=int, required=True, 
        help="Index of the camera to stream (0 to 4)."
    )
    args = parser.parse_args()

    if args.cam_index < 0 or args.cam_index >= len(IPS):
        print("Error: Camera index out of range. Please provide a value between 0 and 4.")
    else:
        run_cam(IPS[args.cam_index])
