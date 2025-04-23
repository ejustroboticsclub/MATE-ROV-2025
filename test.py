import cv2
ip_url = "rtsp://192.168.1.110:8554/videofeed"

pipeline =  (
    f"rtspsrc location={ip_url} latency=0 buffer-mode=auto "
    "! decodebin ! videoconvert ! appsink max-buffers=1 drop=True"
)


cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
while True:
        

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            print("Connection lost for camera Zed, retrying...")
            cap.release()
            break
        cv2.imshow("ZED", frame)
