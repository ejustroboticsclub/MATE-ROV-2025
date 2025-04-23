# 2
import cv2
import os

video_path = "video.mp4"  # Path to your video file
output_dir = "/frames"
os.makedirs(output_dir, exist_ok=True)

resize_scale = 1
cap = cv2.VideoCapture(video_path)

fps = cap.get(cv2.CAP_PROP_FPS)
if fps == 0:
    raise ValueError("❌ Unable to read FPS from video.")
S = 2
frame_interval = int(fps * S)  # One frame every S seconds

print(f"🎥 FPS: {fps}, extracting one frame every {frame_interval} frames")

frame_index = 0  # All frames
saved_index = 0  # Saved frames

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    if frame_index % frame_interval == 0:
        if resize_scale != 1.0:
            frame = cv2.resize(frame, (0, 0), fx=resize_scale, fy=resize_scale)
        frame_path = os.path.join(output_dir, f"frame_{saved_index:04d}.jpg")
        cv2.imwrite(frame_path, frame)
        saved_index += 1

    frame_index += 1

cap.release()
print(f"✅ Extracted {saved_index} frames every {S} seconds to {output_dir}")
