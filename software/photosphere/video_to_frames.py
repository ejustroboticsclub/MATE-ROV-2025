import cv2
import os

# --- Configuration ---
video_path = "/home/abdelrhman/MATE-ROV-2025/software/photosphere/PhotosphereTask.mp4"  # Path to your video file
frames_dir = "frames"  # Directory to save full frames
new_frames = "new_frames"  # Directory to save left halves of frames
resize_scale = 1  # Scale for resizing frames
S = 0.5  # Interval in seconds between frames

# --- Setup ---
os.makedirs(frames_dir, exist_ok=True)
os.makedirs(new_frames, exist_ok=True)

cap = cv2.VideoCapture(video_path)
fps = cap.get(cv2.CAP_PROP_FPS)
if fps == 0:
    raise ValueError("❌ Unable to read FPS from video.")
frame_interval = int(fps * S)

print(f"🎥 FPS: {fps}, extracting one frame every {frame_interval} frames")

frame_index = 0
saved_index = 0

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    if frame_index % frame_interval == 0:
        if resize_scale != 1.0:
            frame = cv2.resize(frame, (0, 0), fx=resize_scale, fy=resize_scale)

        # Save full frame
        frame_filename = f"frame_{saved_index:04d}.jpg"
        frame_path = os.path.join(frames_dir, frame_filename)
        cv2.imwrite(frame_path, frame)

        # Slice and save left half
        height, width = frame.shape[:2]
        left_half = frame[:, : width // 2]
        left_half_path = os.path.join(new_frames, f"left_{frame_filename}")
        cv2.imwrite(left_half_path, left_half)

        saved_index += 1

    frame_index += 1

cap.release()
print(f"✅ Extracted {saved_index} frames every {S} seconds to '{frames_dir}'")
print(f"✅ Saved left halves to '{new_frames}'")
