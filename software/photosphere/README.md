# Video to Panorama Photosphere

This project extracts frames from a video and stitches them into a feather-blended panorama, then converts the panorama into a **360° photosphere** with proper metadata.

---

## Prerequisites

Make sure the following Python libraries and tools are installed:

- Python 3.x
- OpenCV (`cv2`)
- NumPy
- [ExifTool](https://exiftool.org/) (for embedding 360° metadata)

You can install Python dependencies with:

```bash
pip install opencv-python numpy
```

---

## Project Structure

```
project/
├── video_to_frames.py     # Step 1: Extract frames from video
├── stitch_photosphere.py # Step 2: Create panorama and add metadata
├── video.mp4     # Input video file
├── frames/             # Extracted frames (output of step 1)
└── photosphere.jpg            # Final stitched photosphere image
```

---

## Step 1: Extract Frames from Video

`video_to_frames.py` pulls one frame every 2 seconds from a video file.

### Configuration

- `video_path`: Path to your input video file
- `output_dir`: Folder to save extracted frames
- `frame_interval`: Controls frame extraction rate (default: one every 2 seconds)

### Run

```bash
python video_to_frames.py
```

Extracted frames will be saved in `frames/`.

---

## Step 2: Stitch Frames into Panorama

`panorama.py` loads a folder of images, resizes and feather-blends them into a panorama, then embeds 360° metadata.

### Configuration

- `folder_path`: Folder containing input images
- `overlap_width`: Width (in pixels) of overlap region for blending
- `output_image`: Name of final panorama image

### Run

```bash
python panorama.py
```

Outputs:

- `photosphere.jpg`: Final feather-blended panorama
- Adds 360° metadata using `exiftool` for VR/photosphere compatibility

---

## Result

- The output `photosphere.jpg` is now viewable in 360° viewers like PTGUI Viewer.

## Troubleshooting

- **`exiftool` not found**→ Make sure it's installed and available in your `PATH`.
- **FPS = 0**
  → The video might be corrupted or improperly encoded.
