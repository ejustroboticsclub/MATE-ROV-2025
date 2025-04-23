# 3
import cv2
import numpy as np
import os
import subprocess

# --- CONFIG ---
folder_path = "/home/ahmed/pano/enhanced"  # Your folder of input images
output_image = "photosphere.jpg"  # Final output filename
overlap_width = 50  # Width of overlap region for feather blending

# --- Load and Resize Images ---
image_files = sorted(
    [
        f
        for f in os.listdir(folder_path)
        if f.lower().endswith((".jpg", ".jpeg", ".png"))
    ]
)

images = []
min_height = None

for fname in image_files:
    img = cv2.imread(os.path.join(folder_path, fname))
    if img is None:
        continue
    if min_height is None:
        min_height = img.shape[0]
    img_resized = cv2.resize(
        img, (int(img.shape[1] * min_height / img.shape[0]), min_height)
    )
    images.append(img_resized)


# --- Feather Blending ---
def feather_blend(img1, img2, overlap=50):
    h = img1.shape[0]
    result_width = img1.shape[1] + img2.shape[1] - overlap
    blended = np.zeros((h, result_width, 3), dtype=np.uint8)

    # Left part of img1
    blended[:, : img1.shape[1] - overlap] = img1[:, : img1.shape[1] - overlap]

    # Blending region
    for i in range(overlap):
        alpha = i / overlap
        blended[:, img1.shape[1] - overlap + i] = (
            (1 - alpha) * img1[:, img1.shape[1] - overlap + i] + alpha * img2[:, i]
        ).astype(np.uint8)

    # Right part of img2
    blended[:, img1.shape[1] :] = img2[:, overlap:]

    return blended


# --- Merge All Images with Feathering ---
panorama = images[0]
for i in range(1, len(images)):
    panorama = feather_blend(panorama, images[i], overlap=overlap_width)

# --- Save Panorama ---
cv2.imwrite(output_image, panorama)
print(f"✅ Feather-blended panorama saved as: {output_image}")

# --- Add Photosphere Metadata ---
height, width = panorama.shape[:2]
cmd = [
    "exiftool",
    "-overwrite_original",
    "-ProjectionType=equirectangular",
    f"-CroppedAreaImageWidthPixels={width}",
    f"-CroppedAreaImageHeightPixels={height}",
    "-CroppedAreaLeftPixels=0",
    "-CroppedAreaTopPixels=0",
    f"-FullPanoWidthPixels={width}",
    f"-FullPanoHeightPixels={height}",
    output_image,
]

try:
    subprocess.run(cmd, check=True)
    print("✅ 360 metadata added. Image is now a valid photosphere!")
except subprocess.CalledProcessError:
    print("⚠️ exiftool failed. Make sure it is installed and available in PATH.")
