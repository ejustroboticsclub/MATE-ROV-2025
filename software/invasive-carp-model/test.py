import cv2
import numpy as np
import matplotlib.pyplot as plt

# Load the original image
image_path = "image.png"
original_image = cv2.imread(image_path)

# Convert the image to RGB for Matplotlib
original_image_rgb = cv2.cvtColor(original_image, cv2.COLOR_BGR2RGB)


def load_and_draw_region(image, npy_file, color, thickness=3):
    """
    Draws a region on the image using pixel coordinates stored in an .npy file.
    
    Parameters:
        image (numpy.ndarray): The image on which the region will be drawn.
        npy_file (str): Path to the .npy file containing the region's pixel coordinates.
        color (tuple): Color of the region in BGR format (e.g., (255, 0, 0) for red).
        thickness (int): Thickness of the line to draw.
        
    Returns:
        numpy.ndarray: The image with the region drawn.
    """
    overlay = image.copy()
    try:
        # Load the saved pixels
        region_pixels = np.load(npy_file)

        # Draw the region by connecting the pixels
        for point in region_pixels:
            cv2.circle(overlay, (point[0], point[1]), radius=thickness, color=color, thickness=3)

    except Exception as e:
        print(f"Error loading or drawing region from {npy_file}: {e}")

    return overlay


# Example function for each region
def draw_region_1(image):
    return load_and_draw_region(image, "region1.npy", color=(255, 0, 0), thickness=2)  # Red

def draw_region_2(image):
    return load_and_draw_region(image, "region2.npy", color=(0, 255, 0), thickness=2)  # Green

def draw_region_3(image):
    return load_and_draw_region(image, "region3.npy", color=(0, 0, 255), thickness=2)  # Blue

def draw_region_4(image):
    return load_and_draw_region(image, "region4.npy", color=(255, 255, 0), thickness=2)  # Cyan

def draw_region_5(image):
    return load_and_draw_region(image, "region5.npy", color=(255, 0, 255), thickness=2)  # Magenta


# Test the drawing functions (assuming all regions have their corresponding .npy files)
image_with_regions = original_image_rgb.copy()
image_with_regions = draw_region_1(image_with_regions)
image_with_regions = draw_region_2(image_with_regions)
image_with_regions = draw_region_3(image_with_regions)
image_with_regions = draw_region_4(image_with_regions)
image_with_regions = draw_region_5(image_with_regions)

# Display the result
plt.figure(figsize=(8, 8))
plt.imshow(image_with_regions)
plt.title("Image with Regions (Loaded from .npy Files)")
plt.axis("off")
plt.show()

# Save the image with regions
cv2.imwrite("image_with_regions.png", cv2.cvtColor(image_with_regions, cv2.COLOR_RGB2BGR))