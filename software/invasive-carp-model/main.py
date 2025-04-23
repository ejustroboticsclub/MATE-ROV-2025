import cv2
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

# Define the npy file paths
# Each file contains the pixel coordinates of the corresponding region
region1_npy = "/home/nadine/new/MATE-ROV-2025/software/invasive-carp-model/region1.npy"
region2_npy = "/home/nadine/new/MATE-ROV-2025/software/invasive-carp-model/region2.npy"
region3_npy = "/home/nadine/new/MATE-ROV-2025/software/invasive-carp-model/region3.npy"
region4_npy = "/home/nadine/new/MATE-ROV-2025/software/invasive-carp-model/region4.npy"
region5_npy = "/home/nadine/new/MATE-ROV-2025/software/invasive-carp-model/region5.npy"

# Define the output video path
OUTPUT_VIDEO_PATH = "/home/nadine/new/MATE-ROV-2025/software/invasive-carp-model/output_video.mp4"

# Define a unified color for all regions
COLOR = (255, 0, 0)  # Red


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
def draw_region_1(image, color=(255, 0, 0)):
    return load_and_draw_region(image, region1_npy, color=color, thickness=3) 

def draw_region_2(image, color=(0, 255, 0)):
    return load_and_draw_region(image, region2_npy, color=color, thickness=3) 

def draw_region_3(image, color=(0, 0, 255)):
    return load_and_draw_region(image, region3_npy, color=color, thickness=3)  

def draw_region_4(image, color=(255, 255, 0)):
    return load_and_draw_region(image, region4_npy, color=color, thickness=3)  

def draw_region_5(image, color=(255, 0, 255)):
    return load_and_draw_region(image, region5_npy, color=color, thickness=3)


def main():
    # Load the regions data
    regions_data = pd.read_csv("/home/nadine/new/MATE-ROV-2025/software/invasive-carp-model/regions_data.csv")

    # Load the original image
    original_image_path = "/home/nadine/new/MATE-ROV-2025/software/invasive-carp-model/image.png"
    original_image = cv2.imread(original_image_path)
    
    # Convert the image to RGB for Matplotlib
    original_image_rgb = cv2.cvtColor(original_image, cv2.COLOR_BGR2RGB)

    # Define boolean flags for each region to determine if it is draw
    region1_drawn = False
    region2_drawn = False
    region3_drawn = False
    region4_drawn = False
    region5_drawn = False

    frame_size = (original_image.shape[1], original_image.shape[0])
    fps = 30  # 1 frame per second

    # Initialize the video writer
    video_writer = cv2.VideoWriter(OUTPUT_VIDEO_PATH, cv2.VideoWriter_fourcc(*"mp4v"), fps, frame_size)

    # Iterate over each row in the data and create an image for each year
    for index, row in regions_data.iterrows():        
        # Get the year for the current row
        year = row["Year"]
        
        # Draw the regions where content is 'Y' and not already drawn
        for region, value in row.items():
            if value == "Y":
                if region == "Region 1" and not region1_drawn:
                    original_image_rgb = draw_region_1(original_image_rgb, color=COLOR)
                    region1_drawn = True
                elif region == "Region 2" and not region2_drawn:
                    original_image_rgb = draw_region_2(original_image_rgb, color=COLOR)
                    region2_drawn = True
                elif region == "Region 3" and not region3_drawn:
                    original_image_rgb = draw_region_3(original_image_rgb, color=COLOR)
                    region3_drawn = True
                elif region == "Region 4" and not region4_drawn:
                    original_image_rgb = draw_region_4(original_image_rgb, color=COLOR)
                    region4_drawn = True
                elif region == "Region 5" and not region5_drawn:
                    original_image_rgb = draw_region_5(original_image_rgb, color=COLOR)
                    region5_drawn = True
        
        # copy the original image to draw the year
        image_copy = original_image_rgb.copy()

        # convert the image to BGR for OpenCV
        image_copy = cv2.cvtColor(image_copy, cv2.COLOR_RGB2BGR)

        # Add the year as text to the image
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 1
        font_color = (0, 0, 0)  # Black
        thickness = 2
        text_position = (50, 100)  # Position of the year on the image
        cv2.putText(image_copy, f"Year: {year}", text_position, font, font_scale, font_color, thickness)


        # Write the frame to the video
        for _ in range(fps*2):
            video_writer.write(image_copy)
    
    # Release the video writer
    video_writer.release()

if __name__ == "__main__":
    main()