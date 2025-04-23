import cv2
import numpy as np
import matplotlib.pyplot as plt

# Load the original image
image_path = "image.png"
original_image = cv2.imread(image_path)

# Convert the image to RGB for Matplotlib
original_image_rgb = cv2.cvtColor(original_image, cv2.COLOR_BGR2RGB)

def draw_and_save_pixels(image):
    """
    Allows the user to draw regions manually on the image.
    Saves and prints the drawn pixels' coordinates.
    """
    # Copy the image for drawing
    drawing_image = image.copy()
    drawn_pixels = []  # To store the pixels drawn by the user

    # Define mouse callback function
    def draw(event, x, y, flags, param):
        nonlocal drawn_pixels
        if event == cv2.EVENT_LBUTTONDOWN or (flags & cv2.EVENT_FLAG_LBUTTON):  # Draw on left click or drag
            cv2.circle(drawing_image, (x, y), radius=3, color=(255, 0, 0), thickness=3)  # Draw a small circle
            drawn_pixels.append((x, y))  # Save the pixel coordinates

    # Set up the OpenCV window
    cv2.namedWindow("Draw Region")
    cv2.setMouseCallback("Draw Region", draw)

    while True:
        # Display the image
        cv2.imshow("Draw Region", drawing_image)
        key = cv2.waitKey(1) & 0xFF

        # Break the loop on pressing 'q'
        if key == ord("q"):
            break

    # Clean up OpenCV window
    cv2.destroyAllWindows()

    return drawn_pixels


# Test the function
drawn_pixels = draw_and_save_pixels(original_image_rgb)
# Print and return the drawn pixels
print("Drawn Pixels:", drawn_pixels)

# save the drawn pixels
np.save("drawn_pixels.npy", np.array(drawn_pixels))