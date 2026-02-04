import cv2
import numpy as np

def pick_color(event, x, y, flags, param):
    """
    Callback function for mouse events to display HSV values of clicked pixels.
    """
    global hsv_image
    if event == cv2.EVENT_LBUTTONDOWN:  # Left mouse click
        # Get the HSV value at the clicked position
        pixel_hsv = hsv_image[y, x]
        print(f"HSV at ({x}, {y}): {pixel_hsv}")

        # Update a global range if needed
        # Example: Collect values for fine-tuning
        lower_hsv.append(pixel_hsv - np.array([10, 50, 50]))
        upper_hsv.append(pixel_hsv + np.array([10, 50, 50]))

# Initialize global lists to store ranges
lower_hsv = []
upper_hsv = []

# Load the image
image_path = "SceneImg.jpg"  # Replace with your image path
image_bgr = cv2.imread(image_path)
if image_bgr is None:
    raise FileNotFoundError(f"Could not load image: {image_path}")

# Convert BGR to HSV
hsv_image = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)

# Create a window and set the mouse callback function
cv2.namedWindow("Pick HSV")
cv2.setMouseCallback("Pick HSV", pick_color)

while True:
    cv2.imshow("Pick HSV", image_bgr)
    key = cv2.waitKey(1) & 0xFF

    # Press 'q' to quit
    if key == ord('q'):
        break

# Compute average lower and upper bounds if ranges were collected
if lower_hsv and upper_hsv:
    avg_lower_hsv = np.maximum(np.mean(lower_hsv, axis=0).astype(int), 0)
    avg_upper_hsv = np.minimum(np.mean(upper_hsv, axis=0).astype(int), 255)
    print(f"Suggested HSV Range:")
    print(f"Lower: {avg_lower_hsv}")
    print(f"Upper: {avg_upper_hsv}")

cv2.destroyAllWindows()
