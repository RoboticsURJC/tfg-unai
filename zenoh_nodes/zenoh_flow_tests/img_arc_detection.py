import cv2
import numpy as np

# Create a blank image
image = np.zeros((500, 500), dtype=np.uint8)

# Draw a perfect circle on the image
center = (250, 250)
radius = 100
color = 255
thickness = 2  # Negative thickness fills the circle
cv2.circle(image, center, radius, color, thickness)
cv2.rectangle(image, (0, 0), (500, 300), (0, 0, 0), -1)

# Apply Gaussian blur to the image to reduce noise
image_blurred = cv2.GaussianBlur(image, (9, 9), 2)

# Use Hough Circles detection
circles = cv2.HoughCircles(image_blurred, cv2.HOUGH_GRADIENT, dp=1, minDist=20,
                           param1=50, param2=30, minRadius=0, maxRadius=0)



# If circles are detected, draw them
if circles is not None:
    print("detected")
    circles = np.round(circles[0, :]).astype("int")
    for (x, y, r) in circles:
        cv2.circle(image, (x, y), r, 128, 1)
        cv2.rectangle(image, (x - 5, y - 5), (x + 5, y + 5), 128, -1)

# Save the image
cv2.imwrite("arc_detected.png", image)

# Show the image with detected circles
cv2.imshow("Detected Circles", image)
cv2.waitKey(0)
cv2.destroyAllWindows()
