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

print("opening image...")
image = cv2.imread(
    "/home/usanz/Desktop/Uni/22-23/tfg-unai/zenoh_nodes/zenoh_flow_tests/debug_img.png",
    cv2.IMREAD_GRAYSCALE
    )


# Apply Gaussian blur to the image to reduce noise
image_blurred = cv2.GaussianBlur(image, (9, 9), 2)

# Use Hough Circles detection

#for the last images:
#circles = cv2.HoughCircles(image_blurred, cv2.HOUGH_GRADIENT, dp=1, minDist=20,
#                          param1=50, param2=30, minRadius=0, maxRadius=0)
circles = cv2.HoughCircles(image_blurred, cv2.HOUGH_GRADIENT, dp=1, minDist=40,
                           param1=30, param2=25, minRadius=10, maxRadius=100)



# If circles are detected, draw them
if circles is not None:
    print("detected")
    circles = np.round(circles[0, :]).astype("int")
    min_radius = 20
    max_radius = 30
    color = 255
    for (x, y, r) in circles:
        if min_radius <= r <= max_radius:
            cv2.circle(image, (x, y), r, color, 3)
            cv2.rectangle(image, (x - 5, y - 5), (x + 5, y + 5), color, -1)
            color = 128

# Save the image
cv2.imwrite("turtlebot_world_lines_better_resolution_detected.png", image)

# Show the image with detected circles

if image.shape[0] > 1000:
    image = cv2.resize(image, (1000, 1000))  # Specify the width and height you want
cv2.imshow("Detected Circles", image)
cv2.waitKey(0)
cv2.destroyAllWindows()
