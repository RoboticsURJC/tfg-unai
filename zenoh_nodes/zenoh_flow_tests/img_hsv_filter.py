import sys
import cv2
import numpy as np

# Function to process each frame
def process_frame(frame, lower_range, upper_range):
    # Convert BGR image to HSV
    hsv_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Create mask using the color range
    mask = cv2.inRange(hsv_image, lower_range, upper_range)

    # Apply mask to original frame to extract color
    result = cv2.bitwise_and(frame, frame, mask=mask)

    return result

# Callback function for trackbar changes
def update_trackbar(value):
    pass

def main(camera_source):
    # Open camera
    cap = cv2.VideoCapture(camera_source)

    # Check if camera opened successfully
    if not cap.isOpened():
        print("Error: Unable to open camera")
        return

    # Create a window to display the processed frame
    cv2.namedWindow('Processed Frame')

    # Define default HSV color range for yellow
    #lower_range_default = np.array([20, 100, 100])
    #upper_range_default = np.array([40, 255, 255])
    # Define default HSV color range for pink
    lower_range_default = np.array([140, 125, 200])
    upper_range_default = np.array([180, 255, 255])

    # Create trackbars for adjusting HSV color range
    cv2.createTrackbar('Hue Lower', 'Processed Frame',
                       lower_range_default[0], 180, update_trackbar)
    cv2.createTrackbar('Saturation Lower', 'Processed Frame',
                       lower_range_default[1], 255, update_trackbar)
    cv2.createTrackbar('Value Lower', 'Processed Frame',
                       lower_range_default[2], 255, update_trackbar)
    cv2.createTrackbar('Hue Upper', 'Processed Frame',
                       upper_range_default[0], 180, update_trackbar)
    cv2.createTrackbar('Saturation Upper', 'Processed Frame',
                       upper_range_default[1], 255, update_trackbar)
    cv2.createTrackbar('Value Upper', 'Processed Frame',
                       upper_range_default[2], 255, update_trackbar)

    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()

        if not ret:
            print("Error: Unable to capture frame")
            break

        # Get current trackbar positions
        hue_lower = cv2.getTrackbarPos('Hue Lower', 'Processed Frame')
        sat_lower = cv2.getTrackbarPos('Saturation Lower', 'Processed Frame')
        val_lower = cv2.getTrackbarPos('Value Lower', 'Processed Frame')
        hue_upper = cv2.getTrackbarPos('Hue Upper', 'Processed Frame')
        sat_upper = cv2.getTrackbarPos('Saturation Upper', 'Processed Frame')
        val_upper = cv2.getTrackbarPos('Value Upper', 'Processed Frame')

        # Define lower and upper range for HSV color
        lower_range = np.array([hue_lower, sat_lower, val_lower])
        upper_range = np.array([hue_upper, sat_upper, val_upper])

        # Process the frame
        processed_frame = process_frame(frame, lower_range, upper_range)

        # Display the processed frame
        cv2.imshow('Processed Frame', processed_frame)

        # Check for key press to exit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the camera and close OpenCV windows
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python script.py <camera_source>")
        sys.exit(1)
    camera_source = int(sys.argv[1])
    main(camera_source)
