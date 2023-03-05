import pyrealsense2 as rs
import numpy as np
import cv2
import Setup
from Sense import SensingInput
pipeline = Setup.init()
Sense = SensingInput(0,pipeline)
# Create a RealSense pipeline


try:
    while True:
        # Wait for frames
        frames = Sense.pipeline.wait_for_frames()

        # Get the depth frame
        depth_frame = frames.get_depth_frame()

        # Get the color frame
        color_frame = frames.get_color_frame()

        # Convert the depth frame to a NumPy array
        depth_image = np.asanyarray(depth_frame.get_data())

        # Convert the color frame to a NumPy array
        color_image = np.asanyarray(color_frame.get_data())

        # Convert the color image from BGR to RGB


        # Display the depth image
        cv2.imshow("Depth Image", depth_image)

        # Display the color image
        cv2.imshow("Color Image", color_image)

        # Wait for a key press
        key = cv2.waitKey(1)

        # Exit the loop if the 'q' key is pressed
        if key == ord('q'):
            break

finally:
    pipeline.stop()

# Close the OpenCV windows
cv2.destroyAllWindows()