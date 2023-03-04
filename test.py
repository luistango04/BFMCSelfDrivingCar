import pyrealsense2 as rs
import numpy as np
import cv2
import Setup
pipeline = Setup.init()

# Create a RealSense pipeline


try:

        # Wait for frames
        frames = pipeline.wait_for_frames()

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


finally:
    pipeline.stop()

# Close the OpenCV windows
cv2.destroyAllWindows()

import paho.mqtt.client as mqtt
import time

# Define callbacks
def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))

def on_publish(client, userdata, mid):
    print("Message published")

def on_message(client, userdata, message):
    print("Message received: " + str(message.payload.decode()))

# Create a client instance
client = mqtt.Client()

# Set the callbacks
client.on_connect = on_connect
client.on_publish = on_publish
client.on_message = on_message

# Connect to a broker
client.connect("mqtt.eclipseprojects.io", 1883)

# Start the network loop
client.loop_start()

# Publish a message
client.publish("test/topic", "Hello, world!")

# Subscribe to a topic
client.subscribe("test/topic")

# Wait for a message
time.sleep(1)

# Stop the network loop
client.loop_stop()
