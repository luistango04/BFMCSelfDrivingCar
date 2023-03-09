ROBOFLOW_API_KEY = "X3899fIZ5CzTqaMGCEMu"
ROBOFLOW_MODEL = "sign-ldjct/1" 
ROBOFLOW_SIZE = 416

import pyrealsense2 as rs
import cv2
import base64
import numpy as np
import requests
import time

# Construct the Roboflow Infer URL
# (if running locally replace https://detect.roboflow.com/ with eg http://127.0.0.1:9001/)
upload_url = "".join([
    "http://127.0.0.1:9001/",
    ROBOFLOW_MODEL,
    "?api_key=",
    ROBOFLOW_API_KEY,
   
    "&stroke=5"
])

# Get webcam interface via opencv-python
video = cv2.VideoCapture(2)

# Infer via the Roboflow Infer API and return the result
def infer():

    ret, color_image = video.read()

    # Resize (while maintaining the aspect ratio) to improve speed and save bandwidth
    height, width, channels = color_image.shape
    scale = ROBOFLOW_SIZE / max(height, width)
    img = cv2.resize(color_image, (round(scale * width), round(scale * height)))

    # Encode image to base64 string
    retval, buffer = cv2.imencode('.jpg', img)
    img_str = base64.b64encode(buffer)

    # Get prediction from Roboflow Infer API
    resp = requests.post(upload_url, data=img_str, headers={
        "Content-Type": "application/x-www-form-urlencoded"
    }, stream=True).raw

    print(resp.read())

    # Parse result image
    #image = np.asarray(bytearray(resp.read()), dtype="uint8")
    #image = cv2.imdecode(image, cv2.IMREAD_COLOR)

    return img

# Main loop; infers sequentially until you press "q"
while 1:
    # On "q" keypress, exit
    if(cv2.waitKey(1) == ord('q')):
        break

    # Capture start time to calculate fps
    start = time.time()

    # Synchronously get a prediction from the Roboflow Infer API
    image = infer()
    # And display the inference results
    cv2.imshow('image', image)

    # Print frames per second
   # print((1/(time.time()-start)), " fps")

# Release resources when finished
video.release()
cv2.destroyAllWindows()
