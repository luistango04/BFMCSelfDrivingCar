ROBOFLOW_API_KEY = "ABeUmL5hsdoyPceix50P"
ROBOFLOW_MODEL = "tes_1/1" 
ROBOFLOW_SIZE = 416

#import Setup
import pyrealsense2 as rs
import cv2
import base64
import numpy as np
import requests
import time
import json

# Construct the Roboflow Infer URL
# (if running locally replace https://detect.roboflow.com/ with eg http://127.0.0.1:9001/)
upload_url = "".join([
    "http://127.0.0.1:9001/",
    ROBOFLOW_MODEL,
    "?api_key=",
    ROBOFLOW_API_KEY,
   
    "&stroke=5"
])

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

found_rgb = False
for s in device.sensors:
    if s.get_info(rs.camera_info.name) == 'RGB Camera':
        found_rgb = True
        break
if not found_rgb:
    print("The demo requires Depth camera with Color sensor")
    exit(0)

config.enable_stream(rs.stream.depth, 424, 240, rs.format.z16, 30)

if device_product_line == 'L500':
    config.enable_stream(rs.stream.color, 424, 240, rs.format.bgr8, 30)
else:
    config.enable_stream(rs.stream.color, 424, 240, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)

# Infer via the Roboflow Infer API and return the result
def image_():
    # Wait for a coherent pair of frames: depth and color
    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()

    # Convert images to numpy arrays
    color_image = np.asanyarray(color_frame.get_data())

    # Resize (while maintaining the aspect ratio) to improve speed and save bandwidth
    height, width, channels = color_image.shape
    scale = ROBOFLOW_SIZE / max(height, width)
    img = cv2.resize(color_image, (round(scale * width), round(scale * height)))

    return img

def get_prediction_classes(http_response):
    response_dict = http_response.data.decode('utf-8')
    data = json.loads(response_dict)
    class_list = ['Crossing','HighwayE','HighwayX','No Entry','One Way','Parking','Priority','Stop','Traffic Light','Yield']
    classes = []
    for pred in data['predictions']:
        class_name = pred['class']
        class_index = class_list.index(class_name)
        classes.append(class_index)

    return classes

def infer():
    # Encode image to base64 string
    retval, buffer = cv2.imencode('.jpg', image_())
    img_str = base64.b64encode(buffer)

    # Get prediction from Roboflow Infer API
    resp = requests.post(upload_url, data=img_str, headers={
        "Content-Type": "application/x-www-form-urlencoded"
    }, stream=True).raw
    #print(type(resp))
    classes = get_prediction_classes(resp)
   # print(classes)

    return classes

# Main loop; infers sequentially until you press "q"
while 1:
    # On "q" keypress, exit
    if(cv2.waitKey(1) == ord('q')):
        break

    # Synchronously get a prediction from the Roboflow Infer API
    image = image_()
    test = infer()
    print(test)
    # And display the inference results
    cv2.imshow('image', image)

    # Print frames per second
   # print((1/(time.time()-start)), " fps")

# Release resources when finished
#video.release()
cv2.destroyAllWindows()

