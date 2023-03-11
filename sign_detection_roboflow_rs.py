#R.1 STOP/PARKING/CROSSING/STOP detection dist~50-60, mpa 50~85%
#R.2 OTHER dist~25 should be trained again in high res.



import Setup
import cv2
import base64
import numpy as np
import requests
import time
import json

# Construct the Roboflow Infer URL

upload_url = "".join([
    "http://127.0.0.1:9001/",
    ROBOFLOW_MODEL,
    "?api_key=",
    ROBOFLOW_API_KEY,

    "&stroke=5"
])

ROBOFLOW_API_KEY = "ABeUmL5hsdoyPceix50P"
ROBOFLOW_MODEL = "tes_1/1"
ROBOFLOW_SIZE = 416



# Infer via the Roboflow Infer API and return the result


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

def infer(img):
    # Encode image to base64 string
    retval, buffer = cv2.imencode('.jpg', img) ###
    img_str = base64.b64encode(buffer)

    # Get prediction from Roboflow Infer API
    resp = requests.post(upload_url, data=img_str, headers={
        "Content-Type": "application/x-www-form-urlencoded"
    }, stream=True).raw
    classes = get_prediction_classes(resp)
    return classes

"""
while 1:
    # Synchronously get a prediction from the Roboflow Infer API
    infer()
"""

