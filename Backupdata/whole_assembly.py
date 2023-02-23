'''
blah
'''
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
import cv2
import math
import os
import time
import serial

ser = serial.Serial('/dev/ttyACM0', 19200, timeout=0.1)
ser.flush()

# Global parameters

# Gaussian smoothing
kernel_size = 3

# Canny Edge Detector
low_threshold = 50
high_threshold = 150

# Region-of-interest vertices
# We want a trapezoid shape, with bottom edge at the bottom of the image
trap_bottom_width = 0.85  # width of bottom edge of trapezoid, expressed as percentage of image width
trap_top_width = 0.07  # ditto for top edge of trapezoid
trap_height = 0.4  # height of the trapezoid expressed as percentage of image height

# Hough Transform
rho = 2  # distance resolution in pixels of the Hough grid
theta = 1 * np.pi / 180  # angular resolution in radians of the Hough grid
threshold = 15  # minimum number of votes (intersections in Hough grid cell)
min_line_length = 10  # minimum number of pixels making up a line
max_line_gap = 20  # maximum gap in pixels between connectable line segments

prev_error = 0
error = 0

# PID controller parameters
Kp = 0.2
Ki = 0.0
Kd = 0.1
min_output = -20
max_output = 20
last_time = 1

steering_angle = 0

class PIDController:
    def __init__(self, Kp, Ki, Kd, min_output, max_output):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.min_output = min_output
        self.max_output = max_output

        self.last_error = 0
        self.integral_error = 0

    def get_output(self, error, dt):
        dt = int(dt)
        output = 0
        if dt > 0:
            self.integral_error += error * dt
            derivative_error = (error - self.last_error) / dt
            output = self.Kp * error + self.Ki * self.integral_error + self.Kd * derivative_error

            output = min(max(output, self.min_output), self.max_output)

            self.last_error = error

        return output


pid_controller = PIDController(Kp, Ki, Kd, min_output, max_output)


# Helper functions
def grayscale(img):
    """Applies the Grayscale transform
	This will return an image with only one color channel
	but NOTE: to see the returned image as grayscale
	you should call plt.imshow(gray, cmap='gray')"""
    return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)


def canny(img, low_threshold, high_threshold):
    """Applies the Canny transform"""
    return cv2.Canny(img, low_threshold, high_threshold)


def gaussian_blur(img, kernel_size):
    """Applies a Gaussian Noise kernel"""
    return cv2.GaussianBlur(img, (kernel_size, kernel_size), 0)


def region_of_interest(img, vertices):
    """
	Applies an image mask.

	Only keeps the region of the image defined by the polygon
	formed from `vertices`. The rest of the image is set to black.
	"""
    # defining a blank mask to start with
    mask = np.zeros_like(img)

    # defining a 3 channel or 1 channel color to fill the mask with depending on the input image
    if len(img.shape) > 2:
        channel_count = img.shape[2]  # i.e. 3 or 4 depending on your image
        ignore_mask_color = (255,) * channel_count
    else:
        ignore_mask_color = 255

    # filling pixels inside the polygon defined by "vertices" with the fill color
    cv2.fillPoly(mask, vertices, ignore_mask_color)

    # returning the image only where mask pixels are nonzero
    masked_image = cv2.bitwise_and(img, mask)
    return masked_image


def find_slope_intercept(x1, y1, x2, y2):
    slope = (y2 - y1) / (x2 - x1)
    y_intercept = y1 - slope * x1
    return slope, y_intercept


def draw_lines(img, lines, color=[255, 0, 0], thickness=10):
    """
	NOTE: this is the function you might want to use as a starting point once you want to
	average/extrapolate the line segments you detect to map out the full
	extent of the lane (going from the result shown in raw-lines-example.mp4
	to that shown in P1_example.mp4).

	Think about things like separating line segments by their
	slope ((y2-y1)/(x2-x1)) to decide which segments are part of the left
	line vs. the right line.  Then, you can average the position of each of
	the lines and extrapolate to the top and bottom of the lane.

	This function draws `lines` with `color` and `thickness`.
	Lines are drawn on the image inplace (mutates the image).
	If you want to make the lines semi-transparent, think about combining
	this function with the weighted_img() function below
	"""
    # In case of error, don't draw the line(s)
    if lines is None:
        return
    if len(lines) == 0:
        return
    draw_right = True
    draw_left = True

    # Find slopes of all lines
    # But only care about lines where abs(slope) > slope_threshold
    slope_threshold = 0.5
    slopes = []
    new_lines = []
    for line in lines:
        x1, y1, x2, y2 = line[0]  # line = [[x1, y1, x2, y2]]

        # Calculate slope
        if x2 - x1 == 0.:  # corner case, avoiding division by 0
            slope = 999.  # practically infinite slope
        else:
            slope = (y2 - y1) / (x2 - x1)

        # Filter lines based on slope
        if abs(slope) > slope_threshold:
            slopes.append(slope)
            new_lines.append(line)

    lines = new_lines

    # Split lines into right_lines and left_lines, representing the right and left lane lines
    # Right/left lane lines must have positive/negative slope, and be on the right/left half of the image
    right_lines = []
    left_lines = []
    for i, line in enumerate(lines):
        x1, y1, x2, y2 = line[0]
        img_x_center = img.shape[1] / 2  # x coordinate of center of image
        if slopes[i] > 0 and x1 > img_x_center and x2 > img_x_center:
            right_lines.append(line)
        elif slopes[i] < 0 and x1 < img_x_center and x2 < img_x_center:
            left_lines.append(line)

    # Run linear regression to find best fit line for right and left lane lines
    # Right lane lines
    right_lines_x = []
    right_lines_y = []

    for line in right_lines:
        x1, y1, x2, y2 = line[0]

        right_lines_x.append(x1)
        right_lines_x.append(x2)

        right_lines_y.append(y1)
        right_lines_y.append(y2)

    if len(right_lines_x) > 0:
        right_m, right_b = np.polyfit(right_lines_x, right_lines_y, 1)  # y = m*x + b
    else:
        right_m, right_b = 1, 1
        draw_right = False

    # Left lane lines
    left_lines_x = []
    left_lines_y = []

    for line in left_lines:
        x1, y1, x2, y2 = line[0]

        left_lines_x.append(x1)
        left_lines_x.append(x2)

        left_lines_y.append(y1)
        left_lines_y.append(y2)

    if len(left_lines_x) > 0:
        left_m, left_b = np.polyfit(left_lines_x, left_lines_y, 1)  # y = m*x + b
    else:
        left_m, left_b = 1, 1
        draw_left = False

    # Find 2 end points for right and left lines, used for drawing the line
    # y = m*x + b --> x = (y - b)/m
    y1 = img.shape[0]
    y2 = img.shape[0] * (1 - trap_height)

    right_x1 = (y1 - right_b) / right_m
    right_x2 = (y2 - right_b) / right_m

    left_x1 = (y1 - left_b) / left_m
    left_x2 = (y2 - left_b) / left_m

    # Convert calculated end points from float to int
    y1 = int(y1)
    y2 = int(y2)
    right_x1 = int(right_x1)
    right_x2 = int(right_x2)
    left_x1 = int(left_x1)
    left_x2 = int(left_x2)

    # Draw the right and left lines on image
    if draw_right:
        cv2.line(img, (right_x1, y1), (right_x2, y2), color, thickness)
    if draw_left:
        cv2.line(img, (left_x1, y1), (left_x2, y2), color, thickness)

    # for PD
    slope1, y_intercept1 = find_slope_intercept(right_x1, y1, right_x2, y2)
    slope2, y_intercept2 = find_slope_intercept(left_x1, y1, left_x2, y2)
    lane_center = (right_x1 + left_x1) / 2
    image_center = img.shape[1] / 2
    steer_data = [lane_center, image_center, slope1, y_intercept1, slope2, y_intercept2]
    global steering_angle
    steering_angle = round(steering(lane_center, image_center), 2)
    text = str(steering_angle)

    # for PID
    global last_time
    error = lane_center - image_center
    steering_angle = pid_controller.get_output(error, last_time)
    text2 = str(round(steering_angle, 2))
    last_time = last_time + 1

    # Define the font properties
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 1
    font_color = (255, 255, 255)  # white
    line_type = 2
    text_size, _ = cv2.getTextSize(text, font, font_scale, line_type)

    text_x = int((img.shape[1] - text_size[0]) / 2)
    text_y = int((img.shape[0] + text_size[1]) / 2)

    # Write the text on the image
    cv2.putText(img, text, (text_x, text_y), font, font_scale, font_color, line_type)
    cv2.putText(img, text2, (text_x, text_y + 30), font, font_scale, font_color, line_type)

    return img	


def hough_lines(img, rho, theta, threshold, min_line_len, max_line_gap):
    """
	`img` should be the output of a Canny transform.

	Returns an image with hough lines drawn.
	"""
    lines = cv2.HoughLinesP(img, rho, theta, threshold, np.array([]), minLineLength=min_line_len,
                            maxLineGap=max_line_gap)
    line_img = np.zeros((*img.shape, 3), dtype=np.uint8)  # 3-channel RGB image
    drawn_img = draw_lines(line_img, lines)
    return line_img, drawn_img


# Python 3 has support for cool math symbols.

def weighted_img(img, initial_img, α=0.4, β=1., λ=0.):
    """
	`img` is the output of the hough_lines(), An image with lines drawn on it.
	Should be a blank image (all black) with lines drawn on it.

	`initial_img` should be the image before any processing.

	The result image is computed as follows:

	initial_img * α + img * β + λ
	NOTE: initial_img and img must be the same shape!
	"""
    return cv2.addWeighted(initial_img, α, img, β, λ)


def filter_colors(image):
    """
	Filter the image to include only yellow and white pixels
	"""
    # Filter white pixels
    white_threshold = 200  # 130
    lower_white = np.array([white_threshold, white_threshold, white_threshold])
    upper_white = np.array([255, 255, 255])
    white_mask = cv2.inRange(image, lower_white, upper_white)
    white_image = cv2.bitwise_and(image, image, mask=white_mask)

    # Filter yellow pixels
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_yellow = np.array([90, 100, 100])
    upper_yellow = np.array([110, 255, 255])
    yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    yellow_image = cv2.bitwise_and(image, image, mask=yellow_mask)

    # Combine the two above images
    image2 = cv2.addWeighted(white_image, 1., yellow_image, 1., 0.)

    return image2


def annotate_image_array(image_in):
    """ Given an image Numpy array, return the annotated image as a Numpy array """
    # Only keep white and yellow pixels in the image, all other pixels become black
    image = filter_colors(image_in)

    # Read in and grayscale the image
    gray = grayscale(image)

    # Apply Gaussian smoothing
    blur_gray = gaussian_blur(gray, kernel_size)

    # Apply Canny Edge Detector
    edges = canny(blur_gray, low_threshold, high_threshold)

    # Create masked edges using trapezoid-shaped region-of-interest
    imshape = image.shape
    vertices = np.array([[ \
        ((imshape[1] * (1 - trap_bottom_width)) // 2, imshape[0]), \
        ((imshape[1] * (1 - trap_top_width)) // 2, imshape[0] - imshape[0] * trap_height), \
        (imshape[1] - (imshape[1] * (1 - trap_top_width)) // 2, imshape[0] - imshape[0] * trap_height), \
        (imshape[1] - (imshape[1] * (1 - trap_bottom_width)) // 2, imshape[0])]] \
        , dtype=np.int32)

    height = image.shape[0]
    width = image.shape[1]
    vertices = np.array([
        [(0, 180), (0, height), (width, height), (width, 180), (240, 60), (60, 60)]
    ])
    masked_edges = region_of_interest(edges, vertices)

    # Run Hough on edge detected image
    line_image, drawn_img = hough_lines(masked_edges, rho, theta, threshold, min_line_length, max_line_gap)

    # Draw lane lines on the original image
    initial_image = image_in.astype('uint8')
    annotated_image = weighted_img(line_image, initial_image)

    return annotated_image, drawn_img


def annotate_image(input_file, output_file):
    """ Given input_file image, save annotated image to output_file """
    annotated_image, drawn_image = annotate_image_array(input_file)
    #plt.imsave(output_file, annotated_image)
    return annotated_image, drawn_image


def annotate_video(input_file, output_file):
    """ Given input_file video, save annotated video to output_file """
    video = VideoFileClip(input_file)
    annotated_video = video.fl_image(annotate_image_array)
    annotated_video.write_videofile(output_file, audio=False)
    return steer_data


# End helper functions

def steering(xcenter_lane, xcenter_image):
    # Define controller gains
    Kp = 0.1  # Proportional gain
    Kd = 0.01  # Derivative gain

    # Define initial error and derivative of error
    error = xcenter_lane - xcenter_image
    prev_error = 0



    # PD controller
    # while True:
    # Update error and derivative of error
    error = xcenter_lane - xcenter_image
    error_diff = error - prev_error
    prev_error = error

    # Calculate steering angle
    angle = Kp * error + Kd * error_diff

    # Limit the steering angle to the maximum and minimum values
    angle = max(min_angle, min(max_angle, angle))

    # Apply steering angle to the vehicle
    return angle


# Main script
if __name__ == '__main__':
        import pyrealsense2 as rs

        #print("reset start")
        #ctx = rs.context()
        #devices = ctx.query_devices()
        #for dev in devices:
        #        dev.hardware_reset()
        #print("reset done")

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

        #config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        # Start streaming
        pipeline.start(config)

        try:
            while True:

                # Wait for a coherent pair of frames: depth and color
                frames = pipeline.wait_for_frames()
                #depth_frame = frames.get_depth_frame()
                color_frame = frames.get_color_frame()
                if not color_frame:
                    continue

                # Convert images to numpy arrays
                #depth_image = np.asanyarray(depth_frame.get_data())
                color_image = np.asanyarray(color_frame.get_data())

                #start steering
                #ser.write(b'#1:0.17;;\r\n')
		
                annotated_image, drawn_image = annotate_image(color_image, "output.png")
                print("we get angle", steering_angle)
                ser.write(f'#2:{steering_angle};;\r\n'.encode())

                # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
                #depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

                #depth_colormap_dim = depth_colormap.shape
                color_colormap_dim = color_image.shape

                # If depth and color resolutions are different, resize color image to match depth image for display
                #if depth_colormap_dim != color_colormap_dim:
                #    resized_color_image = cv2.resize(color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]), interpolation=cv2.INTER_AREA)
                #    images = np.hstack((resized_color_image, depth_colormap))
                #else:
                #    images = np.hstack((color_image, depth_colormap))

                # Show images
                cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
                cv2.imshow('RealSense', annotated_image)
                cv2.waitKey(1)

                #stop steering
                #ser.write(b'#1:0;;\r\n')

        finally:

            # Stop streaming
            pipeline.stop()
