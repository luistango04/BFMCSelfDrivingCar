import time

import pyrealsense2 as rs

def init():
	global camera_resolutionx
	global camera_resolutiony
	global starttime 
	starttime = time.time()
	camera_resolutionx = 320
	camera_resolutiony = 240
	global xm_per_pix
	global ym_per_pix

	# Defining variables to hold meter-to-pixel conversion
	ym_per_pix = 280 / camera_resolutiony#  ## GUESSING ITS ABOUT THIS FAR Standard lane width is 3.7 cm divided by lane width in 		pixels which is NEEDS TUNING
	# calculated to be approximately 720 pixels not to be confused with frame height
	xm_per_pix = 35  / camera_resolutionx
	starttime = time.time()  ## PROOGRAM START
	

def camerainit():
    import pyrealsense2 as rs

	
    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()
    try:	 
	    for dev in context.query_devices():
	     for sensor in dev.sensors:
	      sensor.stop()
	      sensor.close()
	     dev.close()
 
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

    #config.enable_stream(rs.stream.depth, 320, 240, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, camera_resolutionx, camera_resolutiony, rs.format.bgr8, 30)

    # Start streaming
    pipeline.start(config)
    return pipeline
def pidcarsetting(self, kp, ki, kd, k_t, ser):
    # kp proportional time
    # ki integral coefficient
    # kd derivativecoefficient
    # k_t integraltime
    # ser serial handler

    # write srial fucntions to Car
    ser.write(b'#4:1;;\r\n')
    command = f"#6:{kp};{ki};{kd};{k_t};;\r\n".encode()
    ser.write(command)
    ser.write(b'#5:1;;\r\n')
    ser.readline()

    # parse read line validate system settings.....

    # hand the system until validation

    # after 10 seconds throw exception and rebooot

    # read serial  back to car
    return 1

    def __str__(self):
        return f"yaw_rate: {self.yaw_rate}, lateral_acceleration: {self.lateral_acceleration}, longitudinal_acceleration: {self.longitudinal_acceleration}, speed: {self.speed}, steering_wheel_angle: {self.steering_wheel_angle}, steering_wheel_velocity: {self.steering_wheel_velocity}"
