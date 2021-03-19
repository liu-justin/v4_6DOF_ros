## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

###############################################
##      Open CV and Numpy integration        ##
###############################################

# simple output to windows with openCV and numpy

# create pipeline and config at start
# pipeline_wrapper + pipeline_profile + device + device_product_line seems standard
# set up config for both depth and color(width, height of pixel map, some scale, fps)
# start pipeline
# while True:
#   get frames from pipeline.wait for frames
#   get depth/color frames from frames.get_what_frame
#   convert frame data into numpy array
#   use openCV to alter image to be appealing

import pyrealsense2 as rs
import numpy as np
import cv2

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

depth_scale = 0.0010000000474974513

# Start streaming
pipeline.start(config)

try:
    while True:

        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        if not depth_frame:
            continue

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        depth_image_converted = depth_image/(6/depth_scale)
        depth_image_converted_3d = np.dstack((depth_image_converted,depth_image_converted,depth_image_converted))
        depth_image_converted_3d = np.where((depth_image_converted_3d > 1), 1, depth_image_converted_3d)
        depth_image_converted_3d = np.where((depth_image_converted_3d < 0), 0, depth_image_converted_3d)

        depth_image_255 = (depth_image_converted_3d*255).astype(np.uint8)

        # https://stackoverflow.com/questions/41893029/opencv-canny-edge-detection-not-working-properly
        sigma = 0.33
        v = np.median(depth_image_255)
        lower = int(max(0, (1.0 - sigma) * v))
        upper = int(min(255, (1.0 + sigma) * v))    

        depth_image_canny = cv2.Canny(depth_image_255, lower, upper)

        # https://stackoverflow.com/questions/60259169/how-to-group-nearby-contours-in-opencv-python-zebra-crossing-detection
        # https://www.geeksforgeeks.org/find-and-draw-contours-using-opencv-python/
        contours, hierarchy = cv2.findContours(depth_image_canny, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        # images = np.hstack((depth_image_converted_3d,depth_image_canny))        

        # Show images
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense', depth_image_canny)
        cv2.waitKey(1)

finally:

    # Stop streaming
    pipeline.stop()