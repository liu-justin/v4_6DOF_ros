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
from collections import deque

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

# 10m/s
# store all good contours in this list outside the try loop (try a deque)
# associate an time to live with each contour, every loop decrease the time to live by 1
# on each loop, look in sphere of influence around the good contour to see if any past contours are in the sphere
#   sphere from x,y coords and depth
#   use speed of 5m/s, little under 12 mph; with 30 fps, 0.03333 secs btwn frames --> sphere of 0.166m
#   add current contour to all past contours in sphere,
contour_storage = deque([deque([]),deque([]),deque([]),deque([]),deque([])])

try:
    while True:

        # returns a composite frame
        frames = pipeline.wait_for_frames() 
        depth_frame = frames.get_depth_frame()
        if not depth_frame:
            continue
        depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics

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
        contours, hierarchy = cv2.findContours(depth_image_canny, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for c in contours:
            try:
                ellipse = cv2.fitEllipse(c)
                (x,y), (w,h), angle = ellipse
            except:
                continue

            # removing small contours
            contour_area = cv2.contourArea(c)
            if contour_area < 30:
                continue

            if x*y*w*h <= 0:
                continue
            
            # checking perecentage of contour filled
            ellipse_area = np.pi*w*h/4
            if (contour_area/ellipse_area) < 0.5:
                continue
            
            aspect_ratio = max(w,h) / min(w,h)  
            if aspect_ratio > 6:
                continue

            # removing zero depth
            print(f"{x}, {y}")
            center_depth = depth_frame.get_distance(int(x),int(y))
            if center_depth < 0.1:
                continue

            point = rs.rs2_deproject_pixel_to_point(depth_intrin, [x,y], center_depth)
            # print(point)

            # check all past contours and see spheres of influence
            for i in range(len(contour_storage)-1):
                for past_point in contour_storage[i]:
                    if ((past_point[0][0] - point[0])**2 + (past_point[0][1] - point[1])**2 + (past_point[0][2] - point[2])**2)**0.5 < 0.166*(4-i):
                        past_point.appendleft(point)
                        if len(past_point) > 5:
                            # print(past_point)
                            x = 0


            # if no contours link, then add the contour in its own deque in the 5th deque in contour_storage
            contour_storage[4].append(deque([point]))

            cv2.ellipse(depth_image_converted_3d, ellipse, (0,255,0),2)

        # images = np.hstack((depth_image_converted,depth_image_canny))        

        # Show images
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense', depth_image_converted_3d)
        cv2.waitKey(1)

        # empty 1st deque in contour_storage (ttl = 0)
        contour_storage[0].clear()

        # rotate contour storage left
        contour_storage.rotate(-1)

finally:

    # Stop streaming
    pipeline.stop()