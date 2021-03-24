## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

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

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
profile = pipeline.start(config)
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale() # 0.0010000000474974513

align_to = rs.stream.color
align = rs.align(align_to)

depth_background = np.array([])

# 10m/s
# store all good contours in this list outside the try loop (try a deque)
# associate an time to live with each contour, every loop decrease the time to live by 1
# on each loop, look in sphere of influence around the good contour to see if any past contours are in the sphere
#   sphere from x,y coords and depth
#   use speed of 5m/s, little under 12 mph; with 30 fps, 0.03333 secs btwn frames --> sphere of 0.166m
#   add current contour to all past contours in sphere,
point_storage = deque([deque([]),deque([]),deque([]),deque([]),deque([])])

# fig, ax = plt.subplots()
# ax = fig.add_subplot(111, projection="3d")
# ax.scatter(0, 0, 0)

try:

    while True:
        frames = pipeline.wait_for_frames() 
        aligned_frames = align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        if not depth_frame:
            continue
        depth_image = np.asanyarray(depth_frame.get_data())
        depth_cleaned = (depth_image*(255/(6/depth_scale))).astype(np.uint8)
        depth_cleaned = np.where((depth_cleaned > 255), 255, depth_cleaned)
        depth_cleaned = np.where((depth_cleaned <= 0), 0, depth_cleaned)

        cv2.namedWindow('Main', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('Main', depth_cleaned)
        key = cv2.waitKey(1)

        if key & 0xFF == ord('q') or key == 27:
            # save this depth_image as the background, but need to remove those 0s
            depth_cleaned = (depth_image*(255/(6/depth_scale))).astype(np.uint8)
            depth_cleaned = np.where((depth_cleaned > 255), 255, depth_cleaned)
            depth_cleaned = np.where((depth_cleaned <= 0), 0, depth_cleaned)
            thresh, depth_mask = cv2.threshold(depth_cleaned,1,255,cv2.THRESH_BINARY_INV)
            depth_background = cv2.inpaint(depth_cleaned, depth_mask, 3, cv2.INPAINT_TELEA)
            
            cv2.namedWindow('New', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('New', depth_background)
            cv2.waitKey(100000)
            cv2.destroyAllWindows()
            break

    while True:

        # returns a composite frame
        frames = pipeline.wait_for_frames() 
        aligned_frames = align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        if not depth_frame:
            continue
        depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        depth_cleaned = (depth_image*(255/(6/depth_scale))).astype(np.uint8)
        depth_cleaned = np.where((depth_cleaned > 255), 255, depth_cleaned)
        depth_cleaned = np.where((depth_cleaned <= 0), depth_background, depth_cleaned)
        depth_cleaned_3d = np.dstack((depth_cleaned,depth_cleaned,depth_cleaned))

        thresh, depth_mask = cv2.threshold(depth_cleaned,1,255,cv2.THRESH_BINARY_INV)

        # https://stackoverflow.com/questions/41893029/opencv-canny-edge-detection-not-working-properly
        sigma = 0.33
        v = np.median(depth_cleaned_3d)
        lower = int(max(0, (1.0 - sigma) * v))
        upper = int(min(255, (1.0 + sigma) * v))    

        depth_canny = cv2.Canny(depth_cleaned_3d, lower, upper)

        # https://stackoverflow.com/questions/60259169/how-to-group-nearby-contours-in-opencv-python-zebra-crossing-detection
        # https://www.geeksforgeeks.org/find-and-draw-contours-using-opencv-python/
        contours, hierarchy = cv2.findContours(depth_canny, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for c in contours:
            try:
                ellipse = cv2.minEnclosingCircle(c)
                (x,y), radius = ellipse                
            except:
                continue

            # removing small contours
            contour_area = cv2.contourArea(c)
            if contour_area < 30:
                continue

            # removing weird edge cases
            if x*y*radius <= 0:
                continue
            
            # checking perecentage of contour filled
            # ellipse_area = np.pi*w*h/4
            ellipse_area = np.pi*radius**2
            if (contour_area/ellipse_area) < 0.5:
                continue

            # grabbing from original depth image, without the cleanup
            point = rs.rs2_deproject_pixel_to_point(depth_intrin, [x,y], depth_frame.get_distance(int(x),int(y)))
            if (point[0]*point[1]*point[2] == 0):
                continue
            # ax.scatter(point[0], point[1], point[2])
            # plt.pause(0.01)

            linked = False

            # check all past contours and see spheres of influence
            for i in range(len(point_storage)-1):
                for past_point in point_storage[i]:
                    if ((past_point[0][0] - point[0])**2 + (past_point[0][1] - point[1])**2 + (past_point[0][2] - point[2])**2)**0.5 < 0.166*(4-i):
                        past_point.appendleft(point)
                        linked = True
                        if len(past_point) > 5:
                            print(past_point)


            # # if no contours link, then add the contour in its own deque in the 5th deque in point_storage
            if not linked: 
                point_storage[4].append(deque([point]))

            # cv2.ellipse(depth_image_converted_3d, ellipse, (0,255,0),2)
            cv2.circle(depth_cleaned_3d, (int(x),int(y)), int(radius), (0,255,0),2)

        # images = np.hstack((depth_image_converted,depth_image_canny))        

        # Show images
        cv2.namedWindow('Main', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('Main', depth_cleaned_3d)

        cv2.namedWindow('Canny', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('Canny', depth_canny)
        cv2.waitKey(1)

        # empty 1st deque in point_storage (ttl = 0)
        point_storage[0].clear()

        # rotate contour storage left
        point_storage.rotate(-1)

    plt.show()

finally:

    # Stop streaming
    pipeline.stop()