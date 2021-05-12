import pyrealsense2 as rs
import numpy as np
import cv2
from collections import deque

from Trajectory import Trajectory

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# saves a set of frames in which a ball appears, should probably record all frames btwn first and last frame where it appears

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

fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
ax.axes.set_xlim3d(left=0, right=2)
ax.axes.set_ylim3d(bottom=-2, top=2)
ax.axes.set_zlim3d(bottom=-2, top=2)

i = 0
j = 0

try:
    while True:
        frames = pipeline.wait_for_frames() 
        aligned_frames = align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        # if not depth_frame or color_frame: continue
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
            np.save("depth_frame_background",depth_background)  

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
        current_time = depth_frame.get_timestamp()/1000
        if not depth_frame:
            continue
        depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics

        # Convert images to numpy arrays
        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())
        depth_cleaned = (depth_image*(255/(6/depth_scale))).astype(np.uint8)
        depth_cleaned = np.where((depth_cleaned > 255), 255, depth_cleaned)
        depth_cleaned = np.where((depth_cleaned <= 0), depth_background, depth_cleaned)
        depth_cleaned_3d = np.dstack((depth_cleaned,depth_cleaned,depth_cleaned))

        thresh, depth_mask = cv2.threshold(depth_cleaned,1,255,cv2.THRESH_BINARY_INV)

        sigma = 0.33
        v = np.median(depth_cleaned)
        lower = int(max(0, (1.0 - sigma) * v))
        upper = int(min(255, (1.0 + sigma) * v))    
        depth_canny = cv2.Canny(depth_cleaned, lower, upper)

        contours, hierarchy = cv2.findContours(depth_canny, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for c in contours:
            try:
                ellipse = cv2.minEnclosingCircle(c)
                (x,y), radius = ellipse                
            except: continue
            
            np.save("depth_frame_"+str(i)+"_"+str(j),depth_image)            
            np.save("color_frame_"+str(i)+"_"+str(j),color_image)
            j += 1
            

        # Show images
        cv2.namedWindow('Main', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('Main', depth_cleaned_3d)

        cv2.namedWindow('Canny', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('Canny', depth_canny)
        key = cv2.waitKey(1)

        if key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()       
            break

finally:

    # Stop streaming
    pipeline.stop()


        