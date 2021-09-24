#!/usr/bin/env python3

import rospy

import pyrealsense2 as rs
import numpy as np
import cv2

from modules.Trajectory import Trajectory
from modules import modern_robotics as mr
from modules.MotorController import MotorController

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import v4_6dof.msg as msg

import sys, os

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

# Start streaming
profile = pipeline.start(config)
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale() # 0.0010000000474974513
max_depth = 6
max_depth_scaled = max_depth/depth_scale

# getting transf matrix from camera to robot origin
rot_camera_to_90 = mr.RollPitchYawToRot(0,0,np.pi/4)
transf_camera_to_90 = mr.RpToTrans(rot_camera_to_90, [0,0,0])
transf_90_to_base = np.array([[0, 0,1, 0.07274],\
                              [0,-1,0, 0.06474],\
                              [1, 0,0, -0.0175],\
                              [0, 0,0,       1]])
transf_camera_to_base = transf_90_to_base @ transf_camera_to_90

# initializing globals
trajectories = []
old_trajectories = []
depth_background = np.array([])
betas_depth_to_dia = np.load("/home/justin/catkin_ws/src/v4_6dof/scripts/constants/betas.npy")

mc = MotorController()
rospy.init_node('talker', anonymous=True)

# testing realsense filters comparing to mine
decimate = rs.decimation_filter(2)
hole_filling = rs.hole_filling_filter()
spatial = rs.spatial_filter()
temporal = rs.temporal_filter()

try:
    # getting the background frame
    while True:
        frames = pipeline.wait_for_frames() 
        frames = spatial.process(frames).as_frameset()
        # frames = temporal.process(frames).as_frameset()
        # frames = hole_filling.process(frames).as_frameset()
        depth_frame = frames.get_depth_frame()
        if not depth_frame: continue

        # extracting and cleaning image
        # look into cleaning arrays from intel, like decimate and hole filling
        depth_image = np.asanyarray(depth_frame.get_data())
        depth_cleaned = (depth_image*(255/(max_depth_scaled))).astype(np.uint8)
        depth_cleaned = np.where((depth_cleaned > 255), 255, depth_cleaned)
        depth_cleaned = np.where((depth_cleaned <= 0), 0, depth_cleaned)
        # numbers for bilateral filter tuned in tests/realsense/tune_cleaning
        depth_cleaned = cv2.bilateralFilter(depth_cleaned, 5, 42, 42)

        cv2.namedWindow('Main', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('Main', depth_cleaned)
        key = cv2.waitKey(1)

        if key & 0xFF == ord('q') or key == 27:
            # hole filling filter
            thresh, depth_mask = cv2.threshold(depth_cleaned,50,255,cv2.THRESH_BINARY_INV)
            depth_background = cv2.inpaint(depth_cleaned, depth_mask, 3, cv2.INPAINT_TELEA)

            images = np.hstack((depth_cleaned, depth_mask, depth_background))

            cv2.namedWindow('New', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('New', images)
            cv2.waitKey(100000)
            cv2.destroyAllWindows()
            break

    # at_rest_str = input("is arm at rest position y/n?")
    # at_rest = at_rest_str=="y"
    at_rest = True
    # # if at rest, go to cobra position
    if at_rest: mc.anglePublish([0, -np.pi/2, np.pi/2, 0, -np.pi/2, 0], 0.4, True)

    i = 0

    # trying to find a ping pong ball now
    while True:
        frames = pipeline.wait_for_frames() 
        # frames = hole_filling.process(frames).as_frameset()
        depth_frame = frames.get_depth_frame()
        current_time = depth_frame.get_timestamp()/1000
        
        if not depth_frame: continue
        depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        depth_cleaned = (depth_image*(255/(max_depth_scaled))).astype(np.uint8)
        depth_cleaned = np.where((depth_cleaned > 255), 255, depth_cleaned)
        depth_cleaned = np.where((depth_cleaned <= 5), depth_background, depth_cleaned)
        depth_cleaned = cv2.bilateralFilter(depth_cleaned, 5, 42, 42) # tune these numbers in tune_cleaning
        depth_cleaned_3d = np.dstack((depth_cleaned,depth_cleaned,depth_cleaned))

        # create an canny edge picture
        # values tuned in tests/realsense/tune_cleaning
        sigma = 0.38
        v = np.median(depth_cleaned)
        lower = int(max(0, (1.0 - sigma) * v))
        upper = int(min(255, (1.0 + sigma) * v))    
        depth_canny = cv2.Canny(depth_cleaned, lower, upper)
        depth_canny_3d = np.dstack((depth_canny,depth_canny,depth_canny))

        contours, hierarchy = cv2.findContours(depth_canny, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        np.save("depth_frame_"+str(i),depth_cleaned)
        np.save("depth_3d_"+str(i), depth_cleaned_3d)
        i += 1
        if i > 50:
            cv2.destroyAllWindows()
            break

        images = np.hstack((depth_canny_3d, depth_cleaned_3d))
        cv2.namedWindow('Main', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('Main', images)

        key = cv2.waitKey(1)
        if key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()
            # for t in old_trajectories:
            #     for i in range(len(t.points)):
            #         ax.scatter(t.points[i][0], t.points[i][1], t.points[i][2])
            #         plt.pause(0.01)
            # cv2.waitKey()            
            break

finally:
    # Stop streaming
    pipeline.stop()