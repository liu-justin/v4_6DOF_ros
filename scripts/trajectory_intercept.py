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

import os
print(os.getcwd())


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

fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
ax.axes.set_xlim3d(left=-2, right=2)
ax.axes.set_ylim3d(bottom=-2, top=2)
ax.axes.set_zlim3d(bottom=-2, top=2)

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
betas_depth_to_dia = np.load("/home/brigs/catkin_ws/src/v4_6dof/scripts/constants/betas.npy")

mc = MotorController()
rospy.init_node('talker', anonymous=True)

at_rest_str = input("is arm at rest position y/n?")
at_rest = True if at_rest_str=="y" else False

# if at rest, go to cobra position
if at_rest: mc.anglePublish([0, -np.pi/2, np.pi/2, 0, 0, 0], 2, True)

try:
    # getting the background frame
    while True:
        frames = pipeline.wait_for_frames() 
        aligned_frames = align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        if not depth_frame: continue

        # extracting and cleaning image
        depth_image = np.asanyarray(depth_frame.get_data())
        depth_cleaned = (depth_image*(255/(6/depth_scale))).astype(np.uint8)
        depth_cleaned = np.where((depth_cleaned > 255), 255, depth_cleaned)
        depth_cleaned = np.where((depth_cleaned <= 0), 0, depth_cleaned)
        # numbers for bilateral filter tuned in tests/realsense/tune_cleaning
        depth_cleaned = cv2.bilateralFilter(depth_cleaned, 9, 50, 50)

        cv2.namedWindow('Main', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('Main', depth_cleaned)
        key = cv2.waitKey(1)

        if key & 0xFF == ord('q') or key == 27:
            # save this depth_image as the background, but need to remove those 0s
            # depth_cleaned = (depth_image*(255/(6/depth_scale))).astype(np.uint8)
            # depth_cleaned = np.where((depth_cleaned > 255), 255, depth_cleaned)
            # depth_cleaned = np.where((depth_cleaned <= 0), 0, depth_cleaned)
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
        current_time = depth_frame.get_timestamp()/1000
        
        if not depth_frame: continue
        depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics

        # move thru all registered trajectories
        for traj in trajectories:
            if (len(traj.times) > 3): # 0.180212
                possible, intersection_point, time_until_intersection = traj.checkSphereIntersection([0,0,0], 0.4087037)
                if possible:
                    print(f"point: {intersection_point} time: {time_until_intersection}")
                    intersection_transf = mr.RpToTrans(np.identity(3), intersection_point)

                # mc.transfMatrixPublish(intersection_transf, time_until_intersection)

            # if total time is longer than 3 seconds, kill the trajectory
            if (current_time - traj.init_time) >= 3:
                trajectories.pop(trajectories.index(traj))
                if (len(traj.times) > 3):
                    old_trajectories.append(traj)

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        depth_cleaned = (depth_image*(255/(6/depth_scale))).astype(np.uint8)
        depth_cleaned = np.where((depth_cleaned > 255), 255, depth_cleaned)
        depth_cleaned = np.where((depth_cleaned <= 0), depth_background, depth_cleaned)
        depth_cleaned = cv2.bilateralFilter(depth_cleaned, 5, 42, 42) # tune these numbers in tune_cleaning
        depth_cleaned_3d = np.dstack((depth_cleaned,depth_cleaned,depth_cleaned))

        # create an canny edge picture, and rip data from it
        # values tuned in tests/realsense/tune_cleaning
        sigma = 0.38
        v = np.median(depth_cleaned)
        lower = int(max(0, (1.0 - sigma) * v))
        upper = int(min(255, (1.0 + sigma) * v))    
        depth_canny = cv2.Canny(depth_cleaned, lower, upper)
        depth_canny_3d = np.dstack((depth_canny,depth_canny,depth_canny))

        contours, hierarchy = cv2.findContours(depth_canny, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            try:
                # rectangle, not circle; slow shutter speed/motion blur elongates the circle into oval, need the shorter height
                (x,y), (width, height), angle = cv2.minAreaRect(c)
                diameter = min(width, height)
                depth = depth_frame.get_distance(int(x),int(y))            
            except: continue
                
            # removing weird edge cases
            if x*y*diameter <= 0: continue

            #checking perecentage of contour filled
            contour_area = cv2.contourArea(c)
            if (contour_area < 50): continue
            ellipse_area = np.pi*(diameter/2)**2
            if (contour_area/ellipse_area) < 0.5: continue

            # if depth/diameter relationship does not follow the trend in tests/realsense/depth_daimeter_eq-polyfit, then continue
            calculated_diameter = betas_depth_to_dia[0] + betas_depth_to_dia[1]*depth + betas_depth_to_dia[2]*(depth**2)
            if ((diameter-calculated_diameter)/calculated_diameter) > 0.25: continue

            # grabbing from original depth image, without the cleanup
            # if there was a way to grab from the cleaned array, I would do it
            point = rs.rs2_deproject_pixel_to_point(depth_intrin, [x,y], depth)
            if (point[0]*point[1]*point[2] == 0): continue

            # transf matrix established at the top
            point = transf_camera_to_base @ np.r_[point,1]

            # go thru all trajectories and see if this point fits in the projected path
            added = False
            for t in trajectories:
                success = t.append(current_time, point)
                if success: added = True
            
            # if the point was not in the projected path, then create a new trajectory
            if not added:
                trajectories.append(Trajectory(current_time, point))

            # paint circles onto canny and cleaned
            cv2.circle(depth_canny_3d, (int(x),int(y)), 3, (0,0,255), 1)
            cv2.circle(depth_cleaned_3d, (int(x),int(y)), int(diameter/2), (0,255,0),2)  

        # show images
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

    plt.show()

finally:
    # Stop streaming
    pipeline.stop()