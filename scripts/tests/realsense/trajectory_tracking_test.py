import pyrealsense2 as rs
import numpy as np
import cv2
from collections import deque

from Trajectory import Trajectory

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

trajectories = []

fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
ax.axes.set_xlim3d(left=-2, right=2)
ax.axes.set_ylim3d(bottom=-2, top=2)
ax.axes.set_zlim3d(bottom=0, top=2)

transf_camera_to_base = np.array([[0,0,1,0],\
                                 [0,-1,0,0],\
                                 [1,0,0,0],\
                                 [0,0,0,1]])

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
        time = depth_frame.get_timestamp()/1000
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

        sigma = 0.33
        v = np.median(depth_cleaned_3d)
        lower = int(max(0, (1.0 - sigma) * v))
        upper = int(min(255, (1.0 + sigma) * v))    
        depth_canny = cv2.Canny(depth_cleaned_3d, lower, upper)

        contours, hierarchy = cv2.findContours(depth_canny, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        current_frame_points = []
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
            ellipse_area = np.pi*radius**2
            if (contour_area/ellipse_area) < 0.5:
                continue

            # grabbing from original depth image, without the cleanup
            point = rs.rs2_deproject_pixel_to_point(depth_intrin, [x,y], depth_frame.get_distance(int(x),int(y)))
            if (point[0]*point[1]*point[2] == 0):
                continue
            point = np.dot(transf_camera_to_base, np.r_[point,1])

            added = False
            for t in trajectories:
                success = t.append(time, point)
                if success:
                    added = True
            if not added:
                trajectories.append(Trajectory(time, point))

            cv2.circle(depth_cleaned_3d, (int(x),int(y)), int(radius), (0,255,0),2)  

        # Show images
        cv2.namedWindow('Main', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('Main', depth_cleaned_3d)

        cv2.namedWindow('Canny', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('Canny', depth_canny)
        key = cv2.waitKey(1)

        if key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()
            for t in trajectories:
                for i in range(len(t.points)):
                    ax.scatter(t.points[i][0], t.points[i][1], t.points[i][2])
                    ax.text(t.points[i][0], t.points[i][1], t.points[i][2], str(i))
                    plt.pause(0.01)
            cv2.waitKey(10000)
            
            break

    plt.show()

finally:

    # Stop streaming
    pipeline.stop()