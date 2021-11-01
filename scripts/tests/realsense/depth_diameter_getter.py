import pyrealsense2 as rs
import numpy as np
import cv2
from collections import deque

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# gets values for the different depths/ perceived diameters, and saves them to a npy file

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

depth_background = np.array([])

depths = []
diameters = []

kept_contours = []

try:
    while True:
        frames = pipeline.wait_for_frames() 
        depth_frame = frames.get_depth_frame()
        # if not depth_frame or color_frame: continue
        depth_image = np.asanyarray(depth_frame.get_data())
        depth_cleaned = (depth_image*(255/(max_depth_scaled))).astype(np.uint8)
        depth_cleaned = np.where((depth_cleaned > 255), 255, depth_cleaned)
        depth_cleaned = np.where((depth_cleaned <= 0), 0, depth_cleaned)
        depth_cleaned = cv2.bilateralFilter(depth_cleaned, 5, 42, 42) # tune these numbers in tune_cleaning
        depth_cleaned_3d = np.dstack((depth_cleaned,depth_cleaned,depth_cleaned))

        cv2.namedWindow('Main', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('Main', depth_cleaned)
        key = cv2.waitKey(1)

        if key & 0xFF == ord('q') or key == 27:
            # save this depth_image as the background, but need to remove those 0s
            depth_cleaned = (depth_image*(255/(max_depth_scaled))).astype(np.uint8)
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
        depth_frame = frames.get_depth_frame()
        current_time = depth_frame.get_timestamp()/1000
        if not depth_frame:
            continue
        depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        depth_cleaned = (depth_image*(255/(max_depth_scaled))).astype(np.uint8)
        depth_cleaned = np.where((depth_cleaned > 255), 255, depth_cleaned)
        depth_cleaned = np.where((depth_cleaned <= 5), depth_background, depth_cleaned)
        depth_cleaned = cv2.bilateralFilter(depth_cleaned, 5, 42, 42) # tune these numbers in tune_cleaning
        depth_cleaned_3d = np.dstack((depth_cleaned,depth_cleaned,depth_cleaned))

        sigma = 0.38
        v = np.median(depth_cleaned)
        lower = int(max(0, (1.0 - sigma) * v))
        upper = int(min(255, (1.0 + sigma) * v))    
        depth_canny = cv2.Canny(depth_cleaned, lower, upper)
        depth_canny_3d = np.dstack((depth_canny,depth_canny,depth_canny))

        contours, hierarchy = cv2.findContours(depth_canny, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for c in contours:
            try:
                (x,y), (width, height), angle = cv2.minAreaRect(c)
                diameter = min(width, height)
                depth = depth_frame.get_distance(int(x),int(y))                      
            except: continue

            # removing weird edge cases
            if x*y*diameter <= 0: continue
            if x < 20: continue
            
            # checking perecentage of contour filled
            contour_area = cv2.contourArea(c) 
            ellipse_area = np.pi*(diameter/2)*(diameter/2)
            if (contour_area/ellipse_area) < 0.75: continue
            
            cv2.circle(depth_canny_3d, (int(x),int(y)), 10, (0,0,255), 10)
            cv2.circle(depth_cleaned_3d, (int(x),int(y)), int(diameter/2), (0,255,0),10)
            kept_contours.append(c)
            
        # Show images
        images = np.hstack((depth_canny_3d, depth_cleaned_3d))
        cv2.namedWindow('Main', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('Main', images)

        key = cv2.waitKey(1)

        if key & 0xFF == ord('g') or key == 27:
            print("in g")
            for c in kept_contours:
                try:
                    (x,y), radius = cv2.minEnclosingCircle(c)
                    depth = depth_frame.get_distance(int(x),int(y))                  
                except: continue
                
                # checking perecentage of contour filled
                contour_area = cv2.contourArea(c) 
                ellipse_area = np.pi*radius**2
                if (contour_area/ellipse_area) < 0.4: continue
                
                cv2.drawContours(depth_cleaned_3d, c, 0, (0,255,0), 5)
                cv2.circle(depth_cleaned_3d, (int(x),int(y)), int(radius), (0,255,0),2)

                cv2.namedWindow('Areyousure', cv2.WINDOW_AUTOSIZE)
                cv2.imshow('Areyousure', depth_cleaned_3d)
                key2 = cv2.waitKey(100000)
                if (key2 & 0xFF == ord('y')):
                    depths.append(depth_frame.get_distance(int(x),int(y)))
                    diameters.append(radius*2)
                    cv2.destroyWindow("Areyousure")
                    break

                cv2.destroyWindow("Areyousure")
            
        if key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()
            break

finally:

    # Stop streaming
    pipeline.stop()
    fig = plt.figure()
    ax = fig.add_subplot(111)

    for i in range(len(depths)):
        ax.scatter(depths[i], diameters[i])

    b2, b1, b0 = np.polyfit(depths, diameters, 2)

    x = np.arange(min(depths), max(depths),0.1)
    y = b2*x**2 + b1*x + b0
    y_upper = 1.15*(b2*x**2 + b1*x + b0)
    y_lower = 0.85*(b2*x**2 + b1*x + b0)

    ax.plot(x,y)
    ax.plot(x,y_upper)
    ax.plot(x,y_lower)
    ax.set_xlabel("Depth (m)")
    ax.set_ylabel("Diameter (mm)")

    plt.show()

    save = input("want to save these values?: y/n ")
    if save=="y": np.save("betas_baseball", np.array([b0, b1, b2]))

        