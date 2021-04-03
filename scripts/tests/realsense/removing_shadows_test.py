## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

import numpy as np
import cv2

depth_scale = 0.0010000000474974513

# Convert images to numpy arrays
depth_image = np.load("scripts/tests/realsense/frames/depth_frame_0_7.npy")
color_image = np.load("scripts/tests/realsense/frames/color_frame_0_7.npy")
depth_background = np.load("scripts/tests/realsense/frames/depth_frame_background.npy")

depth_cleaned = (depth_image*(255/(6/depth_scale))).astype(np.uint8)
depth_cleaned = np.where((depth_cleaned > 255), 255, depth_cleaned)
depth_cleaned = np.where((depth_cleaned <= 0), depth_background, depth_cleaned)
depth_cleaned_3d = np.dstack((depth_cleaned,depth_cleaned,depth_cleaned))

depth_cleaned = cv2.bilateralFilter(depth_cleaned, 9, 75, 75)

# https://stackoverflow.com/questions/41893029/opencv-canny-edge-detection-not-working-properly
sigma = 0.33
v = np.median(depth_cleaned_3d)
lower = int(max(0, (1.0 - sigma) * v))
upper = int(min(255, (1.0 + sigma) * v))    

depth_image_canny = cv2.Canny(depth_cleaned, lower, upper)

# https://stackoverflow.com/questions/60259169/how-to-group-nearby-contours-in-opencv-python-zebra-crossing-detection
# https://www.geeksforgeeks.org/find-and-draw-contours-using-opencv-python/
contours, hierarchy = cv2.findContours(depth_image_canny, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

for c in contours:
    try:
        ellipse = cv2.minEnclosingCircle(c)
        (x,y), radius = ellipse                
    except: continue
        
    # removing weird edge cases
    if x*y*radius <= 0: continue

    # # removing small contours
    contour_area = cv2.contourArea(c)
    if contour_area < 2: continue

    cv2.circle(depth_cleaned_3d, (int(x),int(y)), int(radius), (0,255,0),2)
    


# Apply colormap on depth image (image must be converted to 8-bit per pixel first)
depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

depth_colormap_dim = depth_colormap.shape
color_colormap_dim = color_image.shape

# If depth and color resolutions are different, resize color image to match depth image for display
if depth_colormap_dim != color_colormap_dim:
    resized_color_image = cv2.resize(color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]), interpolation=cv2.INTER_AREA)
    images = np.hstack((resized_color_image, depth_colormap))
else:
    images = np.hstack((color_image, depth_colormap))

# Show images
# cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
# cv2.imshow('RealSense', images)

cv2.namedWindow('Canny', cv2.WINDOW_AUTOSIZE)
cv2.imshow('Canny', depth_image_canny)

cv2.namedWindow('Cleaned', cv2.WINDOW_AUTOSIZE)
cv2.imshow('Cleaned', depth_cleaned_3d)

cv2.namedWindow('Filtered', cv2.WINDOW_AUTOSIZE)
cv2.imshow('Filtered', depth_cleaned)

cv2.waitKey(100000)
