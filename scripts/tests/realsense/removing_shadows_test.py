## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

import numpy as np
import cv2

depth_scale = 0.0010000000474974513

# Convert images to numpy arrays
depth_image = np.load("depth_frame.npy")
color_image = np.load("color_frame.npy")

depth_cleaned = (depth_image*(255/(6/depth_scale))).astype(np.uint8)

depth_cleaned = np.where((depth_cleaned > 255), 255, depth_cleaned)
depth_cleaned = np.where((depth_cleaned <= 0), 0, depth_cleaned)
depth_cleaned_3d = np.dstack((depth_cleaned,depth_cleaned,depth_cleaned))

# somehow need to clean up the shadows

thresh, depth_mask = cv2.threshold(depth_cleaned,1,255,cv2.THRESH_BINARY_INV)

depth_image_after_inpaint = cv2.inpaint(depth_cleaned_3d, depth_mask, 3, cv2.INPAINT_NS)

# https://stackoverflow.com/questions/41893029/opencv-canny-edge-detection-not-working-properly
sigma = 0.33
v = np.median(depth_cleaned_3d)
lower = int(max(0, (1.0 - sigma) * v))
upper = int(min(255, (1.0 + sigma) * v))    

depth_image_canny = cv2.Canny(depth_cleaned_3d, lower, upper)

# https://stackoverflow.com/questions/60259169/how-to-group-nearby-contours-in-opencv-python-zebra-crossing-detection
# https://www.geeksforgeeks.org/find-and-draw-contours-using-opencv-python/
contours, hierarchy = cv2.findContours(depth_image_canny, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

for c in contours:
    try:
        # ellipse = cv2.fitEllipse(c)
        # (x,y), (w,h), angle = ellipse
        ellipse = cv2.minEnclosingCircle(c)
        (x,y), radius = ellipse
        
    except:
        continue

    # removing small contours
    contour_area = cv2.contourArea(c)
    if contour_area < 30:
        continue

    if x*y*radius <= 0:
        continue
    
    # checking perecentage of contour filled
    # ellipse_area = np.pi*w*h/4
    ellipse_area = np.pi*radius**2
    if (contour_area/ellipse_area) < 0.35:
        continue
    
    # aspect_ratio = max(w,h) / min(w,h)  
    # if aspect_ratio > 6:
    #     continue

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
cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
cv2.imshow('RealSense', images)

cv2.namedWindow('Inpaint', cv2.WINDOW_AUTOSIZE)
cv2.imshow('Inpaint', depth_image_after_inpaint)

cv2.namedWindow('Depth', cv2.WINDOW_AUTOSIZE)
cv2.imshow('Depth', depth_cleaned_3d)

cv2.namedWindow('Mask', cv2.WINDOW_AUTOSIZE)
cv2.imshow('Mask', depth_mask)

cv2.waitKey(100000)
