## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

import numpy as np
import cv2

depth_scale = 0.0010000000474974513

# Convert images to numpy arrays
depth_image = np.load("scripts/tests/realsense/frames/depth_frame_0_15.npy")
color_image = np.load("scripts/tests/realsense/frames/color_frame_0_15.npy")
depth_background = np.load("scripts/tests/realsense/frames/depth_frame_background.npy")

depth_cleaned = (depth_image*(255/(6/depth_scale))).astype(np.uint8)
depth_cleaned = np.where((depth_cleaned > 255), 255, depth_cleaned)
depth_cleaned = np.where((depth_cleaned <= 0), depth_background, depth_cleaned)
depth_cleaned = cv2.bilateralFilter(depth_cleaned, 9, 50, 50)
depth_cleaned_3d = np.dstack((depth_cleaned,depth_cleaned,depth_cleaned))

cv2.namedWindow('canny_cleaned', cv2.WINDOW_AUTOSIZE)

def on_sigma_trackbar(new_sigma):
    sigma = new_sigma/100
    # sigma = 0.33
    v = np.median(depth_cleaned_3d)
    lower = int(max(0, (1.0 - sigma) * v))
    upper = int(min(255, (1.0 + sigma) * v))    

    depth_canny = cv2.Canny(depth_cleaned, lower, upper)
    depth_canny_3d = np.dstack((depth_canny, depth_canny, depth_canny))

    contours, hierarchy = cv2.findContours(depth_canny, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)
    print(hierarchy)

    for c in contours:
        try:
            (x,y), radius = cv2.minEnclosingCircle(c)              
        except: continue
        cv2.circle(depth_canny_3d, (int(x),int(y)), 3, (0,0,255), 1)
        # removing weird edge cases
        if x*y*radius <= 0: continue
        # # removing small contours
        contour_area = cv2.contourArea(c)
        if contour_area < 2: continue

        cv2.circle(depth_cleaned_3d, (int(x),int(y)), int(radius), (0,255,0),2)

    images = np.hstack((depth_canny_3d, depth_cleaned_3d))
    cv2.imshow('canny_cleaned', images)

cv2.createTrackbar("sigma", "canny_cleaned", 0, 100, on_sigma_trackbar)
on_sigma_trackbar(25)

cv2.waitKey()
