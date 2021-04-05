## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

import numpy as np
import cv2

depth_scale = 0.0010000000474974513

def nothing(x):
    pass

cv2.namedWindow('canny_cleaned', cv2.WINDOW_AUTOSIZE)

depth_image = np.load("scripts/tests/realsense/frames/depth_frame_0_2.npy")
color_image = np.load("scripts/tests/realsense/frames/color_frame_0_2.npy")
depth_background = np.load("scripts/tests/realsense/frames/depth_frame_background.npy")

cv2.createTrackbar("image", "canny_cleaned", 0,39, nothing)
cv2.createTrackbar("sigma", "canny_cleaned", 0, 100, nothing)
cv2.createTrackbar("bilateral_diameter", "canny_cleaned", 1,50, nothing)
cv2.createTrackbar("bilateral_colorspace", "canny_cleaned", 1,100, nothing)
cv2.createTrackbar("bilateral_coordspace", "canny_cleaned", 1,100, nothing)

depth_cleaned = (depth_image*(255/(6/depth_scale))).astype(np.uint8)
depth_cleaned = np.where((depth_cleaned > 255), 255, depth_cleaned)
depth_cleaned = np.where((depth_cleaned <= 0), depth_background, depth_cleaned)
depth_cleaned = cv2.bilateralFilter(depth_cleaned, 9, 50, 50)

depth_cleaned_3d = np.dstack((depth_cleaned,depth_cleaned,depth_cleaned))
sigma = 0.33
v = np.median(depth_cleaned_3d)
lower = int(max(0, (1.0 - sigma) * v))
upper = int(min(255, (1.0 + sigma) * v))    

depth_canny = cv2.Canny(depth_cleaned, lower, upper)
depth_canny_3d = np.dstack((depth_canny, depth_canny, depth_canny))

while 1:
    images = np.hstack((depth_canny_3d, depth_cleaned_3d))
    cv2.imshow('canny_cleaned', images)
    key = cv2.waitKey(1)
    if key & 0xFF == ord('q'):
        break

    depth_image = np.load("scripts/tests/realsense/frames/depth_frame_0_" + str(cv2.getTrackbarPos("image", "canny_cleaned")) + ".npy")
    color_image = np.load("scripts/tests/realsense/frames/color_frame_0_" + str(cv2.getTrackbarPos("image", "canny_cleaned")) + ".npy")
    depth_cleaned = (depth_image*(255/(6/depth_scale))).astype(np.uint8)
    depth_cleaned = np.where((depth_cleaned > 255), 255, depth_cleaned)
    depth_cleaned = np.where((depth_cleaned <= 0), depth_background, depth_cleaned)
    depth_cleaned = cv2.bilateralFilter(depth_cleaned, cv2.getTrackbarPos("bilateral_diameter", "canny_cleaned"),\
                                                       cv2.getTrackbarPos("bilateral_colorspace", "canny_cleaned"),\
                                                       cv2.getTrackbarPos("bilateral_coordspace", "canny_cleaned"))

    depth_cleaned_3d = np.dstack((depth_cleaned,depth_cleaned,depth_cleaned))

    sigma = cv2.getTrackbarPos("sigma", "canny_cleaned")/100
    v = np.median(depth_cleaned_3d)
    lower = int(max(0, (1.0 - sigma) * v))
    upper = int(min(255, (1.0 + sigma) * v))    

    depth_canny = cv2.Canny(depth_cleaned, lower, upper)
    depth_canny_3d = np.dstack((depth_canny, depth_canny, depth_canny))

    contours, hierarchy = cv2.findContours(depth_canny, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)

    for c in contours:
        try:
            # (x,y), radius = cv2.minEnclosingCircle(c)
            (x,y), (width, height), angle = cv2.minAreaRect(c)
            print(f"{width}, {height}")
        except: continue
        cv2.circle(depth_canny_3d, (int(x),int(y)), 3, (0,0,255), 1)
        if x*y <= 0: continue
        contour_area = cv2.contourArea(c)
        if contour_area < 2: continue

        cv2.circle(depth_cleaned_3d, (int(x),int(y)), int(width), (0,255,0),2)

cv2.destroyAllWindows()
