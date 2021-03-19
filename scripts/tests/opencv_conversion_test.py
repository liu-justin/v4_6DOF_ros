import numpy as np
import cv2

depth_scale = 0.0010000000474974513

# Convert images to numpy arrays
image = cv2.imread("C:/Users/justin.liu/Documents/S/v4_6DOF_ros/scripts/tests/image1.jpg")
resized_image = cv2.resize(image, (500, 500))
depth_image = cv2.cvtColor(resized_image, cv2.COLOR_BGR2GRAY)

# depth_image = np.asanyarray(depth_frame.get_data())
depth_image_converted = depth_image/(6/depth_scale)
depth_image_converted_3d = np.dstack((depth_image_converted,depth_image_converted,depth_image_converted))
depth_image_converted_3d = np.where((depth_image_converted_3d > 1), 1, depth_image_converted_3d)
depth_image_converted_3d = np.where((depth_image_converted_3d < 0), 0, depth_image_converted_3d)

depth_image_255 = (depth_image_converted_3d*255).astype(np.uint8)

# https://stackoverflow.com/questions/41893029/opencv-canny-edge-detection-not-working-properly
sigma = 0.33
v = np.median(depth_image_255)
lower = int(max(0, (1.0 - sigma) * v))
upper = int(min(255, (1.0 + sigma) * v))    

depth_image_canny = cv2.Canny(depth_image_255, lower, upper)

# https://stackoverflow.com/questions/60259169/how-to-group-nearby-contours-in-opencv-python-zebra-crossing-detection
# https://www.geeksforgeeks.org/find-and-draw-contours-using-opencv-python/
contours, hierarchy = cv2.findContours(depth_image_canny, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

# images = np.hstack((depth_image_converted_3d,depth_image_canny))        
for c in contours:
    if cv2.contourArea(c) < 50:
        continue
    rect = cv2.minAreaRect(c)
    (x,y), (w,h), angle = rect
    aspect_ratio = max(w,h) / min(w,h)
    if (aspect_ratio > 2):
        continue

    cv2.drawContours(resized_image, c, -1 , (0,255,0),3)

# Show images
cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
cv2.imshow('RealSense', depth_image_canny)
cv2.waitKey(10000)