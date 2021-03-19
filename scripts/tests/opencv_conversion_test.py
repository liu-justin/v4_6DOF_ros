import pyrealsense2 as rs
import numpy as np
import cv2

img = np.zeros([500,500,3])
print(img)

img[:,:,0] = np.ones([500,500])*64
print(img)
img[:,:,1] = np.ones([500,500])*64
img[:,:,2] = np.ones([500,500])*64
print(img)

cv2.imwrite('color_img.jpg', img)
cv2.imshow("image", img)
cv2.waitKey()