import pyrealsense2 as rs
import numpy as np
import cv2

depth_image = np.array([np.array([2899, 2899, 2899, 2912, 2912, 2912]), np.array([2926,2912,2912,2912,2926,2926])])
depth_image_255 = depth_image * 255 // 3000 # selected 3.0 because max range of this camera is 3meters

depth_image_3d = np.dstack((depth_image_255*64,depth_image_255*128,depth_image_255*192))

# Apply colormap on depth image (image must be converted to 8-bit per pixel first)
depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

depth_colormap_dim = depth_colormap.shape