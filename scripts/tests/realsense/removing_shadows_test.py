## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

import numpy as np
import cv2

# Convert images to numpy arrays
depth_image = np.load("depth_frame_2.npy")
color_image = np.load("color_frame_2.npy")

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
cv2.waitKey(10000)
