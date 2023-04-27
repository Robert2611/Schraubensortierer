#!/usr/bin/env python2

import cv2
import numpy as np
import os

# Read image
image_file = os.path.join(os.path.dirname(__file__), "../asset/BlobTest.jpg")
im = cv2.imread(image_file, cv2.IMREAD_GRAYSCALE)

#########


params = cv2.SimpleBlobDetector_Params()
params.filterByArea = True
params.minArea = 1000
params.filterByInertia = False
params.filterByConvexity = False
params.filterByCircularity = False

# Create a detector with the parameters
ver = (cv2.__version__).split('.')
if int(ver[0]) < 3:
    detector = cv2.SimpleBlobDetector(params)
else:
    detector = cv2.SimpleBlobDetector_create(params)

#########

# Detect blobs.
keypoints = detector.detect(im)

# Draw detected blobs as red circles.
# cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
im_with_keypoints = cv2.drawKeypoints(im, keypoints, np.array(
    []), (0, 0, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

font = cv2.FONT_HERSHEY_PLAIN
for kp in keypoints:
    p = (int(kp.pt[0]), int(kp.pt[1]))
    cv2.putText(im_with_keypoints, 'Test', p, font, 1, (0, 0, 0), 1)

# Show keypoints
cv2.imshow("Keypoints", im_with_keypoints)
cv2.waitKey(0)
