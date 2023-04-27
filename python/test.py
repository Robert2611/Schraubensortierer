#!/usr/bin/env python2

import cv2 as cv
import numpy as np
from numpy.linalg import eig
import math
import os
from matplotlib import pyplot as plt


def angle_from_moments(moms):
    return math.atan(moms["mu11"] / (moms["mu20"] - moms["mu02"])) - math.pi/2


def inertia_from_moments(moms):
    denominator = math.sqrt(
        pow(2 * moms["mu11"], 2) + pow(moms["mu20"] - moms["mu02"], 2))
    eps = 1e-2
    if (denominator > eps):
        cosmin = (moms["mu20"] - moms["mu02"]) / denominator
        sinmin = 2 * moms["mu11"] / denominator
        cosmax = -cosmin
        sinmax = -sinmin

        imin = 0.5 * (moms["mu20"] + moms["mu02"]) - 0.5 * \
            (moms["mu20"] - moms["mu02"]) * cosmin - moms["mu11"] * sinmin
        imax = 0.5 * (moms["mu20"] + moms["mu02"]) - 0.5 * \
            (moms["mu20"] - moms["mu02"]) * cosmax - moms["mu11"] * sinmax
        return imin / imax
    else:
        return 1


def write_to_image(im, center, text):
    cv.putText(im,
               text,
               (int(center[0]), int(center[1])),
               cv.FONT_HERSHEY_PLAIN,
               1,
               (0, 0, 255),
               1
               )


# Read image
image_file = os.path.join(os.path.dirname(__file__), "../asset/Realbild2.jpg")
im_bw = cv.imread(image_file, cv.IMREAD_GRAYSCALE)
threshold = 200
minArea = 300
minHoleArea = 10

# make binary image
_retval, bin = cv.threshold(im_bw, threshold, 255, cv.THRESH_BINARY)
# plt.imshow(bin, 'gray')
# plt.show()

# _hierarchy: (next,previus,child,parent)
contours, hierarchy = cv.findContours(
    bin, cv.RETR_TREE, cv.CHAIN_APPROX_TC89_L1)
print("{} Konturen gefunden".format(len(contours)))

# make bgr image for drawing on it
im = cv.cvtColor(im_bw, cv.COLOR_GRAY2BGR)

for idx, cnt in enumerate(contours):
    if idx == 0:
        # skip the surrounding rect
        continue
    # is direct child of surrounding rect
    if hierarchy[0][idx][3] != 124:
        continue
    hole_index = hierarchy[0][idx][2]
    has_hole = hole_index != -1
    has_hole = has_hole and cv.moments(contours[hole_index])[
        "m00"] > minHoleArea
    length = cv.arcLength(cnt, True)
    moms = cv.moments(cnt)
    area = moms["m00"]
    if area < minArea:
        continue
    center = (moms["m10"] / area, moms["m01"] / area)
    circularity = 4 * math.pi * area / (length * length)
    inertia = inertia_from_moments(moms)
    hull = cv.convexHull(cnt)
    hullArea = cv.contourArea(hull)
    if hullArea > 1:
        convexity = area / hullArea
    else:
        convexity = 0
    cv.drawContours(im, contours, idx, (255, 0, 0), 4)
    if has_hole:
        cv.drawContours(im, contours, hole_index, (0, 255, 0), 2)
    # print(area)
    # print(circularity)
    # print(inertia)
    # print(convexity)
    # print(has_hole)
    if convexity < 0.98:
        type = "schraube"
        angle = angle_from_moments(moms)
        p0 = (int(center[0]), int(center[1]))
        p1 = (int(center[0] + math.cos(-angle)*10),
              int(center[1] - math.sin(-angle)*10))
        cv.line(im, p0, p1, (0, 255, 0))
    else:
        if not has_hole:
            continue
        poly = cv.approxPolyDP(cnt, 0.1 * math.sqrt(area), True)
        points = np.array([[p[0] for p in poly]])
        cv.polylines(im, points, True, (0, 255, 255))
        if len(poly) == 6:
            type = "mutter"

        else:
            type = "scheibe"
    write_to_image(im, center, type)
    #write_to_image(im, [center[0], center[1]+10], str(convexity))
   # write_to_image(im, [center[0], center[1]+10], str(convexity))


cv.imwrite("test.png", im)
os.system("test.png")
