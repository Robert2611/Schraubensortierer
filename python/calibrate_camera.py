import numpy as np
import cv2 as cv
import glob

size = (7, 10)
# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((size[0]*size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:size[0], 0:size[1]].T.reshape(-1, 2)
# Arrays to store object points and image points from all the images.
objpoints = []  # 3d point in real world space
imgpoints = []  # 2d points in image plane.
images = glob.glob('../chessboard_*.png')
for fname in images:
    img = cv.imread(fname)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    # Find the chess board corners
    ret, corners = cv.findChessboardCorners(gray, size, cv.CALIB_CB_ADAPTIVE_THRESH)
    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)
        corners2 = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners2)
        # Draw and display the corners
        cv.drawChessboardCorners(img, size, corners2, ret)
        cv.imshow('img', img)
        cv.waitKey(500)
    else:
        print("'{0}' failed".format(fname))

h, w = img.shape[:2]
print(img.shape[:2])

ret, matrix, distortion, r_vecs, t_vecs = cv.calibrateCamera(
    objpoints, imgpoints, (w, h), None, None)

newcameramtx, roi = cv.getOptimalNewCameraMatrix(
    matrix, distortion, (w, h), 1, (w, h))
print("Matrix:")
print(matrix)
print("Optimal Matrix:")
print(newcameramtx)
print("ROI:")
print(roi)
with open("camera_calibration.py", "w") as file:
    file.write("import numpy as np\n")
    file.write("mtx = np.array("+str(np.ndarray.tolist(matrix))+")\n")
    file.write("newcameramtx = np.array(" +
               str(np.ndarray.tolist(newcameramtx))+")\n")
    file.write("distortion = np.array("+str(np.ndarray.tolist(distortion))+")\n")
    file.write("roi="+str(roi)+"\n")
cv.destroyAllWindows()
