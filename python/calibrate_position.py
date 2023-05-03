import numpy as np
import cv2 as cv
import camera_calibration

url = "rtsp://homeassistant:pwtypolig@192.168.178.117/stream1"
vcap = cv.VideoCapture(url)
ret, img = vcap.read()

h,  w = img.shape[:2]
# undistort
img_undistort = cv.undistort(img, camera_calibration.mtx,
                   camera_calibration.distortion, None, camera_calibration.newcameramtx)
x, y, w, h = camera_calibration.roi
img_undistort = img_undistort[y:y+h, x:x+w]
gray = cv.cvtColor(img_undistort, cv.COLOR_BGR2GRAY)

size = (7, 10)
chessboard_tile_size = 25


# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((size[0]*size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:size[0], 0:size[1]
                       ].T.reshape(-1, 2) * chessboard_tile_size

# Find the chess board corners
ret, corners = cv.findChessboardCorners(gray, size, cv.CALIB_CB_ADAPTIVE_THRESH)
# If found, add object points, image points (after refining them)
if ret != True:
    print("Position calibration failed")
    exit(0)
corners2 = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
# Draw and display the corners
#cv.drawChessboardCorners(img_undistort, size, corners, ret)

h, w = img.shape[:2]

ret, r_vecs, t_vecs = cv.solvePnP(objp, corners2, camera_calibration.newcameramtx, None)
with open("position_calibration.py", "w") as file:
    file.write("import numpy as np\n")
    file.write("r_vecs = np.array("+str(np.ndarray.tolist(r_vecs))+")\n")
    file.write("t_vecs = np.array("+str(np.ndarray.tolist(t_vecs))+")\n")

coordinate_system_points = np.array([[0,0,0],[100,0,0],[0,100,0]], np.float32)
backprojected_points, _ = cv.projectPoints(coordinate_system_points, r_vecs, t_vecs, camera_calibration.newcameramtx, None)

cv.line(img_undistort, tuple(backprojected_points[0][0].astype(int)),tuple(backprojected_points[1][0].astype(int)),(0, 255, 255),thickness=5)
cv.line(img_undistort, tuple(backprojected_points[0][0].astype(int)),tuple(backprojected_points[2][0].astype(int)),(0, 255, 255),thickness=5)

cv.imshow('img', img_undistort)
cv.waitKey(2000)
