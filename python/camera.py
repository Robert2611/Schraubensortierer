import cv2
import numpy as np
import glob
import os


class SorterCamera:
    __calibration_image_prefix = "chessboard_"
    __calibration_image_extension = "png"

    def __init__(self, url):
        self.__url = url
        self.__calibration_path = os.path.join(
            os.path.dirname(__file__), "..", "camera_calibration")

    def __save_calibration_value(self, name, value):
        fullpath = os.path.join(self.__calibration_path, name + ".npy")
        np.savetxt(fullpath, value)

    def __load_calibration_value(self, name, dtype=float):
        fullpath = os.path.join(self.__calibration_path, name + ".npy")
        return np.loadtxt(fullpath, dtype=dtype)

    def __take_raw_image(self):
        """ __take_raw_image() -> img

        @brief Take an image from the ip camera
        """
        vcap = cv2.VideoCapture(self.__url)
        _, img = vcap.read()
        vcap.release()
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        return img

    def take_calibration_images(self):
        """take_calibration_images() -> None
        @brief Take 10 images and save them on the harddrive.
                Next image is taken on keypress or after 3s.
                Use those images wih calibrate().
        """
        if not os.path.exists(self.__calibration_path):
            os.makedirs(self.__calibration_path)
        # take 10 images and save them to harddrive
        for i in range(10):
            img = self.__take_raw_image()
            filename = "{0}{1}.{2}".format(
                self.__calibration_image_prefix, i, self.__calibration_image_extension)
            filepath = os.path.join(self.__calibration_path, filename)
            cv2.imwrite(filepath, img)
            cv2.imshow('img', img)
            cv2.waitKey(3000)

    def __find_calibration_pattern(self, img, size):
        """__find_calibration_pattern(img, size) -> retval, object_points, image_points

        @brief Find a chessboard calibration pattern on the given image.

        @param image The cv2 image
        @param size The size of the calibration pattern (vertical count, horizontal count)
        """
        # termination criteria
        criteria = (cv2.TERM_CRITERIA_EPS +
                    cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
        objp = np.zeros((size[0]*size[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0:size[0], 0:size[1]].T.reshape(-1, 2)
        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(
            img, size, cv2.CALIB_CB_ADAPTIVE_THRESH)
        # If found, add object points, image points (after refining them)
        # TODO: remove or refactor
        if len(img.shape) > 2:
            img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        if ret == True:
            corners = cv2.cornerSubPix(
                img, corners, (11, 11), (-1, -1), criteria)
            return (True, objp, corners)
        return (False, None, None)

    def __load_camera_calibration(self):
        """__load_camera_calibration()

        @brief Load position calibration from npy files stored in the calibration folder
        """
        self.matrix = self.__load_calibration_value("matrix")
        self.newcameramtx = self.__load_calibration_value("newcameramtx")
        self.distortion = self.__load_calibration_value("distortion")
        self.roi = self.__load_calibration_value("roi", int)

    def __load_position_calibration(self):
        """__load_camera_calibration()

        @brief Load camera calibration from npy files stored in the calibration folder
        """
        self.r_vecs = self.__load_calibration_value("r_vecs")
        self.t_vecs = self.__load_calibration_value("t_vecs")

    def calibrate(self, size):
        """calibrate(size) -> retval

        @brief Calibrate the camera using the pictures taken with take_calibration_images().
                If successfull, write the resulting matrices and ROI to "camera_calibration.py"

        @param size The size of the calibration pattern (vertical count, horizontal count)
        """
        # Arrays to store object points and image points from all the images.
        objpoints = []  # 3d point in real world space
        imgpoints = []  # 2d points in image plane.
        pattern = "{0}*.{1}".format(self.__calibration_image_prefix,
                                    self.__calibration_image_extension)
        path = os.path.join(self.__calibration_path, pattern)
        images = glob.glob(path)

        for fname in images:
            img = cv2.imread(fname)
            # Find the chess board corners
            ret, object_points, image_points = self.__find_calibration_pattern(
                img, size)
            # If found, add object points, image points (after refining them)
            if ret == True:
                objpoints.append(object_points)
                imgpoints.append(image_points)
                # Draw and display the corners
                cv2.drawChessboardCorners(img, size, image_points, ret)
                cv2.imshow('img', img)
                cv2.waitKey(500)
            else:
                print("'{0}' failed".format(fname))

        if len(imgpoints) == 0:
            return False

        h, w = img.shape[:2]
        ret, matrix, distortion, r_vecs, t_vecs = cv2.calibrateCamera(
            objpoints, imgpoints, (w, h), None, None)

        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(
            matrix, distortion, (w, h), 1, (w, h))
        print("Matrix:")
        print(matrix)
        print("Optimal Matrix:")
        print(newcameramtx)
        print("ROI:")
        print(roi)

        self.__save_calibration_value("matrix", matrix)
        self.__save_calibration_value("newcameramtx", newcameramtx)
        self.__save_calibration_value("distortion", distortion)
        self.__save_calibration_value("roi", roi)

        return True

    def take_undistort_image(self):
        """ take_undistort_image() -> img

        @brief Takes an image and applies distortion correction to it
        """
        img = self.__take_raw_image()
        h,  w = img.shape[:2]

        # undistort
        self.__load_camera_calibration()
        img_undistort = cv2.undistort(
            img, self.matrix, self.distortion, None, self.newcameramtx)
        x, y, w, h = self.roi
        img_undistort = img_undistort[y:y+h, x:x+w]
        return img_undistort

    def calibrate_position(self, size, tile_size_mm):
        """calibrate_position(size, tile_size_mm) -> retval

        @brief Calibrate the position for an already calibrated camera.
                If successfull, write the resulting rotation and translation vectors to "position_calibration.py"

        @param size The size of the calibration pattern (vertical count, horizontal count)
        @param tile_size_mm Size of a single tile inside the chessboard calibration pattern
        """
        img = self.take_undistort_image()
        # Find the chess board corners
        ret, object_values, image_values = self.__find_calibration_pattern(
            img, size)
        # If found, add object points, image points (after refining them)
        if ret != True:
            print("Position calibration failed")
            return False
        # Draw and display the corners
        cv2.drawChessboardCorners(img, size, image_values, ret)

        # load camera caliration from hard drive
        self.__load_camera_calibration()

        # save values to hard drive
        ret, r_vecs, t_vecs = cv2.solvePnP(
            object_values * tile_size_mm, image_values, self.newcameramtx, None)
        self.__save_calibration_value("r_vecs", r_vecs)
        self.__save_calibration_value("t_vecs", t_vecs)

        # draw coordinate system lines
        coordinate_system_points = np.array(
            [[0, 0, 0], [100, 0, 0], [0, 100, 0]], np.float32)
        backprojected_points, _ = cv2.projectPoints(
            coordinate_system_points, r_vecs, t_vecs, self.newcameramtx, None)

        cv2.line(img, tuple(backprojected_points[0][0].astype(int)), tuple(
            backprojected_points[1][0].astype(int)), (0, 255, 255), thickness=5)
        cv2.line(img, tuple(backprojected_points[0][0].astype(int)), tuple(
            backprojected_points[2][0].astype(int)), (0, 255, 255), thickness=5)

        cv2.imshow('img', img)
        cv2.waitKey(2000)

    def get_word_coordinates(self, x, y):
        """get_word_coordinates(x,y)

        @brief Get real world coordinates with z=0 from screen coordinates x and y
        """
        self.__load_camera_calibration()
        self.__load_position_calibration()

        # https://answers.opencv.org/question/62779/image-coordinate-to-world-coordinate-opencv/
        # https://www.fdxlabs.com/calculate-x-y-z-real-world-coordinates-from-a-single-camera-using-opencv/
        # https://stackoverflow.com/questions/12299870/computing-x-y-coordinate-3d-from-image-point

        # using the pinhole camera model
        # s * [u,v,1] = M * ( R * [X,Y,Zconst] + t)
        # with s: scaling, M: camera matrix, R: roation matrix, t: translation vector
        uv1 = np.array([x, y, 1])
        # get inverse of camera matrix
        _, inv_intrinsic = cv2.invert(self.newcameramtx)
        # get rotation matrix from rotation vector and invert it
        rot, _ = cv2.Rodrigues(self.r_vecs)
        _, inv_rot = cv2.invert(rot)

        # => s * (R^-1)*(M^-1)*[u,v,1] = [X,Y,Z] + (R^-1)*t
        # => s * left = right
        left = inv_rot.dot(inv_intrinsic.dot(uv1))
        right = inv_rot.dot(self.t_vecs)
        # find scaling factor so that z = 0
        z_const = 0
        # only using the z (i.e. "2") component of the vectors left and right
        s = (z_const + right[2])/left[2]

        #  => [X,Y,Z] = s * (R^-1)*((M^-1)*[u,v,1] - t)
        pos = inv_rot.dot(s * inv_intrinsic.dot(uv1)-self.t_vecs)
        return pos[0:2]
