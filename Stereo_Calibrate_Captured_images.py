import numpy as np
import cv2
import glob
import re

img_size = (640, 460)

criteria_cal = (cv2.TERM_CRITERIA_EPS + cv2.TermCriteria_MAX_ITER, 30, 0.001)

criteria_stereo_cal = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-5)

def numerical_sort(value):
    numbers = re.compile(r'(\d+)')
    parts = numbers.split(value)
    parts[1::2] = map(int, parts[1::2])
    return parts

def calibrate_camera(dir_path, l_prefix_name, r_prefix_name, size = 0.029, width=8, height=5):

    o_points = np.zeros((height*width, 3), np.float32)
    o_points[:, :2] = np.mgrid[0:width, 0:height].T.reshape(-1, 2)

    o_points = o_points * size

    obj_points = []
    img_points_l = []
    img_points_r = []

    l_images = glob.glob(dir_path + '/' + l_prefix_name +'*.png')
    r_images = glob.glob(dir_path + '/' + r_prefix_name +'*.png')

    l_images = sorted(l_images, key=numerical_sort)
    r_images = sorted(r_images, key=numerical_sort)

    for i,_ in enumerate(l_images):
        l_img = cv2.imread(l_images[i])
        print("shape",l_img.shape)
        r_img = cv2.imread(r_images[i])
        gray_l_img = cv2.cvtColor(l_img, cv2.COLOR_BGR2GRAY)
        gray_r_img = cv2.cvtColor(r_img, cv2.COLOR_BGR2GRAY)

        l_ret, l_corners = cv2.findChessboardCorners(gray_l_img, (width, height), None)
        r_ret, r_corners = cv2.findChessboardCorners(gray_r_img, (width, height), None)

        obj_points.append(o_points)

        if l_ret is True:

            corners_l = cv2.cornerSubPix(gray_l_img, l_corners, (11, 11), (-1, -1), criteria_cal)
            img_points_l.append(corners_l)

            l_img = cv2.drawChessboardCorners(l_img, (width, height), l_corners, l_ret)

        if r_ret is True:

            corners_r = cv2.cornerSubPix(gray_r_img, r_corners, (11, 11), (-1, -1), criteria_cal)
            img_points_r.append(corners_r)

            r_img = cv2.drawChessboardCorners(r_img, (width, height), r_corners, r_ret)

    ret, matrix_l, dist_l, rvecs_l, tvecs_l = cv2.calibrateCamera(obj_points, img_points_l, gray_l_img.shape[::-1], None, None)

    ret, matrix_r, dist_r, rvecs_r, tvecs_r = cv2.calibrateCamera(obj_points, img_points_r, gray_r_img.shape[::-1], None, None)

    return [obj_points, img_points_l, img_points_r, ret, matrix_l, dist_l, matrix_r, dist_r]

def stereo_calibrate(obj_points, img_points_l, img_points_r, img_size, matrix_l, dist_l, matrix_r, dist_r):
    
    flag = 0

    flag |= cv2.CALIB_FIX_INTRINSIC

    retval, matrix_1, dist_1, matrix_2, dist_2, R, T, E, F = cv2.stereoCalibrate(obj_points, img_points_l, img_points_r, matrix_l, dist_l, matrix_r, dist_r, img_size, criteria= criteria_stereo_cal,flags=cv2.CALIB_ZERO_TANGENT_DIST + cv2.CALIB_ZERO_DISPARITY +cv2.CALIB_FIX_TANGENT_DIST +cv2.CALIB_SAME_FOCAL_LENGTH)

    return [retval, matrix_1, dist_1, matrix_2, dist_2, R, T, E, F]

def rectify_stereo_camera(matrix_1, dist_1, matrix_2, dist_2, R, T):

    rotation_1, rotation_2, pose_1, pose_2, Q, roi_left, roi_right = cv2.stereoRectify(matrix_1, dist_1, matrix_2, dist_2, (8, 5), R, T)

    return [rotation_1, rotation_2, pose_1, pose_2, Q, roi_left, roi_right]


dir_path='/home/ubuntu/stereo_images6_undistort/both2'
prefix_left='chessboard-L'
prefix_right='chessboard-R'



def stereo_undistort_rectify(matrix_1, dist_1, rot_1, pose_1,matrix_2, dist_2, rot_2, pose_2):
    img_size_l=(640,460)
    map_w_L, map_h_L = cv2.initUndistortRectifyMap(matrix_1, dist_1, rot_1, pose_1, img_size_l, cv2.CV_32FC1)

    map_w_R, map_h_R = cv2.initUndistortRectifyMap(matrix_2, dist_2, rot_2, pose_2, img_size_l, cv2.CV_32FC1)

    return map_w_L, map_h_L,map_w_R, map_h_R


def calibrate_camera_main():
          
            obj_points, img_points_l, img_points_r, ret, matrix_l, dist_l, matrix_r, dist_r = calibrate_camera(dir_path, prefix_left,prefix_right)
            print('Camera calibrated')
            retval, matrix_1, dist_1, matrix_2, dist_2, R, T, E, F = stereo_calibrate(obj_points, img_points_l, img_points_r, img_size, matrix_l, dist_l, matrix_r, dist_r)
            print('Stereo calibration done')
            rot_1, rot_2, pose_1, pose_2, Q, roi_left, roi_right = rectify_stereo_camera(matrix_1, dist_1, matrix_2, dist_2, R, T)
            print('Stereo camera rectified')

            map_w_L, map_h_L,map_w_R, map_h_R=stereo_undistort_rectify(matrix_1, dist_1, rot_1, pose_1,matrix_2, dist_2, rot_2, pose_2)
            cv_file = cv2.FileStorage('/home/ubuntu/dummy_final_stereo_data1.xml', cv2.FILE_STORAGE_WRITE)

            cv_file.write('matrix_1',matrix_1)
            cv_file.write('matrix_2',matrix_2)
            cv_file.write('dist_1',dist_1)
            cv_file.write('dist_2',dist_2)
            cv_file.write('map_w_L',map_w_L)
            cv_file.write('map_h_L', map_h_L)
            cv_file.write('map_w_R', map_w_R)
            cv_file.write('map_h_R',map_h_R)
            cv_file.write('Q',Q)

            return "success"
