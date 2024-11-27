import numpy as np
import cv2 as cv

# Function to compute disparity map between rectified stereo images
def compute_disparity(rectified_l, rectified_r):
    # Set parameters for StereoSGBM (Semi-Global Block Matching)
    window_size = 7
    min_disp = 0  # Minimum disparity
    max_disp = 64  # Maximum disparity
    num_disp = max_disp - min_disp  # Number of disparities

    # Create left and right stereo matchers
    stereo = cv.StereoSGBM_create(
        minDisparity=min_disp,
        numDisparities=num_disp,
        blockSize=window_size,
        preFilterCap=63,
        uniquenessRatio=15,
        speckleWindowSize=10,
        speckleRange=1,
        disp12MaxDiff=20,
        P1=8 * 3 * window_size ** 2,  # Penalty for small disparities
        P2=32 * 3 * window_size ** 2,  # Penalty for large disparities
        mode=cv.STEREO_SGBM_MODE_SGBM_3WAY
    )

    # Create right stereo matcher
    left_matcher = stereo
    right_matcher = cv.ximgproc.createRightMatcher(left_matcher)

    # Parameters for WLS (Weighted Least Squares) disparity filtering
    l = 70000  # Lambda for regularization
    s = 1.2  # Sigma for color filtering

    # Create WLS filter
    disparity_filter = cv.ximgproc.createDisparityWLSFilter(left_matcher)
    disparity_filter.setLambda(l)
    disparity_filter.setSigmaColor(s)

    # Compute disparity maps for left and right images
    d_l_1 = left_matcher.compute(rectified_l, rectified_r).astype(np.float32) / 16.0
    d_r_1 = right_matcher.compute(rectified_r, rectified_l).astype(np.float32) / 16.0

    # Convert disparities to integer for filtering
    d_l = np.int16(d_l_1)
    d_r = np.int16(d_r_1)

    # Apply WLS filter to disparity map
    d_filter = disparity_filter.filter(d_l, rectified_l, None, d_r)

    # Normalize disparity for visualization
    disparity = cv.normalize(d_filter, None, alpha=0, beta=255, norm_type=cv.NORM_MINMAX)
    filteredImg = np.uint8(disparity)

    # Example: Highlight a specific point in the disparity map
    cv.circle(filteredImg, (int(400), int(122)), 2, (120, 255, 255), 2, cv.LINE_AA)

    # Optionally, apply a color map for better visualization
    imgr2 = cv.applyColorMap(filteredImg, cv.COLORMAP_MAGMA)

    return imgr2

# Function to calculate distance based on disparity
def distance_calc(LX, RX):
    Baseline = 45  # Distance between the stereo cameras in mm
    Disparity = LX - RX  # Difference in x-coordinates of corresponding points
    z_depth = (Baseline * 405) / Disparity  # Calculate depth
    print("Depth (Z):", z_depth)

# Main function (currently commented out)
# This function captures stereo video frames, remaps them using the calibration data, and computes the disparity map.
# def main():
#     cap = cv.VideoCapture(0)  # Capture from stereo camera
#     # Load pre-calibrated stereo camera data
#     cv_file = cv.FileStorage()
#     cv_file.open('/home/ubuntu/dummy_final_stereo_data.xml', cv.FileStorage_READ)
#     matrix_l = cv_file.getNode('matrix_1').mat()
#     matrix_r = cv_file.getNode('matrix_2').mat()
#     dist_l = cv_file.getNode('dist_1').mat()
#     dist_r = cv_file.getNode('dist_2').mat()
#     map_w_L = cv_file.getNode('map_w_L').mat()
#     map_h_L = cv_file.getNode('map_h_L').mat()
#     map_w_R = cv_file.getNode('map_w_R').mat()
#     map_h_R = cv_file.getNode('map_h_R').mat()

#     while True:
#         success, frame = cap.read()
#         # Split stereo frame into left and right images
#         Left_Fram = frame[0:460, 0:640]
#         Right_Fram = frame[0:460, 640:1280]

#         # Rectify the frames using the calibration maps
#         Left_Frame = cv.remap(Left_Fram, map_w_L, map_h_L, cv.INTER_LINEAR)
#         Right_Frame = cv.remap(Right_Fram, map_w_R, map_h_R, cv.INTER_LINEAR)

#         # Compute disparity map and display
#         img = compute_disparity(Left_Frame, Right_Frame)
#         cv.imshow("Disparity Map", img)
#         cv.waitKey(1)

# main()
