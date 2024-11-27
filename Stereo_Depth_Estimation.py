import numpy as np
import cv2 as cv


def compute_disparity(rectified_l, rectified_r):
                        window_size = 7
                        min_disp = 0
                        max_disp = 64
                        num_disp = max_disp - min_disp

                        stereo = cv.StereoSGBM_create(
                            minDisparity = min_disp,
                            numDisparities = num_disp,
                            blockSize= window_size,
                            preFilterCap=63,
                            uniquenessRatio = 15,
                            speckleWindowSize = 10,
                            speckleRange = 1,
                            disp12MaxDiff = 20,
                            P1 = 8*3*window_size**2,
                            P2 = 32*3*window_size**2,
                            mode=cv.STEREO_SGBM_MODE_SGBM_3WAY
                        )

                        left_matcher = stereo
                        right_matcher = cv.ximgproc.createRightMatcher(left_matcher)

                        l = 70000
                        s = 1.2

                        disparity_filter = cv.ximgproc.createDisparityWLSFilter(left_matcher)
                        disparity_filter.setLambda(l)
                        disparity_filter.setSigmaColor(s)

                        d_l_1 = left_matcher.compute(rectified_l, rectified_r).astype(np.float32) / 16.0
                        d_r_1 = right_matcher.compute(rectified_r, rectified_l).astype(np.float32) / 16.0

                        d_l = np.int16(d_l_1)
                        d_r = np.int16(d_r_1)
                        
                        d_filter = disparity_filter.filter(d_l, rectified_l, None, d_r)

                        disparity = cv.normalize(d_filter, None, alpha=0, beta=255, norm_type=cv.NORM_MINMAX)
                        filteredImg = np.uint8(disparity)
                   #     points = cv.reprojectImageTo3D(d_l_1, Q)
                        print("depth",d_filter[395][204])
                        cv.circle(filteredImg,(int(400),int(122)),2,(120,255,255),2,cv.LINE_AA)
                        
                        # points = cv.reprojectImageTo3D(d_filter, Q)
                        # colors = cv.cvtColor(rectified_l, cv.COLOR_BGR2RGB)
                        # mask = d_filter > d_filter.min()
                        # out_points = points[mask]
                        # out_colors = colors[mask]
                        # # append_ply_array(out_points, out_colors)

                        # disparity_scaled = (d_filter - min_disp) / num_disp
                        # disparity_scaled += abs(np.amin(disparity_scaled))
                        # disparity_scaled /= np.amax(disparity_scaled)
                        # disparity_scaled[disparity_scaled < 0] = 0
                        # final= np.array(255 * disparity_scaled, np.uint8) 
                        
                        imgr2=cv.applyColorMap( filteredImg, cv.COLORMAP_MAGMA)
                        
                        return imgr2

def distance_calc(LX,RX):
        Baseline=45
        Disparsity=LX-RX
        z_depth=(Baseline*405)/Disparsity
        print("gg",z_depth)

        print(z_depth)

# def main():
   
#                 cap= cv.VideoCapture(0) 
#                 #load calibrated data from camera_calib file
#                 cv_file = cv.FileStorage()
#                 cv_file.open('/home/ubuntu/dummy_final_stereo_data.xml', cv.FileStorage_READ)
#                 matrix_l = cv_file.getNode('matrix_1').mat()
#                 matrix_r = cv_file.getNode('matrix_2').mat()
#                 dist_l = cv_file.getNode('dist_1').mat()
#                 dist_r = cv_file.getNode('dist_2').mat()
#                 map_w_L=cv_file.getNode('map_w_L').mat()
#                 map_h_L=cv_file.getNode('map_h_L').mat()
#                 map_w_R=cv_file.getNode('map_w_R').mat()
#                 map_h_R=cv_file.getNode('map_h_R').mat()
#                              #define marker parameter
   
                
#                   # 0 -> Right Camera
       
#                 while True:
                       
#                         success, frame = cap.read()

#                         Left_Fram = frame[0:460, 0:640]
#                         Right_Fram = frame[0:460, 640:1280]
#                         Left_Frame = cv.remap(Left_Fram, map_w_L, map_h_L, cv.INTER_LINEAR)
#                         Right_Frame = cv.remap(Right_Fram, map_w_R, map_h_R, cv.INTER_LINEAR)

#                         img=compute_disparity(Left_Frame, Right_Frame)
#                         cv.imshow("Dmap",img)
#                         cv.waitKey(1)


# main()
        
