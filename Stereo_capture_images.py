import numpy as np
import cv2
from Stereo_Calibrate_Captured_images import calibrate_camera_main
# print("worksss")
print('Starting the Calibration. Press and maintain the space bar to exit the script\n')
print('Push (s) to save the image you want and push (c) to see next frame without saving the image')




# Call the two cameras
  # 0 -> Right Camera
  # 2 -> Left Camera
def capture_images():
        cap= cv2.VideoCapture(0) 
        id_image=0
        while True:
            succes1, frame = cap.read()

            leftframe = frame[0:460, 0:640]
            rightframe = frame[0:460, 640:1280]

            k = cv2.waitKey(1)

            if k == 27:
                break
            elif k == ord('s'): # wait for 's' key to save and exit
                cv2.imwrite('/home/ubuntu/stereo_images7/ff/imageL' + str(id_image) + '.png', leftframe)
                cv2.imwrite('/home/ubuntu/stereo_images7/gg/imageR' + str(id_image) + '.png', rightframe)
                print("images saved!")
                id_image += 1

            cv2.imshow('Img 1',leftframe)
            cv2.imshow('Img 2',rightframe)
            if k == ord('q'):
                break;
            elif id_image==50:
                calib_cam=calibrate_camera_main()
                print("dead")
                
                break;

        # Release and destroy all windows before termination
        cap.release()


        cv2.destroyAllWindows() 

