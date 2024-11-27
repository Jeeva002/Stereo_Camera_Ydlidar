import numpy as np
import cv2
from Stereo_Calibrate_Captured_images import calibrate_camera_main

# Print instructions for user interaction
print('Starting the Calibration. Press and maintain the space bar to exit the script\n')
print('Push (s) to save the image you want and push (c) to see the next frame without saving the image')

# Function to capture stereo images
def capture_images():
    cap = cv2.VideoCapture(0)  # Open camera; adjust the index if necessary
    id_image = 0  # Counter for saved images

    while True:
        success1, frame = cap.read()  # Read a frame from the camera
        if not success1:
            print("Failed to read from the camera. Exiting...")
            break

        # Split the captured frame into left and right stereo images
        leftframe = frame[0:460, 0:640]  # Left frame
        rightframe = frame[0:460, 640:1280]  # Right frame

        # Display the stereo frames
        cv2.imshow('Img 1 - Left Camera', leftframe)
        cv2.imshow('Img 2 - Right Camera', rightframe)

        # Wait for a key press and process user input
        k = cv2.waitKey(1)

        if k == 27:  # 'ESC' key to exit
            print("Exiting...")
            break
        elif k == ord('s'):  # 's' key to save the images
            cv2.imwrite(f'/home/ubuntu/stereo_images7/ff/imageL{id_image}.png', leftframe)
            cv2.imwrite(f'/home/ubuntu/stereo_images7/gg/imageR{id_image}.png', rightframe)
            print(f"Images saved: imageL{id_image}.png and imageR{id_image}.png")
            id_image += 1
        elif k == ord('q'):  # 'q' key to quit early
            print("Quitting early...")
            break
        elif id_image == 50:  # Trigger calibration after 50 images
            print("Starting stereo camera calibration...")
            calibrate_camera_main()  # Call the calibration function
            print("Calibration complete. Exiting...")
            break

    # Release resources and close OpenCV windows
    cap.release()
    cv2.destroyAllWindows()

# Uncomment the line below to run the script directly
# capture_images()
