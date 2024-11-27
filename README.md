## Stereo Depth Estimation and Camera Calibration

This repository contains Python scripts to perform **Stereo Depth Estimation**, **Stereo Camera Calibration**, and **Stereo Image Capture**. The scripts use OpenCV to process stereo images and compute depth maps, enabling real-time distance calculations and accurate stereo vision calibration.

---

## Table of Contents
1. [Overview](#overview)
2. [Setup](#setup)
3. [Scripts Description](#scripts-description)
4. [Execution Steps](#execution-steps)
5. [References](#references)

---

## Overview

The project includes three main components:
1. **Stereo Depth Estimation**: Computes disparity maps and calculates depth for stereo images.
2. **Stereo Camera Calibration**: Calibrates and rectifies stereo cameras using chessboard images.
3. **Stereo Image Capture**: Captures stereo images and triggers calibration after sufficient data collection.

---

## Setup

### Prerequisites
- Python 3.8+
- OpenCV (`opencv-python` and `opencv-contrib-python`)

### Installation
Install OpenCV with necessary modules:
```bash
pip install opencv-python opencv-contrib-python numpy
```

---

## Scripts Description

### 1. **Stereo_Depth_Estimation.py**
This script computes a **disparity map** between rectified stereo images and calculates depth from disparity.

- **Key Functions**:
  - `compute_disparity(rectified_l, rectified_r)`: Generates a filtered disparity map and visualizes it using a color map.
  - `distance_calc(LX, RX)`: Computes depth using disparity values and the baseline distance between stereo cameras.

- **Usage**:
  - Input: Rectified stereo image pair.
  - Output: Disparity map and depth values.

---

### 2. **Stereo_Calibrate_Captured_images.py**
This script performs stereo camera calibration using captured chessboard images.

- **Key Functions**:
  - `calibrate_camera(dir_path, l_prefix_name, r_prefix_name, ...)`: Calibrates individual cameras using chessboard images.
  - `stereo_calibrate(...)`: Performs stereo calibration using the intrinsic parameters from both cameras.
  - `rectify_stereo_camera(...)`: Computes rectification maps for stereo image pairs.
  - `stereo_undistort_rectify(...)`: Generates undistortion and rectification maps for stereo images.

- **Output**:
  - Saves calibration data (camera matrices, distortion coefficients, rectification maps) to an XML file.

---

### 3. **Stereo_capture_images.py**
This script captures stereo images from a stereo camera setup and saves them for calibration.

- **Features**:
  - Press `s`: Save the current stereo image pair.
  - Press `ESC`: Exit the capture process.
  - After saving 50 images, the script triggers stereo camera calibration.

- **Key Function**:
  - `capture_images()`: Captures stereo images and saves them to disk.

---

## Execution Steps

### Step 1: Capture Stereo Images
Run the **Stereo_capture_images.py** script:
```bash
python Stereo_capture_images.py
```
- Use the `s` key to save stereo image pairs.
- After 50 images, the script triggers stereo calibration automatically.

### Step 2: Perform Stereo Calibration
Calibration data is automatically saved after 50 images are captured. Alternatively, run the calibration manually:
```bash
python Stereo_Calibrate_Captured_images.py
```
Ensure chessboard images are saved in the specified directory.

### Step 3: Generate Disparity Map and Compute Depth
Run the **Stereo_Depth_Estimation.py** script:
```bash
python Stereo_Depth_Estimation.py
```
- Input: Rectified stereo image pair.
- Output: Disparity map and depth values.

---

## References
- OpenCV Documentation: [https://docs.opencv.org/](https://docs.opencv.org/)
- Stereo Vision Basics: [https://learnopencv.com/](https://learnopencv.com/)
- Real-Time Depth Estimation with OpenCV: [https://opencv-python-tutroals.readthedocs.io/](https://opencv-python-tutroals.readthedocs.io/)

---

For any issues or contributions, feel free to open an issue or submit a pull request!
