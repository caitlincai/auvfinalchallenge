"""computes the camera matrix and distortion coefficients"""

import cv2
import numpy as np
import glob

# 1. Prepare object points for calibration pattern
pattern_size = (7, 6)  # (width, height) = (# inner corners)
square_size = 0.025  # in meters

objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)
objp *= square_size

objpoints = []
imgpoints = []

# 2. Load images and find corners
images = glob.glob('/home/lsc/auvc_ws/calib_images/*.png')
print("Images found:", len(images))
print(images[:5])

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, pattern_size, None)
    print(fname, "Chessboard detected:", ret)
    if not ret:
        continue
    corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1),
                                criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
    objpoints.append(objp)
    imgpoints.append(corners2)

print("Number of calibration images:", len(imgpoints))

# 3. Calibrate
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

# 4. Extract intrinsics
fx = mtx[0, 0]
fy = mtx[1, 1]
cx = mtx[0, 2]
cy = mtx[1, 2]

print("RMS re-projection error:", ret)
print(f"fx={fx:.2f}, fy={fy:.2f}, cx={cx:.2f}, cy={cy:.2f}")
print("Camera matrix:\n", mtx)
print("Distortion coefficients:\n", dist)