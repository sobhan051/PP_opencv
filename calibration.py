import cv2
import numpy as np
import glob

# Set up the checkerboard pattern size (number of internal corners)
checkerboard_size = (9, 6)

# Criteria for corner refinement
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# 3D points in real-world space
objp = np.zeros((checkerboard_size[0] * checkerboard_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:checkerboard_size[0], 0:checkerboard_size[1]].T.reshape(-1, 2)

# Arrays to store object points and image points
objpoints = []  # 3D points in real world space
imgpoints = []  # 2D points in image plane

# Load all calibration images from the folder 'img'
images = glob.glob('cali_pics/*.jpg')
for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Find the checkerboard corners
    ret, corners = cv2.findChessboardCorners(gray, checkerboard_size, None)

    if ret:
        print(f"Checkerboard detected in {fname}")
        objpoints.append(objp)

        # Refine corner locations to sub-pixel accuracy
        corners_refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners_refined)

        # Draw and display the corners
        cv2.drawChessboardCorners(img, checkerboard_size, corners_refined, ret)
        cv2.imshow('img', img)
        cv2.waitKey(500)
    else:
        print(f"No checkerboard detected in {fname}")

cv2.destroyAllWindows()

# Perform camera calibration
ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

# Save the calibration results
np.savez('calibration_params.npz', camera_matrix=camera_matrix, dist_coeffs=dist_coeffs)

print("Camera matrix: \n", camera_matrix)
print("Distortion coefficients: \n", dist_coeffs)
