import cv2
import numpy as np

# Load camera calibration parameters
calibration_data = np.load("calibration_params.npz")
camera_matrix = calibration_data["camera_matrix"]
dist_coeffs = calibration_data["dist_coeffs"]

# Define the ArUco dictionary and parameters
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
aruco_params = cv2.aruco.DetectorParameters()
aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
# Define the size of the ArUco tag (in meters)
tag_size = 0.033  # Replace with the actual size of your ArUco tag

# Define the 3D corners of the ArUco tag in the tag's coordinate system
tag_corners_3d = np.array([
    [-tag_size / 2,  tag_size / 2, 0],
    [ tag_size / 2,  tag_size / 2, 0],
    [ tag_size / 2, -tag_size / 2, 0],
    [-tag_size / 2, -tag_size / 2, 0]
], dtype=np.float32)

# Initialize video capture (replace with your IP webcam URL if needed)
cap = cv2.VideoCapture("http://192.168.1.4:8080/video")  # Use 0 for default camera, or replace with your IP webcam URL

while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Failed to capture frame.")
        break

    # Convert the frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect ArUco markers
    corners, ids, rejected = aruco_detector.detectMarkers(gray)

    if ids is not None:
        # Draw detected markers on the frame
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        # Estimate pose for each detected marker
        for i in range(len(ids)):
            # Get the 2D corners of the detected marker
            tag_corners_2d = corners[i][0]

            # SolvePnP to estimate the pose
            success, rvec, tvec = cv2.solvePnP(tag_corners_3d, tag_corners_2d, camera_matrix, dist_coeffs)

            if success:
                # Draw the pose (axis) on the frame
                cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, tag_size / 2)

                # Print the rotation and translation vectors
                print(f"Marker ID: {ids[i]}")
                print(f"Rotation Vector (rvec):\n{rvec}")
                print(f"Translation Vector (tvec):\n{tvec}")

    # Display the frame
    cv2.imshow("Pose Estimation with ArUco", frame)

    # Exit on 'q' key press
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the video capture and close windows
cap.release()
cv2.destroyAllWindows()