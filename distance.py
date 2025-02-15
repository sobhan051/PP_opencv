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
tag_size = 0.033    # Replace with the actual size of your ArUco tag

# Define the 3D corners of the ArUco tag in the tag's coordinate system
tag_corners_3d = np.array([
    [-tag_size / 2,  tag_size / 2, 0],
    [ tag_size / 2,  tag_size / 2, 0],
    [ tag_size / 2, -tag_size / 2, 0],
    [-tag_size / 2, -tag_size / 2, 0]
], dtype=np.float32)

# Initialize video capture (replace with your IP webcam URL if needed)
cap = cv2.VideoCapture("http://192.168.1.4:8080/video")  # Use 0 for default camera, or replace with your IP webcam URL

def calculate_distance(tvec1, tvec2):
    """
    Calculate the Euclidean distance between two translation vectors.
    """
    return np.linalg.norm(tvec1 - tvec2)

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

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
        for corner in corners:
            cv2.cornerSubPix(gray, corner, (5, 5), (-1, -1), criteria)
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        # Dictionary to store translation vectors for each detected marker
        tvecs = {}

        # Estimate pose for each detected marker
        for i in range(len(ids)):
            # Get the 2D corners of the detected marker
            tag_corners_2d = corners[i][0]

            # SolvePnP to estimate the pose
            success, rvec, tvec = cv2.solvePnP(tag_corners_3d, tag_corners_2d, camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_ITERATIVE)

            if success:
                # Store the translation vector
                tvecs[ids[i][0]] = tvec.flatten()

                # Draw the pose (axis) on the frame
                cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, tag_size / 2)

        # Calculate distance between two specific ArUco tags (e.g., IDs 0 and 1)
        if len(tvecs) >= 2:
            tag_id1, tag_id2 = 6, 0  # Replace with the IDs of the tags you want to measure
            if tag_id1 in tvecs and tag_id2 in tvecs:
                distance = calculate_distance(tvecs[tag_id1], tvecs[tag_id2])
                print(f"Distance between tag {tag_id1} and tag {tag_id2}: {distance:.2f} meters")

                # Display the distance on the frame
                cv2.putText(frame, f"Distance: {distance:.2f} m", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    # Display the frame
    cv2.imshow("Distance Between ArUco Tags", frame)

    # Exit on 'q' key press
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the video capture and close windows
cap.release()
cv2.destroyAllWindows()