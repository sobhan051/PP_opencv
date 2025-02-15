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
    [-tag_size / 2,  tag_size / 2, 0],  # Top-left
    [ tag_size / 2,  tag_size / 2, 0],  # Top-right
    [ tag_size / 2, -tag_size / 2, 0],  # Bottom-right
    [-tag_size / 2, -tag_size / 2, 0]   # Bottom-left
], dtype=np.float32)

# Initialize video capture
cap = cv2.VideoCapture("http://192.168.1.4:8080/video")  # Use 0 for default camera

# Define criteria for subpixel refinement
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

def calculate_cardinal_direction(rvec):
    """
    Calculate the car's cardinal direction based on the Y-axis of the tag.
    """
    # Convert rotation vector to rotation matrix
    rotation_matrix, _ = cv2.Rodrigues(rvec)

    # Extract the Y-axis (green) of the tag
    y_axis = -rotation_matrix[:, 1]

    # Calculate the angle of the Y-axis in the X-Y plane
    angle_rad = np.arctan2(y_axis[0], y_axis[1])  # Use X and Y components
    angle_deg = np.degrees(angle_rad)

    return angle_deg

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

        # Extract corners of the first detected marker
        marker_corners = corners[0]  # Shape: (1, 4, 2)

        # Reshape corners to (N, 1, 2) for cv2.cornerSubPix
        marker_corners = marker_corners.reshape(-1, 1, 2).astype(np.float32)

        # Refine the detected corners
        refined_corners = cv2.cornerSubPix(gray, marker_corners, (5, 5), (-1, -1), criteria)

        # Estimate pose for the detected marker
        success, rvec, tvec = cv2.solvePnP(tag_corners_3d, refined_corners, camera_matrix, dist_coeffs)

        if success:
            # Calculate the angle of the Y-axis
            angle_deg = calculate_cardinal_direction(rvec)

            # Display the angle on the frame
            cv2.putText(frame, f"Angle: {angle_deg:.1f} deg", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            # Draw the pose (axis) on the frame
            cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, tag_size / 2)
        else:
            print("Pose estimation failed.")

    # Display the frame
    cv2.imshow("Car Cardinal Direction", frame)

    # Exit on 'q' key press
    if cv2.waitKey(100) & 0xFF == ord('q'):
        break

# Release resources
cap.release()
cv2.destroyAllWindows()