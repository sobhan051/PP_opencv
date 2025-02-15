# This Code finds all tags and check each pair of tags from tag_mapping
# Calculates distance and angle from robot to object and from robot to targer(tile)
# Switched tags from object to tagert after distance < 0.08 
# After reaching the target removes the pair of tag and do the same for other pairs
# latest version 1.0

import cv2
import numpy as np

# Load camera calibration parameters
calibration_data = np.load("cali_params/calibration_params.npz")
camera_matrix = calibration_data["camera_matrix"]
dist_coeffs = calibration_data["dist_coeffs"]

# Tag configuration
TAG_MAPPING = {1: 4,3: 5}  # Object: Destination mapping
ROBOT_ID = 0
TAG_SIZE = 0.033  # Physical tag size in meters

# ArUco setup
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
aruco_params = cv2.aruco.DetectorParameters()
aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

# State variables
picked_up = False  # Whether the robot has picked up the object
current_object = None  # Current object being processed
current_target = None  # Current target being processed

def get_2d_angle(car_corners, obj_center):
    """
    Calculate angle using 2D image coordinates.
    """
    # Ensure car_corners is in the correct shape (4, 2)
    car_corners = car_corners.reshape(-1, 2)

    # Get car's forward direction from corners (bottom-left to top-left)
    bottom_left = car_corners[3]  # Bottom-left corner
    top_left = car_corners[0]  # Top-left corner
    car_forward = top_left - bottom_left  # Vector from bottom-left to top-left

    # Get direction to object
    car_center = np.mean(car_corners, axis=0)  # Center of the car tag
    direction = obj_center - car_center  # Vector from car center to object center

    # Normalize vectors
    car_forward_norm = car_forward / np.linalg.norm(car_forward)
    direction_norm = direction / np.linalg.norm(direction)

    # Calculate signed angle
    dot = np.dot(car_forward_norm, direction_norm)
    det = car_forward_norm[0] * direction_norm[1] - car_forward_norm[1] * direction_norm[0]
    return np.degrees(np.arctan2(det, dot))

def get_3d_distance(robot_tvec, target_tvec):
    """
    Calculate real-world distance using 3D coordinates.
    """
    return np.linalg.norm(target_tvec - robot_tvec)

def process_tags(frame, tags):
    """
    Main processing logic.
    """
    global picked_up, current_object, current_target, TAG_MAPPING

    # Find robot
    if ROBOT_ID not in tags:
        cv2.putText(frame, "Robot not found", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        return

    robot = tags[ROBOT_ID]

    # If no object is being processed, start with the first pair
    if current_object is None and TAG_MAPPING:
        current_object = list(TAG_MAPPING.keys())[0]
        current_target = TAG_MAPPING[current_object]

    # If an object is being processed
    if current_object is not None:
        if not picked_up:
            # Move to the object
            if current_object not in tags:
                cv2.putText(frame, f"Object {current_object} not found", (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                return

            obj = tags[current_object]

            # Get 2D angle and 3D distance to the object
            angle = get_2d_angle(robot['corners'], obj['center_2d'])
            distance = get_3d_distance(robot['tvec'], obj['tvec'])

            # Display information
            cv2.putText(frame, f"Object {current_object}: {angle:.1f}deg, {distance:.3f}m",
                        (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            if distance > 0.08:
                # SEND COMMAND
                print("MOVE FORWARD")
            else:
                # SEND COMMAND
                print("STOP, PICKUP ITEM")
                picked_up = True  # Mark the object as picked up

            # Draw guidance arrow to the object
            cv2.arrowedLine(frame, tuple(robot['center_2d'].astype(int)),
                            tuple(obj['center_2d'].astype(int)), (0, 0, 255), 2)
        else:
            # Move to the target
            if current_target not in tags:
                cv2.putText(frame, f"Target {current_target} not found", (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                return

            target = tags[current_target]

            # Get 2D angle and 3D distance to the target
            angle = get_2d_angle(robot['corners'], target['center_2d'])
            distance = get_3d_distance(robot['tvec'], target['tvec'])

            # Display information
            cv2.putText(frame, f"Target {current_target}: {angle:.1f}deg, {distance:.3f}m",
                        (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            if distance > 0.08:
                print("MOVE FORWARD")
            else:
                print("PLACE ITEM")
                # Remove the completed pair from the mapping
                del TAG_MAPPING[current_object]
                # Reset state for the next pair
                picked_up = False
                current_object = None
                current_target = None

            # Draw guidance arrow to the target
            cv2.arrowedLine(frame, tuple(robot['center_2d'].astype(int)),
                            tuple(target['center_2d'].astype(int)), (0, 255, 0), 2)

    # Draw robot pose axes
    cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs,
                      robot['rvec'], robot['tvec'], TAG_SIZE / 2)

# Initialize video capture
cap = cv2.VideoCapture("http://192.168.1.3:8080/video")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Detect markers
    corners, ids, _ = aruco_detector.detectMarkers(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
    if ids is None:
        cv2.imshow("Control", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        continue

    # Process detected tags
    tags = {}
    for i, tag_id in enumerate(ids.flatten()):
        # Get 3D pose
        corners_3d = np.array([[-TAG_SIZE / 2, TAG_SIZE / 2, 0],
                               [TAG_SIZE / 2, TAG_SIZE / 2, 0],
                               [TAG_SIZE / 2, -TAG_SIZE / 2, 0],
                               [-TAG_SIZE / 2, -TAG_SIZE / 2, 0]], dtype=np.float32)
        success, rvec, tvec = cv2.solvePnP(corners_3d, corners[i][0],
                                           camera_matrix, dist_coeffs)
        if not success:
            continue

        # Store tag data
        tags[tag_id] = {
            'rvec': rvec,
            'tvec': tvec,
            'corners': corners[i][0],  # Store the corners of the tag
            'center_2d': np.mean(corners[i][0], axis=0)  # 2D center of the tag
        }

    # Run main processing
    process_tags(frame, tags)

    # Show result
    cv2.imshow("Control", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()