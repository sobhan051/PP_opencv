
import cv2
import numpy as np

# Load camera calibration parameters
calibration_data = np.load("calibration_params.npz")
camera_matrix = calibration_data["camera_matrix"]
dist_coeffs = calibration_data["dist_coeffs"]

# Tag configuration
TAG_MAPPING = {1: 4, 3:5}  # Object: Destination mapping
ROBOT_ID = 0
TAG_SIZE = 0.033  # Physical tag size in meters

# ArUco setup
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
aruco_params = cv2.aruco.DetectorParameters()
aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)


def get_2d_angle(car_corners, obj_center):
    """
    Calculate angle using 2D image coordinates.
    """
    car_corners = car_corners.reshape(-1, 2)

    # Get car's forward direction from corners (bottom-left to top-left)
    bottom_left = car_corners[3]
    top_left = car_corners[0]
    car_forward = top_left - bottom_left

    # Get direction to object
    car_center = np.mean(car_corners, axis=0)
    direction = obj_center - car_center

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
    global TAG_MAPPING  # Allow modification of TAG_MAPPING

    if ROBOT_ID not in tags:
        cv2.putText(frame, "Robot not found", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        return

    robot = tags[ROBOT_ID]
    remaining_pairs = list(TAG_MAPPING.items())  # Create a list of pairs to iterate through

    for i in range (3):
        obj_id , target_id = remaining_pairs[0]
        if obj_id not in tags or target_id not in tags:
            continue

        obj = tags[obj_id]
        target = tags[target_id]

        # Get 2D angle and 3D distance for object
        angle_obj = get_2d_angle(robot['corners'], obj['center_2d'])
        distance_obj = get_3d_distance(robot['tvec'], obj['tvec'])

                # Draw guidance elements
        cv2.arrowedLine(frame, tuple(robot['center_2d'].astype(int)),
                    tuple(obj['center_2d'].astype(int)), (0, 0, 255), 2)
        # Display information for object
        cv2.putText(frame, f"Object {obj_id}: {angle_obj:.1f}deg, {distance_obj:.3f}m",
                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        if distance_obj > 0.08:
            print("MOVE FORWARD")
        else:
            print("STOP")

            # Switch to target
            angle_target = get_2d_angle(robot['corners'], target['center_2d'])
            distance_target = get_3d_distance(robot['tvec'], target['tvec'])
                    # Draw guidance elements
            cv2.arrowedLine(frame, tuple(robot['center_2d'].astype(int)),
                        tuple(target['center_2d'].astype(int)), (0, 0, 255), 2)
            # Display information for target
            cv2.putText(frame, f"Target {target_id}: {angle_target:.1f}deg, {distance_target:.3f}m",
                        (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)

            if distance_target < 0.09:
                print(f"Removing Object {obj_id} and Target {target_id}")
                del TAG_MAPPING[obj_id]  # Remove processed pair


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
    if TAG_MAPPING:  # Only process if there are mappings left
        process_tags(frame, tags)

    # Show result
    cv2.imshow("Control", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
