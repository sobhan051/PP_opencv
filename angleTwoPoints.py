import cv2
import numpy as np

# Define the ArUco dictionary and parameters
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
aruco_params = cv2.aruco.DetectorParameters()
aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

# Initialize video capture
cap = cv2.VideoCapture("http://192.168.1.3:8080/video")  # Use 0 for default camera

def find_tag_center(corners):
    """
    Find the center point of an ArUco tag.
    """
    # Reshape corners to (4, 2)
    corners = corners.reshape(4, 2)

    # Calculate the centroid (average of the four corners)
    center = np.mean(corners, axis=0).astype(int)
    return center

def calculate_angle_between_vectors(v1, v2):
    """
    Calculate the signed angle between two 2D vectors.
    Returns angle in degrees.
    """
    dot = v1[0] * v2[0] + v1[1] * v2[1]
    det = v1[0] * v2[1] - v1[1] * v2[0]
    angle_rad = np.arctan2(det, dot)
    return np.degrees(angle_rad)

while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Failed to capture frame.")
        break

    # Convert the frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect ArUco markers
    corners, ids, rejected = aruco_detector.detectMarkers(gray)

    if ids is not None and len(ids) >= 2:
        # Draw detected markers on the frame
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        # Assume the first two detected tags are the car and object
        car_id = 0  # Replace with your car's tag ID
        obj_id = 1  # Replace with your object's tag ID

        # Find indices of car and object tags
        car_indices = np.where(ids == car_id)[0]
        obj_indices = np.where(ids == obj_id)[0]

        if len(car_indices) == 0 or len(obj_indices) == 0:
            print("Car or object tag not detected.")
            continue

        # Get corners for car and object
        car_corners = corners[car_indices[0]][0]  # Shape: (4, 2)
        obj_corners = corners[obj_indices[0]][0]  # Shape: (4, 2)

        # Find the centers of the car and object tags
        car_center = find_tag_center(car_corners)
        obj_center = find_tag_center(obj_corners)

        # Define the car's forward direction (middle of top-left to top-right corner)
        top_left = car_corners[0]  # Top-left corner
        bottom_left = car_corners[3]  # Top-right corner
        car_forward =  top_left - bottom_left  # Vector from top-left to top-right

        # Normalize the car's forward vector
        car_forward_norm = car_forward / np.linalg.norm(car_forward)

        # Compute the direction from the car's center to the object's center
        direction_to_obj = obj_center - car_center
        direction_to_obj_norm = direction_to_obj / np.linalg.norm(direction_to_obj)

        # Calculate the angle between the car's forward direction and the direction to the object
        angle = calculate_angle_between_vectors(car_forward_norm, direction_to_obj_norm)

        # Display the angle on the frame
        cv2.putText(frame, f"Angle: {angle:.1f} deg", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        # Draw the car's forward direction and the line to the object
        cv2.arrowedLine(frame, tuple(car_center), tuple((car_center + car_forward).astype(int)),
                        (0, 255, 0), 2)  # Car's forward direction (green)
        cv2.arrowedLine(frame, tuple(car_center), tuple(obj_center),
                        (0, 0, 255), 2)  # Line to object (red)
    else:
        print("Less than 2 tags detected.")

    # Display the frame
    cv2.imshow("Angle Between Car and Object", frame)

    # Exit on 'q' key press
    if cv2.waitKey(100) & 0xFF == ord('q'):
        break

# Release resources
cap.release()
cv2.destroyAllWindows()