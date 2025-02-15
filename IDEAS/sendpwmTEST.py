import cv2
import numpy as np
import serial
import time

# ... (Rest of your imports and initializations) ...

# Load calibration parameters
calibration_data = np.load('cali_params/calibration_params.npz')
camera_matrix = calibration_data['camera_matrix']
dist_coeffs = calibration_data['dist_coeffs']

# Define the ArUco dictionary and parameters
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
aruco_params = cv2.aruco.DetectorParameters()
aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
# Define the ArUco marker size (in meters)
marker_size = 0.033  # 5 cm

arduino = serial.Serial('COM3', 9600, timeout=1)  # Replace 'COM3' with your Arduino's port
time.sleep(2)

# # Define the 3D corner points of the ArUco marker in the marker's coordinate system
# object_points = np.array([
#     [-marker_size / 2, marker_size / 2, 0],
#     [marker_size / 2, marker_size / 2, 0],
#     [marker_size / 2, -marker_size / 2, 0],
#     [-marker_size / 2, -marker_size / 2, 0]
# ], dtype=np.float32)



# def angleframe():
#     cap = cv2.VideoCapture("http://192.168.1.4:8080/video")
#     ret, frame = cap.read()
#     if not ret:
#         print("Failed to capture frame")
#         return None

#     corners, ids, rejected = aruco_detector.detectMarkers(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))

#     if ids is not None and len(corners) > 0:  # Check if markers were detected
#       try:
#         marker_corners = corners[0][0]
#         success, rvec, tvec = cv2.solvePnP(object_points, marker_corners, camera_matrix, dist_coeffs)
#         if success:
#             rotation_matrix, _ = cv2.Rodrigues(rvec)
#             y_angle = np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0])  # in radians
#             y_angle_deg = np.degrees(y_angle)
#             return y_angle_deg
#         else:
#             print("solvePnP failed")
#             return None
#       except Exception as e:
#             print(f"Error in angleframe: {e}")
#             return None  # Return None on error
#     else:
#         print("No ArUco markers detected.")
#         return None

def send_rotation_command(duration, direction):
    command = f"{direction}{duration}\n"
    arduino.write(command.encode())
    time.sleep(0.1)

send_rotation_command(100, "R")


# def calibrate_rotation(direction, min_angle_change=5):  # Add min_angle_change
#     test_durations = range(250, 1501, 250)
#     angle_changes = []

#     for duration in test_durations:
#         initial_angle = angleframe()
#         time.sleep(3)
#         if initial_angle is None:
#             print(f"Failed to get initial angle for duration {duration}")
#             continue

#         print(f"Testing {direction} rotation, duration: {duration} ms")
#         send_rotation_command(duration, direction)
#         time.sleep(duration / 1000 + 4)

#         new_angle = angleframe()
#         if new_angle is None:
#             print(f"Failed to get new angle for duration {duration}")
#             continue

#         angle_change = new_angle - initial_angle
#         angle_change = (angle_change + 180) % 360 - 180
#         if direction == 'L':
#            if angle_change > 0:
#               angle_change =  - (360- angle_change)
#         if direction == 'R':
#             if angle_change < 0:
#                 angle_change = 360+angle_change

#         # --- KEY CHANGE: Ignore small angle changes ---
#         if abs(angle_change) < min_angle_change:
#             print(f"Ignoring small angle change: {angle_change:.2f} degrees")
#             continue
#         # --------------------------------------------

#         angle_changes.append((duration, angle_change))
#         print(f"Duration: {duration} ms, Angle Change: {angle_change:.2f} degrees")
#         print("UNTAGLE THE WIRES - sleep")
#         time.sleep(15)

#     if angle_changes:
#         durations, changes = zip(*angle_changes)
#         durations = np.array(durations)
#         changes = np.array(changes)
#         slope, intercept = np.polyfit(durations, changes, 1)
#         print(f"Calibration Factor ({direction}): {slope:.4f} degrees/ms, Intercept: {intercept:.2f}")

#         # --- KEY CHANGE:  Plot for Visualization (Optional) ---
#         import matplotlib.pyplot as plt  # Import at the top if you uncomment this
#         plt.figure()
#         plt.scatter(durations, changes, label='Data Points')
#         plt.plot(durations, slope * durations + intercept, color='red', label=f'Fit: y={slope:.4f}x + {intercept:.2f}')
#         plt.xlabel('Duration (ms)')
#         plt.ylabel('Angle Change (degrees)')
#         plt.title(f'Calibration Curve ({direction})')
#         plt.legend()
#         plt.grid(True)
#         plt.show()
#         # -----------------------------------------------------
#         return slope, intercept
#     else:
#         print(f"Calibration failed for {direction}: No angle changes recorded.")
#         return None, None

# def main():
#     print("Calibrating RIGHT rotation...")
#     right_slope, right_intercept = calibrate_rotation('R')

#     print("\nCalibrating LEFT rotation...")
#     left_slope, left_intercept = calibrate_rotation('L')
#     if not all([right_slope,right_intercept,left_slope,left_intercept]):
#         print("Calibration did no complete succesfully. Please run again")
#         return
#     # --- KEY CHANGE: Store calibration data ---
#     calibration_data = {
#         'R': (right_slope, right_intercept),
#         'L': (left_slope, left_intercept)
#     }
#     # -----------------------------------------

#     print("\nCalibration complete.")
#     while True:
#         try:
#             target_angle = float(input("Enter target angle change (degrees, + for right, - for left): "))
#             if target_angle == 0:
#                 print("Target angle cannot be Zero")
#                 continue

#             direction = 'R' if target_angle > 0 else 'L'
#             target_angle = abs(target_angle)

#             # --- KEY CHANGE: Use stored calibration data ---
#             slope, intercept = calibration_data[direction]
#             # -----------------------------------------------

#             required_duration = int((target_angle - intercept) / slope)

#             # --- KEY CHANGE:  Handle potentially negative/small durations ---
#             if required_duration <= 0:
#                 print("Calculated duration is not positive, trying a slightly larger angle...")
#                 # Option 1:  Suggest a minimum angle (more user-friendly)
#                 min_target_angle = abs(intercept) + abs(slope) * 100 # Example: ensure at least 100ms
#                 print(f"Try an angle greater than {min_target_angle:.2f} degrees for {direction} rotation.")
#                 continue

#             print(f"Required Duration: {required_duration} ms")
#             send_rotation_command(required_duration, direction)
#             print("Rotation command sent.")

#         except ValueError:
#             print("Invalid input. Please enter a number.")
#         except KeyboardInterrupt:
#             print("\nExiting.")
#             break

# if __name__ == "__main__":
#     main()