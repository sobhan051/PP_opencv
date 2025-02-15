# calbrate motors for rotating in only one diraction Right or Left and calculate the factor to calcualte the angles. 
# might be wrong logicly.
# important to get new frame cap=cv2.videocapture has to be in the fucntion
# V1.0


import cv2
import numpy as np
import serial
import time
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

# Define the 3D corner points of the ArUco marker in the marker's coordinate system
object_points = np.array([
    [-marker_size / 2, marker_size / 2, 0],
    [marker_size / 2, marker_size / 2, 0],
    [marker_size / 2, -marker_size / 2, 0],
    [-marker_size / 2, -marker_size / 2, 0]
], dtype=np.float32)



def angleframe():
    cap = cv2.VideoCapture("http://192.168.1.4:8080/video")
    ret, frame = cap.read()
    corners, ids, rejected = aruco_detector.detectMarkers(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))

    marker_corners = corners[0][0]
    success, rvec, tvec = cv2.solvePnP(object_points, marker_corners, camera_matrix, dist_coeffs)
    rotation_matrix, _ = cv2.Rodrigues(rvec)
    y_angle = np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0])  # in radians
    y_angle_deg = np.degrees(y_angle)
    if (y_angle_deg < 0):
        y_angle_deg+=360
    return y_angle_deg


def send_rotation_command(duration, direction):
    command = f"{direction}{duration}\n"
    arduino.write(command.encode())
    time.sleep(0.1)
durationR = 50
durationL = 50
while (durationR <= 1000): #change the time needed just to make 180 deg turn 
    for i in range(2):
        angle1 = angleframe()
        send_rotation_command(durationR, "R")
        time.sleep(durationR/1000 + 1)
        angle2 = angleframe()
        angleR = []
        angleDiffer = angle2-angle1
        if (angleDiffer > 180): 
            print(f"Duration Right is {durationR} don't make it higher")
            break
        angleR.append(angleDiffer)
    print(f"Right angle {sum(angleR) / len(angleR)} for this duration: {durationR}")
    durationR += 50
    
while (durationL <= 1000): #change the time needed just to make 180 deg turn 
    for i in range(2):
        angle1 = angleframe()
        send_rotation_command(durationL, "L")
        time.sleep(durationL/1000 + 1)
        angle2 = angleframe()
        angleL = []
        angleDiffer = angle2-angle1
        if (angleDiffer > 180): 
            print(f"Duration Left is {durationL} don't make it higher")
            break
        angleL.append(angleDiffer)
    print(f"Right angle {sum(angleL) / len(angleL)} for this duration: {durationL}")
    durationL += 50

# def send_rotation_command(duration):
#     # Send the rotation duration to the Arduino
#     arduino.write(f"{duration}\n".encode())
#     time.sleep(5)



# def calibrate_rotation():
#     # Test durations in milliseconds
#     test_durations = range(250, 2000, 250)  # 500 ms to 3000 ms in steps of 500 ms
#     angle_changes = []

#     for duration in test_durations:
#         # Get the initial angle

#         initial_angle = angleframe()
#         time.sleep(10)
#         if initial_angle is not None:
#             # Send the rotation command
#             send_rotation_command(duration)
#             print(f"Testing duration: {duration} ms")

#             # Wait for the rotation to complete
#             time.sleep(duration / 1000 + 1)  # Add a small buffer time

#             # Get the new angle
#             new_angle = angleframe()

#             if new_angle is not None:
#                 # Calculate the angle change
#                 angle_change = abs(new_angle - initial_angle)
#                 angle_changes.append(angle_change)
#                 print(f"Duration: {duration} ms, Angle Change: {angle_change:.2f} degrees")
#         print("UNTAGLE THE WIRES sleep")
#         time.sleep(15)

#     # Calculate the calibration factor (degrees per millisecond)
#     if angle_changes:
#         calibration_factor = np.mean(np.array(angle_changes) / np.array(test_durations))
#         print(f"Calibration Factor: {calibration_factor:.4f} degrees/ms")
#         return calibration_factor
#     else:
#         print("Calibration failed: No angle changes recorded.")
#         return None

# def main():
#     # Perform calibration
#     calibration_factor = calibrate_rotation()

#     if calibration_factor is not None:
#         print("Calibration complete. Enter a target angle change (in degrees) to rotate the robot.")
#         while True:
#             try:
#                 target_angle = float(input("Enter target angle change (degrees): "))
#                 if target_angle <= 0:
#                     print("Target angle must be positive.")
#                     continue

#                 # Calculate the required duration
#                 required_duration = int(target_angle / calibration_factor)
#                 print(f"Required Duration: {required_duration} ms")

#                 # Send the rotation command
#                 send_rotation_command(required_duration)
#                 print("Rotation command sent.")

#             except ValueError:
#                 print("Invalid input. Please enter a number.")




# if __name__ == "__main__":
#     main()


