import cv2
import numpy as np
import cv2.aruco as aruco
import socket
import time
import math
import json
from threading import Thread

# Configuration
WIFI_IP = "192.168.4.1"  # ESP32's IP
WIFI_PORT = 8080
CAMERA_ID = 0
MARKER_SIZE_CM = 5
OBJECT_TO_TILE = {1:4, 2:5, 3:6}  # Object marker → Tile marker mapping
ROBOT_MARKER_ID = 0

# Constants
MOVE_TOLERANCE_CM = 2
ROT_TOLERANCE_DEG = 5

class ArUcoDetector:
    def __init__(self):
        self.cap = cv2.VideoCapture(CAMERA_ID)
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.parameters = aruco.DetectorParameters()
        self.detector = aruco.ArucoDetector(self.aruco_dict, self.parameters)
        
        # Camera calibration (update with your values)
        self.cam_matrix = np.array([[800, 0, 320], [0, 800, 240], [0, 0, 1]])
        self.dist_coeffs = np.zeros(4)

    def get_positions(self):
        ret, frame = self.cap.read()
        if not ret: return None
        
        corners, ids, _ = self.detector.detectMarkers(frame)
        positions = {"robot": None, "objects": {}, "tiles": {}}
        
        if ids is not None:
            for i in range(len(ids)):
                marker_id = ids[i][0]
                rvec, tvec, _ = aruco.estimatePoseSingleMarkers(
                    corners[i], MARKER_SIZE_CM, self.cam_matrix, self.dist_coeffs)
                
                # Get position in cm (X,Y) from camera's top-down view
                x = tvec[0][0][0]
                y = tvec[0][0][1]
                angle = math.degrees(math.atan2(y, x))  # Simplified orientation

                if marker_id == ROBOT_MARKER_ID:
                    positions["robot"] = (x, y, angle)
                elif marker_id in OBJECT_TO_TILE.values():
                    positions["tiles"][marker_id] = (x, y)
                else:
                    positions["objects"][marker_id] = (x, y)
        
        return positions

class TaskScheduler:
    @staticmethod
    def get_best_task(robot_state, objects, tiles, object_to_tile_map):
        min_cost = float('inf')
        best_task = None
        
        for obj_id, obj_pos in objects.items():
            if obj_id not in object_to_tile_map:
                continue
                
            tile_id = object_to_tile_map[obj_id]
            tile_pos = tiles.get(tile_id)
            if not tile_pos:
                continue

            # Calculate movement costs
            move_to_obj = math.hypot(robot_state[0]-obj_pos[0], robot_state[1]-obj_pos[1])
            move_to_tile = math.hypot(obj_pos[0]-tile_pos[0], obj_pos[1]-tile_pos[1])
            
            # Calculate rotation costs
            angle_to_obj = math.degrees(math.atan2(obj_pos[1]-robot_state[1], 
                                                 obj_pos[0]-robot_state[0]))
            rot_cost_obj = abs(angle_to_obj - robot_state[2]) * 0.02
            
            angle_to_tile = math.degrees(math.atan2(tile_pos[1]-obj_pos[1],
                                                  tile_pos[0]-obj_pos[0]))
            rot_cost_tile = abs(angle_to_tile) * 0.02  # Assume facing obj after pickup
            
            total_cost = (move_to_obj + move_to_tile) * 0.1 + rot_cost_obj + rot_cost_tile
            
            if total_cost < min_cost:
                min_cost = total_cost
                best_task = (obj_id, obj_pos, tile_pos)
        
        return best_task

class RobotController:
    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((WIFI_IP, WIFI_PORT))
        self.current_angle = 0
        self.current_pos = (0, 0)

    def send_command(self, cmd_type, **kwargs):
        command = {"type": cmd_type, **kwargs}
        self.sock.send(json.dumps(command).encode() + b'\n')

    def rotate_to(self, target_angle):
        angle_diff = (target_angle - self.current_angle) % 360
        if angle_diff > 180:
            angle_diff -= 360
        self.send_command("rotate", angle=angle_diff, speed=80)
        
    def move_to(self, target_pos):
        dx = target_pos[0] - self.current_pos[0]
        dy = target_pos[1] - self.current_pos[1]
        distance = math.hypot(dx, dy)
        self.send_command("move", distance=distance, speed=100)

    def pickup(self):
        self.send_command("gripper", action="pickup")

    def release(self):
        self.send_command("gripper", action="release")

def main():
    detector = ArUcoDetector()
    controller = RobotController()
    placed_objects = set()

    while len(placed_objects) < 3:
        positions = detector.get_positions()
        if not positions or not positions["robot"]:
            continue

        robot_state = positions["robot"]
        controller.current_pos = (robot_state[0], robot_state[1])
        controller.current_angle = robot_state[2]

        # Find best task
        task = TaskScheduler.get_best_task(
            robot_state,
            positions["objects"],
            positions["tiles"],
            OBJECT_TO_TILE
        )

        if not task:
            continue

        obj_id, obj_pos, tile_pos = task
        
        # Move to object
        while math.hypot(robot_state[0]-obj_pos[0], robot_state[1]-obj_pos[1]) > MOVE_TOLERANCE_CM:
            controller.rotate_to(math.degrees(math.atan2(
                obj_pos[1]-robot_state[1], 
                obj_pos[0]-robot_state[0]
            )))
            controller.move_to(obj_pos)
            time.sleep(0.5)
            positions = detector.get_positions()
            robot_state = positions["robot"]

        # Pickup
        controller.pickup()
        time.sleep(2)  # Allow pickup time

        # Move to tile
        while math.hypot(robot_state[0]-tile_pos[0], robot_state[1]-tile_pos[1]) > MOVE_TOLERANCE_CM:
            controller.rotate_to(math.degrees(math.atan2(
                tile_pos[1]-robot_state[1], 
                tile_pos[0]-robot_state[0]
            )))
            controller.move_to(tile_pos)
            time.sleep(0.5)
            positions = detector.get_positions()
            robot_state = positions["robot"]

        # Release
        controller.release()
        placed_objects.add(obj_id)
        time.sleep(1)

    print("All objects placed!")

if __name__ == "__main__":
    main()