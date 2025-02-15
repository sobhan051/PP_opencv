import cv2
import numpy as np

# Define the ArUco dictionary
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)

# Define the size of the ArUco tag (in pixels)
tag_size = 400

# Generate a single ArUco tag
def generate_aruco_tag(id, size):
    tag = np.zeros((size, size), dtype=np.uint8)
    cv2.aruco.generateImageMarker(aruco_dict, id, size, tag, 1)
    return tag

# Example: Generate tags with IDs 0 to 6
tags = [generate_aruco_tag(i, tag_size) for i in range(7)]

# Define A4 size in pixels (300 DPI)
a4_width = 2480
a4_height = 3508

# Create a blank white A4 canvas
a4_canvas = 255 * np.ones((a4_height, a4_width), dtype=np.uint8)

# Define spacing between tags
margin = 150
spacing = 300

# Calculate how many tags fit per row and column
tags_per_row = (a4_width - 2 * margin) // (tag_size + spacing)
tags_per_col = (a4_height - 2 * margin) // (tag_size + spacing)

# Paste tags onto the A4 canvas
for i, tag in enumerate(tags):
    row = i // tags_per_row
    col = i % tags_per_row
    x = margin + col * (tag_size + spacing)
    y = margin + row * (tag_size + spacing)
    a4_canvas[y:y+tag_size, x:x+tag_size] = tag


cv2.imwrite("aruco_tags_a4.png", a4_canvas)