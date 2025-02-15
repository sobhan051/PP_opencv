import cv2

# URL of the video stream from your IP webcam
url = "http://192.168.1.3:8080/video"

# Create a VideoCapture object
cap = cv2.VideoCapture(url)

# Check if the stream is opened successfully
if not cap.isOpened():
    print("Error: Could not open video stream.")
    exit()

# Initialize variables
image_counter = 0  # Counter for naming saved images
max_images = 100  # Maximum number of images to save

# Read and display frames from the stream
while True:
    ret, frame = cap.read()  # Read a frame
    if not ret:
        print("Error: Failed to capture frame.")
        break

    # Display the frame
    cv2.imshow("IP Webcam Stream", frame)

    # Wait for a key press (1 ms delay)
    key = cv2.waitKey(1) & 0xFF

    # If a key is pressed and the image counter is less than 60, save the frame
    if key != 255 and image_counter < max_images:  # 255 means no key pressed
        image_name = f"frame_{image_counter}.jpg"
        cv2.imwrite(image_name, frame)
        print(f"Saved {image_name}")
        image_counter += 1

    # Exit if 'q' is pressed or the image limit is reached
    if key == ord('q') or image_counter >= max_images:
        break

# Release the VideoCapture object and close windows
cap.release()
cv2.destroyAllWindows()

# Notify when the image limit is reached
if image_counter >= max_images:
    print("Reached the limit of 60 images.")