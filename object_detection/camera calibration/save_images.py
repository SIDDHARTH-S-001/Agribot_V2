import cv2
import os

# Create a folder named "images" if it doesn't exist
images_folder = "images"
if not os.path.exists(images_folder):
    os.makedirs(images_folder)

# Capture video from webcam
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Display the frame
    cv2.imshow('frame', frame)

    # Save images when 's' is pressed
    if cv2.waitKey(1) & 0xFF == ord('s'):
        image_name = os.path.join(images_folder, f"image_{cv2.getTickCount()}.png")
        cv2.imwrite(image_name, frame)
        print(f"Image saved: {image_name}")

    # Break the loop when 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the capture
cap.release()
cv2.destroyAllWindows()
