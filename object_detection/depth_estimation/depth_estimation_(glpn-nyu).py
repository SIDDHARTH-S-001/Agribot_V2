## Runs at 1 FPS on Logitech C270 Webcam

import cv2
from transformers import GLPNImageProcessor, GLPNForDepthEstimation
import torch
import numpy as np
from PIL import Image

# Load the GLPN model
processor = GLPNImageProcessor.from_pretrained("vinvino02/glpn-nyu")
model = GLPNForDepthEstimation.from_pretrained("vinvino02/glpn-nyu")

# Function to preprocess the frame for the model
def preprocess_frame(frame):
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    image = Image.fromarray(frame_rgb)
    inputs = processor(images=image, return_tensors="pt")
    return inputs, frame_rgb.shape[:2]

# Function to visualize the depth prediction
def visualize_depth(frame, depth_output):
    output = depth_output.squeeze().cpu().numpy()
    formatted = (output * 255 / np.max(output)).astype("uint8")
    depth_image = cv2.cvtColor(formatted, cv2.COLOR_GRAY2BGR)
    combined_image = np.concatenate((frame, depth_image), axis=1)
    return combined_image

# Main function to process webcam input
# Main function to process webcam input
def process_webcam():
    cap = cv2.VideoCapture(0)  # Change the index if you have multiple webcams
    if not cap.isOpened():
        print("Error: Could not open webcam.")
        return

    while True:
        start_time = cv2.getTickCount()  # Start time of the current frame processing
        ret, frame = cap.read()
        if not ret:
            break

        # Preprocess frame for model input
        inputs, original_size = preprocess_frame(frame)

        # Perform depth estimation
        with torch.no_grad():
            outputs = model(**inputs)
            predicted_depth = outputs.predicted_depth

        # Interpolate to original size
        prediction = torch.nn.functional.interpolate(
            predicted_depth.unsqueeze(1),
            size=original_size,
            mode="bicubic",
            align_corners=False,
        )

        # Visualize the prediction
        combined_image = visualize_depth(frame, prediction)

        # Calculate FPS
        fps = cv2.getTickFrequency() / (cv2.getTickCount() - start_time)
        cv2.putText(combined_image, f"FPS: {int(fps)}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

        # Display the result
        cv2.imshow("Webcam Depth Estimation", combined_image)

        # Check for exit key
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

# Run the webcam processing
process_webcam()
