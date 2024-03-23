import cv2
import torch
import numpy as np
from PIL import Image
from transformers import AutoImageProcessor, AutoModelForDepthEstimation
import time

# Load the depth estimation model and image processor
# Reference link https://huggingface.co/LiheYoung/depth-anything-small-hf
image_processor = AutoImageProcessor.from_pretrained("LiheYoung/depth-anything-small-hf")
model = AutoModelForDepthEstimation.from_pretrained("LiheYoung/depth-anything-small-hf")

# Start capturing video from webcam
cap = cv2.VideoCapture(0)

# Initialize variables for frame rate calculation
start_time = time.time()
frame_count = 0

while True:
    ret, frame = cap.read()  # Read a frame from the webcam
    frame_count += 1

    # Convert the OpenCV frame to PIL image
    frame_pil = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))

    # Prepare image for the model
    inputs = image_processor(images=frame_pil, return_tensors="pt")

    with torch.no_grad():
        outputs = model(**inputs)
        predicted_depth = outputs.predicted_depth

    # Interpolate to original size
    prediction = torch.nn.functional.interpolate(
        predicted_depth.unsqueeze(1),
        size=frame_pil.size[::-1],
        mode="bicubic",
        align_corners=False,
    )

    # Visualize the prediction
    output = prediction.squeeze().cpu().numpy()
    formatted = (output * 255 / np.max(output)).astype("uint8")
    depth = Image.fromarray(formatted)

    # Convert the depth image back to OpenCV format
    depth_cv2 = cv2.cvtColor(np.array(depth), cv2.COLOR_RGB2BGR)

    # Calculate frame rate
    elapsed_time = time.time() - start_time
    fps = frame_count / elapsed_time

    # Overlay frame rate onto the frame
    cv2.putText(frame, f"FPS: {fps:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

    # Display the original frame and the depth image side by side
    frame_with_depth = np.hstack((frame, depth_cv2))
    cv2.imshow('Frame with Depth', frame_with_depth)

    # Break the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the webcam and close the OpenCV windows
cap.release()
cv2.destroyAllWindows()
