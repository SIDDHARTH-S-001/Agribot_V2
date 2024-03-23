## Runs at 0.03 FPS on Logitech C270 Webcam

import cv2
from transformers import DPTImageProcessor, DPTForDepthEstimation
import torch
import numpy as np
from PIL import Image

# Load the model
processor = DPTImageProcessor.from_pretrained("Intel/dpt-swinv2-tiny-256")
model = DPTForDepthEstimation.from_pretrained("Intel/dpt-swinv2-tiny-256")

# Open webcam
cap = cv2.VideoCapture(0)

# Check if the webcam is opened correctly
if not cap.isOpened():
    print("Error: Could not open webcam.")
    exit()

# Main loop
while True:
    # Capture frame-by-frame
    ret, frame = cap.read()

    if not ret:
        print("Error: Couldn't read frame.")
        break

    # Convert frame to PIL Image
    image = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))

    # Prepare image for the model
    inputs = processor(images=image, return_tensors="pt")

    with torch.no_grad():
        outputs = model(**inputs)
        predicted_depth = outputs.predicted_depth

    # Interpolate to original size
    prediction = torch.nn.functional.interpolate(
        predicted_depth.unsqueeze(1),
        size=image.size[::-1],
        mode="bicubic",
        align_corners=False,
    )

    # Visualize the prediction
    output = prediction.squeeze().cpu().numpy()
    formatted = (output * 255 / np.max(output)).astype("uint8")
    depth = Image.fromarray(formatted)

    # Convert the depth image to OpenCV format for display
    depth_cv2 = cv2.cvtColor(np.array(depth), cv2.COLOR_RGB2BGR)

    # Display FPS
    cv2.putText(frame, 'FPS: {:.2f}'.format(1 / cap.get(cv2.CAP_PROP_FPS)), (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

    # Display the frame and the depth image
    cv2.imshow('frame', frame)
    cv2.imshow('depth', depth_cv2)

    # Break the loop when 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the capture
cap.release()
cv2.destroyAllWindows()
