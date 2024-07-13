# -*- coding: utf-8 -*-
"""intel_object_detection_threshold.ipynb

Original file is located at
    https://colab.research.google.com/drive/1XuxYcsjgo_omNu8jZ8WOTHyrHrHqSQia

## Object detection and Threshold/Occupancy maker..

### Object detection
"""

import torch
from pathlib import Path
import cv2
from google.colab import drive
import numpy as np
import time

model_path = 'best.pt'  # Model

# Check if the model file exists
if not Path(model_path).exists():
    raise FileNotFoundError(f"Model file not found at {model_path}")

# Load the model
model = torch.hub.load('ultralytics/yolov5', 'custom', path=model_path, force_reload=True)

# Load an image
img_path = 'saved_images/output/stitched_image_output.png'
if not Path(img_path).exists():
    raise FileNotFoundError(f"Image file not found at {img_path}")

# Perform inference / Prediction
results = model(img_path)


def render_without_labels(results):
    for i, (im, pred) in enumerate(zip(results.ims, results.pred)): # Use results.ims instead of results.imgs
        if pred is not None:
            for *box, conf, cls in reversed(pred):  # xyxy, confidence, class
                label = f'{results.names[int(cls)]} {conf:.2f}' if conf > 0 else f'{results.names[int(cls)]}'
                # Remove label and confidence display
                # Make below comment line executable to get object namings
                # cv2.putText(im, label, (int(box[0]), int(box[1]) - 2), 0, 0.6, [225, 255, 255], thickness=1, lineType=cv2.LINE_AA)
                cv2.rectangle(im, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])), (255, 0, 0), -1)
        yield im

displayed_image_np = list(render_without_labels(results))

# Save the results
for i, img_np in enumerate(displayed_image_np):
    output_image_path = f'output_image_{i}.png'
    cv2.imwrite(output_image_path, img_np)
    print(f"Displayed image saved locally as {output_image_path}")

"""### Normal png creator"""

# Function to apply thresholding to an image
def apply_threshold(image, threshold_value):
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    _, thresholded_image = cv2.threshold(gray_image, threshold_value, 255, cv2.THRESH_BINARY)
    return thresholded_image


# Define threshold value (adjust as needed)
threshold_value = 150

# Apply thresholding to each displayed image and save
for i, img_np in enumerate(displayed_image_np):
    thresholded_img = apply_threshold(img_np, threshold_value)
    output_image_path = f'output_image_{i}_thresholded.png'
    cv2.imwrite(output_image_path, thresholded_img)
    print(f"Thresholded image saved locally as {output_image_path}")

"""### Occcupancy grid maker with 512x512 resolution and with .pgm extension"""

# Function to apply thresholding to an image
def apply_threshold(image, threshold_value):
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    _, thresholded_image = cv2.threshold(gray_image, threshold_value, 255, cv2.THRESH_BINARY)
    return thresholded_image

# Function to resize an image to 512x512 resolution
def resize_image(image, width, height):
    return cv2.resize(image, (width, height), interpolation=cv2.INTER_AREA)

# Assuming you have `displayed_image_np` from your previous code
# displayed_image_np = []  # Placeholder for your list of images

# Define threshold value (adjust as needed)
threshold_value = 150

# Desired output resolution
output_width = 512
output_height = 512

# Apply thresholding, resize and save each displayed image
for i, img_np in enumerate(displayed_image_np):
    # Start timing
    start_time = time.time()

    # Apply thresholding
    thresholded_img = apply_threshold(img_np, threshold_value)

    # Resize to 512x512
    resized_img = resize_image(thresholded_img, output_width, output_height)

    # Save as .pgm file
    output_image_path = f'output_image_{i}_thresholded.pgm'
    cv2.imwrite(output_image_path, resized_img)

    # End timing
    end_time = time.time()

    # Calculate elapsed time
    elapsed_time = end_time - start_time
    print(f"Thresholded image saved locally as {output_image_path} in {elapsed_time:.4f} seconds")

print("All images have been processed and saved.")


