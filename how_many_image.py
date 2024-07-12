import cv2
import numpy as np
import time

# Sample image processing function
def process_image(image):
    # Simulate some image processing (e.g., converting to grayscale)
    return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# Load a sample image
image_path = "saved_images/output/stitched_image_output.png"
image = cv2.imread(image_path)

# Number of images to save for the test
num_images = 100

# Start the timer
start_time = time.time()

# Save processed images
for i in range(num_images):
    processed_image = process_image(image)
    output_path = f"saved_images/output/time_calculation/processed_image_{i}.png"
    cv2.imwrite(output_path, processed_image)

# End the timer
end_time = time.time()

# Calculate elapsed time and images per second
elapsed_time = end_time - start_time
images_per_second = num_images / elapsed_time

print(f"Saved {num_images} images in {elapsed_time:.2f} seconds")
print(f"Images saved per second: {images_per_second:.2f}")
