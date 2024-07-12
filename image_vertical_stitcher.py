from PIL import Image
import numpy as np

# Load the images
image_path_1 = "saved_images/output/stitched_image_2.png"
image_path_2 = "saved_images/output/stitched_image_1.png"

image1 = Image.open(image_path_1)
image2 = Image.open(image_path_2)

# Convert images to numpy arrays
image1_array = np.array(image1)
image2_array = np.array(image2)

# Define the overlap calculation method for vertical stitching
def find_best_row(image1_array, image2_array):
    height, width, _ = image1_array.shape
    min_diff = float('inf')
    best_row = 0
    overlap_start = 0
    
    for overlap in range(height // 4, height):  # Start from 1/4th of the height for finer adjustment
        diff = np.sum(np.abs(image1_array[height - overlap:, :, :] - image2_array[:overlap, :, :]))
        if diff < min_diff:
            min_diff = diff
            best_row = height - overlap
            overlap_start = overlap

    return best_row, overlap_start

# Calculate the best row to stitch the images vertically
best_row, overlap_start = find_best_row(image1_array, image2_array)

# Create a new blank image to hold the stitched result
stitched_image = Image.new('RGB', (image1_array.shape[1], image1_array.shape[0] + image2_array.shape[0] - overlap_start))

# Paste the first image onto the stitched image
stitched_image.paste(image1, (0, 0))

# Paste the second image onto the stitched image at the calculated position
stitched_image.paste(image2, (0, best_row))

# Save and display the stitched image
stitched_image_path = "saved_images/output/stitched_image_output.png"
stitched_image.save(stitched_image_path)
stitched_image.show()
