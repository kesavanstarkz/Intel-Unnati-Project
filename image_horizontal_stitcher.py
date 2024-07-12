from PIL import Image
import numpy as np

# Load the images
image_path_1 = "saved_images/camera_2_latest.png"
image_path_2 = "saved_images/camera_1_latest.png"

image_path_3 = "saved_images/camera_4_latest.png"
image_path_4 = "saved_images/camera_3_latest.png"

def function_for_2_1():

    image1 = Image.open(image_path_1)
    image2 = Image.open(image_path_2)

    # Convert images to numpy arrays
    image1_array = np.array(image1)
    image2_array = np.array(image2)

    # Define the overlap calculation method
    def find_best_column(image1_array, image2_array):
        height, width, _ = image1_array.shape
        min_diff = float('inf')
        best_column = 0
        overlap_start = 0
        
        for overlap in range(width // 2, width):
            diff = np.sum(np.abs(image1_array[:, width - overlap:, :] - image2_array[:, :overlap, :]))
            if diff < min_diff:
                min_diff = diff
                best_column = width - overlap
                overlap_start = overlap

        return best_column, overlap_start

    # Calculate the best column to stitch the images
    best_column, overlap_start = find_best_column(image1_array, image2_array)

    # Create a new blank image to hold the stitched result
    stitched_image = Image.new('RGB', (image1_array.shape[1] + image2_array.shape[1] - overlap_start, image1_array.shape[0]))

    # Paste the first image onto the stitched image
    stitched_image.paste(image1, (0, 0))

    # Paste the second image onto the stitched image at the calculated position
    stitched_image.paste(image2, (best_column, 0))

    # Save and display the stitched image
    stitched_image_path = "saved_images/output/stitched_image_1.png"
    stitched_image.save(stitched_image_path)
    stitched_image.show()


def function_for_4_3():

    image1 = Image.open(image_path_3)
    image2 = Image.open(image_path_4)

    # Convert images to numpy arrays
    image1_array = np.array(image1)
    image2_array = np.array(image2)

    # Define the overlap calculation method
    def find_best_column(image1_array, image2_array):
        height, width, _ = image1_array.shape
        min_diff = float('inf')
        best_column = 0
        overlap_start = 0
        
        for overlap in range(width // 2, width):
            diff = np.sum(np.abs(image1_array[:, width - overlap:, :] - image2_array[:, :overlap, :]))
            if diff < min_diff:
                min_diff = diff
                best_column = width - overlap
                overlap_start = overlap

        return best_column, overlap_start

    # Calculate the best column to stitch the images
    best_column, overlap_start = find_best_column(image1_array, image2_array)

    # Create a new blank image to hold the stitched result
    stitched_image = Image.new('RGB', (image1_array.shape[1] + image2_array.shape[1] - overlap_start, image1_array.shape[0]))

    # Paste the first image onto the stitched image
    stitched_image.paste(image1, (0, 0))

    # Paste the second image onto the stitched image at the calculated position
    stitched_image.paste(image2, (best_column, 0))

    # Save and display the stitched image
    stitched_image_path = "saved_images/output/stitched_image_2.png"
    stitched_image.save(stitched_image_path)
    stitched_image.show()


function_for_2_1()
function_for_4_3()