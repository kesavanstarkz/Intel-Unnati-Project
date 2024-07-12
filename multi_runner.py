import subprocess
import concurrent.futures

# List of Python scripts to execute
scripts = ["image_listener.py", "image_horizontal_stitcher.py", "image_vertical_stitcher.py","how_many_image.py"] ## For image stitching
## scripts.append("intel_object_detection_threshold.py") ## For image stitching + (Object detection + Occupancy grid + .pgm)

# Function to execute a script and return its output
def execute_script(script):
    try:
        result = subprocess.run(['python3', script], capture_output=True, text=True, check=True)
        return f"Output of {script}:\n{result.stdout}"
    except subprocess.CalledProcessError as e:
        return f"Error occurred while executing {script}:\n{e.stderr}"

# Execute each script in parallel
with concurrent.futures.ThreadPoolExecutor() as executor:
    futures = {executor.submit(execute_script, script): script for script in scripts}
    for future in concurrent.futures.as_completed(futures):
        script = futures[future]
        try:
            output = future.result()
            print(output)
        except Exception as e:
            print(f"Exception occurred while executing {script}: {e}")

