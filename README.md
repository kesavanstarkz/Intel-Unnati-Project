# Develop a 2D Occupancy Grid Map of a Room using Overhead Cameras

## Step:1 Setting up the ROS2 environment :

  1. Ubuntu 20.04
  2. Install ROS2 foxy, packages – [3.1.2-3.1.4](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/)
  3. Check `“ros2 topic list”`
  4. export TURTLEBOT3_MODEL=waffle_pi
  5. Steps to create ros2 workspace
     * “mkdir –p ~/turtlebot3_ws/src”
     * Install turtlebot3 packages – [6.1.1,6.1.2](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/)
     * Run the house simulation
  6. Check `“ros2 topic list”`
  7. Locations Overhead cameras [C1: (-5,-2,8) C2:(-5,3,8) C3:(1,-2,8) C4:(1,3,8)]

## Step-2 Adding Overhead Cameras :

  1. Download the infraCam.zip
  2. Extract the folder  `“turtlebot3_camera_house”` and place in the  `“……/turtlebot3_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models”` directory.
  3. Replace the waffle_pi.model in `“…../turtlebot3_ws/src/turtlebot3_simulations/turtlebot3_gazebo/worlds/turtlebot3_houses/”` with the downloaded version.
  4. Change path in lines 6,11,16,21
  5. Source and build the workspace - `"$ source install/setup.bash"`.
  6. Run house simulation: `“ros2 launch turtlebot3_gazebo  turtlebot3_house.launch.py”`
  7. Topics related overhead camera are now visible.

## Step-3 Run simulation and run `multi_runner.py`

`multi_runner.py` executes  `image_listener.py` `image_horizontal_stitcher.py` `image_vertical_stitcher.py` `how_many_image.py` in parallel.

1. `image_listener.py` -- Fetches images from a Gazebo simulation using Python code and stores the images in the `saved_images` folder.
2. `image_horizontal_stitcher.py`-- Stitches images horizontal and stores the stitched image in the `saved_images/{}.png`.
3. `image_vertical_stitcher.py` -- Stitches images vertical and stores the stitched image in the `saved_images/output/{}.png`.
4. `how_many_image.py` -- Helps users determine how many stitched images can be created within a second.

## Step-4 Detect Objects and generate occupancy grid file in .pgm format with a 512x512 resolution.

Implement Object Detection:
    
Use a custom (e.g., YOLO, SSD) to detect objects and update the occupancy grid.
Use `.py` file or `.ipynb` file 

**Note:** To use `intel_object_detection_threshold.py`, we need to make 
\## scripts.append("intel_object_detection_threshold.py") ## For image stitching + (Object detection + Occupancy grid + .pgm) 
executable in `multi_runner.py`. It automatically runs and generates occupancy grid maps for us.
Or else use `intel_object_detection_threshold.ipynb` or run this `intel_object_detection_threshold.py` seperately.

## Step-5 How to evaluate the generated map?

* Run command below (replace path in yaml file)

 ` $ ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=$HOME/map_house.yaml`
