# rl24_hw3_Imbimbo
Ferdinando Dionisio, Vittorio Lisi, Giovanni Gabriele Imbimbo, Emanuele Cifelli

## Overview

This guide provides updated instructions for working with the robotics package, focusing on Gazebo simulation with velocity controllers, positioning tasks, OpenCV integration, and ArUco marker tracking.

---

## Prerequisites

Ensure that the repository has been cloned and the workspace is built:
```bash
git clone https://github.com/GioGab78/rl24_hw3_Imbimbo.git
colcon build
source install/setup.bash
```

## Remember
When cloning the repository with `git clone`, the command downloads a single directory containing all files. This may create directory structure issues. It is recommended to manually move all the folders to the `src` directory or use the following commands:

### Move all folders and files to `src` and delete the parent directory:
```bash
mv ~/ros2_ws/src/rl24_hw3_Imbimbo/* ~/ros2_ws/src/
rm -rf ~/ros2_ws/src/rl24_hw3_Imbimbo
```

---

## Launching Simulation in Gazebo

### Launch the IIWA Robot with Velocity Controller and Camera Enabled
To start the Gazebo simulation with the velocity controller and enable the camera:
```bash
ros2 launch iiwa_bringup iiwa.launch.py command_interface:="velocity" robot_controller:="velocity_controller" use_sim:="true" use_vision:="true"
```

- **Note**: If the camera is not needed, set `use_vision:=false`:
  ```bash
  ros2 launch iiwa_bringup iiwa.launch.py command_interface:="velocity" robot_controller:="velocity_controller" use_sim:="true" use_vision:="false"
  ```

---

## Positioning Commands

### Run the KDL Node for Positioning
In a separate terminal, run the KDL node for positioning tasks:
```bash
ros2 run ros2_kdl_package ros2_kdl_node --ros-args -p cmd_interface:=velocity -p task:=positioning
```

---

## OpenCV Integration

### Launch the OpenCV Node
To use OpenCV functionalities, run the following command:
```bash
ros2 run ros2_opencv ros2_opencv_node
```

---

## ArUco Marker Integration

### Establish Marker Tracking
To integrate with ArUco markers, run the following command:
```bash
ros2 run aruco_ros single --ros-args   -r /image:=/videocamera   -r /camera_info:=/videocamera_info   -p marker_id:=201   -p marker_size:=0.1   -p reference_frame:=camera_link   -p marker_frame:=aruco_marker_frame   -p camera_frame:=camera_link
```

---

## Look-At-Point Command

Once the OpenCV node and ArUco marker tracking are running, you can execute the `look-at-point` task:
```bash
ros2 run ros2_kdl_package ros2_kdl_node --ros-args -p cmd_interface:=velocity -p task:=look-at-point
```

### Testing Look-At-Point
- **Manual Marker Movement**: Use Gazebo’s user interface to move the ArUco marker manually and observe the robot tracking its position.
- **Positional Feedback**: The robot should adjust its view to focus on the ArUco marker.

---

## Marker Pose Monitoring

### Monitor the ArUco Marker Pose
To listen to the marker's pose data, use the following command:
```bash
ros2 topic echo /aruco_single/pose
```

---

## Image Visualization

### Processed Image Monitoring
For better visualization of processed images, open `rqt_image_view` or `rqt_image_plot` and select the `/processed_image` topic:
```bash
rqt_image_view
```

- Navigate to the `/processed_image` topic to see the processed frames.

---

## Notes and Tips

1. **Multiple Terminals**: Ensure you run each node or command in a separate terminal after sourcing the workspace (`source install/setup.bash`).
2. **Video Demonstrations**: In the repository there are videos of the "positioning" and "look at point" tasks.
  - "https://www.youtube.com/watch?v=mJy-RStUUkE" - Positioning
  - "https://www.youtube.com/watch?v=UNbixnNQXAw" - Look_at_Point
3. **Troubleshooting**: If the simulation or tracking does not work as expected:
   - Verify that all required nodes are running.
   - Check for error messages in each terminal.

With these instructions, you can configure and test velocity control, positioning tasks, and ArUco marker tracking in a Gazebo simulation environment.
