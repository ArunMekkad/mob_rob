# mob_rob: ROS 2 Mobile Robot Simulation

A ROS 2 package for simulating and visualizing a mobile robot in Gazebo and RViz2, with SLAM and autonomous navigation using the Nav2 stack.

This package enables users to:
- Simulate a differential drive robot in Gazebo with realistic physics and sensor plugins.
- Perform online SLAM (Simultaneous Localization and Mapping) using SLAM Toolbox, allowing the robot to autonomously build a map of its environment.
- Use the ROS 2 Nav2 stack for autonomous navigation, including path planning, obstacle avoidance, and goal reaching.
- Visualize the robot, its sensors, and navigation stack in RViz2 for debugging and demonstration.
- Easily tune parameters for SLAM, navigation, and robot control to suit different environments or research needs.

---

## Features
- Modular robot description using URDF/Xacro
- Gazebo simulation with ros2_control integration
- SLAM Toolbox for online asynchronous mapping
- Nav2 stack for autonomous navigation
- RViz2 visualization

---

## Dependencies
- ROS 2 Humble (or later)
- Gazebo (comes with ROS 2 desktop)
- slam_toolbox
- nav2_bringup
- teleop_twist_keyboard (for manual control/testing)

### Install dependencies (Ubuntu/Debian):
```sh
sudo apt update
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-gazebo-ros2-control ros-humble-slam-toolbox ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-teleop-twist-keyboard
```
---

## Installation & Build
1. Clone this repository into your ROS 2 workspace `src` directory:
   ```sh
   cd ~/Files/ROS2/ros2_ws/src
   git clone [mob_rob](https://github.com/ArunMekkad/mob_rob.git)
   cd ..
   ```
2. Build the workspace:
   ```sh
   colcon build --packages-select mob_rob
   source install/setup.bash
   ```

---

## How to Run
Launch everything (Gazebo, RViz2, SLAM, Nav2) with a single command:
```sh
ros2 launch mob_rob mob_rob_launch.py
```

- This will:
  - Start Gazebo and spawn the robot
  - Launch SLAM Toolbox for mapping
  - Launch the Nav2 stack for navigation
  - Open RViz2 with the provided config

### Usage
- In RViz2, use the "2D Pose Estimate" tool to set the initial pose.
- Use the "2D Nav Goal" tool to send navigation goals.
- The robot will move in both Gazebo and RViz2, building a map as it explores.

---

## Fine-tuning Parameters
- **SLAM Toolbox:** `config/mapper_params_online_async.yaml`
- **Nav2 Stack:** `config/nav2_params.yaml`
- **Robot Description:** `description/robot.urdf.xacro` and related Xacro files
- **Controller:** `config/ros2_controllers.yaml`

Edit these files to adjust robot, SLAM, or navigation behavior. After changes, rebuild and relaunch.

---

## Troubleshooting
- Make sure all dependencies are installed.
- If you see errors about missing executables, check your ROS 2 installation and sourced environment.
- For controller or movement issues, verify topic remappings and controller configuration.

---

### Future Scope
- **Add Sensors:** Integrate additional sensors (e.g., cameras, IMUs, depth sensors) by editing the robot's URDF/Xacro files and updating Gazebo plugins.
- **Custom Controllers:** Swap or extend the robot's controllers using the ROS 2 control framework for advanced motion or hardware-in-the-loop testing.
- **Deep Learning & Perception:** Subscribe to simulated sensor topics (e.g., camera, lidar) to test deep learning models for perception, object detection, or visual navigation.
- **Multi-Robot Simulation:** Extend the launch files and robot descriptions to support multi-robot scenarios for swarm robotics or collaborative tasks.
- **Real Robot Transition:** The modular design and use of ros2_control make it straightforward to port your algorithms and configurations to real hardware with minimal changes.

## License
See `LICENSE.md` for details.

## Acknowledgement

Huge shoutout to Josh Newans [Articulated Robotics](https://articulatedrobotics.xyz/) for the motivation!

## Maintainer
Arun Mekkad (<mekkad.a@northeastern.edu>)
