# Input Shaping with ROS 2 Joint Trajectories

This repository contains a ROS 2 C++ action client developed to perform input
shaping experiments on an existing robotic crane system using the standard
`control_msgs/FollowJointTrajectory` interface.

The code is intended to operate within an already configured ROS 2 control
pipeline, including MoveIt, inverse kinematics, the
`joint_trajectory_controller`, and vendor-provided motor controllers.
No low-level motor control, firmware, or safety-critical components are modified.

Input shaping is applied at the trajectory command level by shaping joint
velocity profiles over time. This enables vibration reduction experiments to be
conducted in a safe, repeatable, and non-invasive manner on real hardware.

All low-level control loops (current, velocity, and position) remain fully
internal to the motor controllers. This repository interacts with the system
only through standard ROS 2 interfaces.

---

## System Context

The action client sends velocity-based joint trajectories to the existing
`joint_trajectory_controller` via the `FollowJointTrajectory` action interface.
Trajectories are defined using `trajectory_msgs/JointTrajectoryPoint`.

The current implementation uses velocity-only trajectory points. This is
sufficient for early-stage input shaping experiments focused on excitation
control and vibration reduction.

---

## Dependencies

The following dependencies are required:

- ROS 2 Humble
- `rclcpp`
- `rclcpp_action`
- `control_msgs`
- `trajectory_msgs`
- `ros2_control`
- `joint_trajectory_controller`

All dependencies are expected to be available through a standard ROS 2 Humble
installation.

---

## Build Instructions

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
colcon build --packages-select input_shaping_client
source install/setup.bash
```
---
## Running the Action Client

Before running the client, ensure that:
- The robotic crane system is running
- `joint_trajectory_controller` is active
- The following action server is available:

```text
/joint_trajectory_controller/follow_joint_trajectory
```

Run the client:
```
ros2 run input_shaping_client input_shaping_client_node
```
---
## Repository Structure
```
input_shaping_ros2/
├── LICENSE
├── README.md
└── input_shaping_client/
    ├── CMakeLists.txt
    ├── package.xml
    └── src/
        └── input_shaping_client.cpp
```
---
##License
MIT License
2025 Sanjay Maharjan
---
## Author
Sanjay Maharjan
Mechanical Engineering
Lousiana State University
