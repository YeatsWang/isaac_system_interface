# isaac_system_interface

[![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)


**Isaac System Interface** is a ROS2 hardware_interface System plugin for bridging ROS2 control with NVIDIA Isaac Sim joint state/command topics.

---

## Features

- **Bridges ROS2 Control and Isaac Sim:** Seamlessly connects ROS2 controllers with Isaac Sim robots via `/joint_states` and `/joint_command` topics.
- **Supports velocity/position hybrid control.**
- **Thread-safe joint state update.**
- **Easily configurable via URDF.**

---

## Usage

1. **Add to your ROS2 workspace:**
   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/YeatsWang/isaac_system_interface.git
   ```
2. **Build:**
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select isaac_system_interface
   ```
3. **URDF Example:**
   ```xml
    <ros2_control name="IsaacSystemHW" type="system">
    <hardware>
        <plugin>isaac_system_interface/IsaacSystemInterface</plugin>
        <param name="joint_state_topic">/joint_states</param>
        <param name="joint_command_topic">/joint_command</param>
    </hardware>
    <!-- ... your joint definitions ... -->
    <joint name="front_left_wheel_joint">
      <command_interface name="velocity"/>
      <state_interface name="velocity"/>
    </joint>
    <joint name="front_left_steering_joint">
      <command_interface name="position"/>
      <state_interface name="position"/>
    </joint>
    <!-- ... your joint definitions ... -->
    </ros2_control>
   ```

---

## Parameters
- `joint_state_topic`: (default `/joint_states`) Isaac Sim joint state topic to subscribe.
- `joint_command_topic`: (default `/joint_command`) Isaac Sim command topic to publish.

## License
This project is licensed under the Apache-2.0 License - see the [LICENSE](LICENSE) file for details.