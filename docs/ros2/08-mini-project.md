---
id: 08-mini-project
title: Mini Project - Humanoid Nervous System
sidebar_label: Mini Project
description: Integrating all components to build a basic humanoid nervous system.
---

# Mini Project: Humanoid Nervous System

## Goal
In this mini-project, we will integrate all the ROS 2 components and concepts learned in this module to build a basic "nervous system" for our humanoid robot. This includes:
- Running `talker` and `listener` nodes.
- Visualizing the `humanoid.urdf` in RViz2.
- Operating the `control_loop` node with dummy sensor input.
- Sending commands and receiving data through the `ai_bridge` node.

## Integration Steps

1.  **Start the ROS 2 environment:**
    ```bash
    cd ~/ros2_ws
    colcon build --symlink-install
    source install/setup.bash
    ```

2.  **Launch visualization and control:**
    ```bash
    # Open three separate terminals
    # Terminal 1: Launch RViz2 and robot_state_publisher
    ros2 launch ros2_foundations visualize.launch.py

    # Terminal 2: Run the control loop node, which also publishes dummy IMU data
    ros2 run ros2_foundations control_loop

    # Terminal 3: Run the AI bridge node
    ros2 run ros2_foundations ai_bridge
    ```

3.  **Interact with the system:**
    - Observe the robot model in RViz2: The `shoulder_joint` of the humanoid will articulate based on the simulated IMU data from the `control_loop` node.
    - Monitor `ai_bridge` output: The bridge node will log when it receives IMU data and attempts to send it to an "external AI".
    - Test AI commands (optional, requires a separate AI client node publishing to `/ai/command`):
        ```bash
        ros2 topic pub /ai/command std_msgs/String "data: '{\"action\": \"move_head\", \"direction\": \"left\"}'" --once
        ```
        The `ai_bridge` node will log this command.

## Conclusion
This mini-project demonstrates the foundational components of a robot's nervous system using ROS 2, from basic communication to sensor processing and AI integration.
