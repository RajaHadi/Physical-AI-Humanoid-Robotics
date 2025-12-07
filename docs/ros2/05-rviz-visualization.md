---
id: 05-rviz-visualization
title: RViz2 Visualization
sidebar_label: RViz2
description: Visualizing your URDF robot model in RViz2.
---

# RViz2 Visualization

## Introduction to RViz2
RViz2 is a 3D visualizer for ROS 2. It allows you to display sensor data, robot models, and other information in a graphical environment.

## Launching Your Robot Model
To see your `humanoid.urdf` in RViz2, you will typically use a launch file that starts the necessary ROS 2 nodes.

### Code
See `src/ros2_foundations/launch/visualize.launch.py`.

## Manipulating Joints
The `joint_state_publisher_gui` node allows you to interactively manipulate the joints of your robot model in RViz2.

### Steps
1. Launch your robot model with the GUI:
   ```bash
   ros2 launch ros2_foundations visualize.launch.py use_jsp_gui:=true
   ```
2. A GUI window will appear with sliders for each joint. Move them to see your robot articulate.
