---
id: 02-nodes-topics
title: Nodes and Topics
sidebar_label: Nodes & Topics
description: Creating your first ROS 2 nodes in Python.
---

# Nodes and Topics

## Understanding Nodes
A **Node** is a single process that performs a computation. Nodes communicate with each other using Topics, Services, and Actions.

## The Publisher (Talker)
We will create a simple "Talker" node that publishes a text message every second.

### Code
See `src/ros2_foundations/ros2_foundations/talker.py`.

## The Subscriber (Listener)
We will create a "Listener" node that subscribes to the topic and prints the message.

### Code
See `src/ros2_foundations/ros2_foundations/listener.py`.

## Running the Example

1. Build the package:
   ```bash
   cd ~/ros2_ws
   colcon build --symlink-install
   source install/setup.bash
   ```

2. Run Talker:
   ```bash
   ros2 run ros2_foundations talker
   ```

3. Run Listener:
   ```bash
   ros2 run ros2_foundations listener
   ```
