---
id: 01-introduction
title: Introduction to ROS 2
sidebar_label: Intro to ROS 2
description: Understanding the Robot Operating System and setting up your environment.
---

# Introduction to ROS 2

## The "Nervous System" of Robots

Just as a biological nervous system transmits signals between the brain (AI) and the body (sensors/actuators), ROS 2 serves as the communication backbone for modern robots.

## Installation

### Prerequisites
- Ubuntu 22.04 LTS (Jammy Jellyfish)

### Setup Steps
1. Set locale:
   ```bash
   locale  # check for UTF-8
   sudo apt update && sudo apt install locales
   sudo locale-gen en_US en_US.UTF-8
   sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
   export LANG=en_US.UTF-8
   ```

2. Add ROS 2 apt repository...
   *(Refer to official docs for up-to-date steps)*

3. Install ROS 2 Humble:
   ```bash
   sudo apt install ros-humble-desktop
   ```

## Verifying Installation
Run the classic talker/listener example:
```bash
ros2 run demo_nodes_cpp talker
```
