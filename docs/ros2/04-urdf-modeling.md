---
id: 04-urdf-modeling
title: URDF Robot Modeling
sidebar_label: URDF Modeling
description: Describing your robot's physical structure with URDF.
---

# URDF Robot Modeling

## What is URDF?
The Unified Robot Description Format (URDF) is an XML format for describing the physical structure of a robot. It represents the robot as a kinematic tree of links (rigid bodies) connected by joints (revolute, prismatic, fixed, etc.).

## Links and Joints
- **Link**: Defines the physical and visual properties of a segment of the robot (e.g., mass, inertia, shape, color).
- **Joint**: Defines how two links are connected and their relative motion (e.g., rotation, translation).

## Building a Humanoid Torso
We will create a simple URDF for a humanoid torso.

### Code
See `src/ros2_foundations/urdf/humanoid.urdf`

## Visualizing with RViz2
Once the URDF is created, we can visualize it using RViz2.
