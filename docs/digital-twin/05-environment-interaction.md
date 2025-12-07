---
id: 05-environment-interaction
title: Environment and Interaction
sidebar_label: Env & Interaction
description: Modeling terrain, static objects, and dynamic obstacles for humanoid robot simulations.
---

# Environment and Interaction

A robot's environment is as crucial as its own model for realistic simulation. This chapter focuses on how to define and populate the virtual world in which our digital twin operates, enabling meaningful interaction and testing.

## Defining the World (SDF)

The **World File** in SDF (Simulation Description Format) is the top-level entity that defines the simulation environment. It includes:
-   **Physics properties**: Global gravity, physics engine parameters, time step.
-   **Lights**: Ambient, directional, or spot lights to illuminate the scene.
-   **Ground Plane**: A default flat surface for basic interaction.
-   **Static Models**: Fixed objects like walls, furniture, or elevated platforms.
-   **Dynamic Models**: Objects that can move or be manipulated during simulation.

### Example: Basic World File Structure

```xml
<sdf version="1.7">
  <world name="robot_testing_world">
    <physics name="default_physics" default="true" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <scene>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>true</shadows>
    </scene>

    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <!-- Includes for models (e.g., ground plane) -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Your robot model will also be included here -->
    <!-- <include>
      <uri>model://my_humanoid_robot</uri>
    </include> -->

    <!-- Custom terrain or obstacles can be defined below -->
  </world>
</sdf>
```

## Modeling Terrain

For humanoid robots, diverse terrain is critical for testing locomotion and balance.

### Flat Ground
Simple `ground_plane` models are sufficient for basic tests.

### Uneven Terrain / Slopes
More complex terrain can be modeled using heightmaps or collections of primitive shapes (boxes, ramps).

```xml
<!-- Example of a simple ramp -->
<model name="ramp">
  <static>true</static>
  <link name="link">
    <collision name="collision">
      <geometry>
        <box>
          <size>2 1 0.5</size>
        </box>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <box>
          <size>2 1 0.5</size>
        </box>
      </geometry>
      <material>
        <script>
          <uri>file://media/materials/scripts/gazebo.material</uri>
          <name>Gazebo/Orange</name>
        </script>
      </material>
    </visual>
  </link>
  <pose>0 0 0.25 0 -0.2 0</pose> <!-- Positioned and tilted -->
</model>
```

## Static Objects (Obstacles)

Static objects are fixed elements in the environment that the robot can interact with (collide, push against). They typically have their `static` tag set to `true` to indicate they won't be simulated dynamically.

### Example: A Wall Obstacle

```xml
<model name="wall">
  <static>true</static>
  <link name="link">
    <collision name="collision">
      <geometry>
        <box>
          <size>0.1 2 1</size>
        </box>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <box>
          <size>0.1 2 1</size>
        </box>
      </geometry>
      <material>
        <script>
          <uri>file://media/materials/scripts/gazebo.material</uri>
          <name>Gazebo/Grey</name>
        </script>
      </material>
    </visual>
  </link>
  <pose>2 0 0.5 0 0 0</pose>
</model>
```

## Dynamic Obstacles / Manipulable Objects

Dynamic objects can be moved by the physics engine, influenced by collisions with the robot or external forces. These models will have `static` set to `false` or be omitted (default).

### Example: A Movable Box

```xml
<model name="movable_box">
  <static>false</static> <!-- Default, can be omitted -->
  <link name="box_link">
    <pose>0 0 0.5 0 0 0</pose>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.083" ixy="0" ixz="0" iyy="0.083" iyz="0" izz="0.083"/>
    </inertial>
    <collision name="collision">
      <geometry>
        <box>
          <size>1 1 1</size>
        </box>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <box>
          <size>1 1 1</size>
        </box>
      </geometry>
      <material>
        <script>
          <uri>file://media/materials/scripts/gazebo.material</uri>
          <name>Gazebo/Red</name>
        </script>
      </material>
    </visual>
  </link>
</model>
```
To load complex models or reusable components, Gazebo uses `<include>` tags within the world file, referencing models from its model database or local paths.
