---
id: 02-physics-basics
title: Physics Simulation Basics
sidebar_label: Physics Basics
description: Understanding rigid body dynamics, gravity, and friction in robot simulation.
---

# Physics Simulation Basics

## Rigid Body Dynamics

In robot simulation, a **rigid body** is a solid body in which deformation is zero or so small it can be neglected. This simplification allows for computationally efficient simulation of complex mechanical systems. Each link of our humanoid robot model (from Module 1, or more complex ones) will be treated as a rigid body.

The behavior of these rigid bodies is governed by Newton's laws of motion, considering forces, torques, mass, and inertia. Key aspects include:
-   **Mass**: The amount of matter in a body.
-   **Center of Mass (CoM)**: The point where the entire mass of the body is concentrated.
-   **Inertial Tensor**: Describes how mass is distributed around the CoM, influencing rotational dynamics. (Refer to the glossary for more details).

## Gravity Configuration (SDF Example)

Gravity is a fundamental force in any realistic physics simulation. In Gazebo, it is typically configured within the world file (`.world` or included in an SDF model).

```xml
<sdf version="1.7">
  <world name="default">
    <physics name="default_physics" default="true" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
      <ode>
        <solver>
          <type>quick</type>
          <iters>50</iters>
          <precon_iters>0</precon_iters>
          <sor>1.3</sor>
          <rms_error_tolerance>0.0</rms_error_tolerance>
        </solver>
        <constraints>
          <cfm>0.0</cfm>
          <erp>0.2</erp>
        </constraints>
      </ode>
    </physics>
    <gravity>0 0 -9.81</gravity> <!-- Standard Earth gravity -->
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type="adiabatic"/>
    <scene>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>true</shadows>
    </scene>
    <!-- Other world elements like ground plane, lights, models go here -->
  </world>
</sdf>
```
In this example, `<gravity>0 0 -9.81</gravity>` sets the gravitational acceleration vector to point downwards along the Z-axis, simulating Earth's gravity.

## Friction Configuration (SDF Example)

Friction is essential for realistic contact between robot parts and the environment. It defines the resistance to motion when two surfaces are in contact.

Friction parameters are typically defined within the `<surface>` tags of a `<collision>` element in an SDF file.

```xml
<model name="ground_plane">
  <link name="link">
    <collision name="collision">
      <geometry>
        <plane>
          <normal>0 0 1</normal>
          <size>100 100</size>
        </plane>
      </geometry>
      <surface>
        <friction>
          <ode>
            <mu>0.8</mu>     <!-- Coefficient of dynamic friction -->
            <mu2>0.8</mu2>    <!-- Coefficient of static friction (for 2nd friction direction) -->
            <fdir1>1 0 0</fdir1> <!-- First friction direction -->
            <slip1>0.0</slip1>   <!-- Coefficient of sliding friction in fdir1 direction -->
            <slip2>0.0</slip2>   <!-- Coefficient of sliding friction in fdir2 direction -->
          </ode>
        </friction>
        <contact>
          <ode>
            <kp>10000000.0</kp>   <!-- Contact stiffness -->
            <kd>1.0</kd>         <!-- Contact damping -->
            <max_vel>0.1</max_vel> <!-- Max contact velocity -->
            <min_depth>0.001</min_depth> <!-- Min contact depth -->
          </ode>
        </contact>
      </surface>
    </collision>
    <visual name="visual">
      <geometry>
        <plane>
          <normal>0 0 1</normal>
          <size>100 100</size>
        </plane>
      </geometry>
      <material>
        <script>
          <uri>file://media/materials/scripts/gazebo.material</uri>
          <name>Gazebo/Green</name>
        </script>
      </material>
    </visual>
  </link>
</model>
```
In this example, `<mu>` and `<mu2>` define the friction coefficients for the contact surface. These values significantly impact how objects slide or grip against each other.
-   **`mu`**: Coulomb friction coefficient for the primary direction.
-   **`mu2`**: Coulomb friction coefficient for the second direction, orthogonal to `fdir1`.
-   **`fdir1`**: A vector defining the primary direction of friction.
-   **`slip1` / `slip2`**: Coefficients for slip friction (viscous friction), which can be used to model more complex friction behaviors.