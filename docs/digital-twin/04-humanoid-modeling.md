---
id: 04-humanoid-modeling
title: Humanoid Modeling in Gazebo
sidebar_label: Humanoid Models
description: Specific considerations for simulating humanoid robots in Gazebo, including inertia and balancing.
---

# Humanoid Modeling in Gazebo

Simulating humanoid robots presents unique challenges compared to simpler robotic systems. Their complex kinematic chains, high number of degrees of freedom, and inherent instability require careful modeling, especially concerning inertia and balancing.

## Inertia and Mass Distribution

Accurate **Inertial Tensors** are paramount for realistic humanoid simulation. Even small inaccuracies in mass distribution can lead to incorrect dynamics, especially in tasks involving dynamic balance or complex manipulation.

When defining links in URDF/SDF, pay close attention to:
-   **Mass (`<mass>` tag)**: The total mass of the link.
-   **Inertial Tensor (`<inertia>` tag)**: The 3x3 matrix describing mass distribution. This is often calculated by CAD software or approximated for simple geometries.

### Example: Custom Inertia in URDF

```xml
<link name="torso_link">
  <inertial>
    <origin xyz="0 0 0.15" rpy="0 0 0"/>
    <mass value="15.0"/>
    <inertia
      ixx="0.45" ixy="0.0" ixz="0.0"
      iyy="0.40" iyz="0.0"
      izz="0.15"
    />
  </inertial>
  <visual>
    <origin xyz="0 0 0.15" rpy="0 0 0"/>
    <geometry>
      <box size="0.2 0.15 0.3"/>
    </geometry>
    <material name="blue"/>
  </visual>
  <collision>
    <origin xyz="0 0 0.15" rpy="0 0 0"/>
    <geometry>
      <box size="0.2 0.15 0.3"/>
    </geometry>
  </collision>
</link>
```
The `ixx`, `iyy`, `izz` values represent moments of inertia about the x, y, and z axes respectively, while `ixy`, `ixz`, `iyz` represent products of inertia. For symmetric bodies or aligned coordinate systems, products of inertia are often zero.

## Balancing and Stability

Humanoid robots are inherently unstable, requiring active balance control. In simulation, achieving stable locomotion or standing often depends on:

1.  **Accurate Joint Limits**: Defining realistic mechanical stops for joints.
2.  **Contact Modeling**: Proper configuration of friction and restitution for feet-ground contact.
3.  **Controller Fidelity**: The control algorithms themselves must be robust to sensory noise and simulation discrepancies.
4.  **Center of Mass (CoM) Projection**: For static stability, the projection of the robot's CoM must remain within its support polygon (e.g., area defined by its feet). Dynamic stability involves more complex criteria.

## URDF/SDF Structure for Humanoids

Humanoid robot models typically feature:
-   A **base link** (often the pelvis or lower torso).
-   Multiple **kinematic chains** for legs, arms, and torso.
-   **Feet links** with well-defined collision geometries for stable ground contact.
-   Careful placement of **sensors** (IMUs, cameras) on appropriate links.

## Avoiding Common Pitfalls

-   **Self-Collision**: Ensure that robot parts do not excessively collide with themselves. This can be mitigated with proper collision mesh design and, if necessary, disabling collision detection between certain linked bodies.
-   **Joint Instability**: "Jittery" or "exploding" joints often indicate issues with joint limits, high controller gains, or numerical instability in the physics solver. Adjusting `erp` (error reduction parameter) and `cfm` (constraint force mixing) in the `<physics>` tag can sometimes help.
