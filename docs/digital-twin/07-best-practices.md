---
id: 07-best-practices
title: Simulation Best Practices
sidebar_label: Best Practices
description: Identifying common simulation errors, understanding the Sim-to-Real gap, and managing latency.
---

# Simulation Best Practices

Effective robot simulation requires not just understanding how to configure models and environments, but also recognizing common pitfalls and applying best practices. This chapter helps you avoid frustrations and build more robust digital twins.

## Common Simulation Errors

Here's a list of frequently encountered issues in robot simulation and their typical solutions:

1.  **"Jittery" or Unstable Robot Motion**:
    -   **Problem**: Robot links are oscillating, vibrating, or exploding.
    -   **Causes**: High controller gains, overly stiff contact parameters, small collision meshes that penetrate easily, numerical instability in the physics engine.
    -   **Fixes**:
        -   Reduce controller gains (P, I, D).
        -   Adjust physics engine parameters (e.g., lower `kp`, `kd`, increase `max_vel` for contact in SDF).
        -   Ensure collision meshes are not too small or thin, and avoid initial penetrations.
        -   Decrease `max_step_size` or increase solver iterations in the `<physics>` tag.

2.  **Robot Sinks into Ground / Floats Above Ground**:
    -   **Problem**: Robot's base link or feet don't maintain proper contact with the ground plane.
    -   **Causes**: Incorrect `min_depth` for contact surfaces, inappropriate `kp`/`kd` values, or mass/inertia mismatches.
    -   **Fixes**:
        -   Ensure `min_depth` is small but non-zero for ground contacts.
        -   Increase `kp` (contact stiffness) to resist penetration, and `kd` (contact damping) to reduce bounce.
        -   Verify that the robot's mass properties are reasonable.

3.  **Slow Simulation / Low Real-Time Factor**:
    -   **Problem**: Simulation runs slower than real-time, or a very low real-time factor is observed.
    -   **Causes**: Complex models (high polygon count visual meshes, many collision elements), too many sensors, small `max_step_size`, many active contacts.
    -   **Fixes**:
        -   Simplify collision meshes (e.g., use primitives like boxes/spheres instead of detailed visual meshes for collision).
        -   Reduce the number of active sensors or their update rates if not critical.
        -   Increase `max_step_size` in the `<physics>` tag (if numerical stability permits).
        -   Disable shadows or reduce visual quality in the simulator if visual fidelity is not paramount for the task.

4.  **Sensors Not Detecting Objects / Detecting Ghost Objects**:
    -   **Problem**: LiDAR or camera sensors don't perceive objects correctly.
    -   **Causes**: Sensor `range` too small/large, incorrect `min_angle`/`max_angle`, sensor attached to the wrong link, collision filtering issues.
    -   **Fixes**:
        -   Verify sensor parameters (`min`/`max` range, FOV) align with environment scale.
        -   Check the parent link of the sensor in the URDF/SDF.
        -   Ensure collision groups or masks are not inadvertently filtering out objects from sensor detection (Gazebo's `<collision_filter>` can be complex).

## The Sim-to-Real Gap

The "Sim-to-Real" gap refers to the discrepancy between simulated robot behavior and its real-world counterpart. Bridging this gap is one of the most significant challenges in robotics.

### Causes of the Sim-to-Real Gap:
-   **Inaccurate Models**: Imperfect models of robot kinematics, dynamics (mass, friction, inertia), and actuators.
-   **Sensor Fidelity**: Lack of realistic noise, latency, and distortion in simulated sensor data.
-   **Environmental Differences**: Unmodeled aspects of the real environment (e.g., air currents, subtle surface irregularities, lighting variations).
-   **Control Discrepancies**: Control parameters tuned in simulation might not transfer directly to hardware.

### Mitigating the Gap:
-   **Randomization**: Randomizing simulation parameters (mass, friction, sensor noise) can help the AI learn robust policies.
-   **Accurate Parameter Estimation**: Carefully measuring or identifying parameters of the real robot.
-   **Domain Adaptation**: Techniques to transfer knowledge learned in simulation to the real world.
-   **Realistic Sensor Models**: Incorporating realistic noise and bias (as discussed in Chapter 3).

## Managing Latency in Simulation

Latency is the delay between an event (e.g., a motor command, a sensor reading) and its processing. In simulation, latency can arise from:
-   **Simulation Rate**: The rate at which the physics engine updates.
-   **Communication Delays**: Between the simulator and control algorithms (e.g., ROS 2 nodes).
-   **Rendering Rate**: If the simulation is visually heavy.

### Best Practices for Latency:
-   **Decouple Physics and Rendering**: Run the physics engine at a high fixed rate, while rendering can be at a lower, variable rate. This is common in simulators like Gazebo.
-   **Optimize Communication**: Use efficient messaging protocols and minimize data serialization/deserialization overhead.
-   **Monitor Real-Time Factor**: Keep an eye on the simulator's real-time factor (RTF). An RTF < 1 indicates the simulation is not keeping up with real-time, introducing implicit latency.
-   **Profile Performance**: Use profiling tools to identify bottlenecks in your simulation or control code.
