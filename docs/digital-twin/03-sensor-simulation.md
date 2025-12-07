---
id: 03-sensor-simulation
title: Sensors in Simulation
sidebar_label: Sensor Simulation
description: Modeling LiDAR, Depth Cameras, RGB cameras, and IMUs in simulated environments.
---

# Sensors in Simulation

Robot perception relies heavily on sensor data. In a digital twin, accurately simulating these sensors is crucial for developing robust control systems and perception algorithms that can later be transferred to real robots. This involves not only replicating the sensor's output but also modeling its physical properties and noise characteristics.

## Sensor Noise Models

As discussed in the glossary, **Sensor Noise Models** are critical for making simulated data realistic. Without noise, an AI agent might learn patterns that don't exist in the real world, leading to a significant "sim-to-real" gap. Common noise types include:
-   **Gaussian Noise**: Random fluctuations following a normal distribution.
-   **Drift**: A gradual change in sensor bias over time.
-   **Outliers**: Sporadic, erroneous readings.

## LiDAR Configuration (SDF Example)

LiDAR (Light Detection and Ranging) sensors measure distances by emitting laser pulses. They are fundamental for mapping and navigation.

```xml
<sensor name="lidar" type="ray">
  <pose>0 0 0.1 0 0 0</pose> <!-- Relative to its parent link -->
  <ray>
    <scan>
      <horizontal>
        <samples>640</samples>
        <resolution>1</resolution>
        <min_angle>-2.2</min_angle>
        <max_angle>2.2</max_angle>
      </horizontal>
      <vertical>
        <samples>1</samples>
        <resolution>1</resolution>
        <min_angle>0</min_angle>
        <max_angle>0</max_angle>
      </vertical>
    </scan>
    <range>
      <min>0.08</min>
      <max>10.0</max>
      <resolution>0.01</resolution>
    </range>
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.01</stddev> <!-- Standard deviation of the Gaussian noise -->
    </noise>
  </ray>
  <always_on>1</always_on>
  <update_rate>10</update_rate> <!-- 10 Hz update rate -->
  <visualize>true</visualize>
</sensor>
```
This SDF snippet defines a horizontal LiDAR with 640 samples over a 4.4 radian field of view, a range of 0.08m to 10m, and a Gaussian noise model.

## Camera Configuration (SDF Example)

Cameras provide rich visual information about the environment. RGB cameras capture color images, while depth cameras also provide distance information for each pixel.

### RGB Camera

```xml
<sensor name="camera" type="camera">
  <pose>0.05 0 0.05 0 0 0</pose>
  <camera>
    <horizontal_fov>1.047</horizontal_fov> <!-- 60 degrees -->
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.1</near>
      <far>100</far>
    </clip>
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.007</stddev>
    </noise>
  </camera>
  <always_on>1</always_on>
  <update_rate>30</update_rate>
  <visualize>true</visualize>
</sensor>
```

### Depth Camera

Depth cameras are crucial for 3D perception, allowing robots to understand the geometry of their surroundings.

```xml
<sensor name="depth_camera" type="depth">
  <pose>0.05 0 0.05 0 0 0</pose>
  <camera>
    <horizontal_fov>1.047</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
      <format>L_DEPTH</format> <!-- Format for depth data -->
    </image>
    <clip>
      <near>0.1</near>
      <far>10</far>
    </clip>
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.02</stddev>
    </noise>
  </camera>
  <always_on>1</always_on>
  <update_rate>30</update_rate>
  <visualize>false</visualize>
</sensor>
```

## IMU (Inertial Measurement Unit) Configuration (SDF Example)

IMUs measure linear acceleration and angular velocity, providing essential data for robot localization and state estimation.

```xml
<sensor name="imu_sensor" type="imu">
  <pose>0 0 0 0 0 0</pose>
  <imu>
    <angular_velocity>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.0002</stddev>
          <bias_mean>0.0000075</bias_mean>
          <bias_stddev>0.0000008</bias_stddev>
        </noise>
      </x>
      <!-- y and z angular velocity noise similarly configured -->
    </angular_velocity>
    <linear_acceleration>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.017</stddev>
          <bias_mean>0.1</bias_mean>
          <bias_stddev>0.001</bias_stddev>
        </noise>
      </x>
      <!-- y and z linear acceleration noise similarly configured -->
    </linear_acceleration>
  </imu>
  <always_on>1</always_on>
  <update_rate>100</update_rate>
</sensor>
```
IMU noise models can be quite complex, including terms for random walk, bias, and temperature effects. This example shows a basic Gaussian noise with bias for angular velocity and linear acceleration.
