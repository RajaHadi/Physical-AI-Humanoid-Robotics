---
id: 07-ai-bridge
title: AI Agent to ROS 2 Bridge
sidebar_label: AI Bridge
description: Bridging external AI agents with the ROS 2 ecosystem.
---

# AI Agent to ROS 2 Bridge

## The Need for an AI Bridge
Complex AI agents often run outside of the ROS 2 graph, using different communication protocols or data structures. An "AI Bridge" node acts as a translator, converting data between the AI agent's format and ROS 2 messages.

## Implementing a Simple Bridge Node
We will create a Python node that:
1. Subscribes to a simple ROS 2 topic (e.g., `std_msgs/String`) for AI commands.
2. Publishes ROS 2 sensor data (e.g., from our mock IMU) to a format suitable for an external AI agent.

### Code
See `src/ros2_foundations/ros2_foundations/ai_bridge.py`.

## Communication Protocol
Our bridge will use a simple JSON string format for commands from the AI agent and for sending data to it.

### Example AI Command (ROS 2 to AI)
```json
{"command": "get_status"}
```

### Example AI Command (AI to ROS 2)
```json
{"action": "move_head", "direction": "left"}
```
