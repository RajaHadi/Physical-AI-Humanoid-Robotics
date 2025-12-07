---
id: 03-services-actions
title: Services and Actions
sidebar_label: Services & Actions
description: Understanding synchronous and asynchronous communication in ROS 2.
---

# Services and Actions

## ROS 2 Services
Services provide a request/response communication mechanism, ideal for synchronous operations where a client sends a request and waits for a server to respond.

### Defining a Service
A service is defined by a pair of `.srv` files (Request and Response).

## ROS 2 Actions
Actions provide a request/feedback/result communication mechanism, suitable for long-running tasks where clients need feedback on progress and can cancel the goal.

### Defining an Action
An action is defined by a `.action` file (Goal, Result, Feedback).

## Example Implementations
This section will provide example implementations for a basic ROS 2 Service Server/Client and an Action Server/Client.
