# 05 - ROS 2 Action Generation for VLA

This chapter focuses on the Action component of Vision-Language-Action (VLA) systems, detailing how Large Language Models (LLMs) translate cognitive plans into executable robotic behaviors using the ROS 2 Action interface. We cover both standard ROS 2 actions (Nav2, MoveIt) and custom action definitions for complex VLA tasks.

## 5.1 The Role of Actions in ROS 2 Robotics

-   **ROS 2 Communication Primitives**: Actions are used for long-running, goal-oriented tasks with continuous feedback and preemptability. Topics handle continuous data, services manage immediate request/response.
-   **Why Actions for VLA?**: Actions are ideal for VLA due to:
    -   **Goal Management**: Sending high-level commands (e.g., "Navigate to kitchen").
    -   **Continuous Feedback**: Monitoring robot progress (e.g., "Robot is moving," "Object detected").
    -   **Preemptability**: LLM can cancel/modify tasks if new info or plan changes arise.
    -   **Structured Results**: Clear task outcome (success/failure).

## 5.2 Standard ROS 2 Actions: Building Blocks for VLA

-   **Decision**: Explain **standard MoveIt/Nav2 actions as fundamental building blocks**.
-   **Nav2 Actions (Navigation)**:
    -   `NavigateToPose`: Commands robot to a specific 2D pose. LLM plans often translate into sequences of these.
    -   `ComputePathToPose`: Plans a path without immediate execution, allowing LLM inspection.
-   **MoveIt 2 Actions (Manipulation)**:
    -   `MoveGroup`: Controls manipulators for motion planning, collision avoidance, and trajectory execution.
    -   `FollowJointTrajectory`: Executes pre-defined joint trajectories.
-   **Integration in VLA**: LLM-generated plans decompose into these standard actions, called by a high-level executive.

## 5.3 Custom ROS 2 Action Definitions for VLA Tasks

-   **Decision**: Explain **custom ROS 2 action definitions for encapsulating higher-level, application-specific VLA tasks**.
-   **Purpose**: Custom actions define unique robotic behaviors, abstracting multiple standard actions into a single high-level interface for the LLM planner's output.
-   **When to Create Custom Actions**: For tasks combining multiple standard actions with specific logic (e.g., "PickUpObject" involves navigation, perception, arm movement, grasping).
-   **Defining a Custom Action (Conceptual)**:
    ```
    # vla_msgs/action/PickUpObject.action
    # Goal: string object_id, geometry_msgs/Pose object_pose
    # ---
    # Result: bool success, string message
    # ---
    # Feedback: float32 progress_percentage, string current_sub_task
    ```
-   **Implementing a Custom Action Server**: A ROS 2 node implements the custom action logic, typically calling clients for standard Nav2/MoveIt actions.

## 5.4 From LLM Plan to ROS 2 Actions

-   **The Action Graph**: LLM generates an Action Graph (`data-model.md`, `contracts/interfaces.md`) representing the VLA task.
-   **Action Execution Engine**: A high-level ROS 2 node interprets this Action Graph, acting as an action client for standard and custom actions.
-   **Mapping**: Maps abstract plan steps to ROS 2 action calls, providing parameters from the plan and perception data.
-   **Feedback Loop**: Engine provides feedback to the LLM planner for plan adjustments or re-planning.

## 5.5 Examples of ROS 2 Action Flows for VLA

-   **"Pick up the red cup"**: LLM plans `NavigateTo(red_cup_location)` and `PickUpObject(red_cup_id, red_cup_pose)`. Executive calls these, with `PickUpObject` internally using MoveIt.
-   **"Clean the room"**: LLM orchestrates `ExploreRoom`, then iteratively calls `PickUpObject`, `NavigateToPose`, `PlaceObject` for each item.