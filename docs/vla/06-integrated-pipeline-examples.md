# 06 - Integrated Pipeline Examples

This chapter provides concrete, worked examples of Vision-Language-Action (VLA) behaviors, demonstrating the seamless integration of components discussed previously. We detail 2-3 scenarios, illustrating the flow from natural language command to robot execution, including ROS 2 action breakdowns and conceptual message flow diagrams.

## 6.1 Example 1: "Pick Up the Red Cup"

This foundational VLA task integrates speech recognition, LLM planning, visual perception, navigation, and manipulation.

### 6.1.1 Scenario Description

A user commands, "Robot, please pick up the red cup from the table." The robot must hear, understand, plan, navigate, visually confirm, and grasp the cup.

### 6.1.2 VLA Pipeline Flow and ROS 2 Breakdown

1.  **Voice Command (Microphone)**: Human speaks the command.
2.  **Speech-to-Text (Whisper)**: `vla_stt_node` transcribes audio to "pick up the red cup" on `/vla/voice_command`.
3.  **Cognitive Planning (LLM Planner)**: `vla_llm_planner_node` interprets the command, queries `/vla/detected_objects` for "red cup," determines its 3D pose, and generates a **hierarchical plan**. This plan decomposes the command into a **conceptual action graph** of ROS 2 actions.
    *   **Generated Plan**: `NavigateToPose` (near cup) -> `PerceptionQuery` (confirm cup) -> `PickUpObject` (custom action).
    *   **Output**: ROS 2 Topic `/vla/action_plan`.
4.  **Action Execution (High-Level Executive)**: `vla_executive_node` interprets `/vla/action_plan`.
    *   **Execution**: Calls Nav2 `NavigateToPose`. Upon arrival, may issue `PerceptionQuery`. Calls custom `PickUpObject` (internally orchestrates MoveIt for arm/gripper).
    *   **Output**: Publishes status on `/vla/executive_status`.

### 6.1.3 Conceptual Message Flow Diagram

```mermaid
graph TD
    A[Human Voice Command] --> B(Microphone)
    B --> C{vla_stt_node<br>(Whisper STT)}
    C -- transcribed_text (/vla/voice_command) --> D{vla_llm_planner_node<br>(LLM Cognitive Planning)}
    subgraph Robot Perception
        E[Camera/Depth Sensor] --> F{vla_perception_node<br>(Isaac Perception/SLAM)}
        F -- detected_objects (/vla/detected_objects) --> D
    end
    D -- action_plan (/vla/action_plan) --> G{vla_executive_node<br>(Action Execution Engine)}
    G -- NavigateToPose(Goal) --> H[Nav2 Action Server]
    G -- PerceptionQuery(Service/Action) --> F
    G -- PickUpObject(Goal) --> I[Custom PickUpObject Action Server]
    I -- MoveGroup(Goal) --> J[MoveIt2 Action Server]
    H --> K(Humanoid Robot<br>Execution)
    J --> K
    K -- robot_state (/odom, /joint_states) --> D
    K -- sensor_feedback --> F
```

---

## 6.2 Example 2: "Clean the Room"

This complex VLA behavior involves high-level planning and sequential execution in a multi-room environment with static and dynamic obstacles.

### 6.2.1 Scenario Description

User instructs, "Robot, please clean the room. Put all the blocks in the bin and all the cups on the shelf." The robot must hear, understand, generate a comprehensive hierarchical plan, and execute it, adapting to new perception data.

### 6.2.2 VLA Pipeline Flow and ROS 2 Breakdown

1.  **Voice Command & STT**: "clean the room..." is transcribed.
2.  **Cognitive Planning (LLM Planner)**: Interprets the command, performs **hierarchical planning** (e.g., "Explore -> pick/place blocks -> pick/place cups"). Plan accounts for re-planning if objects move.
    *   **Generated Plan**: Iterative calls to `ExploreRoom`, `PickUpObject`, `NavigateToPose`, `PlaceObject`.
    *   **Output**: `/vla/action_plan`.
3.  **Action Execution (High-Level Executive)**: Interprets the Action Graph.
    *   **Execution**: Calls custom `ExploreRoom`. Iteratively calls `PickUpObject`, `NavigateToPose`, `PlaceObject`. Each step uses **perceptual grounding**. Nav2 handles local replanning for dynamic obstacles; LLM executive may re-evaluate.
    *   **Output**: Publishes status (e.g., "Cleaning progress").

### 6.2.3 Conceptual Message Flow Diagram

```mermaid
graph TD
    A[Human Voice Command] --> B(Microphone)
    B --> C{vla_stt_node<br>(Whisper STT)}
    C -- transcribed_text (/vla/voice_command) --> D{vla_llm_planner_node<br>(LLM Cognitive Planning)}
    subgraph Robot Perception
        E[Camera/Depth Sensor] --> F{vla_perception_node<br>(Isaac Perception/SLAM)}
        F -- detected_objects (/vla/detected_objects) --> D
    end
    D -- action_plan (/vla/action_plan) --> G{vla_executive_node<br>(Action Execution Engine)}
    G -- ExploreRoom(Goal) --> H[Custom ExploreRoom Action Server]
    G -- PickUpObject(Goal) --> I[Custom PickUpObject Action Server]
    G -- PlaceObject(Goal) --> J[Custom PlaceObject Action Server]
    I --> K(Humanoid Robot<br>Execution)
    J --> K
    H --> K
    K -- robot_state, sensor_feedback --> F
    K -- robot_state (/odom, /joint_states) --> D
    style G fill:#f9f,stroke:#333,stroke-width:2px
    style D fill:#f9f,stroke:#333,stroke-width:2px
    style F fill:#f9f,stroke:#333,stroke-width:2px
    style C fill:#f9f,stroke:#333,stroke-width:2px
```

---

## 6.3 Example 3: "Follow Me" (Conceptual)

This VLA behavior relies heavily on continuous perception, dynamic re-planning, and user tracking.

### 6.3.1 Scenario Description

User instructs, "Robot, follow me." The robot must identify, continuously track the user, maintain safe distance, and navigate dynamically.

### 6.3.2 VLA Pipeline Flow and ROS 2 Breakdown

1.  **Voice Command & STT**: "Follow me" is transcribed.
2.  **Cognitive Planning (LLM Planner)**: Interprets "follow me" as a continuous goal. Queries perception for the human and generates a **continuous hierarchical plan**: "Track human," "Maintain following distance," "Navigate safely."
    *   **Generated Plan**: Custom `FollowHuman` action.
    *   **Output**: `/vla/action_plan`.
3.  **Action Execution (High-Level Executive)**: Calls custom `FollowHuman` action client.
    *   **Execution**: `FollowHuman` server continuously subscribes to `/vla/detected_objects` for user's updated 3D pose. It calculates a target pose for the robot and calls Nav2 `NavigateToPose`. Nav2 handles local obstacle avoidance; the executive adapts if the user moves unexpectedly.
    *   **Output**: Provides continuous feedback on tracking status.

### 6.3.3 Conceptual Message Flow Diagram

(Similar to previous examples, emphasizing the dynamic loop between perception, planning, and execution, with continuous feedback driving navigation goals.)

---
These examples demonstrate how VLA components integrate to enable complex, natural language-driven robotic behaviors. The LLM's ability to translate high-level human intent into structured, executable robotic plans, grounded in reality through real-time perception and executed via ROS 2 actions, enables robust and adaptive robot behavior.