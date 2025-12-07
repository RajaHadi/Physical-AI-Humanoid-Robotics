# 03 - Cognitive Planning with Large Language Models (LLMs)

This chapter explores how Large Language Models (LLMs) and Vision-Language Models (VLMs) enable cognitive planning in humanoid robots. We delve into hierarchical planning, where LLMs generate high-level strategies from natural language commands, and discuss critical trade-offs of local versus cloud-based LLM inference.

## 3.1 LLMs as Cognitive Planners in Robotics

-   **Beyond Simple Commands**: LLMs interpret complex, nuanced natural language instructions, inferring user intent from ambiguous commands—crucial for intuitive human-robot interaction.
-   **Cognitive Planning**: An LLM breaks down high-level goals (e.g., "Clean the room") into executable sub-goals or actions, reasoning about task decomposition, state transitions, and environmental obstacles.
-   **Contextual Understanding**: LLMs incorporate dialogue history, world knowledge, and real-time perception data (from Chapter 04) for grounded decisions.

## 3.2 Hierarchical Planning for Robustness and Safety

-   **Decision**: Focus on **hierarchical planning with LLMs for robotics**.
-   **Concept**: The LLM acts as a high-level strategist, receiving a command and outputting an abstract plan (e.g., sequence of sub-goals). This plan is then translated into concrete, low-level ROS 2 actions by a separate robotic planning system (task planner, state machine).
-   **Advantages**:
    -   **Robustness & Safety**: LLM influence is constrained by well-tested robotic control systems, preventing unsafe direct commands.
    -   **Explainability**: Abstract plans provide human-readable reasoning traces, simplifying debugging.
    -   **Leverages Existing Robotics**: Integrates seamlessly with ROS 2 frameworks (Nav2, MoveIt) for motion planning and collision checking.
    -   **Scalability**: LLMs excel at long-horizon tasks by breaking them into manageable sub-goals.
-   **Comparison to Direct ROS Action Generation**: Direct generation (LLM outputs raw ROS commands) is less robust and has higher safety risks due to lack of intermediary checks and need for fine-grained LLM control knowledge.

## 3.3 Local vs. Cloud LLM Inference for Robotics

-   **Decision**: Emphasize **local LLM inference for critical real-time and privacy-sensitive tasks**, while discussing cloud and hybrid approaches.
-   **Local LLM Inference**:
    -   **Pros**: Significantly lower latency (no network delays) for real-time control; high privacy (data stays on device); offline operation.
    -   **Cons**: Requires powerful, expensive onboard hardware; limits LLM size/complexity.
-   **Cloud LLM Inference**:
    -   **Pros**: Access to larger, more powerful, and updated LLMs; lower upfront hardware costs.
    -   **Cons**: Introduces network latency (unsuitable for real-time/safety-critical tasks); raises data privacy concerns; incurs high, unpredictable recurring costs; requires network connectivity.
-   **Hybrid Approaches**: Combine local (small, optimized LLMs for real-time) and cloud (large LLMs for complex, less time-critical reasoning) to leverage strengths of both.
-   **Implications for Robotics**: Local inference is generally preferred for humanoid robots requiring immediate reactions, low-latency control, and sensitive data handling.

## 3.4 Representing Plans: The ROS 2 Action Graph

-   **Structured Output**: The LLM's cognitive plan must translate into a structured, machine-readable format for robot execution, often an "action graph."
-   **Action Graph Concept**: A conceptual representation outlining a sequence of ROS 2 actions (standard and custom, Chapter 05) with interdependencies, parameters, and conditional branches.
    -   **Nodes**: Individual ROS 2 actions (e.g., `NavigateToPose`).
    -   **Edges**: Control flow (sequential, parallel, conditional execution).
    -   **Parameters**: Data extracted from LLM's understanding and environment.
-   **Custom Message Types**: Defining custom ROS 2 message types (e.g., `vla_msgs/ActionGraph`) encapsulates the action graph for standardized communication.

## 3.5 Feedback and Re-planning

-   **Continuous Feedback Loop**: The LLM planner continuously receives updates from perception (Chapter 04) and action execution (Chapter 05).
-   **Error Handling and Adaptation**: Feedback is crucial for:
    -   **Monitoring Progress**: Tracking plan execution status.
    -   **Detecting Failures**: Identifying failed sub-actions or unexpected outcomes.
    -   **Dynamic Re-planning**: Interpreting failures or environmental changes to modify plans or request user clarification.
-   **Goal Monitoring**: The LLM continuously monitors whether the overall high-level goal is being achieved.