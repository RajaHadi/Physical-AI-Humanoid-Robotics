import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  // By default, Docusaurus generates a sidebar from the docs folder structure
  tutorialSidebar: [
    {
      type: 'category',
      label: 'ROS 2 Foundations',
      items: [
        'ros2/01-introduction',
        'ros2/02-nodes-topics',
        'ros2/03-services-actions',
        'ros2/04-urdf-modeling',
        'ros2/05-rviz-visualization',
        'ros2/06-control-loops',
        'ros2/07-ai-bridge',
        'ros2/08-mini-project',
      ],
    },
    {
      type: 'category',
      label: 'Digital Twin',
      items: [
        'digital-twin/01-intro-digital-twins',
        'digital-twin/02-physics-basics',
        'digital-twin/03-sensor-simulation',
        'digital-twin/04-humanoid-modeling',
        'digital-twin/05-environment-interaction',
        'digital-twin/06-unity-visualization',
        'digital-twin/07-best-practices',
        'digital-twin/08-linking-modules',
      ],
    },
    {
      type: 'category',
      label: 'NVIDIA Isaac Sim',
      items: [
        'isaac-sim/introduction',
        'isaac-sim/simulation-setup',
        'isaac-sim/perception-pipelines',
        'isaac-sim/navigation-planning',
        'isaac-sim/reinforcement-learning',
        'isaac-sim/integrating-modules',
        'isaac-sim/best-practices',
        'isaac-sim/preparing-module4',
      ],
    },
    {
      type: 'category',
      label: 'VLA Robotics LLM',
      items: [
        'vla/foundations-vla',
        'vla/whisper-language',
        'vla/cognitive-planning',
        'vla/vision-perception',
        'vla/ros2-action-generation',
        'vla/integrated-pipeline-examples',
        'vla/capstone-preparation',
      ],
    },
  ],
};

export default sidebars;
