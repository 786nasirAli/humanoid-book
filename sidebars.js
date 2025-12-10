// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System',
      items: [
        'module-1/intro',
        'module-1/nodes-topics-services',
        'module-1/data-flow-middleware',
        'module-1/python-agent-bridge',
        'module-1/urdf-humanoid-structure',
        'module-1/putting-it-together',
        'module-1/troubleshooting',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: The Digital Twin (Gazebo & Unity)',
      items: [
        'module-2/intro',
        'module-2/physics-simulation',
        'module-2/unity-visualization',
        'module-2/sensor-emulation',
        'module-2/environment-building',
        'module-2/troubleshooting',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac™)',
      items: [
        'module-3/intro',
        'module-3/nvidia-isaac-sim',
        'module-3/isaac-ros-integration',
        'module-3/nav2-bipedal-navigation',
        'module-3/synthetic-data-generation',
        'module-3/training-and-transfer',
        'module-3/troubleshooting',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      items: [
        'module-4/intro',
        'module-4/voice-to-action',
        'module-4/cognitive-planning',
        'module-4/vla-integration',
        'module-4/capstone-project',
        'module-4/troubleshooting',
      ],
    },
  ],
};

module.exports = sidebars;