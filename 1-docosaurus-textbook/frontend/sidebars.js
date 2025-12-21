// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Module 1: Robotic Nervous System (ROS 2)',
      collapsed: false,
      items: [
        'module1/intro',
        'module1/ros2-nodes',
        'module1/rclpy-agents',
        'module1/urdf',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Digital Twin (Gazebo & Unity)',
      collapsed: false,
      items: [
        'module2/intro',
        'module2/gazebo-physics',
        'module2/unity-hri',
        'module2/sensors-lidar',
        'module2/sensors-depth',
      ],
    },
  ],
};

module.exports = sidebars;
