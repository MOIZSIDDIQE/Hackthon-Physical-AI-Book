// @ts-check

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.

 @type {import('@docusaurus/plugin-content-docs').SidebarsConfig}
 */
const sidebars = {
  // Manual sidebar with nested structure for the book
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Part 1: Foundations of Physical AI',
      collapsed: false,
      items: [
        {
          type: 'category',
          label: 'Chapter 1: Introduction to Physical AI',
          collapsed: false,
          items: [
            'part-01-foundations/chapter-01/1.1-concepts',
            'part-01-foundations/chapter-01/1.2-practical',
            'part-01-foundations/chapter-01/1.3-diagrams'
          ]
        },
        {
          type: 'category',
          label: 'Chapter 2: Humanoid Robot Fundamentals',
          collapsed: false,
          items: [
            'part-01-foundations/chapter-02/2.1-concepts',
            'part-01-foundations/chapter-02/2.2-practical',
            'part-01-foundations/chapter-02/2.3-diagrams'
          ]
        },
        {
          type: 'category',
          label: 'Chapter 3: ROS2 and Gazebo Integration',
          collapsed: false,
          items: [
            'part-01-foundations/chapter-03/3.1-concepts',
            'part-01-foundations/chapter-03/3.2-practical',
            'part-01-foundations/chapter-03/3.3-diagrams'
          ]
        }
      ]
    },
    {
      type: 'category',
      label: 'Part 2: Humanoid Design Principles',
      collapsed: false,
      items: [
        {
          type: 'category',
          label: 'Chapter 1: Humanoid Robot Design Principles',
          collapsed: false,
          items: [
            'part-02-humanoid-design/chapter-01/1.1-concepts',
            'part-02-humanoid-design/chapter-01/1.2-practical',
            'part-02-humanoid-design/chapter-01/1.3-diagrams'
          ]
        },
        {
          type: 'category',
          label: 'Chapter 2: Biomechanics and Human Movement Analysis',
          collapsed: false,
          items: [
            'part-02-humanoid-design/chapter-02/2.1-concepts',
            'part-02-humanoid-design/chapter-02/2.2-practical',
            'part-02-humanoid-design/chapter-02/2.3-diagrams'
          ]
        },
        {
          type: 'category',
          label: 'Chapter 3: Balance Control and Locomotion Fundamentals',
          collapsed: false,
          items: [
            'part-02-humanoid-design/chapter-03/3.1-concepts',
            'part-02-humanoid-design/chapter-03/3.2-practical',
            'part-02-humanoid-design/chapter-03/3.3-diagrams'
          ]
        }
      ]
    }
  ]
};

export default sidebars;
