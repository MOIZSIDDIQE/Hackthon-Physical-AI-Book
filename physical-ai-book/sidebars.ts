// @ts-check

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 * - create an ordered group of docs
 * - render a sidebar for each doc of that group
 * - provide next/previous navigation
 *
 * The sidebars can be generated from the filesystem, or explicitly defined here.
 *
 * Create as many sidebars as you want.
 *
 * @type {import('@docusaurus/plugin-content-docs').SidebarsConfig}
 */
const sidebars = {
  // University-grade textbook sidebar structure
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Part 01: Foundations',
      collapsible: true,
      collapsed: false, // Default expanded
      items: [
        {
          type: 'category',
          label: 'Chapter 01: Physical AI Concepts',
          collapsible: true,
          collapsed: false,
          items: [
            'part-01-foundations/chapter-01/1.1-concepts',
            'part-01-foundations/chapter-01/1.2-practical',
            'part-01-foundations/chapter-01/1.3-diagrams'
          ]
        },
        {
          type: 'category',
          label: 'Chapter 02: Humanoid Fundamentals',
          collapsible: true,
          collapsed: true,
          items: [
            'part-01-foundations/chapter-02/2.1-concepts',
            'part-01-foundations/chapter-02/2.2-practical',
            'part-01-foundations/chapter-02/2.3-diagrams'
          ]
        },
        {
          type: 'category',
          label: 'Chapter 03: ROS2 Integration',
          collapsible: true,
          collapsed: true,
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
      label: 'Part 02: Design Principles',
      collapsible: true,
      collapsed: true,
      items: [
        {
          type: 'category',
          label: 'Chapter 01: Humanoid Design',
          collapsible: true,
          collapsed: true,
          items: [
            'part-02-humanoid-design/chapter-01/1.1-concepts',
            'part-02-humanoid-design/chapter-01/1.2-practical',
            'part-02-humanoid-design/chapter-01/1.3-diagrams'
          ]
        },
        {
          type: 'category',
          label: 'Chapter 02: Biomechanics',
          collapsible: true,
          collapsed: true,
          items: [
            'part-02-humanoid-design/chapter-02/2.1-concepts',
            'part-02-humanoid-design/chapter-02/2.2-practical',
            'part-02-humanoid-design/chapter-02/2.3-diagrams'
          ]
        },
        {
          type: 'category',
          label: 'Chapter 03: Balance Control',
          collapsible: true,
          collapsed: true,
          items: [
            'part-02-humanoid-design/chapter-03/3.1-concepts',
            'part-02-humanoid-design/chapter-03/3.2-practical',
            'part-02-humanoid-design/chapter-03/3.3-diagrams'
          ]
        }
      ]
    },
    {
      type: 'category',
      label: 'Part 03: AI Perception',
      collapsible: true,
      collapsed: true,
      items: [
        {
          type: 'category',
          label: 'Chapter 01: Perception Fundamentals',
          collapsible: true,
          collapsed: true,
          items: [
            'part-03-ai-perception/chapter-01/1.1-concepts',
            'part-03-ai-perception/chapter-01/1.2-practical',
            'part-03-ai-perception/chapter-01/1.3-diagrams'
          ]
        },
        {
          type: 'category',
          label: 'Chapter 02: Sensor Integration',
          collapsible: true,
          collapsed: true,
          items: [
            'part-03-ai-perception/chapter-02/2.1-concepts',
            'part-03-ai-perception/chapter-02/2.2-practical',
            'part-03-ai-perception/chapter-02/2.3-diagrams'
          ]
        },
        {
          type: 'category',
          label: 'Chapter 03: Computer Vision',
          collapsible: true,
          collapsed: true,
          items: [
            'part-03-ai-perception/chapter-03/3.1-concepts',
            'part-03-ai-perception/chapter-03/3.2-practical',
            'part-03-ai-perception/chapter-03/3.3-diagrams'
          ]
        }
      ]
    },
    {
      type: 'category',
      label: 'Part 04: Motion Planning',
      collapsible: true,
      collapsed: true,
      items: [
        {
          type: 'category',
          label: 'Chapter 01: Path Planning',
          collapsible: true,
          collapsed: true,
          items: [
            'part-04-motion-planning/chapter-01/1.1-concepts',
            'part-04-motion-planning/chapter-01/1.2-practical',
            'part-04-motion-planning/chapter-01/1.3-diagrams'
          ]
        },
        {
          type: 'category',
          label: 'Chapter 02: Trajectory Optimization',
          collapsible: true,
          collapsed: true,
          items: [
            'part-04-motion-planning/chapter-02/2.1-concepts',
            'part-04-motion-planning/chapter-02/2.2-practical',
            'part-04-motion-planning/chapter-02/2.3-diagrams'
          ]
        },
        {
          type: 'category',
          label: 'Chapter 03: Dynamic Movement',
          collapsible: true,
          collapsed: true,
          items: [
            'part-04-motion-planning/chapter-03/3.1-concepts',
            'part-04-motion-planning/chapter-03/3.2-practical',
            'part-04-motion-planning/chapter-03/3.3-diagrams'
          ]
        }
      ]
    },
    {
      type: 'category',
      label: 'Part 05: Robot Control',
      collapsible: true,
      collapsed: true,
      items: [
        {
          type: 'category',
          label: 'Chapter 01: Control Theory',
          collapsible: true,
          collapsed: true,
          items: [
            'part-05-robot-control/chapter-01/1.1-concepts',
            'part-05-robot-control/chapter-01/1.2-practical',
            'part-05-robot-control/chapter-01/1.3-diagrams'
          ]
        },
        {
          type: 'category',
          label: 'Chapter 02: Feedback Systems',
          collapsible: true,
          collapsed: true,
          items: [
            'part-05-robot-control/chapter-02/2.1-concepts',
            'part-05-robot-control/chapter-02/2.2-practical',
            'part-05-robot-control/chapter-02/2.3-diagrams'
          ]
        },
        {
          type: 'category',
          label: 'Chapter 03: Advanced Control',
          collapsible: true,
          collapsed: true,
          items: [
            'part-05-robot-control/chapter-03/3.1-concepts',
            'part-05-robot-control/chapter-03/3.2-practical',
            'part-05-robot-control/chapter-03/3.3-diagrams'
          ]
        }
      ]
    },
    {
      type: 'category',
      label: 'Part 06: Simulation',
      collapsible: true,
      collapsed: true,
      items: [
        {
          type: 'category',
          label: 'Chapter 01: Simulation Environments',
          collapsible: true,
          collapsed: true,
          items: [
            'part-06-simulation/chapter-01/1.1-concepts',
            'part-06-simulation/chapter-01/1.2-practical',
            'part-06-simulation/chapter-01/1.3-diagrams'
          ]
        },
        {
          type: 'category',
          label: 'Chapter 02: Physics Simulation',
          collapsible: true,
          collapsed: true,
          items: [
            'part-06-simulation/chapter-02/2.1-concepts',
            'part-06-simulation/chapter-02/2.2-practical',
            'part-06-simulation/chapter-02/2.3-diagrams'
          ]
        },
        {
          type: 'category',
          label: 'Chapter 03: Virtual Testing',
          collapsible: true,
          collapsed: true,
          items: [
            'part-06-simulation/chapter-03/3.1-concepts',
            'part-06-simulation/chapter-03/3.2-practical',
            'part-06-simulation/chapter-03/3.3-diagrams'
          ]
        }
      ]
    },
    {
      type: 'category',
      label: 'Part 07: Kinematics',
      collapsible: true,
      collapsed: true,
      items: [
        {
          type: 'category',
          label: 'Chapter 01: Forward Kinematics',
          collapsible: true,
          collapsed: true,
          items: [
            'part-07-kinematics/chapter-01/1.1-concepts',
            'part-07-kinematics/chapter-01/1.2-practical',
            'part-07-kinematics/chapter-01/1.3-diagrams'
          ]
        },
        {
          type: 'category',
          label: 'Chapter 02: Inverse Kinematics',
          collapsible: true,
          collapsed: true,
          items: [
            'part-07-kinematics/chapter-02/2.1-concepts',
            'part-07-kinematics/chapter-02/2.2-practical',
            'part-07-kinematics/chapter-02/2.3-diagrams'
          ]
        },
        {
          type: 'category',
          label: 'Chapter 03: Kinematic Optimization',
          collapsible: true,
          collapsed: true,
          items: [
            'part-07-kinematics/chapter-03/3.1-concepts',
            'part-07-kinematics/chapter-03/3.2-practical',
            'part-07-kinematics/chapter-03/3.3-diagrams'
          ]
        }
      ]
    },
    {
      type: 'category',
      label: 'Part 08: Robot Hardware',
      collapsible: true,
      collapsed: true,
      items: [
        {
          type: 'category',
          label: 'Chapter 01: Hardware Components',
          collapsible: true,
          collapsed: true,
          items: [
            'part-08-robot-hardware/chapter-01/1.1-concepts',
            'part-08-robot-hardware/chapter-01/1.2-practical',
            'part-08-robot-hardware/chapter-01/1.3-diagrams'
          ]
        },
        {
          type: 'category',
          label: 'Chapter 02: Actuators and Sensors',
          collapsible: true,
          collapsed: true,
          items: [
            'part-08-robot-hardware/chapter-02/2.1-concepts',
            'part-08-robot-hardware/chapter-02/2.2-practical',
            'part-08-robot-hardware/chapter-02/2.3-diagrams'
          ]
        },
        {
          type: 'category',
          label: 'Chapter 03: Hardware Integration',
          collapsible: true,
          collapsed: true,
          items: [
            'part-08-robot-hardware/chapter-03/3.1-concepts',
            'part-08-robot-hardware/chapter-03/3.2-practical',
            'part-08-robot-hardware/chapter-03/3.3-diagrams'
          ]
        }
      ]
    },
    {
      type: 'category',
      label: 'Part 09: Ethics and Safety',
      collapsible: true,
      collapsed: true,
      items: [
        {
          type: 'category',
          label: 'Chapter 01: Robot Ethics',
          collapsible: true,
          collapsed: true,
          items: [
            'part-09-ethics/chapter-01/1.1-concepts',
            'part-09-ethics/chapter-01/1.2-practical',
            'part-09-ethics/chapter-01/1.3-diagrams'
          ]
        },
        {
          type: 'category',
          label: 'Chapter 02: Safety and Responsibility',
          collapsible: true,
          collapsed: true,
          items: [
            'part-09-ethics/chapter-02/2.1-concepts',
            'part-09-ethics/chapter-02/2.2-practical',
            'part-09-ethics/chapter-02/2.3-diagrams'
          ]
        },
        {
          type: 'category',
          label: 'Chapter 03: Societal Impact',
          collapsible: true,
          collapsed: true,
          items: [
            'part-09-ethics/chapter-03/3.1-concepts',
            'part-09-ethics/chapter-03/3.2-practical',
            'part-09-ethics/chapter-03/3.3-diagrams'
          ]
        }
      ]
    },
    {
      type: 'category',
      label: 'Part 10: Applications',
      collapsible: true,
      collapsed: true,
      items: [
        {
          type: 'category',
          label: 'Chapter 01: Industrial Applications',
          collapsible: true,
          collapsed: true,
          items: [
            'part-10-applications/chapter-01/1.1-concepts',
            'part-10-applications/chapter-01/1.2-practical',
            'part-10-applications/chapter-01/1.3-diagrams'
          ]
        },
        {
          type: 'category',
          label: 'Chapter 02: Service Robotics',
          collapsible: true,
          collapsed: true,
          items: [
            'part-10-applications/chapter-02/2.1-concepts',
            'part-10-applications/chapter-02/2.2-practical',
            'part-10-applications/chapter-02/2.3-diagrams'
          ]
        },
        {
          type: 'category',
          label: 'Chapter 03: Healthcare Robotics',
          collapsible: true,
          collapsed: true,
          items: [
            'part-10-applications/chapter-03/3.1-concepts',
            'part-10-applications/chapter-03/3.2-practical',
            'part-10-applications/chapter-03/3.3-diagrams'
          ]
        }
      ]
    },
    {
      type: 'category',
      label: 'Part 11: Research',
      collapsible: true,
      collapsed: true,
      items: [
        {
          type: 'category',
          label: 'Chapter 01: Research Trends',
          collapsible: true,
          collapsed: true,
          items: [
            'part-11-research/chapter-01/1.1-concepts',
            'part-11-research/chapter-01/1.2-practical',
            'part-11-research/chapter-01/1.3-diagrams'
          ]
        },
        {
          type: 'category',
          label: 'Chapter 02: Research Methodologies',
          collapsible: true,
          collapsed: true,
          items: [
            'part-11-research/chapter-02/2.1-concepts',
            'part-11-research/chapter-02/2.2-practical',
            'part-11-research/chapter-02/2.3-diagrams'
          ]
        },
        {
          type: 'category',
          label: 'Chapter 03: Experimental Design',
          collapsible: true,
          collapsed: true,
          items: [
            'part-11-research/chapter-03/3.1-concepts',
            'part-11-research/chapter-03/3.2-practical',
            'part-11-research/chapter-03/3.3-diagrams'
          ]
        }
      ]
    },
    {
      type: 'category',
      label: 'Part 12: Future of Humanoids',
      collapsible: true,
      collapsed: true,
      items: [
        {
          type: 'category',
          label: 'Chapter 01: Advanced Systems',
          collapsible: true,
          collapsed: true,
          items: [
            'part-12-future-of-humanoids/chapter-01/1.1-concepts',
            'part-12-future-of-humanoids/chapter-01/1.2-practical',
            'part-12-future-of-humanoids/chapter-01/1.3-diagrams'
          ]
        },
        {
          type: 'category',
          label: 'Chapter 02: Future Technologies',
          collapsible: true,
          collapsed: true,
          items: [
            'part-12-future-of-humanoids/chapter-02/2.1-concepts',
            'part-12-future-of-humanoids/chapter-02/2.2-practical',
            'part-12-future-of-humanoids/chapter-02/2.3-diagrams'
          ]
        },
        {
          type: 'category',
          label: 'Chapter 03: Societal Integration',
          collapsible: true,
          collapsed: true,
          items: [
            'part-12-future-of-humanoids/chapter-03/3.1-concepts',
            'part-12-future-of-humanoids/chapter-03/3.2-practical',
            'part-12-future-of-humanoids/chapter-03/3.3-diagrams'
          ]
        }
      ]
    }
  ]
};

export default sidebars;