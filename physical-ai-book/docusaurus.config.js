// @ts-check
// `@type` JSDoc annotations allow editor autocompletion and type checking
// (when paired with `@ts-check`).
// There are various equivalent ways to declare your Docusaurus config.
// See: https://docusaurus.io/docs/api/docusaurus-config

import {themes as prismThemes} from 'prism-react-renderer';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'A Comprehensive Guide to Building Intelligent Humanoid Robots',
  favicon: 'img/favicon-text.svg',

  // Future flags, see https://docusaurus.io/docs/api/docusaurus-config#future
  future: {
    v4: true, // Improve compatibility with the upcoming Docusaurus v4
  },

  // Set the production url of your site here
  url: process.env.DEPLOYMENT_URL || 'https://your-robotics-book-site.example.com',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: process.env.BASE_URL || '/',

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: 'your-org', // Usually your GitHub org/user name.
  projectName: 'physical-ai-book', // Usually your repo name.

  onBrokenLinks: 'throw',

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: './sidebars.ts',
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/your-org/physical-ai-book/edit/main/',
        },
        blog: {
          showReadingTime: true,
          feedOptions: {
            type: ['rss', 'atom'],
            xslt: true,
          },
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/your-org/physical-ai-book/edit/main/',
          // Useful options to enforce blogging best practices
          onInlineTags: 'warn',
          onInlineAuthors: 'warn',
          onUntruncatedBlogPosts: 'warn',
        },
        theme: {
          customCss: './src/css/custom.css',
        },
      }),
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      // Replace with your project's social card
      image: 'img/robot-social-card.jpg',
      colorMode: {
        defaultMode: 'dark',
        disableSwitch: false,
        respectPrefersColorScheme: true,
      },
      algolia: {
        // The application ID provided by Algolia
        appId: process.env.ALGOLIA_APP_ID || 'PHLZ3CG82D',
        // Public API key: it is safe to commit it
        apiKey: process.env.ALGOLIA_API_KEY || 'a42b2f5e5b6f5a10cfedf6f3a23c8f8b',
        indexName: process.env.ALGOLIA_INDEX_NAME || 'physical-ai-humanoid-robotics',
        contextualSearch: true,
        searchPagePath: 'search',
      },
      navbar: {
        title: 'Physical AI & Humanoid Robotics',
        logo: {
          alt: 'Physical AI & Humanoid Robotics',
          src: 'img/logo-text.svg',
          srcDark: 'img/logo-text-dark.svg',
        },
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'tutorialSidebar',
            position: 'left',
            label: '📚 Book',
          },
          {to: '/blog', label: '📝 Blog', position: 'left'},
          {to: '/search', label: '🔍 Search', position: 'left'}, // Add search to navbar
          {
            to: '/docs/part-01-foundations/chapter-01/1.1-concepts',
            label: 'Get Started',
            position: 'right',
            className: 'button button--primary',
          },
          {
            href: 'https://github.com/your-org/physical-ai-book',
            label: '⭐ GitHub',
            position: 'right',
          },
        ],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: '📖 Documentation',
            items: [
              {
                label: 'Introduction',
                to: '/docs/part-01-foundations/chapter-01/1.1-concepts',
              },
              {
                label: 'Curriculum Overview',
                to: '/docs/intro',
              },
              {
                label: 'Getting Started',
                to: '/docs/part-01-foundations/chapter-01/1.2-practical',
              },
            ],
          },
          {
            title: '🛠️ Resources',
            items: [
              {
                label: 'ROS2 Documentation',
                href: 'https://docs.ros.org/en/humble/',
              },
              {
                label: 'Gazebo Simulation',
                href: 'https://gazebosim.org/',
              },
              {
                label: 'Python Robotics',
                href: 'https://pypi.org/project/robotics-toolkit/',
              },
            ],
          },
          {
            title: '🔗 Community',
            items: [
              {
                label: 'GitHub',
                href: 'https://github.com/your-org/physical-ai-book',
              },
              {
                label: 'Discord',
                href: 'https://discord.gg/robotics',
              },
              {
                label: 'Twitter',
                href: 'https://twitter.com/roboticsbook',
              },
            ],
          },
          {
            title: '📬 Contact',
            items: [
              {
                label: 'Report an Issue',
                href: 'https://github.com/your-org/physical-ai-book/issues',
              },
              {
                label: 'Contribute',
                href: 'https://github.com/your-org/physical-ai-book/blob/main/CONTRIBUTING.md',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics Book. Built with Docusaurus. 🤖`,
      },
      prism: {
        theme: prismThemes.github,
        darkTheme: prismThemes.dracula,
      },
    }),
};

export default config;
