# Physical AI & Humanoid Robotics Book

This repository contains a comprehensive, beginner-friendly book about Physical AI and Humanoid Robotics, built with Docusaurus v2.

## Table of Contents
- [About This Book](#about-this-book)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Local Development](#local-development)
- [Building for Production](#building-for-production)
- [Project Structure](#project-structure)
- [Contributing](#contributing)
- [License](#license)

## About This Book

This book provides a complete learning experience for understanding and building humanoid robots. It covers:
- Fundamentals of robotics
- ROS2 and Gazebo simulation
- Humanoid robot design principles
- Robot control and programming
- AI and perception for robotics
- Real hardware integration
- Capstone projects and next steps

## Prerequisites

Before running this site, you need to have the following installed:

- [Node.js](https://nodejs.org/en/download/) version 18.0 or above
- [npm](https://www.npmjs.com/) or [yarn](https://yarnpkg.com/)

## Installation

1. Clone the repository:
```bash
git clone https://github.com/your-org/physical-ai-book.git
cd physical-ai-book
```

2. Install dependencies:
```bash
npm install
```

## Local Development

1. Start the development server:
```bash
npm start
```

This command starts a local development server and opens up a browser window. Most changes are reflected live without having to restart the server.

The site will be available at `http://localhost:3000`

## Building for Production

To build the website for production:

```bash
npm run build
```

This command generates static content into the `build` directory and can be served using any static contents hosting service.

To test the production build locally:
```bash
npm run serve
```

## Project Structure

```
physical-ai-book/
├── blog/                 # Blog posts
├── docs/                 # Book content
│   ├── fundamentals/     # Fundamentals chapter
│   ├── simulation/       # Simulation chapter
│   ├── humanoid/         # Humanoid design chapter
│   ├── programming/      # Programming chapter
│   ├── ai/               # AI and perception chapter
│   ├── hardware/         # Hardware chapter
│   ├── projects/         # Projects chapter
│   └── intro.md          # Introduction
├── src/
│   ├── components/       # React components
│   ├── css/              # Custom styles
│   └── pages/            # Additional pages
├── static/               # Static files
├── docusaurus.config.js  # Site configuration
├── sidebars.ts           # Sidebar configuration
└── package.json          # Dependencies and scripts
```

## Documentation Structure

The book is organized into the following chapters:

- `docs/intro.md` - Introduction to Physical AI & Humanoid Robotics
- `docs/fundamentals/robotics-basics.md` - Robotics Fundamentals
- `docs/simulation/ros2-gazebo.md` - ROS2 and Gazebo Simulation
- `docs/humanoid/design.md` - Humanoid Robot Design
- `docs/programming/control.md` - Robot Control and Programming
- `docs/ai/perception.md` - AI and Perception for Robotics
- `docs/hardware/real-robot.md` - Real Hardware Integration
- `docs/projects/capstone.md` - Capstone Project
- `docs/next-steps.md` - Next Steps in Robotics

## Customization

### Adding New Content
To add new documentation pages:
1. Create a new `.md` file in the appropriate directory under `docs/`
2. Add the proper frontmatter:
```markdown
---
title: Your Page Title
sidebar_position: X
---
```
3. Update `sidebars.ts` if needed to include the new page in the navigation

### Styling
Custom styles can be added to `src/css/custom.css`.

### Components
Custom React components can be added to `src/components/` and used throughout the site.

## Contributing

We welcome contributions to improve this robotics book! Here's how you can help:

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Make your changes
4. Commit your changes (`git commit -m 'Add amazing feature'`)
5. Push to the branch (`git push origin feature/amazing-feature`)
6. Open a Pull Request

### Content Contributions
- Fix typos or grammatical errors
- Improve explanations or add examples
- Add new content or chapters
- Update outdated information

### Technical Contributions
- Fix bugs in the website
- Improve site performance
- Add new features

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Support

If you encounter any issues or have questions about the content, please open an issue in the GitHub repository.

## Acknowledgments

This book was created using Docusaurus, a modern static website generator optimized for building documentation websites.
