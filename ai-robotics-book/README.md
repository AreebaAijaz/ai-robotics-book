# Physical AI & Humanoid Robotics

A comprehensive educational resource covering the convergence of artificial intelligence and physical robotics systems, with emphasis on humanoid platforms.

## Overview

This book provides a structured curriculum for advanced CS students and AI practitioners transitioning into robotics. It covers:

- **Module 1: ROS 2 Fundamentals** - Robot Operating System architecture, nodes, topics, services, and URDF
- **Module 2: Simulation Environments** - Gazebo, Unity integration, sensor simulation, and digital twins
- **Module 3: NVIDIA Isaac Platform** - Isaac Sim, Isaac ROS, VSLAM, and Nav2 for bipedal robots
- **Module 4: Vision-Language-Action Models** - Voice-to-action pipelines, LLM cognitive planning, multi-modal integration
- **Capstone Project** - Integrated project combining all modules

## Quick Start

### Prerequisites

- Node.js 18+
- npm 9+

### Installation

```bash
# Clone the repository
git clone https://github.com/ai-robotics-book/ai-robotics-book.git
cd ai-robotics-book

# Install dependencies
npm install
```

### Local Development

```bash
# Start development server
npm start
```

This starts a local development server at http://localhost:3000. Most changes are reflected live without restarting.

### Build

```bash
# Create production build
npm run build
```

Generates static content in the `build` directory.

### Deployment

The site automatically deploys to GitHub Pages when changes are pushed to the `main` branch via GitHub Actions.

## Project Structure

```
ai-robotics-book/
├── docs/                    # All book content
│   ├── intro.md            # Introduction
│   ├── module-1-ros2/      # ROS 2 Fundamentals
│   ├── module-2-simulation/# Simulation Environments
│   ├── module-3-isaac/     # NVIDIA Isaac Platform
│   ├── module-4-vla/       # Vision-Language-Action
│   ├── capstone/           # Capstone Project
│   └── resources/          # References & Glossary
├── src/css/                # Custom styles
├── static/img/             # Static images
├── docusaurus.config.js    # Main configuration
├── sidebars.js             # Sidebar configuration
└── package.json            # Dependencies
```

## Technology Stack

- [Docusaurus 3.x](https://docusaurus.io/) - Static site generator
- [Mermaid](https://mermaid.js.org/) - Diagrams as code
- [GitHub Pages](https://pages.github.com/) - Hosting
- [GitHub Actions](https://github.com/features/actions) - CI/CD

## Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/improvement`)
3. Commit your changes (`git commit -am 'Add improvement'`)
4. Push to the branch (`git push origin feature/improvement`)
5. Open a Pull Request

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Acknowledgments

Built with [Docusaurus](https://docusaurus.io/).
