# Humanoid AI Book

This repository contains the Humanoid AI Book, created using Docusaurus and deployed to GitHub Pages. The book focuses on ROS 2 concepts for humanoid robotics, explaining how ROS 2 functions as the "nervous system" of humanoid robots.

## Structure

This book was created using Spec-Kit Plus and Qwen as part of an AI/Spec-driven book creation process:

- `docs/` - Docusaurus markdown files for the book content
- `static/` - Static assets like code examples and images
- `src/` - Docusaurus custom components and CSS
- `specs/` - Specification files used during development

## Building and Running

To build and run the documentation locally:

1. Install Node.js (version 18 or higher)
2. Install dependencies: `npm install`
3. Start the development server: `npm start`
4. The site will be available at http://localhost:3000

## Deployment

The site is automatically deployed to GitHub Pages on pushes to the main branch. The deployment URL is: https://panaversity.github.io/humanoid-book

## Content Overview

Module 1: The Robotic Nervous System covers:
- Introduction to ROS 2 as the nervous system of robots
- ROS 2 Nodes, Topics, and Services
- Real-time data flow and middleware concepts
- Python agent integration using rclpy
- URDF for representing humanoid structure
- Complete integration of all concepts

## Code Examples

Code examples are available in the `static/examples/` directory and can be downloaded and used independently. Examples include:
- Basic publisher/subscriber implementations
- Controller implementations
- URDF models
- Complete system integration examples

## Contributing

To contribute to this book:
1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Submit a pull request"# humanoid-book" 
