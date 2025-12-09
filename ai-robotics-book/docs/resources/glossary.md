---
sidebar_position: 2
title: Glossary
description: Key terms and definitions for Physical AI and Humanoid Robotics
---

# Glossary

## A

**Action (ROS 2)**
: A communication pattern for long-running tasks that provides goal, feedback, and result messages. Unlike services, actions can be preempted and report progress.

**Actuator**
: A device that converts energy into motion. In humanoid robots, actuators (typically electric motors) drive the joints.

**Articulation**
: A connected chain of rigid bodies (links) connected by joints. Humanoid robots are complex articulations with many degrees of freedom.

**ASR (Automatic Speech Recognition)**
: Technology that converts spoken language into text. Used in voice interfaces for robot command input.

## B

**Behavior Tree**
: A hierarchical structure for organizing robot behaviors. Used in Nav2 for composing navigation behaviors from primitive actions.

**Bipedal Locomotion**
: Walking on two legs. Requires dynamic balance control and careful footstep planning.

## C

**Callback**
: A function that is automatically called when an event occurs, such as receiving a message on a ROS 2 topic.

**CoM (Center of Mass)**
: The point where the total mass of a body can be considered concentrated. Critical for balance control in humanoid robots.

**Costmap**
: A grid-based representation of the environment where each cell has a cost indicating traversability. Used for navigation planning.

## D

**DDS (Data Distribution Service)**
: The underlying middleware used by ROS 2 for communication. Provides discovery, QoS, and real-time capabilities.

**Degrees of Freedom (DOF)**
: The number of independent parameters that define the configuration of a mechanical system. A humanoid robot may have 20-40+ DOF.

**Digital Twin**
: A virtual representation of a physical robot that mirrors its state in real-time. Used for monitoring, testing, and predictive maintenance.

**Domain Randomization**
: A technique for improving sim-to-real transfer by randomly varying simulation parameters during training.

## E

**Embodied AI**
: Artificial intelligence systems that have physical form and interact with the real world through sensors and actuators.

**End Effector**
: The device at the end of a robotic arm, such as a gripper or hand. Used for manipulation tasks.

**Executor (ROS 2)**
: A component that schedules and runs callbacks for nodes. Can be single-threaded or multi-threaded.

## F

**Forward Kinematics**
: Computing the position and orientation of the end effector given joint angles.

**Footstep Planning**
: Planning a sequence of foot placements for bipedal locomotion.

## G

**Gazebo**
: An open-source robotics simulator that provides physics, sensor simulation, and ROS integration.

**Grounding**
: The process of connecting abstract language concepts to concrete robot-executable actions and perceptions.

## I

**IMU (Inertial Measurement Unit)**
: A sensor combining accelerometers and gyroscopes to measure linear acceleration and angular velocity.

**Intent Classification**
: Determining the user's intended action from natural language input.

**Inverse Kinematics (IK)**
: Computing joint angles required to achieve a desired end effector position and orientation.

**Isaac Sim**
: NVIDIA's robotics simulation platform built on Omniverse, providing photorealistic rendering and GPU-accelerated physics.

## J

**Joint**
: A connection between two links that allows relative motion. Types include revolute (rotation) and prismatic (translation).

## L

**Launch File**
: A file that specifies how to start multiple ROS 2 nodes with configured parameters.

**LLM (Large Language Model)**
: A neural network trained on large text corpora that can understand and generate natural language.

**Lifecycle Node**
: A ROS 2 node with managed state transitions (unconfigured, inactive, active, finalized) for deterministic startup/shutdown.

**Link**
: A rigid body segment in a robot's kinematic chain, connected to other links via joints.

**Localization**
: Determining the robot's position and orientation within a known map.

## M

**MCP (Model Context Protocol)**
: A protocol for integrating AI models with external tools and data sources.

**Mermaid**
: A JavaScript-based diagramming tool that renders diagrams from text descriptions in Markdown.

**Multi-Modal Perception**
: Combining information from multiple sensor types (vision, depth, proprioception) for robust understanding.

## N

**Nav2**
: The ROS 2 Navigation Stack, providing autonomous navigation capabilities including path planning and obstacle avoidance.

**Node (ROS 2)**
: A process that performs computation in ROS 2. Nodes communicate via topics, services, and actions.

**NLU (Natural Language Understanding)**
: Processing natural language to extract meaning, intent, and parameters.

## O

**Odometry**
: Estimating robot pose change over time using sensor data (wheel encoders, IMU, visual).

**Omniverse**
: NVIDIA's platform for building and operating metaverse applications, underlying Isaac Sim.

## P

**PhysX**
: NVIDIA's physics simulation engine used in Isaac Sim for accurate dynamics and contact simulation.

**Proprioception**
: The robot's sense of its own body state, including joint positions, velocities, and forces.

**Publisher/Subscriber**
: A communication pattern where publishers send messages to topics and subscribers receive them. Supports one-to-many communication.

## Q

**QoS (Quality of Service)**
: ROS 2 settings that control reliability, durability, and deadline of communication.

## R

**rclpy**
: The ROS 2 Client Library for Python.

**ROS 2**
: Robot Operating System 2, a middleware framework for robot software development.

**RTX**
: NVIDIA's ray tracing technology that enables photorealistic rendering in Isaac Sim.

## S

**SDF (Simulation Description Format)**
: An XML format for describing simulation worlds, models, and sensors in Gazebo.

**Service (ROS 2)**
: A request-response communication pattern for synchronous operations.

**Sim-to-Real Transfer**
: Transferring policies or models trained in simulation to work on real robots.

**SLAM (Simultaneous Localization and Mapping)**
: Building a map of an unknown environment while simultaneously tracking the robot's position within it.

**Slot Filling**
: Extracting specific parameters from natural language commands (e.g., extracting "kitchen" from "go to the kitchen").

## T

**TF (Transform)**
: ROS 2's system for tracking coordinate frames over time, enabling transformation between different reference frames.

**Topic (ROS 2)**
: A named bus for publish-subscribe communication in ROS 2.

## U

**URDF (Unified Robot Description Format)**
: An XML format for describing robot kinematics, dynamics, visual appearance, and collision geometry.

**USD (Universal Scene Description)**
: A file format for describing 3D scenes, used by Omniverse and Isaac Sim.

## V

**VLA (Vision-Language-Action)**
: Models that directly map visual observations and language commands to robot actions.

**VLM (Vision-Language Model)**
: Models that process both visual and textual inputs to understand scenes and answer questions.

**VSLAM (Visual SLAM)**
: SLAM using camera sensors as the primary input.

## W

**Whole-Body Control**
: Controlling all joints of a humanoid robot simultaneously to achieve tasks while maintaining balance.

**Workspace (ROS 2)**
: A directory containing ROS 2 packages with their source code, build artifacts, and installed files.

## X

**Xacro**
: An XML macro language for creating modular and parameterized URDF descriptions.

## Z

**ZMP (Zero Moment Point)**
: A point on the ground where the sum of horizontal inertial and gravity forces produces zero moment. Critical for bipedal balance.

---

This glossary covers key terms used throughout the Physical AI & Humanoid Robotics curriculum. Terms are cross-referenced where relevant.
