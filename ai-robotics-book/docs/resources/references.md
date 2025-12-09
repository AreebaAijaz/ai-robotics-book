---
sidebar_position: 1
title: References
description: Bibliography and external resources for Physical AI and Humanoid Robotics
---

# References

## Official Documentation

### ROS 2

- [ROS 2 Documentation](https://docs.ros.org/en/jazzy/) - Official ROS 2 Jazzy documentation
- [ROS 2 Tutorials](https://docs.ros.org/en/jazzy/Tutorials.html) - Step-by-step tutorials
- [rclpy API](https://docs.ros2.org/latest/api/rclpy/) - Python client library reference
- [Nav2 Documentation](https://navigation.ros.org/) - Navigation stack for ROS 2

### NVIDIA Isaac

- [Isaac Sim Documentation](https://docs.omniverse.nvidia.com/isaacsim/latest/index.html) - Official Isaac Sim docs
- [Isaac ROS](https://nvidia-isaac-ros.github.io/) - Isaac ROS packages
- [Omniverse Documentation](https://docs.omniverse.nvidia.com/) - Omniverse platform docs

### Gazebo

- [Gazebo Harmonic](https://gazebosim.org/docs/harmonic/) - Latest Gazebo documentation
- [ros_gz](https://github.com/gazebosim/ros_gz) - ROS 2 Gazebo integration

## Research Papers

### Humanoid Robotics

1. **Whole-Body Control**
   - Sentis, L., & Khatib, O. (2005). "Synthesis of whole-body behaviors through hierarchical control of behavioral primitives." *International Journal of Humanoid Robotics*, 2(04), 505-518.

2. **Bipedal Locomotion**
   - Kajita, S., et al. (2003). "Biped walking pattern generation by using preview control of zero-moment point." *ICRA 2003*.

3. **Balance Control**
   - Pratt, J., et al. (2006). "Capture point: A step toward humanoid push recovery." *Humanoids 2006*.

### Vision-Language-Action Models

1. **RT-2: Vision-Language-Action Models**
   - Brohan, A., et al. (2023). "RT-2: Vision-Language-Action Models Transfer Web Knowledge to Robotic Control." *arXiv:2307.15818*.

2. **PaLM-E: An Embodied Multimodal Language Model**
   - Driess, D., et al. (2023). "PaLM-E: An Embodied Multimodal Language Model." *ICML 2023*.

3. **RT-1: Robotics Transformer**
   - Brohan, A., et al. (2022). "RT-1: Robotics Transformer for Real-World Control at Scale." *arXiv:2212.06817*.

### Visual SLAM

1. **ORB-SLAM3**
   - Campos, C., et al. (2021). "ORB-SLAM3: An Accurate Open-Source Library for Visual, Visual-Inertial and Multi-Map SLAM." *IEEE T-RO*.

2. **VINS-Mono**
   - Qin, T., et al. (2018). "VINS-Mono: A Robust and Versatile Monocular Visual-Inertial State Estimator." *IEEE T-RO*.

### Learning for Robotics

1. **Imitation Learning**
   - Argall, B. D., et al. (2009). "A survey of robot learning from demonstration." *Robotics and autonomous systems*.

2. **Reinforcement Learning for Locomotion**
   - Hwangbo, J., et al. (2019). "Learning agile and dynamic motor skills for legged robots." *Science Robotics*.

## Books

### Robotics Fundamentals

1. **Springer Handbook of Robotics** (2nd Ed.)
   - Siciliano, B., & Khatib, O. (Eds.) (2016)
   - Comprehensive reference covering all aspects of robotics

2. **Modern Robotics: Mechanics, Planning, and Control**
   - Lynch, K. M., & Park, F. C. (2017)
   - Excellent foundation in robot kinematics and dynamics

3. **Probabilistic Robotics**
   - Thrun, S., Burgard, W., & Fox, D. (2005)
   - Essential for understanding SLAM and localization

### Computer Vision

1. **Multiple View Geometry in Computer Vision**
   - Hartley, R., & Zisserman, A. (2003)
   - Foundation for visual SLAM

2. **Computer Vision: Algorithms and Applications**
   - Szeliski, R. (2022) - 2nd Edition
   - Comprehensive computer vision reference

### Machine Learning

1. **Deep Learning**
   - Goodfellow, I., Bengio, Y., & Courville, A. (2016)
   - Foundation for neural network architectures

2. **Reinforcement Learning: An Introduction**
   - Sutton, R. S., & Barto, A. G. (2018) - 2nd Edition
   - Essential for robot learning

## Online Courses

### Robotics

- [MIT 6.4210: Robotic Manipulation](https://manipulation.csail.mit.edu/) - Russ Tedrake
- [Stanford CS223A: Introduction to Robotics](https://cs.stanford.edu/groups/manips/teaching/cs223a/)
- [ETH Robot Dynamics](https://rsl.ethz.ch/education-students/lectures/rob-dyn.html)

### Deep Learning for Robotics

- [Stanford CS231n: CNNs for Visual Recognition](http://cs231n.stanford.edu/)
- [Berkeley CS287: Advanced Robotics](https://people.eecs.berkeley.edu/~pabbeel/cs287-fa19/)
- [CMU 16-831: Statistical Techniques in Robotics](https://www.cs.cmu.edu/~16831-f14/)

## Software Libraries

### ROS 2 Packages

| Package | Description | Link |
|---------|-------------|------|
| `nav2` | Navigation stack | [nav2](https://github.com/ros-planning/navigation2) |
| `moveit2` | Motion planning | [moveit2](https://github.com/ros-planning/moveit2) |
| `ros2_control` | Hardware abstraction | [ros2_control](https://github.com/ros-controls/ros2_control) |
| `image_pipeline` | Image processing | [image_pipeline](https://github.com/ros-perception/image_pipeline) |

### Machine Learning

| Library | Description | Link |
|---------|-------------|------|
| PyTorch | Deep learning framework | [pytorch.org](https://pytorch.org/) |
| Transformers | NLP models | [huggingface.co](https://huggingface.co/transformers/) |
| OpenCV | Computer vision | [opencv.org](https://opencv.org/) |
| Open3D | 3D data processing | [open3d.org](http://www.open3d.org/) |

### Speech Processing

| Library | Description | Link |
|---------|-------------|------|
| Whisper | Speech recognition | [github.com/openai/whisper](https://github.com/openai/whisper) |
| Vosk | Offline speech recognition | [alphacephei.com/vosk](https://alphacephei.com/vosk/) |
| pyttsx3 | Text-to-speech | [pyttsx3](https://pypi.org/project/pyttsx3/) |

## Humanoid Robot Platforms

### Commercial

| Platform | Manufacturer | Notes |
|----------|--------------|-------|
| Atlas | Boston Dynamics | Advanced research platform |
| Digit | Agility Robotics | Commercial bipedal robot |
| Optimus | Tesla | In development |
| Figure 01 | Figure | AI-focused humanoid |

### Research/Open Source

| Platform | Organization | Notes |
|----------|--------------|-------|
| Cassie | Agility Robotics | Research bipedal platform |
| Unitree H1 | Unitree | Affordable humanoid |
| BRUCE | Westwood Robotics | Open-source biped |

## Community Resources

### Forums and Discussion

- [ROS Discourse](https://discourse.ros.org/) - Official ROS community forum
- [Isaac Sim Forums](https://forums.developer.nvidia.com/c/omniverse/simulation/) - NVIDIA community
- [Robotics Stack Exchange](https://robotics.stackexchange.com/) - Q&A site

### GitHub Organizations

- [ros2](https://github.com/ros2) - ROS 2 organization
- [NVIDIA-ISAAC-ROS](https://github.com/NVIDIA-ISAAC-ROS) - Isaac ROS packages
- [ros-planning](https://github.com/ros-planning) - Navigation and motion planning
- [ros-controls](https://github.com/ros-controls) - Control libraries

## Datasets

### Robot Manipulation

- **RoboSet** - Large-scale robot manipulation dataset
- **Open X-Embodiment** - Cross-robot learning dataset
- **BridgeData V2** - Real robot manipulation data

### Scene Understanding

- **ScanNet** - Indoor RGB-D dataset with annotations
- **Matterport3D** - Indoor environment dataset
- **NYU Depth V2** - RGB-D indoor scenes

---

This reference list is not exhaustive but provides starting points for deeper exploration of Physical AI and Humanoid Robotics topics.
