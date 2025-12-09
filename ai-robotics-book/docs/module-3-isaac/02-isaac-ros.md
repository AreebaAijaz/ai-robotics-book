---
sidebar_position: 2
title: Isaac ROS Integration
description: Connecting NVIDIA Isaac Sim with ROS 2 for robot development
---

# Isaac ROS Integration

## Learning Outcomes

By the end of this chapter, you will be able to:

1. **Configure** the Isaac ROS bridge for ROS 2 communication
2. **Publish** sensor data from Isaac Sim to ROS 2 topics
3. **Subscribe** to control commands from ROS 2 nodes
4. **Implement** action servers for complex robot behaviors
5. **Debug** communication issues between Isaac and ROS 2

## Prerequisites

- Completed Chapter 1: Isaac Sim Overview
- ROS 2 Jazzy installed and configured
- Humanoid robot imported into Isaac Sim

## Isaac ROS Architecture

Isaac Sim communicates with ROS 2 through the OmniGraph action graph system:

```mermaid
graph LR
    subgraph "Isaac Sim"
        OG[OmniGraph]
        PUB[ROS Publishers]
        SUB[ROS Subscribers]
        SRV[ROS Services]
    end

    subgraph "ROS 2"
        T1[/joint_states]
        T2[/cmd_vel]
        T3[/camera/image]
        N[ROS 2 Nodes]
    end

    OG --> PUB
    OG --> SUB
    OG --> SRV
    PUB --> T1
    PUB --> T3
    T2 --> SUB
    T1 --> N
    T3 --> N
    N --> T2

    style OG fill:#76b900,color:white
    style N fill:#c8e6c9
```

## Setting Up ROS 2 Bridge

### Enable ROS 2 Extension

```python
from omni.isaac.kit import SimulationApp

# Must enable ROS 2 bridge before creating app
CONFIG = {
    "headless": False,
    "renderer": "RayTracedLighting",
}

simulation_app = SimulationApp(CONFIG)

# Enable ROS 2 extensions
from omni.isaac.core.utils.extensions import enable_extension
enable_extension("omni.isaac.ros2_bridge")

# Now ROS 2 is available
import rclpy
rclpy.init()
```

### OmniGraph Setup for ROS 2

```python
import omni.graph.core as og

def create_ros_bridge_graph():
    """Create OmniGraph for ROS 2 communication."""

    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": "/World/ROS_Bridge", "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("PublishJointState", "omni.isaac.ros2_bridge.ROS2PublishJointState"),
                ("SubscribeJointState", "omni.isaac.ros2_bridge.ROS2SubscribeJointState"),
                ("PublishClock", "omni.isaac.ros2_bridge.ROS2PublishClock"),
            ],
            keys.SET_VALUES: [
                ("PublishJointState.inputs:targetPrim", "/World/Humanoid"),
                ("PublishJointState.inputs:topicName", "joint_states"),
                ("SubscribeJointState.inputs:topicName", "joint_commands"),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "SubscribeJointState.inputs:execIn"),
            ],
        }
    )
```

## Publishing Sensor Data

### Joint State Publisher

```python
from omni.isaac.core.utils.stage import get_current_stage
from omni.isaac.core.articulations import Articulation
import omni.isaac.ros2_bridge as ros2_bridge


class JointStatePublisher:
    """Publish joint states to ROS 2."""

    def __init__(self, robot_prim_path: str, topic_name: str = "/joint_states"):
        self.robot_prim_path = robot_prim_path
        self.topic_name = topic_name

        # Create the publisher using OmniGraph
        self._setup_publisher()

    def _setup_publisher(self):
        """Set up OmniGraph publisher node."""
        import omni.graph.core as og

        keys = og.Controller.Keys
        og.Controller.edit(
            {"graph_path": "/World/JointStateGraph", "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    ("tick", "omni.graph.action.OnPlaybackTick"),
                    ("publisher", "omni.isaac.ros2_bridge.ROS2PublishJointState"),
                ],
                keys.SET_VALUES: [
                    ("publisher.inputs:targetPrim", self.robot_prim_path),
                    ("publisher.inputs:topicName", self.topic_name),
                ],
                keys.CONNECT: [
                    ("tick.outputs:tick", "publisher.inputs:execIn"),
                ],
            }
        )
```

### Camera Publisher

```python
def create_camera_publisher(camera_prim_path: str, topic_name: str = "/camera/image"):
    """Create camera image publisher."""
    import omni.graph.core as og

    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": "/World/CameraGraph", "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("tick", "omni.graph.action.OnPlaybackTick"),
                ("camera_helper", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
            ],
            keys.SET_VALUES: [
                ("camera_helper.inputs:cameraPrim", camera_prim_path),
                ("camera_helper.inputs:topicName", topic_name),
                ("camera_helper.inputs:type", "rgb"),
                ("camera_helper.inputs:frameId", "camera_optical_frame"),
            ],
            keys.CONNECT: [
                ("tick.outputs:tick", "camera_helper.inputs:execIn"),
            ],
        }
    )


def create_depth_publisher(camera_prim_path: str, topic_name: str = "/camera/depth"):
    """Create depth image publisher."""
    import omni.graph.core as og

    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": "/World/DepthGraph", "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("tick", "omni.graph.action.OnPlaybackTick"),
                ("depth_helper", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
            ],
            keys.SET_VALUES: [
                ("depth_helper.inputs:cameraPrim", camera_prim_path),
                ("depth_helper.inputs:topicName", topic_name),
                ("depth_helper.inputs:type", "depth"),
                ("depth_helper.inputs:frameId", "camera_depth_frame"),
            ],
            keys.CONNECT: [
                ("tick.outputs:tick", "depth_helper.inputs:execIn"),
            ],
        }
    )
```

### IMU Publisher

```python
def create_imu_publisher(imu_prim_path: str, topic_name: str = "/imu/data"):
    """Create IMU data publisher."""
    import omni.graph.core as og

    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": "/World/IMUGraph", "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("tick", "omni.graph.action.OnPlaybackTick"),
                ("imu_sensor", "omni.isaac.sensor.IsaacReadIMU"),
                ("imu_publisher", "omni.isaac.ros2_bridge.ROS2PublishImu"),
            ],
            keys.SET_VALUES: [
                ("imu_sensor.inputs:imuPrim", imu_prim_path),
                ("imu_publisher.inputs:topicName", topic_name),
                ("imu_publisher.inputs:frameId", "imu_link"),
            ],
            keys.CONNECT: [
                ("tick.outputs:tick", "imu_sensor.inputs:execIn"),
                ("imu_sensor.outputs:execOut", "imu_publisher.inputs:execIn"),
                ("imu_sensor.outputs:angVel", "imu_publisher.inputs:angularVelocity"),
                ("imu_sensor.outputs:linAcc", "imu_publisher.inputs:linearAcceleration"),
                ("imu_sensor.outputs:orientation", "imu_publisher.inputs:orientation"),
            ],
        }
    )
```

## Subscribing to Commands

### Joint Command Subscriber

```python
class JointCommandSubscriber:
    """Subscribe to joint commands from ROS 2."""

    def __init__(self, robot: Articulation, topic_name: str = "/joint_commands"):
        self.robot = robot
        self.topic_name = topic_name
        self._setup_subscriber()

    def _setup_subscriber(self):
        """Set up OmniGraph subscriber node."""
        import omni.graph.core as og

        keys = og.Controller.Keys
        og.Controller.edit(
            {"graph_path": "/World/JointCommandGraph", "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    ("tick", "omni.graph.action.OnPlaybackTick"),
                    ("subscriber", "omni.isaac.ros2_bridge.ROS2SubscribeJointState"),
                    ("articulation", "omni.isaac.core_nodes.IsaacArticulationController"),
                ],
                keys.SET_VALUES: [
                    ("subscriber.inputs:topicName", self.topic_name),
                    ("articulation.inputs:targetPrim", self.robot.prim_path),
                    ("articulation.inputs:robotPath", self.robot.prim_path),
                ],
                keys.CONNECT: [
                    ("tick.outputs:tick", "subscriber.inputs:execIn"),
                    ("subscriber.outputs:execOut", "articulation.inputs:execIn"),
                    ("subscriber.outputs:jointNames", "articulation.inputs:jointNames"),
                    ("subscriber.outputs:positionCommand", "articulation.inputs:positionCommand"),
                    ("subscriber.outputs:velocityCommand", "articulation.inputs:velocityCommand"),
                    ("subscriber.outputs:effortCommand", "articulation.inputs:effortCommand"),
                ],
            }
        )
```

### Twist Command Subscriber (for mobile base)

```python
def create_twist_subscriber(robot_prim_path: str, topic_name: str = "/cmd_vel"):
    """Create Twist command subscriber for velocity control."""
    import omni.graph.core as og

    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": "/World/TwistGraph", "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("tick", "omni.graph.action.OnPlaybackTick"),
                ("subscriber", "omni.isaac.ros2_bridge.ROS2SubscribeTwist"),
                ("diff_controller", "omni.isaac.wheeled_robots.DifferentialController"),
            ],
            keys.SET_VALUES: [
                ("subscriber.inputs:topicName", topic_name),
                ("diff_controller.inputs:wheelRadius", 0.05),
                ("diff_controller.inputs:wheelDistance", 0.3),
            ],
            keys.CONNECT: [
                ("tick.outputs:tick", "subscriber.inputs:execIn"),
                ("subscriber.outputs:linearVelocity", "diff_controller.inputs:linearVelocity"),
                ("subscriber.outputs:angularVelocity", "diff_controller.inputs:angularVelocity"),
            ],
        }
    )
```

## Complete Integration Example

### Humanoid ROS 2 Bridge

```python
#!/usr/bin/env python3
"""Complete Isaac Sim - ROS 2 integration for humanoid robot."""

from omni.isaac.kit import SimulationApp

CONFIG = {"headless": False, "renderer": "RayTracedLighting"}
simulation_app = SimulationApp(CONFIG)

from omni.isaac.core import World
from omni.isaac.core.utils.extensions import enable_extension
from omni.isaac.core.articulations import Articulation
import omni.graph.core as og
import numpy as np

# Enable ROS 2
enable_extension("omni.isaac.ros2_bridge")


class HumanoidROS2Bridge:
    """Complete ROS 2 bridge for humanoid robot in Isaac Sim."""

    def __init__(self, robot_prim_path: str):
        self.robot_prim_path = robot_prim_path
        self.world = World(stage_units_in_meters=1.0)

        # Add ground and robot
        self.world.scene.add_default_ground_plane()
        self._import_robot()

        # Setup ROS 2 communication
        self._setup_ros2_bridge()

    def _import_robot(self):
        """Import humanoid URDF."""
        from omni.isaac.urdf import _urdf

        urdf_interface = _urdf.acquire_urdf_interface()
        import_config = _urdf.ImportConfig()
        import_config.fix_base = False
        import_config.default_drive_type = _urdf.UrdfJointTargetType.JOINT_DRIVE_POSITION

        # Import URDF (adjust path as needed)
        # urdf_interface.import_robot(...)

        self.robot = Articulation(self.robot_prim_path)

    def _setup_ros2_bridge(self):
        """Configure all ROS 2 publishers and subscribers."""
        keys = og.Controller.Keys

        # Main ROS bridge graph
        og.Controller.edit(
            {"graph_path": "/World/ROS2Bridge", "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    # Tick node
                    ("tick", "omni.graph.action.OnPlaybackTick"),

                    # Publishers
                    ("pub_joint_state", "omni.isaac.ros2_bridge.ROS2PublishJointState"),
                    ("pub_clock", "omni.isaac.ros2_bridge.ROS2PublishClock"),
                    ("pub_tf", "omni.isaac.ros2_bridge.ROS2PublishTransformTree"),

                    # Subscribers
                    ("sub_joint_cmd", "omni.isaac.ros2_bridge.ROS2SubscribeJointState"),

                    # Articulation controller
                    ("articulation_ctrl", "omni.isaac.core_nodes.IsaacArticulationController"),
                ],
                keys.SET_VALUES: [
                    # Joint state publisher
                    ("pub_joint_state.inputs:targetPrim", self.robot_prim_path),
                    ("pub_joint_state.inputs:topicName", "joint_states"),

                    # TF publisher
                    ("pub_tf.inputs:targetPrims", [self.robot_prim_path]),

                    # Joint command subscriber
                    ("sub_joint_cmd.inputs:topicName", "joint_commands"),

                    # Articulation controller
                    ("articulation_ctrl.inputs:targetPrim", self.robot_prim_path),
                    ("articulation_ctrl.inputs:robotPath", self.robot_prim_path),
                ],
                keys.CONNECT: [
                    # Publishers
                    ("tick.outputs:tick", "pub_joint_state.inputs:execIn"),
                    ("tick.outputs:tick", "pub_clock.inputs:execIn"),
                    ("tick.outputs:tick", "pub_tf.inputs:execIn"),

                    # Subscriber to controller
                    ("tick.outputs:tick", "sub_joint_cmd.inputs:execIn"),
                    ("sub_joint_cmd.outputs:execOut", "articulation_ctrl.inputs:execIn"),
                    ("sub_joint_cmd.outputs:jointNames", "articulation_ctrl.inputs:jointNames"),
                    ("sub_joint_cmd.outputs:positionCommand", "articulation_ctrl.inputs:positionCommand"),
                ],
            }
        )

    def run(self):
        """Run the simulation loop."""
        self.world.reset()
        self.robot.initialize()

        while simulation_app.is_running():
            self.world.step(render=True)

        simulation_app.close()


if __name__ == "__main__":
    bridge = HumanoidROS2Bridge("/World/Humanoid")
    bridge.run()
```

## Debugging Tips

### Verify ROS 2 Topics

```bash
# In a separate terminal with ROS 2 sourced
ros2 topic list
ros2 topic echo /joint_states
ros2 topic hz /joint_states
```

### Common Issues

| Issue | Solution |
|-------|----------|
| No topics visible | Check ROS 2 bridge extension is enabled |
| Low publish rate | Verify simulation is playing (not paused) |
| Wrong frame IDs | Check TF tree and frame_id settings |
| Command not applied | Verify articulation controller connections |

## Exercises

### Exercise 1: Basic Bridge Setup

1. Create an Isaac Sim scene with your humanoid
2. Set up joint state publisher
3. Verify data in ROS 2 using `ros2 topic echo`

### Exercise 2: Bidirectional Control

1. Add joint command subscriber
2. Create a ROS 2 node that commands joint positions
3. Verify robot moves in Isaac Sim

### Exercise 3: Sensor Integration

1. Add camera and IMU publishers
2. Visualize camera in RViz
3. Plot IMU data using `rqt_plot`

## Assessment Questions

1. **How does OmniGraph enable ROS 2 communication in Isaac Sim?**

2. **What is the difference between publishing at simulation rate vs. fixed rate?**

3. **How do you ensure proper TF tree configuration for ROS 2 navigation?**

4. **What debugging steps would you take if joint commands aren't being applied?**

## Summary

This chapter covered Isaac ROS integration:

- **OmniGraph** provides the execution framework for ROS 2 nodes
- **Publishers** send sensor data and robot state to ROS 2
- **Subscribers** receive commands from ROS 2 nodes
- **Debugging** requires checking both Isaac Sim and ROS 2 sides

Next, we'll explore Visual SLAM for humanoid localization.

---

**Next**: [Visual SLAM](./vslam)
