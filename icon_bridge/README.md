# icon_bridge

A Flowstate-ROS bridge plugin that bridges the AgentBridge and AgentBridgeJoint actions on ICON to ROS.

## Interfaces

## Quickstart
To run the bridge:

```bash
# Flowstate-ROS Bridge plugin
ros2 launch icon_bridge flowstate_ros_bridge_launch.py

# Run a test script
ros2 run icon_bridge test_icon_bridge.py
```

# todo
1. Package as flowstate service

2. Log feedback of states from the actions. Use pubsub

3. Replace task_setting configuration files .pbtxt

4. Fix test_icon_bridge.py script

5. add documentation for functions
