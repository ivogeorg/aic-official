---
generated_by: repo-deep-dive workflow
date: 2026-04-22
depth: deep
---

# API Reference — AI for Industry Challenge (AIC) Toolkit

> **Runtime:** ROS 2 Kilted Kaiju  
> **Simulation:** Gazebo (via ros_gz_bridge)  
> **Policy language:** Python 3 (dynamically loaded into `aic_model`)

This document exhaustively covers every ROS 2 topic, service, action, message type, Python SDK export, CLI interface, and configuration contract exposed by the AIC toolkit.

---

## Table of Contents

1. [ROS 2 Topic API](#1-ros-2-topic-api)
   - 1.1 [Sensor / State Input Topics](#11-sensor--state-input-topics)
   - 1.2 [Controller Command Output Topics](#12-controller-command-output-topics)
   - 1.3 [Scoring / Internal Topics](#13-scoring--internal-topics)
2. [ROS 2 Service API](#2-ros-2-service-api)
3. [ROS 2 Action API](#3-ros-2-action-api)
4. [Message Type Reference](#4-message-type-reference)
   - 4.1 [aic_task_interfaces/msg/Task](#41-aic_task_interfacesmsgtask)
   - 4.2 [aic_model_interfaces/msg/Observation](#42-aic_model_interfacesmsgobservation)
   - 4.3 [aic_control_interfaces/msg/MotionUpdate](#43-aic_control_interfacesmsgmotionupdate)
   - 4.4 [aic_control_interfaces/msg/JointMotionUpdate](#44-aic_control_interfacesmsgjointmotionupdate)
   - 4.5 [aic_control_interfaces/msg/ControllerState](#45-aic_control_interfacesmsgcontrollerstate)
   - 4.6 [aic_control_interfaces/msg/TargetMode](#46-aic_control_interfacesmsgtargetmode)
   - 4.7 [aic_control_interfaces/msg/TrajectoryGenerationMode](#47-aic_control_interfacesmsgtrajectory_generationmode)
5. [Service Type Reference](#5-service-type-reference)
   - 5.1 [aic_control_interfaces/srv/ChangeTargetMode](#51-aic_control_interfacessvr-changetargetmode)
   - 5.2 [aic_engine_interfaces/srv/ResetJoints](#52-aic_engine_interfacessvrresetjoints)
   - 5.3 [aic_training_interfaces/srv/ExpandXacro](#53-aic_training_interfacessvrexpandxacro)
   - 5.4 [std_srvs/srv/Trigger — tare_force_torque_sensor](#54-std_srvssrvtrigger--tare_force_torque_sensor)
   - 5.5 [std_srvs/srv/Empty — cancel_task](#55-std_srvssrvempty--cancel_task)
6. [Action Type Reference](#6-action-type-reference)
   - 6.1 [aic_task_interfaces/action/InsertCable](#61-aic_task_interfacesactioninsertcable)
7. [SDK / Library API (Python)](#7-sdk--library-api-python)
   - 7.1 [aic_model.policy.Policy](#71-aic_modelpolicypolicy)
   - 7.2 [aic_model.policy.MoveRobotCallback](#72-aic_modelpolicymoverobotcallback)
   - 7.3 [aic_model.policy.GetObservationCallback](#73-aic_modelpolicygetobservationcallback)
   - 7.4 [aic_model.policy.SendFeedbackCallback](#74-aic_modelpolicysendFeedbackcallback)
   - 7.5 [aic_model.aic_model.AicModel (LifecycleNode)](#75-aic_modelaic_modelaicmodel-lifecyclenode)
8. [CLI Interface](#8-cli-interface)
9. [Events / Streaming](#9-events--streaming)
10. [Authentication & Authorization](#10-authentication--authorization)
11. [Error Reference](#11-error-reference)
12. [Engine Configuration Schema](#12-engine-configuration-schema)
13. [Controller Configuration Parameters](#13-controller-configuration-parameters)

---

## 1. ROS 2 Topic API

### 1.1 Sensor / State Input Topics

These topics carry sensory data into the participant's policy. They are all published by the evaluation component (not by the participant).

| Topic | Message Type | Rate | Description |
|-------|-------------|------|-------------|
| `/left_camera/image` | `sensor_msgs/msg/Image` | Lazily published | Rectified image from left wrist camera |
| `/left_camera/camera_info` | `sensor_msgs/msg/CameraInfo` | Lazily published | Calibration data for left wrist camera |
| `/center_camera/image` | `sensor_msgs/msg/Image` | Lazily published | Rectified image from center wrist camera |
| `/center_camera/camera_info` | `sensor_msgs/msg/CameraInfo` | Lazily published | Calibration data for center wrist camera |
| `/right_camera/image` | `sensor_msgs/msg/Image` | Lazily published | Rectified image from right wrist camera |
| `/right_camera/camera_info` | `sensor_msgs/msg/CameraInfo` | Lazily published | Calibration data for right wrist camera |
| `/fts_broadcaster/wrench` | `geometry_msgs/msg/WrenchStamped` | 50 Hz | 6-DOF force/torque at the wrist |
| `/joint_states` | `sensor_msgs/msg/JointState` | ≥500 Hz (controller rate) | Arm joint positions, velocities, efforts |
| `/gripper_state` | `sensor_msgs/msg/JointState` | — | End-effector/gripper joint state |
| `/tf` | `tf2_msgs/msg/TFMessage` | Dynamic | Dynamic coordinate frame transforms |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | Latched | Static coordinate frame transforms |
| `/aic_controller/controller_state` | `aic_control_interfaces/msg/ControllerState` | 500 Hz | Real-time TCP pose, velocity, tracking error, FTS tare |
| `observations` | `aic_model_interfaces/msg/Observation` | 20 Hz | Time-synchronized composite snapshot (published by `aic_adapter`) |

> **Source:** `aic_interfaces/aic_control_interfaces/`, `docs/aic_interfaces.md:34-64`, `aic_bringup/config/ros_gz_bridge_config.yaml`, `aic_adapter/src/aic_adapter.cpp:58-100`

The `observations` topic is a composite message assembled by `aic_adapter` at 20 Hz by synchronizing the raw sensor streams above. The joint ordering in `observations.joint_states` is normalized to:

```
[shoulder_pan_joint, shoulder_lift_joint, elbow_joint,
 wrist_1_joint, wrist_2_joint, wrist_3_joint, gripper/left_finger_joint]
```
> Source: `aic_adapter/src/aic_adapter.cpp:80-86`

---

### 1.2 Controller Command Output Topics

These topics are **published by the participant policy** to command the robot. Only one mode is active at a time (Cartesian or joint); the mode must be selected via the `/aic_controller/change_target_mode` service before commands are accepted.

| Topic | Message Type | Max Rate | Description |
|-------|-------------|---------|-------------|
| `/aic_controller/pose_commands` | `aic_control_interfaces/msg/MotionUpdate` | 10–30 Hz (recommended) | Cartesian-space position or velocity target |
| `/aic_controller/joint_commands` | `aic_control_interfaces/msg/JointMotionUpdate` | 10–30 Hz (recommended) | Joint-space position or velocity target |

> **Source:** `docs/aic_interfaces.md:69-79`, `aic_model/aic_model/aic_model.py:107-112`

The controller bridges these 10–30 Hz policy commands to the robot hardware at 500 Hz with safety clamping and impedance control.

---

### 1.3 Scoring / Internal Topics

These topics are consumed by the evaluation/scoring system. Participants do not publish to them.

| Topic | Message Type | Source | Description |
|-------|-------------|--------|-------------|
| `/scoring/tf` | `tf2_msgs/msg/TFMessage` | Gazebo bridge (cable_0..4/pose, task_board/pose_static) | Ground-truth pose of cable links and task board for scoring |
| `/scoring/insertion_event` | `std_msgs/msg/String` | Gazebo bridge (cable_0..1/insertion_event) | Event fired by `ScoringPlugin` when a plug reaches insertion threshold |
| `/aic/gazebo/contacts/off_limit` | `ros_gz_interfaces/msg/Contacts` | Gazebo `OffLimitContactsPlugin` | Contact events with off-limit items on the task board |

> **Source:** `aic_bringup/config/ros_gz_bridge_config.yaml`, `aic_engine/config/sample_config.yaml:5-44`

---

## 2. ROS 2 Service API

All services listed below and their endpoints:

| Service Name | Type | Provider | Available to Participant |
|---|---|---|---|
| `/aic_controller/change_target_mode` | `aic_control_interfaces/srv/ChangeTargetMode` | `aic_controller` | Yes |
| `/aic_controller/tare_force_torque_sensor` | `std_srvs/srv/Trigger` | `aic_controller` | Training only (disabled during eval) |
| `cancel_task` | `std_srvs/srv/Empty` | `aic_model` | Yes (cancels active InsertCable goal) |
| `/expand_xacro` | `aic_training_interfaces/srv/ExpandXacro` | evaluation env | Training only |
| `/aic_engine/reset_joints` (internal) | `aic_engine_interfaces/srv/ResetJoints` | `aic_engine` | No (internal) |

> **Source:** `docs/aic_interfaces.md:83-91`, `aic_model/aic_model/aic_model.py:86-88`, `aic_engine/src/aic_engine.hpp:365`

---

## 3. ROS 2 Action API

| Action Name | Type | Server | Description |
|---|---|---|---|
| `/insert_cable` | `aic_task_interfaces/action/InsertCable` | `aic_model` | Triggers autonomous cable insertion; called by `aic_engine` |

> **Source:** `aic_interfaces/aic_task_interfaces/action/InsertCable.action`, `aic_model/aic_model/aic_model.py:97-106`, `docs/aic_interfaces.md:52-56`

---

## 4. Message Type Reference

### 4.1 `aic_task_interfaces/msg/Task`

> **File:** `aic_interfaces/aic_task_interfaces/msg/Task.msg`

Describes a single cable insertion task sent as the goal payload of the `InsertCable` action.

| Field | Type | Description |
|-------|------|-------------|
| `id` | `string` | Unique task identifier |
| `cable_type` | `string` | Type of cable, e.g. `"sfp_sc"` |
| `cable_name` | `string` | Name of the cable entity, e.g. `"sfp_sc"` |
| `plug_type` | `string` | Type of plug, e.g. `"sfp"` or `"sc"` |
| `plug_name` | `string` | Name of the plug entity, e.g. `"sfp_tip"`, `"sc_tip"` |
| `port_type` | `string` | Type of port, e.g. `"sfp"` or `"sc"` |
| `port_name` | `string` | Name of the port on the module, e.g. `"sfp_port_0"`, `"sc_port_base"` |
| `target_module_name` | `string` | Name of the module/component, e.g. `"nic_card_mount_0"`, `"sc_port_1"` |
| `time_limit` | `uint64` | Seconds from task receipt by which the task must complete |

**Example (from `aic_engine/config/sample_config.yaml:142-150`):**
```yaml
cable_type: "sfp_sc"
cable_name: "cable_0"
plug_type: "sfp"
plug_name: "sfp_tip"
port_type: "sfp"
port_name: "sfp_port_0"
target_module_name: "nic_card_mount_0"
time_limit: 180
```

---

### 4.2 `aic_model_interfaces/msg/Observation`

> **File:** `aic_interfaces/aic_model_interfaces/msg/Observation.msg`

A time-synchronized snapshot of the sensor suite, delivered at 20 Hz by `aic_adapter`.

| Field | Type | Description |
|-------|------|-------------|
| `left_image` | `sensor_msgs/Image` | Image from left wrist camera |
| `left_camera_info` | `sensor_msgs/CameraInfo` | Calibration for left camera |
| `center_image` | `sensor_msgs/Image` | Image from center wrist camera |
| `center_camera_info` | `sensor_msgs/CameraInfo` | Calibration for center camera |
| `right_image` | `sensor_msgs/Image` | Image from right wrist camera |
| `right_camera_info` | `sensor_msgs/CameraInfo` | Calibration for right camera |
| `wrist_wrench` | `geometry_msgs/WrenchStamped` | 6-DOF force/torque at the wrist |
| `joint_states` | `sensor_msgs/JointState` | Arm + gripper joint state (7 joints, normalized order) |
| `controller_state` | `aic_control_interfaces/ControllerState` | Real-time TCP pose, velocity, tracking error, FTS tare |

**Accessing image timestamp (example from `WaveArm.py:61-63`):**
```python
t = (observation.center_image.header.stamp.sec
     + observation.center_image.header.stamp.nanosec / 1e9)
```

---

### 4.3 `aic_control_interfaces/msg/MotionUpdate`

> **File:** `aic_interfaces/aic_control_interfaces/msg/MotionUpdate.msg`  
> **Published to:** `/aic_controller/pose_commands`

Cartesian-space motion command to the robot arm.

| Field | Type | Description |
|-------|------|-------------|
| `header` | `std_msgs/Header` | `frame_id` must be `"base_link"` (global frame) or `"gripper/tcp"` (TCP frame); `stamp` should be `now()` |
| `pose` | `geometry_msgs/Pose` | Target Cartesian pose for TCP. Used when `trajectory_generation_mode` is `MODE_POSITION`. If `frame_id` is `"gripper/tcp"`, interpreted as offset from current position |
| `velocity` | `geometry_msgs/Twist` | Target TCP velocity. Used when `trajectory_generation_mode` is `MODE_VELOCITY`. Relative to `frame_id` |
| `target_stiffness` | `float64[36]` | 6×6 stiffness matrix, row-major. Controls how strongly the robot resists deviating from the target. Higher = stiffer |
| `target_damping` | `float64[36]` | 6×6 damping matrix, row-major. Reduces oscillations. Tuned relative to stiffness |
| `feedforward_wrench_at_tip` | `geometry_msgs/Wrench` | Optional external force/torque applied at TCP. Useful for constant contact forces |
| `wrench_feedback_gains_at_tip` | `float64[6]` | Force/torque feedback gains. Must be in `[0, 0.95]` to avoid instability |
| `trajectory_generation_mode` | `TrajectoryGenerationMode` | `MODE_POSITION` (follows `pose`) or `MODE_VELOCITY` (follows `velocity`). `MODE_UNSPECIFIED` causes message to be ignored |

**Default values from `Policy.set_pose_target()` (`aic_model/aic_model/policy.py:94-138`):**
```python
stiffness = [90.0, 90.0, 90.0, 50.0, 50.0, 50.0]   # diagonal
damping    = [50.0, 50.0, 50.0, 20.0, 20.0, 20.0]   # diagonal
feedforward_wrench = Wrench(force=(0,0,0), torque=(0,0,0))
wrench_feedback_gains = [0.5, 0.5, 0.5, 0.0, 0.0, 0.0]
trajectory_generation_mode = MODE_POSITION
```

**CLI example (position target, `docs/aic_controller.md:123-161`):**
```bash
ros2 service call /aic_controller/change_target_mode \
  aic_control_interfaces/srv/ChangeTargetMode "{target_mode: {mode: 1}}"

ros2 topic pub --once /aic_controller/pose_commands \
  aic_control_interfaces/msg/MotionUpdate "{
    header: { frame_id: 'base_link' },
    pose: {
      position: {x: -0.501, y: -0.175, z: 0.2},
      orientation: {x: 0.7071068, y: 0.7071068, z: 0.0, w: 0.0}
    },
    target_stiffness: [85,0,0,0,0,0, 0,85,0,0,0,0, 0,0,85,0,0,0,
                       0,0,0,85,0,0, 0,0,0,0,85,0, 0,0,0,0,0,85],
    target_damping:   [75,0,0,0,0,0, 0,75,0,0,0,0, 0,0,75,0,0,0,
                       0,0,0,75,0,0, 0,0,0,0,75,0, 0,0,0,0,0,75],
    feedforward_wrench_at_tip: {force: {x:0,y:0,z:0}, torque: {x:0,y:0,z:0}},
    wrench_feedback_gains_at_tip: [0,0,0,0,0,0],
    trajectory_generation_mode: {mode: 2}
  }"
```

**CLI example (velocity target, `docs/aic_controller.md:164-192`):**
```bash
ros2 topic pub --once /aic_controller/pose_commands \
  aic_control_interfaces/msg/MotionUpdate "{
    header: { frame_id: 'gripper/tcp' },
    velocity: { linear: {x: 0.025, y: 0.0, z: 0.0},
                angular: {x: 0.0, y: 0.0, z: 0.25} },
    target_stiffness: [85,0,...],
    target_damping: [75,0,...],
    trajectory_generation_mode: {mode: 1}
  }"
```

---

### 4.4 `aic_control_interfaces/msg/JointMotionUpdate`

> **File:** `aic_interfaces/aic_control_interfaces/msg/JointMotionUpdate.msg`  
> **Published to:** `/aic_controller/joint_commands`

Joint-space motion command to the robot arm.

| Field | Type | Description |
|-------|------|-------------|
| `target_state` | `trajectory_msgs/JointTrajectoryPoint` | Target joint values. `positions` used with `MODE_POSITION`; `velocities` used with `MODE_VELOCITY` |
| `target_stiffness` | `float64[]` | Per-joint stiffness. Array length must match joint count (6) |
| `target_damping` | `float64[]` | Per-joint damping. Array length must match joint count (6) |
| `trajectory_generation_mode` | `TrajectoryGenerationMode` | `MODE_POSITION` or `MODE_VELOCITY` |
| `target_feedforward_torque` | `float64[]` | Optional per-joint feedforward torque |

**CLI example (position target, `docs/aic_controller.md:211-224`):**
```bash
ros2 service call /aic_controller/change_target_mode \
  aic_control_interfaces/srv/ChangeTargetMode "{target_mode: {mode: 2}}"

ros2 topic pub --once /aic_controller/joint_commands \
  aic_control_interfaces/msg/JointMotionUpdate "{
    target_state: { positions: [0.0, -1.57, -1.57, -1.57, 1.57, 0] },
    target_stiffness: [85.0, 85.0, 85.0, 85.0, 85.0, 85.0],
    target_damping: [75.0, 75.0, 75.0, 75.0, 75.0, 75.0],
    trajectory_generation_mode: {mode: 2}
  }"
```

**CLI example (velocity target, `docs/aic_controller.md:226-234`):**
```bash
ros2 topic pub --once /aic_controller/joint_commands \
  aic_control_interfaces/msg/JointMotionUpdate "{
    target_state: { velocities: [0.025, 0.025, 0.025, 0.025, 0.025, 0.025] },
    target_stiffness: [85.0, 85.0, 85.0, 85.0, 85.0, 85.0],
    target_damping: [75.0, 75.0, 75.0, 75.0, 75.0, 75.0],
    trajectory_generation_mode: {mode: 1}
  }"
```

---

### 4.5 `aic_control_interfaces/msg/ControllerState`

> **File:** `aic_interfaces/aic_control_interfaces/msg/ControllerState.msg`  
> **Published on:** `/aic_controller/controller_state` at 500 Hz

Real-time state feedback from the controller.

| Field | Type | Description |
|-------|------|-------------|
| `header` | `std_msgs/Header` | Timestamp and frame_id |
| `tcp_pose` | `geometry_msgs/Pose` | Current TCP position and orientation |
| `tcp_velocity` | `geometry_msgs/Twist` | Current TCP linear and angular velocity |
| `reference_tcp_pose` | `geometry_msgs/Pose` | Target TCP pose being tracked |
| `tcp_error` | `float64[6]` | Pose error from current to target TCP: `[x, y, z, rx, ry, rz]` |
| `reference_joint_state` | `trajectory_msgs/JointTrajectoryPoint` | Reference joint commands computed by controller |
| `target_mode` | `TargetMode` | Currently active target mode (`MODE_CARTESIAN` or `MODE_JOINT`) |
| `fts_tare_offset` | `geometry_msgs/WrenchStamped` | Tare offset applied to force-torque readings; `header.frame_id` gives the reference frame |

This message is also available inside every `Observation` as `controller_state`.

---

### 4.6 `aic_control_interfaces/msg/TargetMode`

> **File:** `aic_interfaces/aic_control_interfaces/msg/TargetMode.msg`

Enum-style message selecting the active control mode.

| Constant | Value | Meaning |
|----------|-------|---------|
| `MODE_UNSPECIFIED` | `0` | No mode set; controller will ignore commands |
| `MODE_CARTESIAN` | `1` | Accept `MotionUpdate` on `/aic_controller/pose_commands` |
| `MODE_JOINT` | `2` | Accept `JointMotionUpdate` on `/aic_controller/joint_commands` |

**Field:**

| Field | Type | Description |
|-------|------|-------------|
| `mode` | `uint8` | One of the constants above |

**Default at startup:** `MODE_CARTESIAN` (set by `aic_bringup/config/aic_ros2_controllers.yaml:29`)

---

### 4.7 `aic_control_interfaces/msg/TrajectoryGenerationMode`

> **File:** `aic_interfaces/aic_control_interfaces/msg/TrajectoryGenerationMode.msg`

Selects how a motion target is interpreted within `MotionUpdate` or `JointMotionUpdate`.

| Constant | Value | Meaning |
|----------|-------|---------|
| `MODE_UNSPECIFIED` | `0` | Message is ignored |
| `MODE_VELOCITY` | `1` | Use velocity fields; ignore position/pose fields |
| `MODE_POSITION` | `2` | Use position/pose fields; ignore velocity fields |

**Field:**

| Field | Type | Description |
|-------|------|-------------|
| `mode` | `uint8` | One of the constants above |

---

## 5. Service Type Reference

### 5.1 `aic_control_interfaces/srv/ChangeTargetMode`

> **File:** `aic_interfaces/aic_control_interfaces/srv/ChangeTargetMode.srv`  
> **Endpoint:** `/aic_controller/change_target_mode`

Switch the controller between Cartesian and joint target modes. Must be called before sending commands of the new type; the controller ignores commands from the non-active mode.

**Request:**

| Field | Type | Description |
|-------|------|-------------|
| `target_mode` | `TargetMode` | Desired mode: `{mode: 1}` = Cartesian, `{mode: 2}` = Joint |

**Response:**

| Field | Type | Description |
|-------|------|-------------|
| `success` | `bool` | `true` if mode was changed successfully |

**CLI:**
```bash
# Switch to Cartesian mode
ros2 service call /aic_controller/change_target_mode \
  aic_control_interfaces/srv/ChangeTargetMode "{target_mode: {mode: 1}}"

# Switch to joint mode
ros2 service call /aic_controller/change_target_mode \
  aic_control_interfaces/srv/ChangeTargetMode "{target_mode: {mode: 2}}"
```

> **Source:** `docs/aic_controller.md:70-78`, `aic_model/aic_model/aic_model.py:114-116,312-320`

---

### 5.2 `aic_engine_interfaces/srv/ResetJoints`

> **File:** `aic_interfaces/aic_engine_interfaces/srv/ResetJoints.srv`  
> **Used by:** `aic_engine` internally between trials

Resets named robot joints to specified initial positions. Used by the engine to home the robot between trials.

**Request:**

| Field | Type | Description |
|-------|------|-------------|
| `joint_names` | `string[]` | Names of joints to reset |
| `initial_positions` | `float64[]` | Target positions for each joint (radians) |

**Response:**

| Field | Type | Description |
|-------|------|-------------|
| `success` | `bool` | `true` if reset succeeded |
| `message` | `string` | Human-readable result message |

> **Source:** `aic_interfaces/aic_engine_interfaces/srv/ResetJoints.srv`, `aic_engine/src/aic_engine.hpp:365`

---

### 5.3 `aic_training_interfaces/srv/ExpandXacro`

> **File:** `aic_utils/aic_training_interfaces/srv/ExpandXacro.srv`  
> **Endpoint:** `/expand_xacro`  
> **Available:** Training environment only

Expands a xacro file from an installed ROS package into its XML representation. Enables programmatic entity spawning during training without filesystem access to the eval environment.

**Request:**

| Field | Type | Description |
|-------|------|-------------|
| `package_name` | `string` | Name of the ROS package containing the xacro file |
| `relative_path` | `string` | Path to the xacro file relative to the package root |
| `xacro_arguments` | `string[]` | Additional xacro arguments (key:=value format) |

**Response:**

| Field | Type | Description |
|-------|------|-------------|
| `success` | `bool` | `true` if xacro expansion succeeded |
| `xml` | `string` | The expanded XML string |
| `message` | `string` | Error message if `success` is `false` |

> **Source:** `aic_utils/aic_training_interfaces/srv/ExpandXacro.srv`, `docs/aic_interfaces.md:91`

---

### 5.4 `std_srvs/srv/Trigger` — `tare_force_torque_sensor`

> **Endpoint:** `/aic_controller/tare_force_torque_sensor`  
> **Available:** Training only; **disabled during evaluation**

Zeros (tares) the force-torque sensor readings. Call before each training episode (before spawning cables or starting teleoperation). The tared offset is published in `ControllerState.fts_tare_offset`.

**Request:** (empty)

**Response:**

| Field | Type | Description |
|-------|------|-------------|
| `success` | `bool` | `true` if tare succeeded |
| `message` | `string` | Result message |

**CLI:**
```bash
ros2 service call /aic_controller/tare_force_torque_sensor std_srvs/srv/Trigger
```

> **Source:** `docs/aic_controller.md:92-102`, `aic_engine/src/aic_engine.hpp:366`

---

### 5.5 `std_srvs/srv/Empty` — `cancel_task`

> **Endpoint:** `cancel_task` (relative to `aic_model` node namespace)  
> **Provider:** `aic_model` node  
> **Available to participants:** Yes

Aborts the currently active `InsertCable` action goal. If `goal_handle` is active, calls `goal_handle.abort()`. Useful for external interrupts or emergency stops.

**Request:** (empty)

**Response:** (empty)

**CLI:**
```bash
ros2 service call /cancel_task std_srvs/srv/Empty
```

**Python:**
```python
# From aic_model/test/cancel_task.py:30-36
client = node.create_client(Empty, "cancel_task")
future = client.call_async(Empty.Request())
rclpy.spin_until_future_complete(node, future, timeout_sec=1.0)
```

> **Source:** `aic_model/aic_model/aic_model.py:86-88,156-161`, `aic_model/test/cancel_task.py`

---

## 6. Action Type Reference

### 6.1 `aic_task_interfaces/action/InsertCable`

> **File:** `aic_interfaces/aic_task_interfaces/action/InsertCable.action`  
> **Action server:** `/insert_cable` (served by `aic_model`)  
> **Called by:** `aic_engine`

The primary interface triggering the autonomous cable insertion task. `aic_engine` sends a goal; the participant's `Policy.insert_cable()` is invoked.

**Goal (Request):**

| Field | Type | Description |
|-------|------|-------------|
| `task` | `aic_task_interfaces/msg/Task` | Full task descriptor (see [Task](#41-aic_task_interfacesmsgtask)) |

**Result (Response):**

| Field | Type | Description |
|-------|------|-------------|
| `success` | `bool` | `true` if the cable was successfully inserted |
| `message` | `string` | Human-readable success or failure description |

**Feedback:**

| Field | Type | Description |
|-------|------|-------------|
| `message` | `string` | Progress update string published via `send_feedback()` |

**State machine:**

The action server in `aic_model` (`aic_model/aic_model/aic_model.py:165-308`) enforces:
- **REJECT** if node is not in `active` lifecycle state
- **REJECT** if another goal is currently active (only one goal at a time)
- **ACCEPT** otherwise → launches `Policy.insert_cable()` in a background thread
- **CANCEL** via action client cancel request → `goal_handle.canceled()`
- **ABORT** via `cancel_task` service → `goal_handle.abort()`

**Cancellation:**

```bash
# Via action client cancel (standard ROS 2 action cancel)
ros2 action send_goal /insert_cable aic_task_interfaces/action/InsertCable \
  "{task: {id: 'test', cable_type: 'sfp_sc', time_limit: 60}}"
# Then cancel via separate terminal or action client API

# Via cancel_task service (immediate abort)
ros2 service call /cancel_task std_srvs/srv/Empty
```

---

## 7. SDK / Library API (Python)

The `aic_model` Python package (`aic_model/aic_model/`) provides the primary SDK for implementing policies.

### 7.1 `aic_model.policy.Policy`

> **File:** `aic_model/aic_model/policy.py:70-149`

Abstract base class that all participant policies must subclass.

```python
from aic_model.policy import Policy

class MyPolicy(Policy):
    def __init__(self, parent_node): ...
    def insert_cable(self, task, get_observation, move_robot, send_feedback) -> bool: ...
```

#### Constructor

```python
def __init__(self, parent_node)
```

| Parameter | Type | Description |
|-----------|------|-------------|
| `parent_node` | `rclpy.node.Node` | The `AicModel` lifecycle node; passed automatically at runtime |

#### `get_logger()`

```python
def get_logger(self) -> rclpy.impl.rcutils_logger.RcutilsLogger
```

Returns the ROS 2 logger from the parent node. Use for all logging output.

#### `get_clock()`

```python
def get_clock(self) -> rclpy.clock.Clock
```

Returns the node's clock (simulation-time aware when `use_sim_time:=true`).

#### `time_now()`

```python
def time_now(self) -> rclpy.time.Time
```

Returns the current time from the node's clock (sim-time aware).

> Source: `aic_model/aic_model/policy.py:82-83`

#### `sleep_for(duration_sec)`

```python
def sleep_for(self, duration_sec: float) -> None
```

Sleeps for the given duration using the node's clock (sim-time aware). Preferred over `time.sleep()` which ignores simulation time.

| Parameter | Type | Description |
|-----------|------|-------------|
| `duration_sec` | `float` | Duration to sleep in seconds |

> Source: `aic_model/aic_model/policy.py:85-87`

#### `set_pose_target(move_robot, pose, frame_id, stiffness, damping)`

```python
def set_pose_target(
    self,
    move_robot: MoveRobotCallback,
    pose: geometry_msgs.msg.Pose,
    frame_id: str = "base_link",
    stiffness: list = [90.0, 90.0, 90.0, 50.0, 50.0, 50.0],
    damping: list = [50.0, 50.0, 50.0, 20.0, 20.0, 20.0],
) -> None
```

Convenience method. Constructs a `MotionUpdate` with diagonal stiffness/damping matrices and sensible defaults, then calls `move_robot`. Intended as the simplest way to command a Cartesian pose target.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `move_robot` | `MoveRobotCallback` | — | Callback to send the motion command |
| `pose` | `geometry_msgs/Pose` | — | Target TCP position and orientation |
| `frame_id` | `str` | `"base_link"` | Reference frame. Also accepts `"gripper/tcp"` |
| `stiffness` | `list[float]` (6) | `[90,90,90,50,50,50]` | Diagonal of 6×6 stiffness matrix `[tx, ty, tz, rx, ry, rz]` |
| `damping` | `list[float]` (6) | `[50,50,50,20,20,20]` | Diagonal of 6×6 damping matrix |

Fixed internal defaults:
- `feedforward_wrench_at_tip`: zeros
- `wrench_feedback_gains_at_tip`: `[0.5, 0.5, 0.5, 0.0, 0.0, 0.0]`
- `trajectory_generation_mode`: `MODE_POSITION`

> Source: `aic_model/aic_model/policy.py:89-138`

#### `insert_cable(task, get_observation, move_robot, send_feedback)` *(abstract)*

```python
@abstractmethod
def insert_cable(
    self,
    task: aic_task_interfaces.msg.Task,
    get_observation: GetObservationCallback,
    move_robot: MoveRobotCallback,
    send_feedback: SendFeedbackCallback,
) -> bool
```

The sole abstract method. Called by `aic_model` when `aic_engine` sends an `InsertCable` action goal. Runs in a dedicated background thread.

| Parameter | Type | Description |
|-----------|------|-------------|
| `task` | `Task` | Full task descriptor including cable/plug/port names and time limit |
| `get_observation` | `GetObservationCallback` | Callable that returns the latest `Observation` (or `None` if not yet received) |
| `move_robot` | `MoveRobotCallback` | Callable to send Cartesian or joint motion commands |
| `send_feedback` | `SendFeedbackCallback` | Callable to publish progress strings as action feedback |

**Returns:** `bool` — `True` if insertion succeeded, `False` otherwise. A return value of `None` is treated as `False` with a warning.

> Source: `aic_model/aic_model/policy.py:140-149`, `aic_model/aic_model/aic_model.py:236-248`

---

### 7.2 `aic_model.policy.MoveRobotCallback`

> **File:** `aic_model/aic_model/policy.py:38-64`

A `typing.Protocol` class describing the callable signature provided to `insert_cable()` for commanding robot motion.

```python
class MoveRobotCallback(Protocol):
    def __call__(
        self,
        motion_update: aic_control_interfaces.msg.MotionUpdate = None,
        joint_motion_update: aic_control_interfaces.msg.JointMotionUpdate = None,
    ) -> None: ...
```

**Constraints:**
- Exactly one of `motion_update` or `joint_motion_update` must be provided (not both, not neither).
- If `motion_update` is provided and the current target mode is not `MODE_CARTESIAN`, the mode is automatically switched before publishing.
- If `joint_motion_update` is provided and the current target mode is not `MODE_JOINT`, the mode is automatically switched before publishing.

> Source: `aic_model/aic_model/policy.py:38-64`, `aic_model/aic_model/aic_model.py:190-229`

---

### 7.3 `aic_model.policy.GetObservationCallback`

> **File:** `aic_model/aic_model/policy.py:35`

```python
GetObservationCallback = Callable[[], Observation]
```

A callable that takes no arguments and returns the most recent `Observation` message, or `None` if no observation has been received yet.

**Usage:**
```python
observation = get_observation()
if observation is None:
    # no data yet
    return
image = observation.center_image
wrench = observation.wrist_wrench
```

---

### 7.4 `aic_model.policy.SendFeedbackCallback`

> **File:** `aic_model/aic_model/policy.py:67`

```python
SendFeedbackCallback = Callable[[str], None]
```

A callable that takes a single `str` and publishes it as the `message` field of an `InsertCable.Feedback` message on the action's feedback channel. Use for debugging and monitoring during evaluation.

**Usage:**
```python
send_feedback("approaching port")
send_feedback(f"tcp_error: {observation.controller_state.tcp_error}")
```

---

### 7.5 `aic_model.aic_model.AicModel` (LifecycleNode)

> **File:** `aic_model/aic_model/aic_model.py:53-335`

The ROS 2 Lifecycle node that hosts the participant's policy. Participants do not typically interact with this class directly; it is invoked via `ros2 run aic_model aic_model`.

**ROS 2 Parameter:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `policy` | `string` | `"WaveArm"` | Fully-qualified Python module + class name of the policy to load, e.g. `"my_package.MyPolicy"` |
| `use_sim_time` | `bool` | — | Standard ROS 2 parameter; enables simulation time |

**Lifecycle transitions handled:**

| Transition | Method | Behavior |
|-----------|--------|---------|
| `configure` | `on_configure()` | Instantiates `self._policy = PolicyClass(self)` |
| `activate` | `on_activate()` | Sets `is_active = True`; enables lifecycle publishers |
| `deactivate` | `on_deactivate()` | Sets `is_active = False`; disables lifecycle publishers |
| `cleanup` | `on_cleanup()` | Sets `self._policy = None` |
| `shutdown` | `on_shutdown()` | Destroys publishers and subscriptions |

**Internal publishers created:**

| Publisher | Topic | Type | QoS depth |
|-----------|-------|------|-----------|
| `motion_update_pub` | `/aic_controller/pose_commands` | `MotionUpdate` | 2 (lifecycle) |
| `joint_motion_update_pub` | `/aic_controller/joint_commands` | `JointMotionUpdate` | 2 (lifecycle) |

**Internal subscriptions:**

| Subscription | Topic | Type | QoS depth |
|-------------|-------|------|-----------|
| `observation_sub` | `observations` | `Observation` | 10 |

**Dynamic policy loading:** The `policy` parameter is interpreted as a dotted Python module path. The class name is derived from the last component after `.`. For example, `"my_package.ros.MyPolicy"` → loads module `my_package.ros.MyPolicy` and looks for class `MyPolicy`.

> Source: `aic_model/aic_model/aic_model.py:56-79`

---

## 8. CLI Interface

### `ros2 run aic_model aic_model`

Starts the `AicModel` lifecycle node.

**Syntax:**
```bash
ros2 run aic_model aic_model --ros-args \
  -p use_sim_time:=true \
  -p policy:=<module.ClassName>
```

**Parameters:**

| Flag | Type | Default | Description |
|------|------|---------|-------------|
| `-p policy:=<value>` | `string` | `WaveArm` | Python module + class to load as the policy |
| `-p use_sim_time:=<bool>` | `bool` | `false` | Enable simulation time |

**Examples:**
```bash
# Run with the built-in WaveArm example policy
ros2 run aic_model aic_model --ros-args -p use_sim_time:=true -p policy:=aic_example_policies.ros.WaveArm

# Run with a custom policy
ros2 run aic_model aic_model --ros-args -p use_sim_time:=true -p policy:=my_policy_node.WaveArm
```

> Source: `docs/policy.md:171`, `aic_model/aic_model/aic_model.py:56-57`

---

### `ros2 launch aic_bringup aic_gz_bringup.launch.py`

Brings up the full simulation environment (Gazebo, robot, sensors, controllers).

```bash
ros2 launch aic_bringup aic_gz_bringup.launch.py
```

---

### `ros2 launch aic_bringup spawn_cable.launch.py`

Spawns a cable entity in the simulation.

```bash
ros2 launch aic_bringup spawn_cable.launch.py
```

---

### `ros2 launch aic_bringup spawn_task_board.launch.py`

Spawns the task board in the simulation.

```bash
ros2 launch aic_bringup spawn_task_board.launch.py
```

---

### `ros2 run aic_bringup home_robot`

Sends the robot to its home pose using either `aic_controller` (Cartesian mode) or the `joint_trajectory_controller`.

```bash
ros2 run aic_bringup home_robot --ros-args \
  -p use_aic_controller:=true \
  -p controller_namespace:=aic_controller
```

**Parameters:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `use_aic_controller` | `bool` | `true` | Use `aic_controller`; if `false`, uses `FollowJointTrajectory` action |
| `controller_namespace` | `string` | `"aic_controller"` | Namespace for the controller services and topics |

> Source: `aic_bringup/scripts/home_robot.py:43-48`

---

### `ros2 run aic_bringup test_impedance`

Utility script for testing impedance control parameters.

```bash
ros2 run aic_bringup test_impedance
```

> Source: `aic_bringup/scripts/test_impedance.py` (see also `docs/aic_controller.md:195,209`)

---

### `pixi run ros2 run aic_model aic_model`

Recommended way to run within the Pixi workspace environment:

```bash
pixi shell   # Enter Pixi environment
ros2 run aic_model aic_model --ros-args -p use_sim_time:=true -p policy:=my_package.MyPolicy
```

After modifying a package:
```bash
pixi reinstall <package_name>
```

> Source: `docs/policy.md:171-175`

---

## 9. Events / Streaming

### Insertion Event

**Topic:** `/scoring/insertion_event`  
**Type:** `std_msgs/msg/String`  
**Source:** Gazebo `ScoringPlugin` (via `ros_gz_bridge`)  
**Bridges from Gazebo topics:** `/cable_0/insertion_event`, `/cable_1/insertion_event`

Published by the simulation plugin when a cable plug crosses the insertion threshold into a port. The payload is a string identifying the insertion event. This is consumed by `aic_engine` scoring logic.

> Source: `aic_bringup/config/ros_gz_bridge_config.yaml:99-112`, `aic_engine/config/sample_config.yaml:39-40`

### Off-Limit Contacts Event

**Topic:** `/aic/gazebo/contacts/off_limit`  
**Type:** `ros_gz_interfaces/msg/Contacts`  
**Source:** Gazebo `OffLimitContactsPlugin`

Published whenever the robot makes contact with off-limit items on the task board. Contacts are used in scoring to penalize invalid interactions.

> Source: `aic_bringup/config/ros_gz_bridge_config.yaml:50-55`, `aic_engine/config/sample_config.yaml:24-26`

### Action Feedback Streaming

The `InsertCable` action supports streaming feedback messages from the policy back to `aic_engine` and any action client subscribers. Feedback is published via `send_feedback(str)` inside `insert_cable()`.

**Delivery guarantees:** Best-effort; feedback is published on the ROS 2 action feedback channel. Not guaranteed to arrive if there is no active subscriber.

> Source: `aic_model/aic_model/aic_model.py:231-234`, `aic_interfaces/aic_task_interfaces/action/InsertCable.action`

### Observation Stream

**Topic:** `observations`  
**Type:** `aic_model_interfaces/msg/Observation`  
**Rate:** 20 Hz  
**Source:** `aic_adapter` node

Time-synchronized composite of all sensor streams. The adapter maintains short deques of each sensor stream and assembles the composite at 20 Hz. Participants receive this via the `get_observation()` callback inside `insert_cable()`.

---

## 10. Authentication & Authorization

The AIC toolkit uses no authentication on ROS 2 topics, services, or actions. All communication is local to the Docker network defined in `docker/docker-compose.yaml`.

**Network isolation:**
- The participant's model container communicates with the evaluation component via ROS 2 (DDS middleware). The default RMW is Zenoh, configured in `docker/aic_eval/aic_zenoh_config.json5` and `docker/aic_model/zenoh_config_model_session.sh`.
- The Zenoh router configuration (`docker/rmw_zenohd/`) establishes the bridge between the model and evaluation containers.

**Access restrictions enforced at evaluation:**
- `/aic_controller/tare_force_torque_sensor` is **disabled**; the engine calls it automatically before each trial.
- Participants cannot access ground-truth pose data (TF from Gazebo simulation internals) unless the engine is started with `--ground_truth` (training mode only).

> Source: `docs/aic_controller.md:97-102`, `aic_engine/src/aic_engine.hpp:388`

---

## 11. Error Reference

### Action Goal Rejection

The `InsertCable` action server at `/insert_cable` rejects goals in the following cases:

| Condition | Response | Source |
|-----------|----------|--------|
| `aic_model` lifecycle state is not `active` | `GoalResponse.REJECT` with error log | `aic_model/aic_model/aic_model.py:166-168` |
| Another goal is currently active | `GoalResponse.REJECT` with error log | `aic_model/aic_model/aic_model.py:170-175` |

### Action Goal Termination

| Condition | Result | `success` | `message` |
|-----------|--------|-----------|-----------|
| `policy.insert_cable()` returns `True` | `succeed()` | `true` | — |
| `policy.insert_cable()` returns `False` | `succeed()` | `false` | — |
| `policy.insert_cable()` returns `None` | `succeed()` | `false` | Warning logged |
| Action client sends cancel | `canceled()` | `false` | `"Canceled via action client"` |
| `cancel_task` service called | abort via `goal_handle.abort()` | `false` | `"Canceled via cancel_task service"` |
| `aic_model` deactivated mid-task | terminate loop | `false` | `"Canceled via cancel_task service"` |

> Source: `aic_model/aic_model/aic_model.py:249-310`

### `move_robot()` Errors

| Condition | Behavior |
|-----------|---------|
| Both `motion_update` and `joint_motion_update` provided | Logs error, returns `False` |
| Neither provided | Logs error, returns `False` |
| Mode switch service call fails | Logs error; command not published |

> Source: `aic_model/aic_model/aic_model.py:204-229`, `aic_model/aic_model/aic_model.py:312-320`

### Policy Loading Errors

| Condition | Behavior |
|-----------|---------|
| Module not found (`importlib.import_module` fails) | Fatal log; node raises exception and exits |
| Class not found in module | Fatal log; `LookupError` raised; node exits |

> Source: `aic_model/aic_model/aic_model.py:62-79`

### Controller Tracking Error Reset

If the tracking error is large and does not decrease within the configured `tracking_error.timeout` (default `2.0 s`), the controller resets the target to the current pose to prevent accumulated error from causing sudden motion when the robot moves out of collision.

> Source: `docs/aic_controller.md:25-26`, `aic_bringup/config/aic_ros2_controllers.yaml:112-116`

### Engine / Trial State Errors

The engine (`aic_engine`) tracks task outcomes:

| `TaskState` | Meaning |
|-------------|---------|
| `Uninitialized` (0) | Task not yet started |
| `TaskRequested` (1) | Goal sent to `aic_model` |
| `TaskStarted` (2) | Goal execution has begun |
| `TaskCompleted` (3) | Task succeeded |
| `TaskRejected` (4) | Goal was rejected by `aic_model` |
| `TaskFailed` (5) | Task returned `success=false` |
| `TimeLimitExceeded` (6) | `Task.time_limit` elapsed before completion |

> Source: `aic_engine/src/aic_engine.hpp:113-121`

---

## 12. Engine Configuration Schema

> **File:** `aic_engine/config/sample_config.yaml`

The `aic_engine` is configured via a YAML file. The file defines scoring topics, scene geometry, and one or more trial definitions.

### Top-Level Keys

| Key | Description |
|-----|-------------|
| `scoring.topics` | List of ROS topics to record for scoring |
| `task_board_limits` | Rail translation limits in meters for NIC, SC, and mount rails |
| `trials` | Map of trial ID → trial configuration |
| `robot.home_joint_positions` | Named joint positions for the robot's home pose |

### `scoring.topics[]`

Each entry:

| Field | Type | Description |
|-------|------|-------------|
| `topic.name` | `string` | ROS topic name |
| `topic.type` | `string` | ROS message type |
| `topic.latched` | `bool` (optional) | Whether to latch the topic |

### `task_board_limits`

| Key | Fields | Description |
|-----|--------|-------------|
| `nic_rail` | `min_translation`, `max_translation` | NIC card rail limits (meters from center) |
| `sc_rail` | `min_translation`, `max_translation` | SC port rail limits (meters from center) |
| `mount_rail` | `min_translation`, `max_translation` | Mount rail limits (meters from center) |

### `trials.<trial_id>.scene.task_board`

| Field | Description |
|-------|-------------|
| `pose.{x,y,z,roll,pitch,yaw}` | Task board world pose (meters, radians) |
| `nic_rail_<N>.entity_present` | Whether a NIC card is on rail N |
| `nic_rail_<N>.entity_name` | Name of the NIC card entity |
| `nic_rail_<N>.entity_pose.{translation,roll,pitch,yaw}` | NIC card pose on the rail |
| `sc_rail_<N>.entity_present` | Whether an SC port is on rail N |
| `sc_rail_<N>.entity_name` | SC port entity name |
| `lc_mount_rail_<N>.entity_present` | LC mount presence |
| `sfp_mount_rail_<N>.entity_present` | SFP mount presence |
| `sc_mount_rail_<N>.entity_present` | SC mount presence |

### `trials.<trial_id>.scene.cables.<cable_id>`

| Field | Description |
|-------|-------------|
| `pose.gripper_offset.{x,y,z}` | Cable attachment offset from gripper origin (meters) |
| `pose.{roll,pitch,yaw}` | Cable orientation (radians) |
| `attach_cable_to_gripper` | `true` to attach the cable to the robot gripper |
| `cable_type` | Cable type string: `"sfp_sc_cable"` or `"sfp_sc_cable_reversed"` |

### `trials.<trial_id>.tasks.<task_id>`

All fields are the same as `aic_task_interfaces/msg/Task` fields. See [Section 4.1](#41-aic_task_interfacesmsgtask).

### `robot.home_joint_positions`

Named joint positions (radians) for homing:

```yaml
robot:
  home_joint_positions:
    shoulder_pan_joint: -0.1597
    shoulder_lift_joint: -1.3542
    elbow_joint: -1.6648
    wrist_1_joint: -1.6933
    wrist_2_joint: 1.5710
    wrist_3_joint: 1.4110
```

---

## 13. Controller Configuration Parameters

> **File:** `aic_controller/src/aic_controller_parameters.yaml`  
> **Active values:** `aic_bringup/config/aic_ros2_controllers.yaml`

All parameters are under the `aic_controller` namespace.

> **Note:** The configuration is **fixed** during evaluation. All participants use the same controller settings.

### Top-Level Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `joints` | `string[]` | — | Joint names controlled: `shoulder_pan_joint`, `shoulder_lift_joint`, `elbow_joint`, `wrist_1_joint`, `wrist_2_joint`, `wrist_3_joint` |
| `control_mode` | `string` | `"impedance"` | Control law: `"impedance"` (only supported mode) |
| `target_mode` | `string` | `"cartesian"` | Default target mode: `"cartesian"` or `"joint"` |
| `control_frequency` | `double` | `500.0` | Control loop frequency in Hz |
| `enable_parameter_update_without_reactivation` | `bool` | `true` | Allow live parameter updates |
| `admittance_controller_namespace` | `string` | `"admittance_controller"` | Chained admittance controller namespace |

### `kinematics`

| Parameter | Type | Active Value | Description |
|-----------|------|-------------|-------------|
| `plugin_name` | `string` | `kinematics_interface_kdl/KinematicsInterfaceKDL` | Kinematics solver plugin |
| `plugin_package` | `string` | `kinematics_interface` | Package providing the solver |
| `base` | `string` | `base_link` | Kinematic chain base frame |
| `tip` | `string` | `gripper/tcp` | Tool center point frame |
| `alpha` | `double` | `0.0005` | Damping coefficient for Jacobian pseudo-inverse |

### `force_torque_sensor`

| Parameter | Type | Active Value | Description |
|-----------|------|-------------|-------------|
| `name` | `string` | `AtiForceTorqueSensor` | FTS name as defined in URDF |

### `clamp_to_limits` (Cartesian safety bounds)

| Parameter | Type | Active Value | Description |
|-----------|------|-------------|-------------|
| `min_translational_position` | `double[3]` | `[-5.0, -5.0, -5.0]` | Min XYZ (meters) |
| `max_translational_position` | `double[3]` | `[5.0, 5.0, 5.0]` | Max XYZ (meters) |
| `min_translational_velocity` | `double[3]` | `[-0.25, -0.25, -0.25]` | Min XYZ velocity (m/s) |
| `max_translational_velocity` | `double[3]` | `[0.25, 0.25, 0.25]` | Max XYZ velocity (m/s) |
| `min_rotation_angle` | `double[3]` | `[-π, -π, -π]` | Min rotation (radians) |
| `max_rotation_angle` | `double[3]` | `[π, π, π]` | Max rotation (radians) |
| `max_rotational_velocity` | `double` | `2.0` | Max rotation velocity (rad/s) |
| `tracking_error.timeout` | `double` | `2.0` | Seconds before tracking error triggers target reset |
| `tracking_error.min_angular_change` | `double` | `0.01` | Min angular change to consider robot moving toward target (rad) |
| `tracking_error.min_translation_change` | `double` | `0.01` | Min translation change to consider robot moving toward target (m) |
| `tracking_error.min_translation_error` | `double` | `0.2` | Min translation error to trigger clamping (m); must exceed `min_translation_change` |

### `impedance` (Cartesian impedance parameters)

| Parameter | Type | Active Value | Description |
|-----------|------|-------------|-------------|
| `gravity_compensation` | `bool` | `true` | Enable gravitational force compensation |
| `stiffness_smoothing_constant` | `double` | `0.1` | Exponential smoothing constant for stiffness matrix (0=no change, 1=no smoothing) |
| `damping_smoothing_constant` | `double` | `0.1` | Exponential smoothing constant for damping matrix |
| `activation_percentage` | `double` | `0.0` | Joint-limit activation potential range (0–1) |
| `pose_error_integrator.gain` | `double[6]` | `[0,0,0,0,0,0]` | Integral gains for pose error |
| `pose_error_integrator.bound` | `double[6]` | `[10,10,10,2,2,2]` | Integrator bounds (N for translational, Nm for rotational) |
| `maximum_wrench` | `double[6]` | `[10,10,10,10,10,10]` | Wrench magnitude limit after feedback + feedforward |
| `nullspace.target_configuration` | `double[6]` | `[0.6,-1.5708,-1.5708,-1.5708,1.5708,0.6]` | Preferred joint configuration in nullspace |
| `nullspace.stiffness` | `double[6]` | `[0,0,0,0,0,0]` | Nullspace stiffness |
| `nullspace.damping` | `double[6]` | `[10,10,10,10,10,10]` | Nullspace damping |
| `feedforward_interpolation.min_wrench` | `double[6]` | `[-40,-40,-40,-5,-5,-5]` | Minimum feedforward wrench (N/Nm) |
| `feedforward_interpolation.max_wrench` | `double[6]` | `[40,40,40,5,5,5]` | Maximum feedforward wrench (N/Nm) |
| `feedforward_interpolation.max_wrench_dot` | `double[6]` | `[500,500,500,25,25,25]` | Max rate of feedforward wrench change |
| `default_values.control_stiffness` | `double[6]` | `[75,75,75,75,75,75]` | Default diagonal stiffness values |
| `default_values.control_damping` | `double[6]` | `[35,35,35,35,35,35]` | Default diagonal damping values |
| `default_values.offset_wrench` | `double[6]` | `[0,0,0,0,0,0]` | Payload offset wrench in base frame |

### `joint_impedance` (Joint impedance parameters)

| Parameter | Type | Active Value | Description |
|-----------|------|-------------|-------------|
| `interpolator.min_value` | `double[6]` | `[-40,-40,-40,-6,-6,-6]` | Minimum feedforward torque (Nm) |
| `interpolator.max_value` | `double[6]` | `[40,40,40,6,6,6]` | Maximum feedforward torque (Nm) |
| `interpolator.max_step_size` | `double[6]` | `[1,1,1,1,1,1]` | Maximum torque interpolation step size per cycle |
| `default_values.control_stiffness` | `double[6]` | `[100,100,100,50,50,50]` | Default per-joint stiffness |
| `default_values.control_damping` | `double[6]` | `[40,40,40,15,15,15]` | Default per-joint damping |
