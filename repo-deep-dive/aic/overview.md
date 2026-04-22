# AIC Toolkit — Codebase Overview

## What It Does

The AIC Toolkit is a complete simulation-to-evaluation platform for an industrial cable-insertion robotics competition. Its concrete capabilities:

- **Trial Orchestration** — `aic_engine` runs multi-trial sessions from a YAML config: spawns task boards and cables in Gazebo, drives the participant model through ROS 2 Lifecycle states (configure → activate → task → deactivate → cleanup), records results, and writes scores to `$AIC_RESULTS_DIR` (default `$HOME/aic_results`).

- **Simulation Environment** — `aic_bringup` launches a UR5e arm with Robotiq Hand-E gripper in Gazebo Harmonic. The scene contains a modular task board with fiber-optic ports (SC, SFP, LC) on NIC cards and standalone mounts. Cables (e.g., `sfp_sc_cable`, `sfp_sc_cable_reversed`) spawn in the gripper or in free space. Environment is parameterized at launch time (robot position, board pose, cable type, ground truth poses).

- **Impedance/Force Robot Control** — `aic_controller` (a `ros2_control` plugin) runs at 500 Hz, implementing Cartesian and joint impedance control with configurable stiffness/damping matrices. It accepts high-level Cartesian or joint targets from participants and handles safety clamping and trajectory interpolation.

- **Sensor Fusion** — `aic_adapter` aggregates three camera streams (left/center/right Basler acA2440-20gc at 1152×1024, 20 FPS), wrist force-torque (ATI AXIA80-M20), joint states, and controller state into a single `Observation` message published at ~20 Hz.

- **Scoring** — Three-tier automated scoring per trial (max 100 pts):
  - *Tier 1*: Model validity — verifies required ROS topics are being published with sufficient frequency.
  - *Tier 2*: Performance — trajectory smoothness, cycle time, force/contact penalties, recorded via ROS bag.
  - *Tier 3*: Task success — cable insertion detected by `CablePlugin` (Gazebo), scored by proximity, partial insertion, and full insertion.

- **Participant Policy Framework** — `aic_model` is a ready-to-use ROS 2 Lifecycle node that dynamically imports a Python `Policy` subclass at runtime (configured by `policy_module` and `policy_class` ROS parameters). Participants implement a single `insert_cable()` method; all lifecycle, action server, and pub/sub boilerplate is handled.

- **Example Policies** — Three reference implementations in `aic_example_policies`:
  - `WaveArm` — minimal skeleton, no insertion logic.
  - `CheatCode` — uses ground-truth TF frames to compute exact target poses; for debugging only.
  - `RunACT` — loads a pretrained ACT (Action Chunking with Transformers) model from HuggingFace (`grkw/aic_act_policy`).

- **Teleoperation & Data Collection** — `aic_teleoperation` provides keyboard and SpaceMouse control of the robot in joint or Cartesian space. `lerobot_robot_aic` integrates with the LeRobot framework for recording teleoperation datasets and training policies (ACT, Diffusion Policy, etc.).

- **Alternative Simulators** — `aic_mujoco` converts the Gazebo SDF world to MuJoCo MJCF (via `sdformat_mjcf` + custom `add_cable_plugin.py`) and launches with `ros2_control`, providing the same controller interface. `aic_isaac` provides Isaac Sim integration stubs.

- **Containerized Deployment** — Docker Compose (`docker/docker-compose.yaml`) runs two services: `eval` (full Gazebo + engine evaluation stack) and `model` (participant container). Communication uses Zenoh middleware with ACL-based access control.

---

## Architecture

```
┌────────────────────────────── EVALUATION COMPONENT ──────────────────────────────┐
│                                                                                    │
│  aic_engine (C++)                                                                  │
│  ├─ Drives model lifecycle (configure/activate/deactivate/cleanup)                │
│  ├─ Sends InsertCable action goals                                                 │
│  ├─ Spawns task board + cable via Gazebo service calls                            │
│  └─ Writes score JSON to $AIC_RESULTS_DIR                                         │
│                                                                                    │
│  aic_bringup (Python launch)                                                       │
│  ├─ Gazebo Harmonic server + ROS-GZ bridge                                        │
│  ├─ robot_state_publisher, joint_state_broadcaster                                │
│  ├─ aic_controller (ros2_control plugin, 500 Hz)                                  │
│  └─ aic_adapter node                                                              │
│                                                                                    │
│  Gazebo Plugins (aic_gazebo)                                                       │
│  ├─ CablePlugin      — detachable joint FSM, insertion detection                  │
│  ├─ ScoringPlugin    — publishes scoring protobuf on update                       │
│  ├─ OffLimitContactsPlugin — publishes off-limit collision events                 │
│  └─ ResetJointsPlugin / WorldSdfGeneratorPlugin                                   │
│                                                                                    │
│  aic_scoring (C++)                                                                 │
│  ├─ ScoringTier1 — topic stats monitor (frequency / message count)                │
│  └─ ScoringTier2 — ROS bag recorder + plug/port connection tracker                │
│                                                                                    │
└────────────────────────────────────────────────────────────────────────────────────┘
           │  ROS topics / actions / services (Zenoh DDS transport)
           ▼
┌────────────────────────── PARTICIPANT MODEL COMPONENT ──────────────────────────────┐
│                                                                                      │
│  aic_model (Python, ROS 2 LifecycleNode)                                            │
│  ├─ Loads user Policy via policy_module / policy_class ROS params                  │
│  ├─ Action server: /insert_cable → calls Policy.insert_cable()                     │
│  ├─ Publishes: /aic_controller/pose_commands, /aic_controller/joint_commands       │
│  └─ Subscribes: /aic_adapter/observation                                           │
│                                                                                      │
│  Policy (user code — aic_example_policies or custom)                                │
│  └─ insert_cable(task, get_observation, move_robot, send_feedback)                 │
│                                                                                      │
└──────────────────────────────────────────────────────────────────────────────────────┘
```

### Data Flow (single task)

1. `aic_engine` sends `InsertCable` action goal → `aic_model` action server.
2. `aic_model` calls `Policy.insert_cable(task, get_observation, move_robot, send_feedback)`.
3. Policy calls `get_observation()` → `aic_model` fetches latest `Observation` from `/aic_adapter/observation`.
4. Policy calls `move_robot(pose=..., stiffness=..., damping=...)` → `aic_model` publishes `MotionUpdate` to `/aic_controller/pose_commands`.
5. `aic_controller` interpolates to 500 Hz, computes torques via Cartesian impedance, sends to Gazebo joints.
6. `CablePlugin` monitors collision geometry; when cable plug enters port, publishes insertion event on `/scoring/insertion_event`.
7. `aic_scoring/ScoringTier2` records the ROS bag; `aic_engine` reads results and writes per-trial score.

### Package Map

| Package | Language | Role |
|---------|----------|------|
| `aic_engine` | C++ | Trial orchestrator / state machine |
| `aic_bringup` | Python (launch) | Simulation bringup |
| `aic_controller` | C++ | ros2_control impedance plugin |
| `aic_adapter` | C++ | Sensor aggregator |
| `aic_model` | Python | Participant node framework |
| `aic_interfaces` | msg/srv/action | ROS 2 IDL definitions |
| `aic_gazebo` | C++ | Gazebo system plugins |
| `aic_scoring` | C++ | Tier 1 & Tier 2 scorers |
| `aic_description` | URDF/SDF | Robot + environment models |
| `aic_assets` | SDF/mesh | 3D assets for scene |
| `aic_example_policies` | Python | Reference policy implementations |
| `aic_utils/aic_teleoperation` | Python | Keyboard/SpaceMouse teleop |
| `aic_utils/lerobot_robot_aic` | Python | LeRobot data collection + training |
| `aic_utils/aic_mujoco` | mixed | MuJoCo simulation integration |
| `aic_utils/aic_training_utils` | C++/Python | Shared training utilities |

---

## Public Surface

### ROS 2 Topics

| Direction | Topic | Message Type | Publisher | Purpose |
|-----------|-------|--------------|-----------|---------|
| OUT (read) | `/aic_adapter/observation` | `aic_model_interfaces/Observation` | `aic_adapter` | Fused sensor snapshot (3 cameras + FT + joints + controller state) |
| OUT (read) | `/aic_controller/controller_state` | `aic_control_interfaces/ControllerState` | `aic_controller` | Current TCP pose/velocity, reference, error, target mode |
| OUT (read) | `/left_camera/image` | `sensor_msgs/Image` | ros-gz bridge | Left Basler camera, 1152×1024 |
| OUT (read) | `/center_camera/image` | `sensor_msgs/Image` | ros-gz bridge | Center camera |
| OUT (read) | `/right_camera/image` | `sensor_msgs/Image` | ros-gz bridge | Right camera |
| OUT (read) | `/joint_states` | `sensor_msgs/JointState` | ros-gz bridge | UR5e + gripper joint positions/velocities |
| OUT (read) | `/ft_sensor/wrench` | `geometry_msgs/WrenchStamped` | ros-gz bridge | ATI AXIA80 force/torque |
| IN (write) | `/aic_controller/pose_commands` | `aic_control_interfaces/MotionUpdate` | participant | Cartesian impedance target |
| IN (write) | `/aic_controller/joint_commands` | `aic_control_interfaces/JointMotionUpdate` | participant | Joint impedance target |

### ROS 2 Actions

| Action | Type | Server | Purpose |
|--------|------|--------|---------|
| `/insert_cable` | `aic_task_interfaces/action/InsertCable` | `aic_model` | Engine sends cable task; participant streams feedback, returns success/failure |

### ROS 2 Services

| Service | Type | Server | Purpose |
|---------|------|--------|---------|
| `/aic_controller/change_target_mode` | `aic_control_interfaces/srv/ChangeTargetMode` | `aic_controller` | Switch between `MODE_CARTESIAN` (1) and `MODE_JOINT` (2) |
| `/aic_controller/tare_force_torque_sensor` | `std_srvs/Empty` | `aic_controller` | Zero the FT sensor |
| `/<node>/get_state` | `lifecycle_msgs/GetState` | `aic_model` | Query lifecycle state |
| `/<node>/change_state` | `lifecycle_msgs/ChangeState` | `aic_model` | Drive lifecycle transitions |
| `reset_joints` | `aic_engine_interfaces/srv/ResetJoints` | Gazebo plugin | Reset robot to home configuration |

### Message Field Reference

**`aic_control_interfaces/MotionUpdate`** — Cartesian impedance command:
```
std_msgs/Header header          # frame_id: "gripper/tcp" or "base_link"
geometry_msgs/Pose pose         # target Cartesian pose
geometry_msgs/Twist velocity    # target Cartesian velocity
float64[36] target_stiffness   # 6×6 stiffness matrix (N/m, Nm/rad)
float64[36] target_damping     # 6×6 damping matrix
geometry_msgs/Wrench feedforward_wrench_at_tip
float64[6] wrench_feedback_gains_at_tip  # [0, 0.95]
TrajectoryGenerationMode trajectory_generation_mode  # 1=VELOCITY, 2=POSITION
```

**`aic_task_interfaces/action/InsertCable` Goal**:
```
Task task
  string id                 # unique task ID
  string cable_type         # e.g. "sfp_sc"
  string cable_name         # SDF entity name
  string plug_type          # e.g. "sfp_module"
  string plug_name
  string port_type          # target port type
  string port_name
  string target_module_name # board module containing the port
  float64 time_limit        # seconds
```

### CLI Commands

```bash
# Launch simulation (primary entry point)
ros2 launch aic_bringup aic_gz_bringup.launch.py \
  ground_truth:=true \
  start_aic_engine:=true \
  cable_type:=sfp_sc_cable \
  launch_rviz:=true

# Run engine with custom trial config
ros2 run aic_engine aic_engine \
  --ros-args -p config_file_path:=/path/to/trials.yaml \
             -p ground_truth:=false \
             -p model_node_name:=aic_model

# Run example policy
ros2 launch aic_example_policies run_policy.launch.py \
  policy:=CheatCode ground_truth:=true

# Keyboard teleoperation (joint space)
pixi run joint-teleop

# Keyboard teleoperation (Cartesian space)
pixi run cartesian-teleop

# LeRobot data recording
pixi run lerobot-record \
  --robot.type=aic_controller --robot.id=aic \
  --teleop.type=aic_keyboard_ee --teleop.id=aic \
  --dataset.repo_id=myuser/my_dataset \
  --dataset.single_task="Insert SFP cable"

# LeRobot policy training
pixi run lerobot-train \
  --dataset.repo_id=myuser/my_dataset \
  --policy.type=act \
  --output_dir=outputs/act_run1

# Score a recording (Tier 1)
ros2 run aic_scoring tier1 /path/to/tier1.yaml

# Build eval Docker image
docker build -f docker/aic_eval/Dockerfile -t aic_eval .

# Build model Docker image
docker build -f docker/aic_model/Dockerfile -t aic_model .

# Run full evaluation stack locally
docker compose -f docker/docker-compose.yaml up
```

### Launch File Arguments (`aic_gz_bringup.launch.py`)

| Argument | Type | Default | Purpose |
|----------|------|---------|---------|
| `robot_x/y/z` | float | `0.0` | Robot base position in world |
| `robot_roll/pitch/yaw` | float | `0.0` | Robot base orientation |
| `task_board_x/y/z` | float | `0.48/0.0/0.0` | Task board pose |
| `ground_truth` | bool | `false` | Publish ground-truth TF frames for plugs/ports |
| `start_aic_engine` | bool | `false` | Launch engine for automated trial execution |
| `shutdown_on_aic_engine_exit` | bool | `false` | Kill simulation when engine finishes |
| `spawn_task_board` | bool | `true` | Spawn task board in Gazebo |
| `spawn_cable` | bool | `true` | Spawn cable entity |
| `attach_cable_to_gripper` | bool | `true` | Weld cable to gripper at spawn |
| `cable_type` | string | `sfp_sc_cable` | Cable SDF variant (`sfp_sc_cable` or `sfp_sc_cable_reversed`) |
| `launch_rviz` | bool | `true` | Start RViz visualization |
| `gazebo_gui` | bool | `false` | Show Gazebo GUI |
| `use_sim_time` | bool | `true` | Use `/clock` topic |

### Environment Variables

| Variable | Default | Purpose |
|----------|---------|---------|
| `AIC_RESULTS_DIR` | `$HOME/aic_results` | Output directory for trial score JSON files |
| `GZ_BUILD_FROM_SOURCE` | unset | Set to `1` when building Gazebo from source (MuJoCo integration) |
| `MUJOCO_DIR` | unset | Path to MuJoCo installation (MuJoCo integration) |
| `MUJOCO_PLUGIN_PATH` | unset | Path to MuJoCo plugins directory |

### Configuration Files

**`aic_engine/config/sample_config.yaml`** — trial definition:
```yaml
trials:
  trial_1:
    scene:
      task_board:
        pose: {position: {x, y, z}, orientation: {x, y, z, w}}
        nic_rails: [...]      # 5 rails with NIC cards
        sc_rails:  [...]      # 2 rails with SC ports
        mount_rails: [...]    # 6 mount rails (LC, SFP, SC)
      cable: {type: sfp_sc_cable, pose: ...}
    tasks:
      - {id: task_1, cable_type: sfp_sc, plug_type: sfp_module, port_type: sc_port, ...}
robot:
  home_joint_positions: [0, -1.57, 1.57, -1.57, -1.57, 0]  # radians
scoring:
  subscriptions: [/joint_states, /tf, /ft_sensor/wrench, ...]
```

**`aic_controller/src/aic_controller_parameters.yaml`** — key controller params:
```yaml
kinematics:
  plugin: "kinematics_interface_kdl/KinematicsInterfaceKDL"
  tip_link: "gripper/tcp"
control_frequency: 500.0   # Hz
cartesian_impedance:
  stiffness: [1000, 1000, 1000, 50, 50, 50]  # N/m, Nm/rad defaults
  damping:   [50, 50, 50, 5, 5, 5]
```

---

## Integration Guide

### Prerequisites and Setup

```bash
# 1. Install Docker (with GPU support if available)
# 2. Install Pixi (Python environment manager)
curl -fsSL https://pixi.sh/install.sh | bash

# 3. Clone the repository
git clone https://github.com/intrinsic-dev/aic.git && cd aic

# 4. Install Python dependencies
pixi install

# 5. Pull the evaluation Docker image (provided by organizers)
docker pull <eval-image-from-organizers>

# 6. Build the model Docker image (participant)
docker build -f docker/aic_model/Dockerfile -t aic_model .
```

### Authentication

- **Zenoh (runtime communication):** The eval container runs a Zenoh router with ACL rules (`docker/aic_eval/aic_zenoh_config.json5`). The model container connects as a peer (`docker/aic_model/zenoh_config_model_session.sh`). Credentials are provided in `docker/aic_eval/credentials.txt` and are pre-configured in the Docker Compose network.
- **AWS ECR (submission):** Authenticate with `aws ecr get-login-password | docker login --username AWS --password-stdin <registry>`, then push the tagged `aic_model` image to the assigned ECR repository.

### Integration Pattern 1: Simple Policy (Recommended)

Implement `Policy` base class (`aic_model/aic_model/policy.py`):

```python
from aic_model.policy import Policy, MoveRobotCallback
from aic_task_interfaces.action import InsertCable

class MyPolicy(Policy):
    def insert_cable(
        self,
        task: InsertCable.Goal,
        get_observation,       # () -> Observation
        move_robot: MoveRobotCallback,
        send_feedback,         # (str) -> None
    ) -> bool:
        while not self._task_done:
            obs = get_observation()
            # Process obs.left_image, obs.center_image, obs.ft_sensor, etc.
            move_robot(
                pose=target_pose,          # geometry_msgs/Pose in "base_link"
                stiffness=[500]*6,         # linear then angular
                damping=[30]*6,
            )
            self.sleep_for(0.05)           # 20 Hz loop
        return True
```

Configure at launch:
```bash
ros2 run aic_model aic_model \
  --ros-args -p policy_module:=my_package.my_policy \
             -p policy_class:=MyPolicy
```

### Integration Pattern 2: Custom ROS 2 Lifecycle Node

For full control, bypass `aic_model` and implement your own node:

```python
import rclpy
from rclpy.lifecycle import LifecycleNode
from aic_task_interfaces.action import InsertCable
from rclpy.action import ActionServer

class MyAicModel(LifecycleNode):
    def __init__(self):
        super().__init__("aic_model")  # name MUST be "aic_model"
        self._action_server = ActionServer(
            self, InsertCable, "/insert_cable", self._execute_cb
        )

    def on_configure(self, state): ...
    def on_activate(self, state): ...
    # Must implement full lifecycle: configure, activate, deactivate, cleanup, shutdown
```

Requirements per challenge rules: node named `aic_model`, starts unconfigured, implements `/insert_cable` action server.

### Integration Pattern 3: Loading a Trained Neural Network

```python
import torch
from aic_model.policy import Policy

class RunMyNet(Policy):
    def insert_cable(self, task, get_observation, move_robot, send_feedback):
        # Load model once (use on_configure hook for efficiency)
        model = torch.load("my_policy.pt").eval()

        for _ in range(int(task.task.time_limit * 20)):  # 20 Hz
            obs = get_observation()
            img = obs.center_image  # sensor_msgs/Image
            ft  = obs.ft_sensor     # geometry_msgs/WrenchStamped

            # Convert to tensors, run inference
            action = model(preprocess(img, ft))
            move_robot(pose=action_to_pose(action))
            self.sleep_for(0.05)
        return False
```

### Integration Pattern 4: LeRobot Data Collection and Training

```bash
# Collect teleoperation data
pixi run lerobot-record \
  --robot.type=aic_controller --robot.id=aic \
  --teleop.type=aic_spacemouse --teleop.id=aic \
  --robot.teleop_target_mode=cartesian \
  --robot.teleop_frame_id=base_link \
  --dataset.repo_id=${HF_USER}/aic_dataset \
  --dataset.single_task="Insert SFP-SC cable"

# Train ACT policy
pixi run lerobot-train \
  --dataset.repo_id=${HF_USER}/aic_dataset \
  --policy.type=act \
  --output_dir=outputs/act_run1

# Use trained model via RunACT example:
# Set policy_module:=aic_example_policies.run_act
# Set policy_class:=RunACT
# Set hf_repo_id:=${HF_USER}/aic_dataset
```

### Integration Pattern 5: MuJoCo Simulation

```bash
# Convert Gazebo world to MuJoCo
gz service -s /world/aic/generate_world_sdf --reqtype gz.msgs.SdfGeneratorConfig \
  --reptype gz.msgs.StringMsg --timeout 5000 \
  --req 'global_entity_gen_flags: ...' > /tmp/aic.sdf

python3 aic_utils/aic_mujoco/add_cable_plugin.py /tmp/aic.sdf

# Launch with ros2_control (same controller interface)
ros2 launch aic_mujoco aic_mujoco_bringup.launch.py
```

### Extension Points

| Extension Point | How to Use |
|----------------|------------|
| `Policy` base class | Subclass `aic_model.policy.Policy`, implement `insert_cable()` |
| Custom Dockerfile | Extend `docker/aic_model/Dockerfile`; add system deps before the `COPY` layer |
| Additional Pixi envs | Add `[feature.*]` sections to `pixi.toml`; run `pixi install` |
| Custom trial configs | Write YAML following `aic_engine/config/sample_config.yaml` schema; pass via `config_file_path` |
| MuJoCo scene | Run `add_cable_plugin.py` after SDF export; modify MJCF partitions in `aic_mujoco/` |

### Known Limitations

- **ROS 2 Kilted Kaiju only** for official evaluation. Other distributions (Humble, Jazzy) are unsupported and inter-distro communication is not guaranteed.
- **Zenoh transport** is required; standard RMW DDS will not reach across the eval/model Docker network boundary.
- **Ground truth TF** (`ground_truth:=true`) is available in simulation only — not available during official evaluation runs.
- **CheatCode policy** relies on ground truth TF; it will not work when `ground_truth:=false`.
- **GPU** is strongly recommended (32 GB+ RAM, 4–8 CPU cores for simulation); CPU-only runs are slower and may drop sensor frames.
- The `aic_model` framework does not support C++ policies; custom C++ nodes must implement the full lifecycle interface from scratch.

---

## Operational Notes

### Deployment Model

- **Local development:** Docker Compose (`docker/docker-compose.yaml`) runs two containers on an internal bridge network:
  - `eval`: full Gazebo + `aic_engine` + `aic_adapter` + `aic_controller` + `aic_scoring`
  - `model`: participant container (built from `docker/aic_model/Dockerfile`)
- **Official evaluation:** Organizers run the `eval` container; participants submit a tagged `aic_model` image to AWS ECR.
- **Native development:** Supported on Ubuntu 24.04 with ROS 2 Kilted and Gazebo Harmonic installed natively; Pixi manages Python deps.

### Observability

- **Scores:** Written as JSON/YAML files to `$AIC_RESULTS_DIR` at end of each trial.
- **ROS bags:** `ScoringTier2` records a bag of joint states, TF, FT sensor, and off-limit contact events during each trial execution. Bags stored alongside score output.
- **Logging:** All nodes use standard ROS 2 logging (`rclcpp` / `rclpy`). Set `RCUTILS_LOGGING_SEVERITY=DEBUG` for verbose output.
- **RViz:** `aic_bringup` launches `aic_bringup/rviz/aic.rviz` preconfigured with camera, TF, and robot model displays.
- **Gazebo GUI:** Disabled by default (`gazebo_gui:=false`); enable for visual inspection.
- **Troubleshooting doc:** `docs/troubleshooting.md` covers common policy failures, lifecycle timeout issues, and Zenoh connectivity problems.

### Resource Requirements

| Resource | Minimum | Recommended |
|----------|---------|-------------|
| CPU cores | 4 | 8 |
| RAM | 16 GB | 32 GB |
| GPU | None | NVIDIA (CUDA), any VRAM |
| Disk | 20 GB | 50 GB (Docker images + bags) |
| OS | Ubuntu 24.04 | Ubuntu 24.04 |

### Data Persistence

- Trial results: `$AIC_RESULTS_DIR/` (JSON per trial)
- ROS bags: written alongside results during scoring
- Policy weights: managed externally (HuggingFace Hub recommended; `pixi.toml` includes `huggingface-hub` dependency)
- No database; all state is file-based and ephemeral across container restarts
