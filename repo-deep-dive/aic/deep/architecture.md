---
generated_by: repo-deep-dive workflow
date: 2026-04-22
depth: deep
---

# Module Architecture

## Module Map

| Package | Purpose | Key Exports / Entry Points | Intra-repo Dependencies |
|---|---|---|---|
| **aic_engine** | Orchestrates trial execution: discovers the participant model, sequences lifecycle transitions, spawns simulation entities, invokes tasks via action, and writes scored results | `Engine` class (`aic_engine/src/aic_engine.hpp:181`); `main()` at `aic_engine/src/main.cpp` | aic_interfaces/aic_task_interfaces, aic_interfaces/aic_engine_interfaces, aic_interfaces/aic_control_interfaces, aic_scoring |
| **aic_model** | ROS 2 LifecycleNode template that dynamically loads a participant-supplied `Policy` subclass and exposes the `/insert_cable` action server | `AicModel` (`aic_model/aic_model/aic_model.py:53`); `Policy` ABC (`aic_model/aic_model/policy.py:70`) | aic_interfaces/aic_task_interfaces, aic_interfaces/aic_model_interfaces, aic_interfaces/aic_control_interfaces |
| **aic_controller** | 500 Hz ros2_control plugin implementing Cartesian- and joint-space impedance control with gravity compensation, safety clamping, and trajectory interpolation | `AicController` plugin (`aic_controller/src/aic_controller.cpp`); plugin XML at `aic_controller/aic_controller_plugin.xml` | aic_interfaces/aic_control_interfaces |
| **aic_adapter** | Aggregates and time-synchronises camera images, joint states, wrench, and controller state into a single `Observation` message | `AicAdapterNode` (`aic_adapter/src/aic_adapter.cpp`) | aic_interfaces/aic_model_interfaces, aic_interfaces/aic_control_interfaces |
| **aic_interfaces/aic_task_interfaces** | ROS 2 IDL definitions for tasks: `Task.msg`, `InsertCable.action` | `Task`, `InsertCable` | — |
| **aic_interfaces/aic_control_interfaces** | ROS 2 IDL definitions for robot commands: `MotionUpdate.msg`, `JointMotionUpdate.msg`, `ControllerState.msg`, `ChangeTargetMode.srv`, `TargetMode.msg` | `MotionUpdate`, `JointMotionUpdate`, `ControllerState`, `ChangeTargetMode` | — |
| **aic_interfaces/aic_model_interfaces** | ROS 2 IDL definitions for sensor aggregation: `Observation.msg` | `Observation` | aic_interfaces/aic_control_interfaces |
| **aic_interfaces/aic_engine_interfaces** | ROS 2 IDL definitions for engine infrastructure: `ResetJoints.srv` | `ResetJoints` | — |
| **aic_gazebo** | Gazebo Harmonic plugins: cable dynamics, off-limit contact detection, joint reset, world SDF generation, and scoring event emission | `CablePlugin`, `OffLimitContactsPlugin`, `ResetJointsPlugin`, `ScoringPlugin`, `WorldSdfGeneratorPlugin` | aic_interfaces/aic_engine_interfaces, scoring.proto |
| **aic_scoring** | Evaluates trial recordings: Tier 1 validates topic publication rates; Tier 2 evaluates cable-insertion quality from recorded bags | `ScoringTier1`, `ScoringTier2` (`aic_scoring/src/`) | aic_interfaces/aic_engine_interfaces |
| **aic_bringup** | Launch files and configs that bring up Gazebo, the UR5e robot, ros2_control, the Gazebo-ROS 2 bridge, and optionally the engine | `aic_gz_bringup.launch.py`, `spawn_cable.launch.py`, `spawn_task_board.launch.py` | aic_description, aic_controller, aic_adapter, aic_engine, aic_gazebo |
| **aic_description** | URDF/SDF robot and environment descriptions: UR5e with sensors, task board, cable models | `ur_gz.urdf.xacro`, `task_board.urdf.xacro`, `cable.sdf.xacro`, `aic.sdf` | aic_assets |
| **aic_assets** | 3D meshes (`.glb`) and SDF templates for cables, task board modules, connectors | `.glb` mesh files; `cable.sdf.erb` template | — |
| **aic_example_policies** | Reference Python `Policy` implementations demonstrating different control strategies | `aic_example_policies/__init__.py` | aic_model |
| **aic_utils/aic_teleoperation** | Teleoperation utility for human-controlled data collection | `setup.py`, `pixi.toml` | aic_interfaces/aic_control_interfaces |
| **aic_utils/aic_mujoco** | MuJoCo simulation bridge for policy training | `CMakeLists.txt`, `mujoco.repos` | aic_interfaces |
| **aic_utils/aic_isaac** | Isaac Sim integration utilities | `README.md` (BSD-3 licensed) | aic_interfaces |
| **aic_utils/aic_training_interfaces** | ROS 2 IDL definitions used during training data collection | CMakeLists.txt | — |
| **aic_utils/aic_training_utils** | Utilities for capturing and replaying training demonstrations | CMakeLists.txt | aic_utils/aic_training_interfaces |
| **aic_utils/lerobot_robot_aic** | LeRobot framework integration bridge for AIC hardware | `main.py` (`aic_utils/lerobot_robot_aic/main.py`) | aic_interfaces/aic_control_interfaces |

---

## Key Abstractions

### 1. `Engine` — Trial Orchestrator
- **Location**: `aic_engine/src/aic_engine.hpp:181`
- **Domain role**: Central state machine that drives the full evaluation loop: discovers the participant model node, exercises its ROS 2 lifecycle, spawns simulation entities, dispatches `InsertCable` action goals, and collects scores.
- **State enums** (`aic_engine.hpp:71–121`): `EngineState` (`Uninitialized`, `Initialized`, `Running`, `Error`, `Completed`) and `TrialState` (`ModelReady` → `SimulatorReady` → `ScoringReady` → `TasksExecuting` → `AllTasksCompleted`) and `TaskState` (`TaskRequested` → `TaskStarted` → `TaskCompleted / TaskFailed / TimeLimitExceeded`).
- **Implementors**: Singleton; instantiated in `aic_engine/src/main.cpp`.
- **Users**: Invoked by the ROS 2 bringup launch file when `start_aic_engine:=true`.

### 2. `Policy` — Participant Plugin Interface
- **Location**: `aic_model/aic_model/policy.py:70`
- **Domain role**: Abstract base class that defines the contract participants must fulfil. The single abstract method `insert_cable()` receives a `Task`, an `Observation` getter, a `move_robot` callback, and a `send_feedback` callback. Helper methods (`set_pose_target`, `sleep_for`) are provided on the base class.
- **Implementors**: Any class in a participant container that subclasses `Policy` and overrides `insert_cable`; example implementations live in `aic_example_policies/`.
- **Users**: `AicModel.on_configure()` dynamically imports and instantiates the policy class named by the `policy` ROS parameter (`aic_model/aic_model/aic_model.py:~90`).

### 3. `AicModel` — Lifecycle Node Harness
- **Location**: `aic_model/aic_model/aic_model.py:53`
- **Domain role**: ROS 2 `LifecycleNode` that owns the `/insert_cable` action server, publishes to `/aic_controller/pose_commands` and `/aic_controller/joint_commands`, subscribes to `/observations`, and enforces challenge rules (no motion before activation, goal rejection before activation). Runs the policy in a background thread and handles cancellation.
- **Implementors**: Shipped as the participant framework; participants extend it only via the `Policy` plugin, not by subclassing `AicModel`.
- **Users**: `Engine` manages its lifecycle via `lifecycle_msgs` services (`aic_engine/src/aic_engine.cpp`).

### 4. `InsertCable` — Task Action Contract
- **Location**: `aic_interfaces/aic_task_interfaces/CMakeLists.txt` (defines the `.action` file)
- **Domain role**: The ROS 2 action type that is the sole interface between `Engine` and `AicModel`. Goal field carries a `Task` message; result carries `bool success` + `string message`; feedback carries a progress `string`.
- **Implementors**: Action server in `AicModel`; action client in `Engine` (`aic_engine/src/aic_engine.cpp`).
- **Users**: `Engine` sends one goal per task per trial and waits up to `time_limit` seconds.

### 5. `Observation` — Sensor Aggregation Message
- **Location**: `aic_interfaces/aic_model_interfaces/CMakeLists.txt`
- **Domain role**: Bundles three camera images + camera infos, a `WrenchStamped`, `JointState`, and `ControllerState` into a single timestamped snapshot. Represents everything the policy can legally observe about the world.
- **Implementors**: `AicAdapterNode` assembles and publishes it on `/observations` (`aic_adapter/src/aic_adapter.cpp`).
- **Users**: `AicModel` subscribes and caches the latest value; it is passed to `Policy.insert_cable()` via `get_observation` callback.

### 6. `MotionUpdate` / `JointMotionUpdate` — Impedance Command Messages
- **Location**: `aic_interfaces/aic_control_interfaces/CMakeLists.txt`
- **Domain role**: `MotionUpdate` carries a Cartesian target pose, stiffness (6×6 row-major), damping (6×6), feedforward wrench, and wrench feedback gains. `JointMotionUpdate` carries per-joint target positions/velocities, per-joint stiffness and damping, and feedforward torques. Together they are the only control knobs available to the policy.
- **Implementors**: Assembled in `Policy.set_pose_target()` (`policy.py:~100`) or directly by participant code.
- **Users**: `AicController` subscribes on `/aic_controller/pose_commands` and `/aic_controller/joint_commands`.

### 7. `AicController` — ros2_control Plugin
- **Location**: `aic_controller/src/aic_controller.cpp`; declared in `aic_controller/aic_controller_plugin.xml`
- **Domain role**: A `controller_interface::ControllerInterface` plugin that runs at 500 Hz. Implements Cartesian impedance (`τ = Jᵀ[Kp(x_des−x) + Kd(ẋ_des−ẋ) + Wf] + τ_null`) and joint impedance (`τ = Kp(q_des−q) + Kd(q̇_des−q̇) + τf`) with gravity compensation, command clamping, and trajectory interpolation. Publishes `ControllerState` at each cycle.
- **Implementors**: Loaded by `ros2_control` hardware abstraction layer at bringup.
- **Users**: Consumes command topics from `AicModel`; drives simulated (Gazebo) or real UR5e joints.

### 8. `ScoringTier1` / `ScoringTier2` — Scoring Backends
- **Location**: `aic_scoring/src/ScoringTier1.cc`, `aic_scoring/src/ScoringTier2.cc`
- **Domain role**: `ScoringTier1` validates that required ROS topics were published above minimum rates during a trial. `ScoringTier2` evaluates cable insertion quality (plug/port pose proximity, contact forces) from recorded bags and the Gazebo scoring plugin events.
- **Implementors**: Invoked by `Engine::score_trial()` and `Engine::score_run()`.
- **Users**: `Engine`; configuration provided via `aic_scoring/config/tier1.yaml` and `tier2.yaml`.

### 9. `Trial` / `Task` — Engine Data Model
- **Location**: `aic_engine/src/aic_engine.hpp:136–147` (`Trial`); `aic_interfaces/aic_task_interfaces` (`Task.msg`)
- **Domain role**: `Trial` (C++ struct) groups a scene configuration (task board pose, cable attachment, mount layout) with an ordered list of `Task` objects and records their execution history (`TaskAttempt`). `Task` (ROS 2 message) carries cable/plug/port identifiers and a per-task `time_limit`.
- **Implementors**: Parsed from YAML by `Engine::initialize()`.
- **Users**: `Engine::handle_trial()` iterates tasks; goals built from `Task` msg are sent to `AicModel`.

### 10. `AicAdapterNode` — Sensor Fusion Node
- **Location**: `aic_adapter/src/aic_adapter.cpp`
- **Domain role**: Subscribes to all raw sensor streams (3 cameras, camera infos, wrench, joint states, controller state, TF), buffers them in deques, time-aligns them using `tf2`, and emits a single `Observation` per camera frame. Hides low-level multiplexing from the policy.
- **Implementors**: Standalone ROS 2 node launched by `aic_bringup`.
- **Users**: `AicModel` subscribes to `/observations`.

---

## Data Flow

### Use Case 1: Engine Executes a Single Trial Task

```
YAML config (aic_engine/config/sample_config.yaml)
    │  parsed at Engine::initialize()  (aic_engine.cpp)
    ▼
Engine::handle_trial(trial)
    │
    ├─ Engine::check_model()
    │   calls /aic_model/get_state  (lifecycle_msgs/srv/GetState)
    │   calls /aic_model/change_state CONFIGURE, then ACTIVATE
    │
    ├─ Engine::ready_simulator(trial)
    │   calls /gz_server/spawn_entity  (simulation_interfaces/srv/SpawnEntity)
    │     → Gazebo spawns task_board + cable SDF
    │   calls /aic_controller/tare_force_torque_sensor  (std_srvs/srv/Trigger)
    │
    ├─ Engine::ready_scoring(trial)
    │   registers plug/port connections with ScoringTier2
    │   starts ros2bag recording of scoring topics
    │
    └─ For each task → Engine sends InsertCable goal
           action client → /insert_cable  (aic_task_interfaces/action/InsertCable)
               ▼
           AicModel::insert_cable_execute_callback()  (aic_model.py:~170)
               spawns action_thread_func in background thread
               ▼
           Policy.insert_cable(task, get_observation, move_robot, send_feedback)
           (participant implementation)
               ▼
           Policy calls move_robot(motion_update=MotionUpdate(...))
               ▼
           AicModel publishes MotionUpdate on /aic_controller/pose_commands
               ▼
           AicController::update() at 500 Hz  (aic_controller.cpp)
             Clamp → Interpolate → CartesianImpedanceAction
             τ = Jᵀ[Kp(x_des−x) + Kd(ẋ_des−ẋ)] + τ_null
               ▼
           Gazebo simulates robot motion; cable plug moves toward port
               ▼
           ScoringPlugin emits contact events (aic_gazebo/src/ScoringPlugin.cc)
               ▼
           Engine::score_trial() → ScoringTier2 evaluates bag
               ▼
           Engine::reset_after_trial() → deactivate model, delete entities,
             call /scoring/reset_joints  (aic_engine_interfaces/srv/ResetJoints)
               ▼
           TrialScore written to YAML by Engine::score_run()
```

Key functions/line anchors:
- `Engine::handle_trial` — `aic_engine/src/aic_engine.cpp`
- `Engine::ready_simulator` — `aic_engine/src/aic_engine.cpp`
- `AicModel::insert_cable_execute_callback` — `aic_model/aic_model/aic_model.py:~170`
- `Policy.set_pose_target` — `aic_model/aic_model/policy.py:~100`
- `AicController::update` — `aic_controller/src/aic_controller.cpp`

---

### Use Case 2: Sensor Data Path — World State to Policy Observation

```
Gazebo Harmonic (physics at ~1000 Hz)
    │
    ├─ 3× wrist cameras → image topics via gz-ros2-bridge
    │     /left_camera/image, /center_camera/image, /right_camera/image
    │     (sensor_msgs/msg/Image, ~30 Hz each)
    │
    ├─ ATI F/T sensor → /fts_broadcaster/wrench
    │     (geometry_msgs/msg/WrenchStamped, ~50 Hz)
    │
    ├─ UR5e joint encoders → /joint_states
    │     (sensor_msgs/msg/JointState, ~125 Hz)
    │
    └─ AicController publishes at 500 Hz → /aic_controller/controller_state
          (aic_control_interfaces/msg/ControllerState)
          contains: tcp_pose, tcp_velocity, tcp_error, reference_joint_state
              │
              ▼
      AicAdapterNode::*_callback()  (aic_adapter/src/aic_adapter.cpp)
        buffers each stream into a per-type std::deque
        on each camera frame:
          → picks nearest-timestamp entries from other deques
          → resolves TF2 transforms for camera frames
          → assembles aic_model_interfaces/msg/Observation
          → publishes on /observations (~30 Hz)
              │
              ▼
      AicModel::observation_callback()  (aic_model.py)
        stores latest Observation in self._latest_observation
              │
              ▼
      Policy.insert_cable(..., get_observation, ...)
        calls get_observation() → returns cached Observation
        reads observation.controller_state.tcp_pose for current EEF pose
        reads observation.wrist_wrench for force sensing
        reads observation.left_image / center_image / right_image for vision
```

---

### Use Case 3: Robot Homing Between Trials

```
Engine::reset_after_trial(trial)  (aic_engine.cpp)
    │
    ├─ Engine::deactivate_model_node()
    │   calls /aic_model/change_state DEACTIVATE
    │
    ├─ Engine::reset_simulator(trial)
    │   for each spawned entity:
    │     calls /gz_server/delete_entity
    │
    └─ Engine::home_robot()
        ├─ deactivate aic_controller via /controller_manager/switch_controller
        ├─ calls /scoring/reset_joints  (aic_engine_interfaces/srv/ResetJoints)
        │   → ResetJointsPlugin in Gazebo teleports joints to home positions
        │     (aic_gazebo/src/ResetJointsPlugin.cc)
        └─ re-activate aic_controller
```

---

## Dependency Graph

```
                         ┌─────────────────────────────┐
                         │       aic_interfaces/        │
                         │  aic_task_interfaces         │
                         │  aic_control_interfaces      │
                         │  aic_model_interfaces        │
                         │  aic_engine_interfaces       │
                         └────────────┬────────────────┘
                                      │ (used by all)
           ┌──────────────────────────┼────────────────────────────┐
           │                          │                            │
    ┌──────▼──────┐          ┌────────▼───────┐          ┌────────▼────────┐
    │ aic_engine  │◄─action──│   aic_model    │──cmd────►│ aic_controller  │
    │             │  server  │  (+ Policy ABC)│  topics  │  (ros2_control  │
    │ orchestrates│          │               │           │   plugin)       │
    │ lifecycle & │          └────────┬──────┘           └────────┬────────┘
    │ scoring     │                   │ /observations              │state
    └──────┬──────┘                   │                           │
           │                  ┌───────▼──────┐                    │
           │                  │ aic_adapter  │◄───────────────────┘
           │                  │ (sensor mux) │
           │                  └──────────────┘
           │
    ┌──────▼──────┐     ┌──────────────┐     ┌──────────────┐
    │ aic_scoring │     │  aic_gazebo  │     │ aic_bringup  │
    │ (Tier1/2)   │     │  (plugins)   │     │ (launch +    │
    └─────────────┘     └──────────────┘     │  config)     │
                                              └──────┬───────┘
                                                     │ depends on
                                    ┌────────────────┼────────────┐
                             ┌──────▼──────┐  ┌──────▼──────┐   │
                             │aic_description│ │  aic_assets │   │
                             │(URDF/SDF)   │  │  (meshes)   │   │
                             └─────────────┘  └─────────────┘   │
                                                                  │
                                             ┌────────────────────▼──┐
                                             │      aic_utils/        │
                                             │  aic_teleoperation     │
                                             │  aic_mujoco            │
                                             │  aic_isaac             │
                                             │  aic_training_*        │
                                             │  lerobot_robot_aic     │
                                             └────────────────────────┘
```

**Circular dependencies**: None detected. All dependency edges flow from orchestration (`aic_engine`) and participant (`aic_model`) layers down toward shared interfaces (`aic_interfaces`) and simulation infrastructure (`aic_gazebo`, `aic_description`).

---

## Notable Architectural Decisions

### 1. ROS 2 Lifecycle Node as the Participant Contract
**Evidence**: `aic_model/aic_model/aic_model.py:53` (`AicModel(LifecycleNode)`); engine methods `configure_model_node`, `activate_model_node`, `deactivate_model_node`, `cleanup_model_node` in `aic_engine/src/aic_engine.cpp`.

The engine enforces that the participant node starts in `Unconfigured` state, transitions through `Configured` before activation, and publishes no motion commands prior to `Active`. This is actively verified: the engine attempts a dummy action goal in `Configured` state and asserts it is rejected. The decision couples the challenge rules tightly to the ROS 2 lifecycle protocol, providing a language-agnostic behavioural contract enforceable at runtime without code inspection.

### 2. Plugin-Based Policy Loading via Python `importlib`
**Evidence**: `aic_model/aic_model/aic_model.py:~90`; `aic_model/aic_model/policy.py:70` (`class Policy(ABC)`).

Rather than requiring participants to subclass `AicModel`, the framework dynamically loads a Python class named by the `policy` ROS parameter at `on_configure` time. This separates boilerplate (lifecycle management, topic wiring, thread safety) from algorithm (the `insert_cable` method). Participants ship only a `Policy` subclass in any Python package; the harness handles everything else. The tradeoff is Python-only policy code; participants wanting C++ must implement the full `AicModel` interface from scratch (documented in `docs/challenge_rules.md`).

### 3. Observation Aggregator as a Dedicated Node (`aic_adapter`)
**Evidence**: `aic_adapter/src/aic_adapter.cpp`; `Observation.msg` in `aic_interfaces/aic_model_interfaces`.

Rather than subscribing to raw sensor topics directly, the policy receives a single `Observation` message that bundles all sensor streams. The adapter node handles deque-based time alignment and TF resolution. This choice means the policy sees a consistent world snapshot and the sensor-fusion logic is centralised and reusable. The downside is an extra hop adding latency; because the adapter fires on camera frames (~30 Hz), high-frequency wrench or joint data is downsampled.

### 4. Impedance Control at 500 Hz with Command Interpolation
**Evidence**: `aic_controller/src/aic_controller_parameters.yaml` (`control_frequency: 500`); `aic_controller/src/aic_controller.cpp` (CartesianImpedanceAction, JointImpedanceAction, clamping, interpolation logic); `aic_bringup/config/aic_ros2_controllers.yaml`.

The controller receives sparse policy commands (10–30 Hz) and interpolates them to 500 Hz internally before computing impedance torques. This decouples policy loop rate from control loop stability. Stiffness and damping matrices are tunable per command, enabling the policy to express soft compliance for insertion tasks and stiff tracking for free-space motion without switching controllers. The `change_target_mode` service (`ChangeTargetMode.srv`) allows runtime switching between Cartesian and joint modes without restarting the controller.

### 5. YAML-Driven Trial Configuration
**Evidence**: `aic_engine/config/sample_config.yaml`; `Engine::initialize()` in `aic_engine/src/aic_engine.cpp`; `aic_engine/README.md`.

All trial geometry (task board pose, mount layout, cable attachment, per-task time limits) is expressed in a YAML file parsed at engine startup, not hard-coded. This allows organizers to vary scene parameters across competition rounds without recompiling. The same mechanism lets participants create their own test scenarios locally. The `WorldSdfGeneratorPlugin` (`aic_gazebo/src/WorldSdfGeneratorPlugin.cc`) generates Gazebo world SDF at runtime from this configuration, avoiding static world files.

### 6. Two-Container Deployment via Docker Compose
**Evidence**: `docker/docker-compose.yaml`; `docker/aic_eval/Dockerfile`; `docker/aic_model/Dockerfile`; `docs/submission.md`.

The evaluation stack is split into an immutable `aic_eval` container (engine, controller, adapter, Gazebo) and a participant-supplied `aic_model` container. They communicate exclusively over ROS 2 (Zenoh RMW, configured via `aic_eval/aic_zenoh_config.json5`). This boundary means the evaluation infrastructure is fixed and tamper-resistant while participants can use any base image or language runtime inside their container, subject to the ROS 2 topic/service/action contract.

### 7. Scoring Decoupled from Real-Time Execution via Bag Replay
**Evidence**: `aic_scoring/src/ScoringTier2.cc`; engine calls `ready_scoring()` before tasks and `score_trial()` after; `aic_scoring/config/tier2.yaml` lists recorded topics.

Tier 2 scoring is computed post-trial by replaying a `ros2bag` recording rather than evaluating live. This makes scoring reproducible and auditable, and allows organizers to compute additional metrics offline without modifying the real-time pipeline. The `ScoringPlugin` in Gazebo emits contact events into the bag (via `scoring.proto`, `aic_gazebo/proto/scoring.proto`) to capture insertion events with microsecond accuracy.
