---
generated_by: repo-deep-dive workflow
date: 2026-04-22
depth: deep
---

# Integration & Extension Guide — AI for Industry Challenge (AIC) Toolkit

**Audience:** Engineers building a "God-mode" automation dashboard that orchestrates multiple AIC evaluation pipelines running in parallel.

**Codebase snapshot:** `bfbf3ab` on `main` — ROS 2 Kilted Kaiju, Gazebo (via `ros_gz`), Zenoh RMW.

---

## Table of Contents

1. [Integration Patterns](#1-integration-patterns)
   - 1.1 [As a CLI Subprocess](#11-as-a-cli-subprocess)
   - 1.2 [As a ROS 2 Action Client](#12-as-a-ros-2-action-client-service-style)
   - 1.3 [As a Library (Direct Python Import)](#13-as-a-library-direct-python-import)
   - 1.4 [As a Platform Adapter (Policy Plug-in)](#14-as-a-platform-adapter-policy-plug-in)
2. [Extension Points](#2-extension-points)
   - 2.1 [Policy Plug-in Interface](#21-policy-plug-in-interface)
   - 2.2 [aic_engine YAML Configuration](#22-aic_engine-yaml-configuration)
   - 2.3 [Scoring Hook (bag recording + score output)](#23-scoring-hook-bag-recording--score-output)
   - 2.4 [Gazebo Plugins](#24-gazebo-plugins)
   - 2.5 [ROS 2 Controller Plug-in](#25-ros-2-controller-plug-in)
3. [Automation Use Cases](#3-automation-use-cases)
   - 3.1 [Batch / Bulk Trial Runs](#31-batch--bulk-trial-runs)
   - 3.2 [Scheduled / Cron Workflows](#32-scheduled--cron-workflows)
   - 3.3 [Event-Driven Triggers](#33-event-driven-triggers)
   - 3.4 [Multi-Repo / Multi-Project Parallel Execution](#34-multi-repo--multi-project-parallel-execution)
4. [Parallel Deployment](#4-parallel-deployment)
   - 4.1 [Port and Network Configuration](#41-port-and-network-configuration)
   - 4.2 [State Isolation Strategies](#42-state-isolation-strategies)
   - 4.3 [Credential and Secret Management](#43-credential-and-secret-management)
   - 4.4 [Shared vs Isolated Filesystem Resources](#44-shared-vs-isolated-filesystem-resources)
   - 4.5 [Concurrency and Rate-Limit Constraints](#45-concurrency-and-rate-limit-constraints)
5. [Limitations & Gotchas](#5-limitations--gotchas)

---

## 1. Integration Patterns

### 1.1 As a CLI Subprocess

**When to use:** Your dashboard launches and monitors an evaluation run (eval container + model container) without needing fine-grained programmatic access to ROS internals. This is the dominant pattern for an orchestration dashboard because the two Docker containers encapsulate all ROS complexity.

**Setup:** Docker Compose is the canonical launcher. The `docker/docker-compose.yaml` defines two services (`eval` and `model`) with a shared internal network.

```yaml
# docker/docker-compose.yaml (abridged, your customisations in-line)
services:
  eval:
    image: aic_eval:latest
    environment:
      - AIC_RESULTS_DIR=/results          # where scores land
      - AIC_ENGINE_CONFIG=/config/my.yaml  # custom trial config
      - ENABLE_ACL=true
    volumes:
      - ./results/run-${RUN_ID}:/results
      - ./configs:/config
    ...
  model:
    image: my-solution:v1
    environment:
      - AIC_ROUTER_ADDR=eval:7447
    ...
```

**Invocation pattern from a Python orchestrator:**

```python
import subprocess, os, json, pathlib, shutil

def run_evaluation(run_id: str, config_path: str, model_image: str) -> dict:
    results_dir = pathlib.Path(f"results/{run_id}")
    results_dir.mkdir(parents=True, exist_ok=True)

    env = {
        **os.environ,
        "RUN_ID": run_id,
        "COMPOSE_PROJECT_NAME": f"aic_{run_id}",   # isolates networks/volumes
        "MODEL_IMAGE": model_image,
        "CONFIG_PATH": str(config_path),
        "RESULTS_DIR": str(results_dir),
    }

    proc = subprocess.run(
        ["docker", "compose", "-f", "docker/docker-compose.yaml",
         "up", "--abort-on-container-exit", "--exit-code-from", "eval"],
        env=env,
        capture_output=True,
        text=True,
        timeout=7200,  # 2-hour hard limit
    )

    score_file = results_dir / "score.yaml"
    if score_file.exists():
        import yaml
        return yaml.safe_load(score_file.read_text())
    return {"success": False, "exit_code": proc.returncode, "stderr": proc.stderr[-2000:]}
```

**Output parsing:** `aic_engine` writes a `score.yaml` file to the directory pointed to by `AIC_RESULTS_DIR` (`aic_engine/src/aic_engine.cpp`, constructor, line ~320). The file is a YAML serialization of the `Score` struct (see `aic_engine/src/aic_engine.hpp`):

```yaml
# Example score.yaml structure (synthesized from aic_engine.hpp Score struct)
total_score: 85.3
trials:
  trial_1:
    tier1_score: 1.0
    tier2_score: 0.85
    tier3_score: 0.0
    success: true
  trial_2: ...
```

**Caveats:**
- The `eval` container exits with `EXIT_SUCCESS` only when the engine state machine reaches `Completed` (`aic_engine/src/main.cpp:24`). Any infrastructure failure exits non-zero — but a partial run where all tasks failed also exits 0 (scoring is internal). Parse `score.yaml` rather than relying solely on exit code.
- `docker compose up --abort-on-container-exit` causes all containers to stop when *any* container exits. If your model crashes, Compose stops the eval container mid-run. Use `--exit-code-from eval` to capture the eval exit code.
- Container startup order is not guaranteed by health checks; the engine's `ModelReady` state waits for the `aic_model` lifecycle node to appear on the network (timeout: `model_discovery_timeout`, default 300 s).

**Performance considerations:** Each run requires a GPU (Gazebo rendering). Serial runs on a single GPU host are the safe default; see §4 for parallel strategies.

---

### 1.2 As a ROS 2 Action Client (Service-Style)

**When to use:** Your dashboard needs to programmatically trigger individual cable-insertion tasks, query intermediate state, or inject synthetic tasks during development — without waiting for the full engine state machine. Useful for policy smoke-testing and CI-style unit tests.

**Setup:** Your client node must be on the same ROS 2 network as a running `aic_model` node (either inside the container via `ros2 run` or via Zenoh bridge).

**Minimal working example (Python):**

```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from aic_task_interfaces.action import InsertCable
from aic_task_interfaces.msg import Task

class AICClient(Node):
    def __init__(self):
        super().__init__("aic_dashboard_client")
        self._client = ActionClient(self, InsertCable, "/insert_cable")

    def send_task(self, cable_type: str, plug_type: str, port_type: str,
                  target_module: str, time_limit: int = 180) -> bool:
        self._client.wait_for_server(timeout_sec=10.0)

        goal = InsertCable.Goal()
        goal.task = Task(
            id="dashboard_task_001",
            cable_type=cable_type,
            cable_name="cable_0",
            plug_type=plug_type,
            plug_name="plug_sfp_0",
            port_type=port_type,
            port_name="port_sfp_0",
            target_module_name=target_module,
            time_limit=time_limit,
        )

        future = self._client.send_goal_async(
            goal,
            feedback_callback=lambda fb: self.get_logger().info(fb.feedback.message),
        )
        rclpy.spin_until_future_complete(self, future)
        result_future = future.result().get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return result_future.result().result.success

def main():
    rclpy.init()
    client = AICClient()
    ok = client.send_task("sfp", "sfp_plug", "sfp_port", "nic_card_0")
    print("success:", ok)
    rclpy.shutdown()
```

**Interface definition:** `aic_interfaces/aic_task_interfaces/action/InsertCable.action`
```
# Goal
aic_task_interfaces/Task task
---
# Result
bool success
string message
---
# Feedback
string message
```

**Caveats:**
- `aic_model` must be in the `active` lifecycle state before it will accept goals. Sending a goal to a `configured` node results in an immediate rejection (by design — the engine validates this behaviour). You must call the lifecycle `activate` service first.
- The action server is single-threaded for goal processing (see `aic_model/aic_model/aic_model.py` — one `execute_callback` at a time). Concurrent goals are rejected.
- Cross-container action calls require Zenoh; within a container, DDS works natively.

---

### 1.3 As a Library (Direct Python Import)

**When to use:** You want to embed the `Policy` base class in your own Python package, unit-test policy implementations outside ROS, or introspect observation data structures without launching any infrastructure.

**Setup:** Add `aic_model` to your `pixi.toml` or install it via `pip install -e aic_model/`.

```toml
# Your pixi.toml
[dependencies]
aic_model = { path = "aic_model/" }
```

**Direct import — policy unit testing:**

```python
# test_my_policy.py
from unittest.mock import MagicMock, AsyncMock
from my_package.policies.my_policy import MyPolicy
from aic_model_interfaces.msg import Observation
from aic_task_interfaces.msg import Task

def make_mock_policy():
    """Instantiate a policy without any ROS 2 runtime."""
    node = MagicMock()                         # fake rclpy Node
    node.get_logger.return_value = MagicMock()
    node.get_clock.return_value = MagicMock()
    return MyPolicy(node=node)

def test_insert_cable_returns_bool():
    policy = make_mock_policy()
    task = Task(id="t1", cable_type="sfp", time_limit=180)

    obs = Observation()                        # empty — fill fields as needed
    get_obs = MagicMock(return_value=obs)
    move_robot = MagicMock()
    send_feedback = MagicMock()

    result = policy.insert_cable(task, get_obs, move_robot, send_feedback)
    assert isinstance(result, bool)
```

**Key import paths:**
- `aic_model.policy.Policy` — abstract base class (`aic_model/aic_model/policy.py`)
- `aic_model.aic_model.AICModel` — the full lifecycle node (requires ROS 2 runtime)

**Caveats:**
- `aic_model_interfaces`, `aic_task_interfaces`, `aic_control_interfaces` are ROS 2 message packages. They cannot be imported in a plain Python environment without a ROS 2 installation or the pixi `robostack` environment configured in `pixi.toml`. The mock pattern above sidesteps this for pure logic tests.
- `Policy` uses `rclpy` types (`Clock`, `Logger`) injected via the constructor — mock these rather than importing `rclpy` in test environments.

**Performance considerations:** Policy logic runs synchronously inside the action executor's thread. Blocking `sleep_for()` calls (`policy.py:sleep_for`) hold the GIL and prevent any other Python callbacks. Use `rclpy.spin` in a background thread if you mix library usage with live ROS subscriptions.

---

### 1.4 As a Platform Adapter (Policy Plug-in)

**When to use:** You implement a custom policy (neural network, classical planner, etc.) and slot it into the `aic_model` node framework dynamically — without modifying any framework code.

The `aic_model` node discovers and loads your policy class at startup via `importlib` (`aic_model/aic_model/aic_model.py`, `on_configure` callback):

```python
# aic_model/aic_model/aic_model.py (on_configure, simplified)
module = importlib.import_module(self.policy_module)
cls = getattr(module, self.policy_class)
self.policy = cls(node=self)
```

Two ROS 2 parameters control loading:
- `policy_module` — Python module path (e.g. `my_package.policies.my_policy`)
- `policy_class` — Class name within that module (e.g. `MyPolicy`)

**Canonical adapter pattern:**

```python
# my_package/policies/my_policy.py
from aic_model.policy import Policy, MoveRobotCallback, SendFeedbackCallback
from aic_task_interfaces.msg import Task
from aic_model_interfaces.msg import Observation


class MyPolicy(Policy):
    """Minimal compliant policy implementation."""

    def __init__(self, node):
        super().__init__(node)
        # one-time setup: load model weights, initialise planners, etc.
        self._model = load_my_model("/weights/checkpoint.pt")

    def insert_cable(
        self,
        task: Task,
        get_observation,          # Callable[[], Observation]
        move_robot: MoveRobotCallback,
        send_feedback: SendFeedbackCallback,
    ) -> bool:
        """
        Called once per task. Must return True (success) or False (failure).
        This runs in a dedicated executor thread — blocking is allowed.
        """
        send_feedback("starting insertion")

        for step in range(1000):
            obs: Observation = get_observation()
            if obs is None:
                self.sleep_for(0.05)
                continue

            # --- your policy logic here ---
            action = self._model.predict(obs)

            # Cartesian control (Pose + impedance params)
            self.set_pose_target(
                move_robot=move_robot,
                pose=action.pose,
                stiffness=action.stiffness,   # list[float], length 36 (6×6 row-major)
                damping=action.damping,
            )

            if action.done:
                return True

            self.sleep_for(0.05)    # ~20 Hz policy loop

        return False  # time limit reached without success
```

**Registration in launch / ROS parameters:**

```python
# In your launch file
Node(
    package="aic_model",
    executable="aic_model",
    parameters=[{
        "policy_module": "my_package.policies.my_policy",
        "policy_class": "MyPolicy",
    }],
)
```

**Or via CLI:**

```bash
ros2 run aic_model aic_model \
  --ros-args \
  -p policy_module:=my_package.policies.my_policy \
  -p policy_class:=MyPolicy
```

**Caveats:**
- The policy module must be importable in the Python environment where `aic_model` runs. Inside the Docker container, install your package before launching.
- `insert_cable()` is called from a non-main thread. All ROS 2 calls (`move_robot`, `get_observation`, etc.) go through thread-safe realtime buffers — do not create new publishers or subscriptions inside `insert_cable()`.
- `get_observation()` returns the most recently buffered `Observation` or `None` if no data has arrived yet. Your policy must handle `None` gracefully.

---

## 2. Extension Points

### 2.1 Policy Plug-in Interface

**File:** `aic_model/aic_model/policy.py`

```python
# aic_model/aic_model/policy.py — full abstract interface

from abc import ABC, abstractmethod
from typing import Callable, Optional
import rclpy.clock, rclpy.duration

# Callback types
MoveRobotCallback = Callable[..., None]     # publishes MotionUpdate or JointMotionUpdate
SendFeedbackCallback = Callable[[str], None]

class Policy(ABC):
    def __init__(self, node):
        self._node = node

    # ---- Mandatory override ----
    @abstractmethod
    def insert_cable(
        self,
        task,               # aic_task_interfaces/Task
        get_observation,    # Callable[[], Optional[Observation]]
        move_robot: MoveRobotCallback,
        send_feedback: SendFeedbackCallback,
    ) -> bool: ...

    # ---- Provided utilities ----
    def get_logger(self): ...
    def get_clock(self) -> rclpy.clock.Clock: ...
    def time_now(self): ...
    def sleep_for(self, seconds: float): ...

    def set_pose_target(
        self,
        move_robot: MoveRobotCallback,
        pose,               # geometry_msgs/Pose
        stiffness=None,     # list[float] len 36, defaults to [90,90,90,50,50,50] diagonal
        damping=None,       # list[float] len 36, defaults to [50,50,50,20,20,20] diagonal
        feedforward_wrench=None,
        trajectory_mode=None,   # TrajectoryGenerationMode
    ): ...
```

**How to implement:** Create a concrete subclass, override `insert_cable()`, configure the two ROS parameters (`policy_module`, `policy_class`) — see §1.4 for a complete example.

---

### 2.2 aic_engine YAML Configuration

The engine reads a single YAML file at startup (parameter `config_file`, default `sample_config.yaml`). Every trial, scene component, cable, and task is declarative.

**Extension:** Add new trials, new task board configurations, or custom cable placements without modifying any C++ code.

**Schema (from `aic_engine/config/sample_config.yaml`):**

```yaml
scoring:
  topics:                   # list of ROS topic names recorded to bag
    - /observations
    - /aic_controller/pose_commands
    # ... add your own diagnostic topics here

task_board_limits:          # physical rail limits in metres
  nic_rail:   {min: -0.0215, max: 0.0234}
  sc_rail:    {min: -0.06,   max: 0.055}
  mount_rail: {min: -0.09425, max: 0.09425}

trials:
  my_custom_trial:
    scene:
      task_board:
        pose: {x: 0.0, y: 0.5, z: 0.1, roll: 0.0, pitch: 0.0, yaw: 1.5708}
        nic_rail_0:
          entity_present: true
          entity_name: nic_card_0
          entity_pose: {position: 0.0}
        sc_rail_0:
          entity_present: true
          entity_name: sc_connector_0
          entity_pose: {position: 0.0}
      cables:
        cable_0:
          pose:
            gripper_offset: {x: 0.0, y: 0.0, z: 0.1}
            roll: 0.0
            pitch: 0.0
            yaw: 0.0
          attach_to_gripper: true
          cable_type: sfp

    tasks:
      task_1:
        cable_type: sfp
        cable_name: cable_0
        plug_type: sfp_plug
        plug_name: plug_sfp_0
        port_type: sfp_port
        port_name: port_sfp_0
        target_module_name: nic_card_0
        time_limit: 180         # seconds

robot:
  home_joint_positions:
    shoulder_pan_joint: 0.0
    shoulder_lift_joint: -1.5708
    elbow_joint: 1.5708
    wrist_1_joint: -1.5708
    wrist_2_joint: -1.5708
    wrist_3_joint: 0.0
```

**Loading path:** `aic_engine/src/aic_engine.cpp` `initialize()` → `load_config()` → parses via `YAML::LoadFile`.

**Environment variable override for results directory:**

```bash
export AIC_RESULTS_DIR=/path/to/output
```

---

### 2.3 Scoring Hook (Bag Recording + Score Output)

The engine automatically records ROS 2 bag files and writes `score.yaml` at the end of a run. You extend this by:

1. **Adding topics to bag recording** — list additional topics under `scoring.topics` in your config YAML.
2. **Post-processing score.yaml** — read the YAML file from `AIC_RESULTS_DIR` in your dashboard after the run completes.
3. **Tier 2/3 scoring** — `aic_scoring/src/ScoringTier2.cc` and `ScoringTier3` are C++ nodes that subscribe to `/aic_gazebo/insertion_events` and compute precision/efficiency metrics. They are launched as part of `aic_gz_bringup.launch.py`. To add custom metrics, implement a new ROS 2 node that subscribes to the bag topics and writes additional fields to the score file.

**Score file location:**

```
${AIC_RESULTS_DIR}/score.yaml
${AIC_RESULTS_DIR}/<trial_name>_<timestamp>.bag/   # rosbag2 directory
```

---

### 2.4 Gazebo Plugins

Custom Gazebo (Ignition) plugins are registered in SDF world files or via `<plugin>` tags in xacro. Existing plugins in `aic_gazebo/src/`:

| Plugin | File | Purpose |
|---|---|---|
| `CablePlugin` | `CablePlugin.cc/.hh` | Simulates cable dynamics and attachment |
| `OffLimitContactsPlugin` | `OffLimitContactsPlugin.cc/.hh` | Detects illegal contact events |
| `ResetJointsPlugin` | `ResetJointsPlugin.cc/.hh` | Resets joint positions between trials |
| `ScoringPlugin` | `ScoringPlugin.cc` | Fires insertion events for scoring |
| `WorldSdfGeneratorPlugin` | `WorldSdfGeneratorPlugin.cc/.hh` | Generates world SDF dynamically |

**How to add a new Gazebo plugin:**

```cpp
// my_plugin/MyPlugin.cc
#include <gz/sim/System.hh>

class MyPlugin : public gz::sim::System,
                 public gz::sim::ISystemConfigure,
                 public gz::sim::ISystemUpdate {
public:
    void Configure(const gz::sim::Entity &entity,
                   const std::shared_ptr<const sdf::Element> &sdf,
                   gz::sim::EntityComponentManager &ecm,
                   gz::sim::EventManager &eventMgr) override;

    void Update(const gz::sim::UpdateInfo &info,
                gz::sim::EntityComponentManager &ecm) override;
};

GZ_ADD_PLUGIN(MyPlugin, gz::sim::System,
    MyPlugin::ISystemConfigure,
    MyPlugin::ISystemUpdate)
```

Register in your SDF world:
```xml
<plugin filename="libMyPlugin.so" name="my_ns::MyPlugin">
  <my_param>value</my_param>
</plugin>
```

---

### 2.5 ROS 2 Controller Plug-in

`aic_controller` is registered as a `ros2_control` controller plugin (`aic_controller/aic_controller_plugin.xml`). The plugin ID is `aic_controller/AICController`.

You can replace the impedance controller with your own by:
1. Implementing the `controller_interface::ControllerInterface` base class.
2. Registering your plugin in a `plugin.xml`.
3. Replacing `aic_controller` with your plugin name in `aic_bringup/config/aic_ros2_controllers.yaml`.

The controller communicates with `aic_model` via:
- **Input:** `/aic_controller/pose_commands` (`MotionUpdate`) and `/aic_controller/joint_commands` (`JointMotionUpdate`)
- **Output:** `/aic_controller/motion_update` and `/aic_controller/joint_motion_update` (state feedback)
- **Service:** `/aic_controller/change_target_mode` (`ChangeTargetMode.srv`) and `/aic_controller/tare_force_torque_sensor`

---

## 3. Automation Use Cases

### 3.1 Batch / Bulk Trial Runs

**Scenario:** Run N different policy checkpoints against the same set of trials, collect all scores.

```python
# batch_eval.py
import asyncio, subprocess, pathlib, yaml, json
from dataclasses import dataclass

@dataclass
class RunSpec:
    run_id: str
    model_image: str
    config: str

async def run_one(spec: RunSpec, semaphore: asyncio.Semaphore) -> dict:
    async with semaphore:
        results_dir = pathlib.Path(f"results/{spec.run_id}")
        results_dir.mkdir(parents=True, exist_ok=True)

        proc = await asyncio.create_subprocess_exec(
            "docker", "compose",
            "-f", "docker/docker-compose.yaml",
            "--project-name", f"aic_{spec.run_id}",
            "up", "--abort-on-container-exit", "--exit-code-from", "eval",
            env={
                **__import__("os").environ,
                "COMPOSE_PROJECT_NAME": f"aic_{spec.run_id}",
                "MODEL_IMAGE": spec.model_image,
                "CONFIG_PATH": spec.config,
                "RESULTS_DIR": str(results_dir),
            },
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.PIPE,
        )
        await proc.communicate()

        score_file = results_dir / "score.yaml"
        score = yaml.safe_load(score_file.read_text()) if score_file.exists() else {}
        score["run_id"] = spec.run_id
        score["model_image"] = spec.model_image
        return score

async def batch(specs: list[RunSpec], max_parallel: int = 2):
    sem = asyncio.Semaphore(max_parallel)  # GPU constraint — see §4
    tasks = [run_one(s, sem) for s in specs]
    results = await asyncio.gather(*tasks)

    with open("batch_results.json", "w") as f:
        json.dump(results, f, indent=2)
    return results

if __name__ == "__main__":
    runs = [
        RunSpec("run_001", "my-solution:checkpoint-100", "configs/trial_set_a.yaml"),
        RunSpec("run_002", "my-solution:checkpoint-200", "configs/trial_set_a.yaml"),
        RunSpec("run_003", "my-solution:checkpoint-300", "configs/trial_set_b.yaml"),
    ]
    asyncio.run(batch(runs, max_parallel=2))
```

**Key point:** Set `max_parallel` to match available GPU count (one GPU per Gazebo instance — see §4.5).

---

### 3.2 Scheduled / Cron Workflows

**Scenario:** Nightly regression runs after model training completes.

```bash
#!/bin/bash
# cron_eval.sh — scheduled by cron or a CI runner

set -euo pipefail

DATE=$(date +%Y%m%d_%H%M%S)
CHECKPOINT=$(ls -t checkpoints/*.pt | head -1)
IMAGE_TAG="my-solution:nightly-${DATE}"

# Build model image with latest checkpoint
docker build \
  --build-arg CHECKPOINT="${CHECKPOINT}" \
  -t "${IMAGE_TAG}" \
  -f docker/aic_model/Dockerfile .

# Run evaluation
python3 batch_eval.py \
  --image "${IMAGE_TAG}" \
  --config configs/nightly.yaml \
  --output "results/nightly/${DATE}"

# Report to dashboard API
curl -s -X POST https://your-dashboard/api/results \
  -H "Content-Type: application/json" \
  -d @"results/nightly/${DATE}/score.yaml"
```

**Cron entry:**
```cron
0 2 * * * /opt/aic/cron_eval.sh >> /var/log/aic_nightly.log 2>&1
```

---

### 3.3 Event-Driven Triggers

**Scenario:** Automatically evaluate a new Docker image when it is pushed to a registry.

```python
# webhook_handler.py (FastAPI example)
from fastapi import FastAPI, BackgroundTasks
import subprocess, asyncio

app = FastAPI()

async def evaluate_image(image_tag: str, run_id: str):
    """Triggered by registry webhook."""
    # Pull the new image
    subprocess.run(["docker", "pull", image_tag], check=True)

    # Run evaluation (reuse batch_eval logic)
    from batch_eval import RunSpec, run_one
    import asyncio
    sem = asyncio.Semaphore(1)
    result = await run_one(
        RunSpec(run_id, image_tag, "configs/qualification.yaml"), sem
    )

    # Post result back
    print(f"Run {run_id} completed: {result}")

@app.post("/webhook/image-push")
async def on_image_push(payload: dict, background: BackgroundTasks):
    image_tag = payload["repository"]["name"] + ":" + payload["push_data"]["tag"]
    run_id = f"webhook_{payload['push_data']['pusher']}_{image_tag.replace(':', '_')}"
    background.add_task(evaluate_image, image_tag, run_id)
    return {"status": "queued", "run_id": run_id}
```

**Scenario 2:** Trigger re-evaluation when the trial config YAML changes (e.g., new scoring conditions added).

```python
# watch_config.py — uses watchdog
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler
import subprocess

class ConfigChangeHandler(FileSystemEventHandler):
    def on_modified(self, event):
        if event.src_path.endswith(".yaml"):
            print(f"Config changed: {event.src_path} — re-running evaluation")
            subprocess.Popen(["python3", "batch_eval.py", "--config", event.src_path])

observer = Observer()
observer.schedule(ConfigChangeHandler(), path="configs/", recursive=False)
observer.start()
observer.join()
```

---

### 3.4 Multi-Repo / Multi-Project Parallel Execution

**Scenario:** Your dashboard manages multiple teams' policies concurrently, each in an isolated environment.

```python
# multi_team_eval.py
import asyncio, pathlib

TEAMS = {
    "team_alpha": {"image": "registry/team-alpha:latest", "config": "configs/phase1.yaml"},
    "team_beta":  {"image": "registry/team-beta:latest",  "config": "configs/phase1.yaml"},
    "team_gamma": {"image": "registry/team-gamma:latest", "config": "configs/phase2.yaml"},
}

async def evaluate_team(team_id: str, cfg: dict, sem: asyncio.Semaphore):
    run_id = f"{team_id}_{__import__('time').strftime('%Y%m%d_%H%M%S')}"
    # Each team gets its own Compose project → isolated network, ports, volumes
    env = {
        "COMPOSE_PROJECT_NAME": f"aic_{team_id}",
        "MODEL_IMAGE": cfg["image"],
        "CONFIG_PATH": cfg["config"],
        "RESULTS_DIR": f"results/{run_id}",
        # Offset Zenoh router port per team to avoid binding conflicts on shared host
        "ZENOH_ROUTER_PORT": str(7447 + list(TEAMS).index(team_id)),
    }
    async with sem:
        proc = await asyncio.create_subprocess_exec(
            "docker", "compose", "-f", "docker/docker-compose.yaml",
            "up", "--abort-on-container-exit",
            env={**__import__("os").environ, **env},
        )
        await proc.wait()
    return run_id

async def main():
    sem = asyncio.Semaphore(2)   # 2 GPUs available
    tasks = [evaluate_team(tid, cfg, sem) for tid, cfg in TEAMS.items()]
    results = await asyncio.gather(*tasks)
    print("Completed runs:", results)

asyncio.run(main())
```

---

## 4. Parallel Deployment

### 4.1 Port and Network Configuration

The eval and model containers communicate over a private Docker network using **Zenoh** as the RMW middleware. The Zenoh router is started in the `eval` container and listens on TCP port **7447**.

**For each parallel evaluation instance, you must use a distinct:**
1. `COMPOSE_PROJECT_NAME` — Docker Compose uses this to namespace networks, volumes, and container names. Without it, containers from different runs collide.
2. Zenoh router TCP port — if running multiple eval containers on the same host with host-network mode, bind each router to a different port. In bridge-network mode (default), containers are isolated and port 7447 can be reused.

**Recommended port allocation for N parallel instances:**

```bash
# Instance i gets router port 7447 + i
# Configure in docker/aic_eval/zenoh_config_router.sh:
ZENOH_PORT=$((7447 + INSTANCE_ID))
```

**Bridge network isolation (preferred):** Docker Compose bridge networks are namespaced by `COMPOSE_PROJECT_NAME`. Two projects named `aic_run_001` and `aic_run_002` each get their own `aic_run_001_default` and `aic_run_002_default` networks — no cross-contamination of ROS 2 / DDS / Zenoh traffic.

---

### 4.2 State Isolation Strategies

The AIC stack has several sources of mutable state:

| State | Location | Isolation Strategy |
|---|---|---|
| ROS 2 bag files | `AIC_RESULTS_DIR` | Mount a unique host path per run |
| `score.yaml` | `AIC_RESULTS_DIR` | Same as above |
| Gazebo state | In-process (eval container) | One Gazebo process per container — already isolated |
| Joint reset state | Gazebo plugin `ResetJointsPlugin` | Per-container — isolated |
| ros2_control state | `aic_controller` in-process | Per-container — isolated |
| Zenoh router ACL credentials | `docker/aic_eval/credentials.txt` | Mount unique credentials file per run or disable ACL in dev |

**Volume mount pattern:**

```yaml
# docker-compose.yaml override for parallel runs
services:
  eval:
    volumes:
      - type: bind
        source: ${RESULTS_DIR}
        target: /results
      - type: bind
        source: ${CONFIG_PATH}
        target: /config/trial_config.yaml
        read_only: true
```

---

### 4.3 Credential and Secret Management

Zenoh ACL (Access Control List) is controlled by `ENABLE_ACL` environment variable in both containers.

**Credentials file:** `docker/aic_eval/credentials.txt` — contains username/password pairs for Zenoh authentication. **Do not commit real credentials.** For parallel runs:

1. Generate per-run credentials and mount at container startup:
   ```bash
   # generate_creds.sh
   RUN_ID=$1
   echo "user_${RUN_ID}:$(openssl rand -base64 16)" > "/tmp/creds_${RUN_ID}.txt"
   ```
2. Reference the generated file in Compose:
   ```yaml
   environment:
     - CREDENTIALS_FILE=/run/secrets/zenoh_creds
   secrets:
     - zenoh_creds
   secrets:
     zenoh_creds:
       file: /tmp/creds_${RUN_ID}.txt
   ```

For development/CI with no external network exposure, set `ENABLE_ACL=false` to skip authentication entirely.

---

### 4.4 Shared vs Isolated Filesystem Resources

| Resource | Shared safe? | Notes |
|---|---|---|
| `aic_eval` Docker image | Yes | Read-only; all runs use the same image |
| `aic_model` Docker image | Yes (per team) | Read-only; mount checkpoint as volume if hot-swapping |
| Trial config YAMLs | Yes (read-only) | Mount as `read_only: true` |
| Results directory | **No** | Each run must get a unique directory |
| Bag recording directory | **No** | Written inside `AIC_RESULTS_DIR` |
| 3D asset files (`aic_assets/`) | Yes | Bundled in eval image; read-only |
| Gazebo resource paths | Per-container | Set via `GZ_SIM_RESOURCE_PATH` env; eval image sets via `aic_gazebo.sh.in` hook |

**GPU memory:** Gazebo uses the GPU for rendering. If multiple Gazebo instances run on the same GPU simultaneously, VRAM pressure can cause rendering failures or OOM kills. See §4.5.

---

### 4.5 Concurrency and Rate-Limit Constraints

| Constraint | Limit | Mitigation |
|---|---|---|
| GPU per Gazebo instance | 1 GPU recommended | Use `asyncio.Semaphore(gpu_count)` in orchestrator |
| Zenoh router port | 1 per eval container on host network | Assign unique ports; use bridge network to avoid |
| `aic_model` action server | 1 concurrent goal | Engine sends one goal at a time; not a dashboard concern |
| Model discovery timeout | 300 s (configurable) | Set `model_discovery_timeout` in launch args; increase for slow container startups |
| Task time limit | Per-task (default 180 s) | Set `time_limit` in trial config YAML |
| Engine max retries | 5 (hardcoded in `aic_engine.cpp`) | Infrastructure errors retry 5× before aborting trial |
| Controller update rate | 500 Hz | Cannot be reduced without controller parameter changes |
| Observation publish rate | ~20 Hz (`aic_adapter`) | Policy loop should not exceed 20 Hz without custom adapter |
| Clock sync wait | 10 s (hardcoded in engine) | Ensure Gazebo starts before engine times out |

---

## 5. Limitations & Gotchas

### 5.1 Lifecycle Node Strict Sequencing

The `aic_engine` implements a precise state machine that validates the `aic_model` node's lifecycle behaviour (`aic_engine/src/aic_engine.cpp`, `check_model()`, `configure_model()`). Specifically:

- The engine sends an `InsertCable` goal to `aic_model` while it is in the **configured** state and verifies it is **rejected**. If your model accepts goals in the configured state, the engine reports infrastructure failure and aborts.
- After `activate`, the engine sends another goal and expects **acceptance**.
- Any deviation from this expected lifecycle behaviour causes the trial to fail at the `ModelReady` stage, not at the task execution stage — making it look like an infrastructure problem rather than a policy bug.

### 5.2 Single-Threaded Action Execution

`aic_model` uses a `MultiThreadedExecutor` for ROS callbacks, but `insert_cable()` runs in a single dedicated thread per action goal. The action server will reject concurrent goals. This means:

- You cannot pipeline overlapping tasks.
- If your policy calls `sleep_for()` for long periods, no other Python callbacks run in that thread — but ROS 2 callbacks in other threads (subscriptions) continue normally.

### 5.3 `get_observation()` Returns Stale Data

The observation callback in `aic_model` stores the **most recent** `Observation` message. Between adapter publishes (~50 ms apart), `get_observation()` returns the same object repeatedly. If your policy checks timestamps to detect new data:

```python
obs = get_observation()
if obs is None or obs.joint_states.header.stamp == self._last_stamp:
    self.sleep_for(0.05)
    continue
self._last_stamp = obs.joint_states.header.stamp
```

### 5.4 Camera Timestamp Synchronisation Tolerance

`aic_adapter` requires all three camera images to have timestamps within **1 ms** of each other (`aic_adapter/src/aic_adapter.cpp`). If the Gazebo camera plugins drift apart by more than 1 ms (can happen under CPU load), the adapter will drop observation frames silently. Your policy loop will see `get_observation()` return the same stale observation indefinitely.

**Detection:** Monitor the frequency at which observation timestamps advance. If it stalls, the adapter is dropping frames due to camera sync failure.

### 5.5 Joint Settling Check Before Tasks Start

Before executing each task, the engine waits for all joint velocities to fall below **1e-3 rad/s** (`aic_engine.cpp`, `ready_simulator()`). If the robot doesn't settle (e.g., due to impedance instability after homing), the trial hangs until the internal timeout. This is a common failure mode when custom impedance parameters cause oscillation.

### 5.6 Force-Torque Sensor Taring is Per-Trial

The engine calls `tare_force_torque_sensor` before each trial. If this service call fails or times out, the FT readings will have a non-zero offset for the entire trial, affecting impedance control and potentially triggering the wrench safety limits (±40 N translational, ±5 Nm rotational in `aic_ros2_controllers.yaml`).

### 5.7 Xacro Expansion Happens at Runtime

Entity spawning during trial setup uses `xacro` to expand URDF/SDF templates at runtime (`aic_engine.cpp`, `spawn_entity()`). If `xacro` is not on the PATH in the eval container or the template references missing mesh files, spawning silently fails and the engine logs a service call timeout. Always verify asset paths are bundled in the eval image.

### 5.8 `AIC_RESULTS_DIR` Must Exist Before Engine Starts

The engine creates the results directory during `initialize()`. If `AIC_RESULTS_DIR` points to a path whose parent does not exist, the engine throws and enters an error state. Always `mkdir -p` the results parent before starting the eval container.

### 5.9 Zenoh Router Must Be Reachable Before Model Container Starts

The model container's entrypoint validates `AIC_ROUTER_ADDR` and attempts to connect to the Zenoh router. If the eval container's router is not yet accepting connections (still starting Gazebo), the model container may fail to connect. `docker-compose`'s `depends_on` with a health check on port 7447 is the robust solution:

```yaml
services:
  model:
    depends_on:
      eval:
        condition: service_healthy
eval:
  healthcheck:
    test: ["CMD", "nc", "-z", "localhost", "7447"]
    interval: 5s
    timeout: 3s
    retries: 10
```

### 5.10 No Horizontal Scaling of the Eval Component

The eval + Gazebo stack is designed for a single robot instance per container. There is no built-in sharding or distributed trial execution. Horizontal scaling means running N independent Docker Compose stacks, each with their own Gazebo, ROS 2 graph, and Zenoh router. The engine has no awareness of other running instances.

### 5.11 Bag Recording Can Fill Disk

With 10+ sensor topics recorded at high frequency (cameras at 20 Hz, joint states at 500 Hz) for multi-minute trials, bag files grow quickly — easily 1–5 GB per trial. For a dashboard running N concurrent evaluations, ensure the host has sufficient disk, and implement cleanup of old bags:

```bash
# Prune bags older than 7 days
find /results -name "*.bag" -mtime +7 -exec rm -rf {} +
```

---

*Generated from codebase snapshot `bfbf3ab` on 2026-04-22.*
