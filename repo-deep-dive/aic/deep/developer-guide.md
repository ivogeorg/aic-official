---
generated_by: repo-deep-dive workflow
date: 2026-04-22
depth: deep
---

# Developer Guide — AI for Industry Challenge (AIC) Toolkit

## Prerequisites

### Hardware Requirements

| Resource | Minimum | Cloud Eval Instance |
|---|---|---|
| OS | Ubuntu 24.04 | Ubuntu 24.04 |
| CPU | 4–8 cores | 64 vCPU |
| RAM | 32 GB | 256 GiB |
| GPU | NVIDIA RTX 2070+ | NVIDIA L4 Tensor Core |
| VRAM | 8 GB | 24 GiB |

GPU is optional but strongly recommended. Without one, Gazebo's GlobalIllumination renderer will cause severely degraded real-time factor (RTF).

### Required CLI Tools

| Tool | Version / Notes | Install |
|---|---|---|
| **Docker Engine** | Any current release | [docs.docker.com/engine/install](https://docs.docker.com/engine/install/) |
| **Distrobox** | Package-manager install | `sudo apt install distrobox` (Ubuntu) |
| **Pixi** | ≥ 0.63.2 (pinned in CI) | `curl -fsSL https://pixi.sh/install.sh \| sh` |
| **AWS CLI** | Required for submission only | [aws.amazon.com/cli](https://aws.amazon.com/cli/) |

Optional (NVIDIA GPU users only):

```bash
# Install NVIDIA Container Toolkit and configure Docker runtime
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

### ROS 2 Distribution

The official evaluation environment is **ROS 2 Kilted Kaiju**. Inter-distro communication is unsupported. All Pixi channels are pinned to `robostack-kilted`.

---

## Setup: Zero to Running

### 1. Clone and Install Dependencies

```bash
mkdir -p ~/ws_aic/src
cd ~/ws_aic/src
git clone https://github.com/intrinsic-dev/aic

cd ~/ws_aic/src/aic
pixi install          # downloads conda + PyPI packages, builds local ROS packages
```

Expected output: Pixi resolves the lock file, downloads packages, and prints a success message. A `.pixi/` directory is created containing the full environment.

> **Important:** Pixi does not track source changes automatically. After editing any package, run `pixi reinstall <package_name>` to pick up changes.

### 2. Pull and Start the Evaluation Container

```bash
export DBX_CONTAINER_MANAGER=docker

docker pull ghcr.io/intrinsic-dev/aic/aic_eval:latest

# With NVIDIA GPU:
distrobox create -r --nvidia -i ghcr.io/intrinsic-dev/aic/aic_eval:latest aic_eval
# Without NVIDIA GPU (remove --nvidia):
distrobox create -r -i ghcr.io/intrinsic-dev/aic/aic_eval:latest aic_eval

distrobox enter -r aic_eval

# Inside the container:
/entrypoint.sh ground_truth:=false start_aic_engine:=true
```

Expected output: Two windows open — **Gazebo** (3-D simulation) and **RViz** (sensor visualization). The terminal prints `No node with name 'aic_model' found. Retrying...` — this is normal. The engine waits up to 30 seconds (`model_discovery_timeout_seconds`) for a policy node to connect.

> The container must be started **before** the policy node (Step 3). The eval container hosts the Zenoh router on TCP port 7447.

### 3. Run an Example Policy (from Host)

In a separate terminal on the **host** (not inside Distrobox):

```bash
cd ~/ws_aic/src/aic
pixi run ros2 run aic_model aic_model \
  --ros-args -p use_sim_time:=true \
             -p policy:=aic_example_policies.ros.WaveArm
```

Expected output: A task board and gripper-attached cable appear in Gazebo. The eval terminal logs trial progress (Trial 1/3, 2/3, 3/3) and prints scores. Results are written to `$HOME/aic_results/` (override with `$AIC_RESULTS_DIR`).

---

## Development Workflow

### Day-to-Day Commands

```bash
# Enter the Pixi environment (gives a shell with all ROS + Python packages active)
pixi shell

# Run any ROS 2 command inside Pixi
pixi run ros2 <subcommand>

# After changing a local package, reinstall it
pixi reinstall ros-kilted-my-policy-node

# Start the eval container (separate terminal)
export DBX_CONTAINER_MANAGER=docker
distrobox enter -r aic_eval -- /entrypoint.sh ground_truth:=false start_aic_engine:=true

# Run policy against live eval container
pixi run ros2 run aic_model aic_model \
  --ros-args -p use_sim_time:=true \
             -p policy:=my_policy_node.MyPolicy

# Run eval locally via Docker Compose (no Distrobox needed)
docker compose -f docker/docker-compose.yaml up
```

### Build-Run-Debug Cycle (Python Policy)

1. Edit policy source code.
2. `pixi reinstall ros-kilted-<package-name>` — rebuilds and reinstalls the package.
3. Re-launch the `aic_model` node.

There is no hot-reload. Each iteration requires a reinstall.

Tip: You can use `pixi shell` and then `pip install -e <package>` for a faster editable install during development, but this bypasses Pixi and may cause subtle side effects.

### Creating a New Policy Package

```bash
pixi shell
ros2 pkg create my_policy_node --build-type ament_python
```

Add AIC ROS dependencies to `package.xml` (`aic_control_interfaces`, `aic_model`, `aic_model_interfaces`, `aic_task_interfaces`, plus standard ROS message packages).

Create `my_policy_node/pixi.toml` declaring AIC packages as `host-dependencies` and `build-dependencies` using local paths (e.g., `{ path = "../aic_interfaces/aic_control_interfaces" }`).

Register the new package in the root `pixi.toml` under `[dependencies]`:
```toml
ros-kilted-my-policy-node = { path = "my_policy_node" }
```

Implement a class that inherits from `aic_model.Policy` and defines `insert_cable()`. Start from the `WaveArm` example:

```bash
cp aic_example_policies/aic_example_policies/ros/WaveArm.py \
   my_policy_node/my_policy_node/WaveArm.py
```

### Port and Middleware Defaults

| Service | Default |
|---|---|
| Zenoh router (TCP) | 7447 |
| RMW implementation | `rmw_zenoh_cpp` |
| Pixi env activation script | `pixi_env_setup.sh` (sets `RMW_IMPLEMENTATION`, `ZENOH_CONFIG_OVERRIDE`) |
| Observation publish rate | 20 Hz |

Log verbosity is standard ROS 2 (`RCUTILS_LOGGING_SEVERITY`, or pass `--log-level <level>` to ros2 run).

---

## Testing

### Test Framework

There are no automated unit/integration tests run through a test runner in the standard sense. Verification is done by:

1. **Running the policy against the eval container** with `pixi run ros2 run aic_model aic_model ...` and observing trial outcomes.
2. **Docker Compose end-to-end test** — `docker compose -f docker/docker-compose.yaml up` brings up both the eval and model containers in an isolated internal network and runs full trial sequences.
3. **Manual lifecycle tests** — scripts in `aic_model/test/` exercise lifecycle transitions: `cancel_task.py`, `create_and_cancel_task.py`, `cycle_through_lifecycle.py`.

CI uses `ros-tooling/action-ros-ci@v0.4` with `skip-tests: true`, meaning the CI build confirms the workspace compiles but does not run tests.

### Running Manual Tests

```bash
# Run lifecycle test scripts inside the Pixi environment
pixi run python aic_model/test/cycle_through_lifecycle.py
pixi run python aic_model/test/cancel_task.py
```

### Style and Type Checks (run by CI)

```bash
# C/C++ (clang-format 19, rules from .clang-format)
# Run locally via Docker using the same action as CI, or:
clang-format --style=file -i aic_controller/src/*.cpp

# Python import order (isort 5.13.2, config in .isort.cfg)
pixi run isort -c aic_utils/lerobot_robot_aic

# Python type checking (pyright 1.1.408, config in pyrightconfig.json)
pixi run pyright
```

### Test Organization

| Path | Purpose |
|---|---|
| `aic_model/test/` | ROS 2 lifecycle node integration scripts |
| `aic_example_policies/` | Reference implementations used to validate scoring |
| `aic_engine/config/sample_config.yaml` | 3-trial scenario used for quick end-to-end verification |

---

## Build & Packaging

### Local Workspace (Pixi)

```bash
pixi install          # first-time install from pixi.lock
pixi reinstall <pkg>  # rebuild a specific package after source changes
```

Pixi resolves conda packages from `robostack-kilted` and `conda-forge`, and PyPI packages (e.g., `lerobot==0.5.1`, `mujoco==3.5.0`) declared under `[pypi-dependencies]` in `pixi.toml`.

### Eval Container (aic_eval)

Built from `docker/aic_eval/Dockerfile` on top of `ros:kilted-ros-base`:

```bash
# Build locally (rarely needed — use the published image instead)
docker compose -f docker/docker-compose.yaml build eval
```

The Dockerfile:
1. Adds the OSRF Gazebo apt repository.
2. Imports external repos via `vcs import` using `aic.repos`.
3. Installs ROS dependencies with `rosdep`.
4. Builds the workspace with `colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --merge-install`.

Skipped rosdep keys: `gz-cmake3 DART libogre-dev libogre-next-2.3-dev rosetta`.

### Model / Submission Container (aic_model)

Built from `docker/aic_model/Dockerfile` on top of `ros:kilted-ros-core`:

```bash
# Build submission image
docker compose -f docker/docker-compose.yaml build model
```

The Dockerfile installs Pixi, copies the relevant source packages, runs `pixi install --locked`, and sets up an entrypoint that configures Zenoh and calls `pixi run ros2 run aic_model aic_model`.

To use a custom policy:
1. Copy the Dockerfile: `cp docker/aic_model/Dockerfile docker/my_policy/Dockerfile`
2. Add a `COPY` line for your package directory.
3. Update `CMD` to reference your policy class.
4. Update `docker/docker-compose.yaml` to point to the new Dockerfile.

### CI Pipelines

| Workflow | Trigger | What It Does |
|---|---|---|
| `build.yml` | push to `main`, PRs (non-`.md` files) | `colcon build` inside `ros:kilted-ros-base` container |
| `pixi.yml` | push/PR touching `aic_interfaces/`, `aic_utils/`, `pixi.toml`, `pixi.lock` | `isort` check + `pyright` type check |
| `style.yml` | every push and PR | `clang-format` (C/C++), `black`/`isort` (Python) |
| `build_aic_eval_image.yml` | manual dispatch | Builds and optionally pushes the `aic_eval` Docker image |

---

## Configuration Reference

### `aic_engine` ROS Parameters

Passed via `--ros-args -p <param>:=<value>` or in a launch file.

| Parameter | Type | Default | Required | Purpose |
|---|---|---|---|---|
| `config_file_path` | string | `""` | **Yes** | Path to trial configuration YAML |
| `model_node_name` | string | `"aic_model"` | No | Name of participant lifecycle node |
| `adapter_node_name` | string | `"aic_adapter_node"` | No | Adapter node name |
| `gripper_frame_name` | string | `"gripper/tcp"` | No | TF frame for gripper TCP |
| `ground_truth` | bool | `false` | No | Publish ground truth poses from task board |
| `skip_model_ready` | bool | `false` | No | Skip model readiness checks (testing only) |
| `skip_ready_simulator` | bool | `false` | No | Skip entity spawning/deletion (testing only) |
| `endpoint_ready_timeout_seconds` | int | `10` | No | Timeout for required endpoints |
| `model_discovery_timeout_seconds` | int | `30` | No | Timeout to find the participant model |
| `model_configure_timeout_seconds` | int | `60` | No | Timeout for model configuration |
| `model_activate_timeout_seconds` | int | `60` | No | Timeout for model activation |
| `model_deactivate_timeout_seconds` | int | `60` | No | Timeout for model deactivation |
| `model_cleanup_timeout_seconds` | int | `60` | No | Timeout for model cleanup |
| `model_shutdown_timeout_seconds` | int | `60` | No | Timeout for model shutdown |
| `use_sim_time` | bool | `false` | No | Use `/clock` (simulation time) instead of wall time |

### `aic_model` ROS Parameters

| Parameter | Type | Default | Purpose |
|---|---|---|---|
| `policy` | string | — | Fully qualified Python class to load, e.g. `aic_example_policies.ros.WaveArm` |
| `use_sim_time` | bool | `false` | Use simulation clock |

### Environment Variables

| Variable | Default | Scope | Purpose |
|---|---|---|---|
| `AIC_RESULTS_DIR` | `$HOME/aic_results` | Eval container / host | Where scoring data and bag files are written |
| `RMW_IMPLEMENTATION` | `rmw_zenoh_cpp` | Both containers | ROS 2 middleware selection |
| `ZENOH_CONFIG_OVERRIDE` | See `pixi_env_setup.sh` | Both containers | Semicolon-delimited Zenoh config overrides |
| `ZENOH_ROUTER_CONFIG_URI` | `/aic_zenoh_config.json5` | Eval container | Path to Zenoh router JSON5 config |
| `AIC_ROUTER_ADDR` | — | Model container | Address of the Zenoh router (`host:7447`) |
| `AIC_EVAL_PASSWD` | — | Eval container | Password for eval Zenoh auth (ACL mode) |
| `AIC_MODEL_PASSWD` | — | Both containers | Password for model Zenoh auth (ACL mode) |
| `AIC_ENABLE_ACL` | `false` | Both containers | Enable Zenoh access control (`true` / `1`) |
| `DBX_CONTAINER_MANAGER` | `podman` | Host | Force Distrobox to use Docker (`export DBX_CONTAINER_MANAGER=docker`) |
| `AWS_PROFILE` | — | Host | AWS credential profile for ECR submission |

### Trial Configuration YAML (`aic_engine/config/sample_config.yaml`)

The top-level keys are:

| Key | Purpose |
|---|---|
| `scoring.topics` | List of ROS topics the scoring system subscribes to |
| `task_board_limits` | Min/max rail translation limits (meters) for NIC, SC, and mount rails |
| `trials.<trial_id>.scene.task_board` | Task board pose + per-rail entity configuration |
| `trials.<trial_id>.scene.cables` | Cable pose (gripper offset + RPY) and type |
| `trials.<trial_id>.tasks.<task_id>` | Cable/plug/port names, target module, `time_limit` (seconds, sim clock) |
| `robot.home_joint_positions` | Named joint angles (radians) for robot home pose |

`time_limit` is enforced against simulation time, not wall time.

---

## Contributing Conventions

### Branch Naming

No explicit convention is documented. Infer from recent commits: feature/fix branches merged via PR.

### Commit Message Style

Conventional Commits style is used in practice:

```
<type>: <short description> (#<PR number>)
```

Examples from recent history:
- `docs: add troubleshooting notes for policies failing only on the portal (#500)`
- `minor bug fix to nic card (#491)`
- `add arg for setting verbosity (#490)`

### Code Style

| Language | Formatter / Linter | Config |
|---|---|---|
| C/C++ | `clang-format` v19 | `.clang-format` in repo root |
| Python imports | `isort` 5.13.2 | `.isort.cfg` |
| Python types | `pyright` 1.1.408 | `pyrightconfig.json` |
| Python style | `black` (enforced by `style.yml`) | Default settings |

Style checks run on every push and pull request via the `style` GitHub Actions workflow. PRs that fail style checks will not merge.

### PR Process

1. Fork or branch from `main`.
2. Ensure `style` and `build` CI checks pass.
3. Submit PR against `main`.
4. No CONTRIBUTING.md exists; follow the existing style shown in the repo.

---

## Common Failure Modes

### 1. `Error: no such container aic_eval`

**Symptom:** `distrobox enter -r aic_eval` fails immediately.

**Cause:** Distrobox defaults to Podman; Docker is required here.

**Fix:**
```bash
export DBX_CONTAINER_MANAGER=docker
```
Add this to your shell profile to avoid repeating it.

---

### 2. `aic_engine` times out waiting for `aic_model` (30-second discovery budget)

**Symptom:** Eval container prints `No node with name 'aic_model' found. Retrying...` then exits.

**Cause A:** The policy node was not started within the 30-second window (`model_discovery_timeout_seconds`).

**Fix A:** Start the policy node promptly after the eval container is running, or increase `model_discovery_timeout_seconds` when starting the engine.

**Cause B:** Heavy top-level imports (e.g., `import torch`) in the policy module run during the 30-second discovery window and exceed it before the node registers.

**Fix B:** Move heavy imports into the policy class `__init__`. The `on_configure` lifecycle callback has its own 60-second budget (`model_configure_timeout_seconds`).

---

### 3. Gazebo runs at low RTF (< 0.5)

**Symptom:** Simulation is slower than real time; physics and scoring become unreliable.

**Cause A:** Integrated GPU is handling OpenGL rendering instead of the discrete GPU.

**Diagnosis:** `glxinfo -B` — check the renderer string. `nvidia-smi` — `gz sim` should appear.

**Fix A:** `sudo prime-select nvidia`, then log out and back in.

**Cause B:** No dedicated GPU available.

**Fix B:** Disable GlobalIllumination in `aic_description/world/aic.sdf` by setting `<enabled>false</enabled>` in both GI blocks (lines ~39 and ~109). This degrades visual quality and may affect vision-based policies.

---

### 4. `pixi reinstall` not picking up source changes

**Symptom:** Code changes are not reflected when running the policy.

**Cause:** Pixi does not detect source file changes automatically.

**Fix:** Always run `pixi reinstall ros-kilted-<package-name>` after editing a local package. Do not rely on re-running `pixi run` alone.

---

### 5. RTX 50xx GPU not recognized by PyTorch

**Symptom:**
```
UserWarning: NVIDIA GeForce RTX 5090 with CUDA capability sm_120 is not compatible with the current PyTorch installation.
```

**Cause:** `lerobot==0.5.1` pins an older PyTorch version that does not support sm_120 (RTX 50xx).

**Fix:** Add to the root `pixi.toml`:
```toml
[pypi-options.dependency-overrides]
torch = ">=2.7.1"
torchvision = ">=0.22.1"
```

---

### 6. Policy passes locally but fails silently on submission portal

**Symptom:** Local `docker compose up` succeeds; portal shows `Failed` with no logs.

**Cause A:** `task.time_limit` uses simulation clock, not wall clock. If the policy uses `time.time()` or `time.sleep()`, divergence between sim and wall time (which varies on cloud hardware) causes unexpected behavior.

**Fix A:** Use the ROS node clock for all time-based logic inside the policy.

**Cause B:** Top-level module imports exceed the 30-second discovery budget on the portal (cloud may be slower to import large libraries).

**Fix B:** Move `import torch` and similar heavy imports inside the policy class `__init__`.

**Cause C:** Docker login session expired for ECR (tokens are valid for 12 hours).

**Fix C:** Re-run the ECR authentication step:
```bash
aws ecr get-login-password --region us-east-1 | \
  docker login --username AWS --password-stdin 973918476471.dkr.ecr.us-east-1.amazonaws.com
```

---

### 7. Zenoh watchdog warnings at startup

**Symptom:**
```
WARN Watchdog Validator ... error setting scheduling priority for thread: OS(1), will run with priority 48.
```

**Cause:** Zenoh's shared memory watchdog cannot elevate its thread priority without `CAP_SYS_NICE`.

**Fix:** None required — this is harmless and can be safely ignored. Verify shared memory is working with `ls -lh /dev/shm | grep zenoh`.

---

### 8. ECR push fails with "tag already exists" or "no basic auth credentials"

**Symptom A:** `push failed: tag immutable`.

**Fix A:** ECR tags are immutable. Increment the version tag on every push (`:v2`, `:v3`, or use a git SHA).

**Symptom B:** `no basic auth credentials`.

**Fix B:** ECR tokens expire after 12 hours. Re-authenticate:
```bash
export AWS_PROFILE=<team_name>
aws ecr get-login-password --region us-east-1 | \
  docker login --username AWS --password-stdin 973918476471.dkr.ecr.us-east-1.amazonaws.com
```
