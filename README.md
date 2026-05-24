# SFM Pedestrian Simulation — OAT Sensitivity Analysis

Extending the Social Force Model (ESFM) to understand how parameter tuning affects pedestrian crowd realism. Uses **One-At-A-Time (OAT) sensitivity analysis** across three Gazebo environments of increasing complexity, compared against ETH/UCY ground-truth pedestrian datasets.

---

## Current Status

| Environment | Status | Agents | Runs | Config Mode | Key Finding |
|-------------|--------|--------|------|-------------|-------------|
| **Café** | Complete | 12 | 15 | Mode 2 (random) | Force-factor results unreliable; only speed phase valid |
| **Central Tunnel** | Complete | 40 | 19 | Mode 1 (custom) | Bidirectional corridor; obstacle trapping observed |
| **Delta** | Complete | 80 | 19 | Mode 1 (custom) | Obstacle-free triangle; cleanest OAT data |

---

## Quick Start (TL;DR)

If the workspace is already built and sourced, run the full Delta baseline:

```bash
# Terminal A — start simulation HEADLESS (recommended for recording runs)
cd ~/sfm_ws_fresh
source install/setup.bash
ros2 launch hunav_gazebo_wrapper delta_no_robot.launch.py \
  configuration_file:=domenic/delta/delta_baseline.yaml gui:=false

# Terminal B — record 120 s of trajectories (after launch settles)
cd ~/sfm_ws_fresh
source install/setup.bash
python3 src/analysis/run_recording.py 1
```

Output lands in `sim_results/delta/delta_baseline/run_1/`. Repeat with `2`, `3`, … for additional runs.

If this is a fresh clone, follow the [Setup](#setup) section first.

---

## Setup

### 1. Prerequisites

Tested on **Ubuntu 22.04 + ROS 2 Humble + Gazebo Classic 11**.

Install ROS 2 Humble per [docs.ros.org](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html), then:

```bash
sudo apt update
sudo apt install -y \
  ros-humble-desktop \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-nav2-bringup \
  python3-colcon-common-extensions \
  python3-pip \
  git cmake build-essential

pip3 install numpy pandas matplotlib scipy pyyaml
```

The HuNavSim core needs three extra libraries (not on apt):

```bash
# 1) lightsfm (Social Force Model library)
git clone https://github.com/robotics-upo/lightsfm.git ~/lightsfm
cd ~/lightsfm && mkdir -p build && cd build && cmake .. && make && sudo make install

# 2) BehaviorTree.CPP + BehaviorTree.ROS2
sudo apt install -y ros-humble-behaviortree-cpp-v3
# (or build from source: https://github.com/BehaviorTree/BehaviorTree.CPP)
```

`people_msgs` is already vendored in `src/people/` — no separate clone needed.

### 2. Build the workspace

```bash
cd ~/sfm_ws_fresh
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Every new terminal that runs the simulation needs:

```bash
source ~/sfm_ws_fresh/install/setup.bash
```

### 3. Sanity-check the install

```bash
cd ~/sfm_ws_fresh
bash test_services.sh
```

You should see `/hunav_start_recording`, `/hunav_stop_recording`, and `/compute_agents` listed once a simulation is running.

---

## How It Works

### Agent configuration (YAML)

Each scenario is a YAML file specifying every agent's SFM parameters:

```yaml
agent1:
  id: 1
  group_id: -1          # -1 = individual, positive int = group membership
  max_vel: 1.28         # max walking speed (m/s), clamped [0.0, 1.8]
  radius: 0.4           # body radius (m)
  goal_radius: 0.3      # distance to goal that counts as "reached" (m)
  cyclic_goals: true    # loop through goals repeatedly
  init_pose: {x: -12.0, y: 2.5, z: 1.001, h: 0.0}
  behavior:
    type: Regular
    configuration: 1                # 0=default, 1=CUSTOM, 2=random normal, 3=random uniform
    social_force_factor: 11.5       # person-person repulsion   [5.0, 20.0]
    goal_force_factor: 3.2          # attraction to goal        [2.0, 5.0]
    obstacle_force_factor: 15.0     # wall/object repulsion     [2.0, 50.0]
    other_force_factor: 15.0        # auxiliary forces          [0.0, 25.0]
  goals: [3, 15]                    # references to global_goals (cyclic)
```

### Configuration modes (`behavior.configuration`)

| Mode | Name | Effect on force factors |
|------|------|-------------------------|
| 0 | DEFAULT | Overwritten with hardcoded defaults (SF=5, GF=2, OF=10) |
| 1 | CUSTOM | YAML values used **exactly** as written — required for OAT |
| 2 | RANDOM_NORMAL | Overwritten with random normal samples each launch |
| 3 | RANDOM_UNIFORM | Overwritten with random uniform samples each launch |

> **Critical discovery**: the café tests used `configuration: 2`, so the loader overwrote YAML force factors with random normal samples at each launch. Only `max_vel` (Phase 3) was unaffected. Central Tunnel and Delta use `configuration: 1` for deterministic control.

Source: `src/hunav_sim/hunav_agent_manager/src/hunav_loader.cpp` (~lines 120–230).
Constants: `src/hunav_sim/hunav_msgs/msg/AgentBehavior.msg`.

### SFM force parameters

| Parameter | Controls | Range | Higher means |
|-----------|----------|-------|--------------|
| `social_force_factor` | Person-to-person repulsion | [5.0, 20.0] | More avoidance, wider spacing |
| `goal_force_factor` | Attraction to goal | [2.0, 5.0] | More direct paths, less wandering |
| `obstacle_force_factor` | Wall/object repulsion | [2.0, 50.0] | Stay further from walls |
| `max_vel` | Maximum walking speed | [0.0, 1.8] m/s | Faster walking (realistic 0.8–1.5) |
| `other_force_factor` | Auxiliary forces | [0.0, 25.0] | Context-dependent extra repulsion |

Clamps are enforced in `hunav_loader.cpp` when `configuration != 1`.

### OAT methodology

Each phase varies **exactly one** parameter, holding everything else at baseline:

```
Baseline (3 runs)     → reference measurements
Phase 1 (low / high)  → only social_force_factor changes  (×0.55 / ×1.70)
Phase 2 (low / high)  → only goal_force_factor changes    (×0.60 / ×1.60)
Phase 3 (low / high)  → only max_vel changes              (×0.75 / ×1.25)
Phase 4 (low / high)  → only obstacle_force_factor changes (×0.50 / ×2.00)
```

Phase 4 is present in Central Tunnel and Delta only. Café tested three phases (no obstacle phase).

**Scaling**: every agent is scaled by the same proportional factor — individual variation is preserved, only the parameter under test is varied.

### YAML files per environment

The exact scale factor used to generate each YAML:

#### Café — `src/hunav_gazebo_wrapper/scenarios/domenic/cafe/`

| File | Parameter varied | Scale |
|------|------------------|-------|
| `cafe_oat_balanced_baseline.yaml` | — | reference |
| `cafe_oat_social_low.yaml` | Social Force | ×0.579 |
| `cafe_oat_social_high.yaml` | Social Force | ×2.371 |
| `cafe_oat_goal_low.yaml` | Goal Force | ×0.469 |
| `cafe_oat_goal_high.yaml` | Goal Force | ×2.033 |
| `cafe_oat_speed_slow.yaml` | Max Velocity | ×0.700 |
| `cafe_oat_speed_fast.yaml` | Max Velocity | ×1.330 |

#### Central Tunnel — `src/hunav_gazebo_wrapper/scenarios/domenic/central_tunnel/`

| File | Parameter varied | Scale |
|------|------------------|-------|
| `ct_baseline.yaml` | — | reference |
| `ct_phase1_social_low.yaml` | Social Force | ×0.55 |
| `ct_phase1_social_high.yaml` | Social Force | ×1.70 |
| `ct_phase2_goal_low.yaml` | Goal Force | ×0.60 |
| `ct_phase2_goal_high.yaml` | Goal Force | ×1.60 |
| `ct_phase3_speed_low.yaml` | Max Velocity | ×0.75 |
| `ct_phase3_speed_high.yaml` | Max Velocity | ×1.25 |
| `ct_phase4_obstacle_low.yaml` | Obstacle Force | ×0.50 |
| `ct_phase4_obstacle_high.yaml` | Obstacle Force | ×2.00 |

#### Delta — `src/hunav_gazebo_wrapper/scenarios/domenic/delta/`

Same naming pattern as Central Tunnel (`delta_baseline.yaml`, `delta_phase1_social_high.yaml`, …), same scale factors.

YAMLs are generated by `generate_pure_yamls.py` (café), `generate_central_tunnel_yamls.py`, and `generate_delta_yamls.py` in the same scenario directories. To regenerate:

```bash
cd ~/sfm_ws_fresh/src/hunav_gazebo_wrapper/scenarios/domenic
python3 generate_delta_yamls.py            # writes new delta/*.yaml + BT files
colcon build --symlink-install              # from workspace root, to install YAML/BT to share/
```

---

## Running a Test

The full loop is: **launch → record → analyse → repeat**.

### Step 1 — Launch the simulation

Each environment has its own launch file. The `configuration_file` argument is relative to the installed scenarios directory.

> **⚠️ Run headless when recording.** The Gazebo client (`gzclient`) is *very* GPU/CPU-intensive — it renders every agent every frame and will starve the SFM/behaviour-tree threads under load. Recordings made with the GUI on will exhibit lag spikes that corrupt trajectories and inflate collision counts. The problem scales sharply with agent count: café (12 agents) is bearable, but **central tunnel (40 agents) and delta (80 agents) should always be recorded with `gui:=false`**. Only enable the GUI when you want to *look at the environment* (verify spawn positions, inspect obstacles, debug a single scenario) — never while collecting OAT data.

**Headless (recommended for all recording runs):**

```bash
# Open Terminal A and source the workspace
cd ~/sfm_ws_fresh
source install/setup.bash

# Café — headless
ros2 launch hunav_gazebo_wrapper cafe_no_robot.launch.py \
  configuration_file:=domenic/cafe/cafe_oat_balanced_baseline.yaml gui:=false

# Central Tunnel — headless (REQUIRED for clean data)
ros2 launch hunav_gazebo_wrapper central_tunnel_no_robot.launch.py \
  configuration_file:=domenic/central_tunnel/ct_baseline.yaml gui:=false

# Delta — headless (REQUIRED for clean data)
ros2 launch hunav_gazebo_wrapper delta_no_robot.launch.py \
  configuration_file:=domenic/delta/delta_baseline.yaml gui:=false
```

**With GUI (visual inspection only — do not record):**

```bash
# Default behaviour (gui:=true). Use only to look at the world, not to record.
ros2 launch hunav_gazebo_wrapper delta_no_robot.launch.py \
  configuration_file:=domenic/delta/delta_baseline.yaml
```

The launch sequence is identical for all three:

1. `hunav_loader` reads YAML configuration.
2. After 2 s: `hunav_gazebo_world_generator` creates the world with agents.
3. After 2 s more: Gazebo server launches the generated world.
4. `hunav_agent_manager` runs the SFM and behaviour trees.
5. `hunav_evaluator` waits for recording requests.

Wait until Gazebo is fully loaded and agents are visible before recording.

### Step 2 — Record 120 s of trajectories

In a **second terminal**:

```bash
cd ~/sfm_ws_fresh
source install/setup.bash
python3 src/analysis/run_recording.py <run_id>
```

`<run_id>` is an integer (1, 2, 3, …). The script:

1. Updates `install/hunav_evaluator/share/hunav_evaluator/config/metrics.yaml` with the result path.
2. Calls `/hunav_start_recording`.
3. Waits 120 seconds.
4. Calls `/hunav_stop_recording`.
5. Saves trajectories + metrics to `sim_results/<env>/<phase>/run_<id>/`.

For each phase run all required runs (3 for baseline, 2 for each low/high), shutting down Gazebo (Ctrl-C) and relaunching with the next YAML between phases.

### Step 3 — Full run plan

The complete test schedule per environment:

| Phase | YAML (Delta example) | Runs |
|-------|----------------------|------|
| Baseline | `delta_baseline.yaml` | 1, 2, 3 |
| Phase 1 — Social low | `delta_phase1_social_low.yaml` | 4, 5 |
| Phase 1 — Social high | `delta_phase1_social_high.yaml` | 6, 7 |
| Phase 2 — Goal low | `delta_phase2_goal_low.yaml` | 8, 9 |
| Phase 2 — Goal high | `delta_phase2_goal_high.yaml` | 10, 11 |
| Phase 3 — Speed low | `delta_phase3_speed_low.yaml` | 12, 13 |
| Phase 3 — Speed high | `delta_phase3_speed_high.yaml` | 14, 15 |
| Phase 4 — Obstacle low | `delta_phase4_obstacle_low.yaml` | 16, 17 |
| Phase 4 — Obstacle high | `delta_phase4_obstacle_high.yaml` | 18, 19 |

The exact per-run breakdown lives in:

- `CAFE_TESTING_PARAMETERS.txt`
- `CENTRAL_TUNNEL_TESTING_PARAMETERS.txt`
- `DELTA_TESTING_PARAMETERS.txt`

---

## Analysing Results

### Single-phase dashboard

```bash
cd ~/sfm_ws_fresh
python3 src/analysis/crowd_dynamics_evaluator.py \
  --custom-dataset run_1 sim_results/delta/delta_baseline/run_1/true_pos_.csv \
  --custom-dataset run_2 sim_results/delta/delta_baseline/run_2/true_pos_.csv \
  --custom-dataset run_3 sim_results/delta/delta_baseline/run_3/true_pos_.csv \
  --output-dir sim_results/delta/delta_baseline/analysis \
  --dt 0.1
```

### Per-agent parameter variation plot

```bash
python3 src/analysis/plot_parameter_variation.py \
  --scenario-dir src/hunav_gazebo_wrapper/scenarios/domenic/delta/ \
  --output sim_results/delta/parameter_variation_individual_agents.png
```

### Regenerate every plot in the project

```bash
cd ~/sfm_ws_fresh
bash regenerate_all_plots.sh
```

This rebuilds per-phase dashboards, per-environment summaries, and the cross-environment comparison for all three environments.

### Verify metrics against ground truth (ETH Univ)

```bash
python3 src/analysis/verify_metrics.py
```

---

## Evaluation Metrics (3 tiers)

**Tier 1 — Population-level** (one scalar per scene)

| Metric | Measures | ETH Univ | UCY Zara01 |
|--------|----------|----------|------------|
| Speed Mean (m/s) | Average walking velocity | 1.46 | 1.10 |
| Collision Rate (/ag/s) | Body overlaps (<0.5 m) | 0.0004 | 0.0022 |
| Near-Miss Rate (/ag/s) | Close passes (<1.0 m) | 0.014 | 0.058 |
| Mean Density (ag/m²) | Agents in measurement area | 0.054 | 0.054 |
| Mean Flow (ag/m/s) | Density × speed | 0.080 | 0.061 |

**Tier 2 — Agent-level** (per pedestrian, aggregated)

| Metric | Measures | ETH Univ | UCY Zara01 |
|--------|----------|----------|------------|
| Path Efficiency (0–1) | Displacement / arc length | 0.966 | 0.964 |
| Speed Variability (CV) | Intra-agent speed fluctuation | 0.163 | 0.138 |
| Acceleration (m/s²) | Velocity-change magnitude | 0.74 | 0.14 |
| Heading Jerk (rad/fr³) | Directional smoothness | 0.467 | 0.107 |

**Tier 3 — Pairwise / group-level**

| Metric | Measures | ETH Univ |
|--------|----------|----------|
| Min Inter-Agent Dist (m) | Nearest-neighbour distance | 1.88 |
| Min Dist 5th %ile (m) | Closest encounters | 0.60 |

---

## Environments

### Café (15 m × 20 m) — 15 runs complete
- 12 agents, 10 goal positions among café tables and furniture
- Random-crossing goals in a cluttered indoor space
- `configuration: 2` → force-factor phases unreliable (random overwrite)
- 15 runs: 3 baseline + 4 per phase × 3 phases (social, goal, speed)

### Central Tunnel (38.5 m × 17.5 m) — 19 runs complete
- 40 agents: 20 westbound, 20 eastbound (~30 m traversal)
- Bidirectional corridor flow with furniture obstacles
- Mix: 21 individuals + 5 dyads + 3 triads (group walking via behaviour trees)
- `configuration: 1` → deterministic OAT
- 19 runs: 3 baseline + 4 per phase × 4 phases
- Known limitation: SFM lacks path planning; agents occasionally trapped by obstacle AABB forces

### Delta (triangle, ~55 m sides) — 19 runs complete
- 80 agents: 27/27/26 across 3 triangle corners
- Three-way crossing flow, obstacle-free (cabinets removed to avoid AABB trapping)
- Mix: 29 individuals + 9 dyads + 6 triads (15 groups)
- `configuration: 1` → deterministic OAT
- 19 runs: 3 baseline + 4 per phase × 4 phases
- Cleanest environment for isolating pure SFM parameter effects

### Why these design choices?

- **40 / 80 agents** vs café's 12: café was too dense (12 in 300 m²) so collisions dominated; the larger envs match ETH/UCY scale (150–430 peds) more honestly.
- **Bidirectional corridor**: creates natural collision opportunities at the centre and matches the ETH-Univ topology.
- **Groups**: real pedestrians walk in pairs/trios. Group members share goals and walk via the `GroupWalkNode` behaviour tree.
- **Config mode 1**: ensures YAML values are honoured exactly — true OAT isolation.

---

## Results Summary (phase averages)

### Central Tunnel (40 agents, config mode 1)

| Phase | Speed (m/s) | Coll Rate | Near-Miss Rate | Path Eff | Heading Jerk | Min Dist (m) |
|-------|-------------|-----------|----------------|----------|--------------|--------------|
| Baseline | 0.784 | 0.048 | 0.971 | 0.413 | 0.107 | 1.591 |
| Social | 0.729 | 0.041 | 0.914 | 0.558 | 0.092 | 1.550 |
| Goal | 0.773 | 0.042 | 0.965 | 0.435 | 0.106 | 1.611 |
| Speed | 0.742 | 0.049 | 0.919 | 0.544 | 0.091 | 1.576 |
| Obstacle | 0.695 | 0.046 | 0.959 | 0.532 | 0.091 | 1.572 |

### Delta (80 agents, config mode 1)

| Phase | Speed (m/s) | Coll Rate | Near-Miss Rate | Path Eff | Heading Jerk | Min Dist (m) |
|-------|-------------|-----------|----------------|----------|--------------|--------------|
| Baseline | 0.181 | 0.034 | 1.283 | 0.998 | 0.001 | 1.081 |
| Social | 0.201 | 0.050 | 1.328 | 0.986 | 0.007 | 1.045 |
| Goal | 0.208 | 0.040 | 1.129 | 0.979 | 0.008 | 1.115 |
| Speed | 0.145 | 0.019 | 1.355 | 0.996 | 0.026 | 1.059 |
| Obstacle | 0.201 | 0.044 | 1.346 | 0.981 | 0.006 | 1.046 |

### Ground-truth benchmarks

| Dataset | Peds | Speed (m/s) | Coll Rate | Path Eff | Heading Jerk | Min Dist (m) |
|---------|------|-------------|-----------|----------|--------------|--------------|
| ETH Univ | 360 | 1.46 | 0.0004 | 0.966 | 0.467 | 1.88 |
| ETH Hotel | 389 | 1.29 | 0.0010 | 0.910 | 0.518 | 1.65 |
| UCY Zara01 | 148 | 1.10 | 0.0022 | 0.964 | 0.107 | 1.57 |
| UCY Zara02 | 204 | 1.11 | 0.0185 | 0.948 | 0.145 | 1.23 |
| UCY Univ S1 | 415 | 0.64 | 0.0941 | 0.870 | 0.220 | 0.80 |

---

## Troubleshooting

### Agents not spawning
- Inspect `hunav_loader` output in Terminal A for parameter warnings.
- Verify the YAML agent count matches the `agents:` list at the top of the file.
- Ensure `yaml_base_name` matches the BT file naming pattern.

### Behaviour-tree files not found
The BT system looks for `<install>/behavior_trees/{yaml_base_name}__agent_{id}_bt.xml`. After generating new YAMLs run `colcon build --symlink-install` so the BT XMLs are copied into the install share directory.

### Force factors not taking effect
- Check the `configuration` field — modes 0/2/3 override YAML values.
- Look for clamping messages in `hunav_loader` output.
- Use mode 1 (CUSTOM) for deterministic parameter control.

### Path efficiency near zero
- Agents are oscillating between social-force and goal-force.
- Try raising `goal_force_factor` or lowering `social_force_factor`.
- In cluttered envs (e.g. café), this is also a symptom of obstacle-AABB trapping.

### Recording service unavailable
- Confirm the launch in Terminal A finished setup (Gazebo visible, agents spawned).
- Re-run `bash test_services.sh` to verify `/hunav_start_recording` exists.
- Both terminals must source `install/setup.bash`.

---

## Directory Structure

```
├── README.md                                    # ← you are here
├── CAFE_TESTING_PARAMETERS.txt                  # Café 15-run test plan
├── CENTRAL_TUNNEL_TESTING_PARAMETERS.txt        # Central tunnel 19-run test plan
├── DELTA_TESTING_PARAMETERS.txt                 # Delta 19-run test plan
├── regenerate_all_plots.sh                      # Rebuild every analysis figure
├── test_services.sh                             # Sanity-check ROS services
│
├── src/analysis/                                # Analysis & recording scripts
│   ├── crowd_dynamics_evaluator.py              # Crowd dynamics evaluation (sim + ground truth)
│   ├── plot_parameter_variation.py              # Per-agent SFM parameter variation plots
│   ├── run_recording.py                         # 120 s ROS2 recording trigger
│   └── verify_metrics.py                        # Metric computation verification (ETH Univ)
│
├── src/hunav_gazebo_wrapper/
│   ├── launch/                                  # cafe_/central_tunnel_/delta_no_robot.launch.py
│   ├── worlds/                                  # cafe.world, central_tunnel.world, delta.world
│   ├── scenarios/domenic/
│   │   ├── generate_pure_yamls.py               # Café YAML generator
│   │   ├── generate_central_tunnel_yamls.py     # Central tunnel YAML + BT generator
│   │   ├── generate_delta_yamls.py              # Delta YAML + BT generator
│   │   ├── cafe/            (7 YAMLs)
│   │   ├── central_tunnel/  (9 YAMLs)
│   │   └── delta/           (9 YAMLs)
│   └── behavior_trees/
│       ├── cafe/            (12 BT XMLs)
│       ├── central_tunnel/  (40 BT XMLs)
│       └── delta/           (80 BT XMLs)
│
├── src/hunav_sim/
│   ├── hunav_agent_manager/   # C++ SFM computation, BT execution, YAML loading
│   ├── hunav_evaluator/       # Python metrics computation, trajectory export
│   ├── hunav_rviz2_panel/     # RViz2 interactive agent editor
│   └── hunav_msgs/            # ROS2 message/service definitions
│
├── src/DATASETS/
│   ├── eth/                   # ETH Zurich datasets (hotel, univ)
│   └── ucy/                   # UCY datasets (zara01/02/03, students001/003)
│
└── sim_results/
    ├── cafe/                  # 15 runs + per-phase analysis/
    ├── central_tunnel/        # 19 runs + per-phase analysis/
    └── delta/                 # 19 runs + per-phase analysis/
```

---

## Acknowledgements

Anthropic's **Claude Opus 4.6** and **Claude Opus 4.7** contributed heavily to this codebase. In particular, they were used to:

- Generate the analysis and plotting pipeline (`src/analysis/crowd_dynamics_evaluator.py`, `plot_parameter_variation.py`, `verify_metrics.py`) and the `regenerate_all_plots.sh` orchestration.
- Implement the metric equations — speed/CV, collision and near-miss rates, path efficiency, heading jerk, density/flow, inter-agent distances — and align them with the ETH/UCY ground-truth conventions.
- Author the scenario generators (`generate_pure_yamls.py`, `generate_central_tunnel_yamls.py`, `generate_delta_yamls.py`) including the proportional OAT scaling, clamping logic, and per-agent BT XML emission.
- Wire up the recording workflow (`run_recording.py`) and the ROS 2 service plumbing.
- Diagnose the `configuration: 2` confound in the café runs and design the deterministic mode 1 fix that underpins the central tunnel and delta results.
- Draft and consolidate the project documentation in this README.

The underlying HuNavSim / hunav_gazebo_wrapper / lightsfm stack is the work of the original authors at robotics-upo (see citations in `src/hunav_sim/README.md`).

