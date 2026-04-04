# SFM Pedestrian Simulation — OAT Sensitivity Analysis

Extending the Social Force Model (ESFM) to understand how parameter tuning affects pedestrian crowd realism. Uses **One-At-A-Time (OAT) sensitivity analysis** across three Gazebo environments of increasing complexity, compared against ETH/UCY ground truth pedestrian datasets.

## Current Status

| Environment | Status | Agents | Runs | Config Mode | Key Finding |
|-------------|--------|--------|------|-------------|-------------|
| **Café** | Complete | 12 | 15 | Mode 2 (random) | Force factor results unreliable; only speed phase valid |
| **Central Tunnel** | Complete | 40 | 19 | Mode 1 (custom) | Bidirectional corridor; obstacle trapping observed |
| **Delta** | Complete | 80 | 19 | Mode 1 (custom) | Obstacle-free triangle; cleanest OAT data |

## How It Works

### 1. Agent Configuration (YAML)

Each scenario is defined by a YAML file specifying every agent's SFM parameters:

```yaml
agent1:
  id: 1
  group_id: -1          # -1 = individual, positive = group membership
  max_vel: 1.28         # maximum walking speed (m/s), clamped [0.0, 1.8]
  radius: 0.4           # agent body radius (m)
  goal_radius: 0.3      # distance to goal = "reached" (m)
  cyclic_goals: true    # loop through goals repeatedly
  init_pose:
    x: -12.0            # spawn X
    y: 2.5              # spawn Y
    z: 1.001            # spawn Z (slightly above ground)
    h: 0.0              # heading (radians)
  behavior:
    type: Regular
    configuration: 1     # 0=default, 1=CUSTOM, 2=random normal, 3=random uniform
    social_force_factor: 11.5    # person-person repulsion [5.0, 20.0]
    goal_force_factor: 3.2       # attraction toward goal [2.0, 5.0]
    obstacle_force_factor: 15.0  # wall/object repulsion [2.0, 50.0]
    other_force_factor: 15.0     # auxiliary forces [0.0, 25.0]
  goals: [3, 15]        # references to global_goals (cyclic)
```

**Configuration modes** (set in `behavior.configuration`):
- `0` — Default: overwrites all force factors with hardcoded defaults (SF=5, GF=2, OF=10)
- `1` — Custom: uses YAML values exactly as specified (**required for OAT**)
- `2` — Random Normal: overwrites force factors with random normal samples each launch
- `3` — Random Uniform: overwrites force factors with random uniform samples each launch

> **Critical discovery**: The café tests used `configuration: 2`, which caused the loader
> to overwrite YAML force factors with random normal values at each launch. Only `max_vel`
> (Phase 3) was unaffected. Central Tunnel and Delta use `configuration: 1` for deterministic control.

### 2. SFM Force Parameters

The Social Force Model computes agent movement as a sum of forces:

| Parameter | Controls | Range | Higher = |
|-----------|----------|-------|----------|
| `social_force_factor` | Person-to-person repulsion | [5.0, 20.0] | More avoidance, wider spacing |
| `goal_force_factor` | Attraction toward goal | [2.0, 5.0] | More direct paths, less wandering |
| `obstacle_force_factor` | Wall/object repulsion | [2.0, 50.0] | Stay further from walls |
| `max_vel` | Maximum walking speed | [0.0, 1.8] m/s | Faster walking (realistic: 0.8–1.5) |
| `other_force_factor` | Auxiliary forces | [0.0, 25.0] | Context-dependent extra repulsion |

Ranges are **enforced by clamping** in `hunav_loader.cpp` (when `configuration != 1`).

### 3. OAT Methodology

Each phase varies **exactly one parameter** while keeping everything else at baseline:

```
Baseline (3 runs)     → reference measurements
Phase 1 (low / high)  → only social_force_factor changes  (×0.55 / ×1.70)
Phase 2 (low / high)  → only goal_force_factor changes    (×0.60 / ×1.60)
Phase 3 (low / high)  → only max_vel changes              (×0.75 / ×1.25)
Phase 4 (low / high)  → only obstacle_force_factor changes (×0.50 / ×2.00)
```

Phase 4 is present in Central Tunnel and Delta only. Café tested 3 phases (no obstacle phase).

**Scaling**: All agents are scaled by the same proportional factor. This preserves
individual variation while isolating the parameter effect.

### 4. Execution

```bash
# Terminal 1: Launch simulation
ros2 launch hunav_gazebo_wrapper delta_no_robot.launch.py \
  configuration_file:=domenic/delta/delta_baseline.yaml

# Terminal 2: Record 120 seconds of trajectory data
python3 src/analysis/run_recording.py <run_id>

# After all runs: Evaluate crowd dynamics metrics
python3 src/analysis/crowd_dynamics_evaluator.py \
  --custom-dataset run_1 sim_results/delta/delta_baseline/run_1/true_pos_.csv \
  --custom-dataset run_2 sim_results/delta/delta_baseline/run_2/true_pos_.csv \
  --custom-dataset run_3 sim_results/delta/delta_baseline/run_3/true_pos_.csv \
  --output-dir sim_results/delta/delta_baseline/analysis

# Generate per-agent parameter variation plot
python3 src/analysis/plot_parameter_variation.py \
  --scenario-dir src/hunav_gazebo_wrapper/scenarios/domenic/delta/ \
  --output sim_results/delta/parameter_variation_individual_agents.png
```

### 5. Evaluation Metrics (3 Tiers)

**Tier 1 — Population-Level** (scalar per scene):

| Metric | Measures | ETH Univ | UCY Zara01 |
|--------|----------|----------|------------|
| Speed Mean (m/s) | Average walking velocity | 1.46 | 1.10 |
| Collision Rate (/ag/s) | Body overlaps (<0.5m) | 0.0004 | 0.0022 |
| Near-Miss Rate (/ag/s) | Close passes (<1.0m) | 0.014 | 0.058 |
| Mean Density (ag/m²) | Agents in measurement area | 0.054 | 0.054 |
| Mean Flow (ag/m/s) | Density × speed | 0.080 | 0.061 |

**Tier 2 — Agent-Level** (per pedestrian, aggregated):

| Metric | Measures | ETH Univ | UCY Zara01 |
|--------|----------|----------|------------|
| Path Efficiency (0–1) | Displacement / arc length | 0.966 | 0.964 |
| Speed Variability (CV) | Intra-agent speed fluctuation | 0.163 | 0.138 |
| Acceleration (m/s²) | Velocity change magnitude | 0.74 | 0.14 |
| Heading Jerk (rad/fr³) | Directional smoothness | 0.467 | 0.107 |

**Tier 3 — Pairwise/Group-Level**:

| Metric | Measures | ETH Univ |
|--------|----------|----------|
| Min Inter-Agent Dist (m) | Nearest-neighbour distance | 1.88 |
| Min Dist 5th %ile (m) | Closest encounters | 0.60 |

## Environments

### Café (15m × 20m) — 15 Runs Complete
- 12 agents, 10 goal positions among café tables and furniture
- Random-crossing goals in a cluttered indoor space
- `configuration: 2` — force factor phases unreliable (random overwrite)
- 15 runs: 3 baseline + 4 per phase × 3 phases (social, goal, speed)

### Central Tunnel (38.5m × 17.5m) — 19 Runs Complete
- 40 agents: 20 westbound, 20 eastbound (~30m traversal)
- Bidirectional corridor flow with furniture obstacles
- Mix: 21 individuals + 5 dyads + 3 triads (group walking via behaviour trees)
- `configuration: 1` — deterministic OAT
- 19 runs: 3 baseline + 4 per phase × 4 phases (social, goal, speed, obstacle)
- Known limitation: SFM lacks path planning; agents occasionally trapped by obstacle AABB forces

### Delta (triangle, ~55m sides) — 19 Runs Complete
- 80 agents: 27/27/26 across 3 triangle corners
- Three-way crossing flow, obstacle-free (cabinets removed to avoid AABB trapping)
- Mix: 29 individuals + 9 dyads + 6 triads (15 groups)
- `configuration: 1` — deterministic OAT
- 19 runs: 3 baseline + 4 per phase × 4 phases (social, goal, speed, obstacle)
- Cleanest environment for isolating pure SFM parameter effects

## Directory Structure

```
├── CAFE_TESTING_PARAMETERS.txt                  # Café 15-run test plan
├── CENTRAL_TUNNEL_TESTING_PARAMETERS.txt        # Central tunnel 19-run test plan
├── DELTA_TESTING_PARAMETERS.txt                 # Delta 19-run test plan
├── CAPSTONE_GUIDE.md                            # Paper-writing guide
├── METHODOLOGY.md                               # OAT methodology details
├── INVESTIGATION_FINDINGS.md                    # SFM obstacle handling analysis
│
├── src/analysis/                                # Analysis & recording scripts
│   ├── crowd_dynamics_evaluator.py              # Crowd dynamics evaluation (sim + ground truth)
│   ├── plot_parameter_variation.py              # Per-agent SFM parameter variation plots
│   ├── run_recording.py                         # 120s ROS2 recording trigger
│   └── verify_metrics.py                        # Metric computation verification (ETH Univ)
│
├── src/hunav_gazebo_wrapper/
│   ├── launch/
│   │   ├── cafe_no_robot.launch.py
│   │   ├── central_tunnel_no_robot.launch.py
│   │   └── delta_no_robot.launch.py
│   ├── worlds/
│   │   ├── cafe.world
│   │   ├── central_tunnel.world
│   │   └── delta.world
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

## Results Summary (Phase Averages)

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

### Ground Truth Benchmarks

| Dataset | Peds | Speed (m/s) | Coll Rate | Path Eff | Heading Jerk | Min Dist (m) |
|---------|------|-------------|-----------|----------|--------------|--------------|
| ETH Univ | 360 | 1.46 | 0.0004 | 0.966 | 0.467 | 1.88 |
| ETH Hotel | 389 | 1.29 | 0.0010 | 0.910 | 0.518 | 1.65 |
| UCY Zara01 | 148 | 1.10 | 0.0022 | 0.964 | 0.107 | 1.57 |
| UCY Zara02 | 204 | 1.11 | 0.0185 | 0.948 | 0.145 | 1.23 |
| UCY Univ S1 | 415 | 0.64 | 0.0941 | 0.870 | 0.220 | 0.80 |
