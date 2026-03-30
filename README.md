# SFM Pedestrian Simulation — OAT Sensitivity Analysis

Extending the Social Force Model (SFM) to understand how parameter tuning affects pedestrian realism. Uses **One-At-A-Time (OAT) sensitivity analysis** across multiple Gazebo environments, compared against ETH/UCY ground truth pedestrian datasets.

## Current Status

| Environment | Status | Agents | Runs | Key Result |
|-------------|--------|--------|------|------------|
| **Café** | ✅ Complete | 12 | 15 | Phase 2 (Goal Force) reduced collisions 19% |
| **Central Tunnel** | 🔧 Ready to run | 40 | 19 planned | Bidirectional corridor flow with groups |

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
- `1` — Custom: uses YAML values exactly as specified (**recommended for OAT**)
- `2` — Random Normal: overwrites force factors with random normal samples each launch
- `3` — Random Uniform: overwrites force factors with random uniform samples each launch

> **Important discovery**: The café tests used `configuration: 2`, which means the loader
> overwrote YAML force factors with random normal values at each launch. Only `max_vel`
> (Phase 3) was unaffected. Central tunnel tests use `configuration: 1` to fix this.

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
Phase 1 (low / high)  → only social_force_factor changes
Phase 2 (low / high)  → only goal_force_factor changes
Phase 3 (low / high)  → only max_vel changes
Phase 4 (low / high)  → only obstacle_force_factor changes (central tunnel only)
```

**Scaling**: All agents are scaled by the same factor proportionally. This preserves
individual variation while isolating the parameter effect.

### 4. Execution

```bash
# Terminal 1: Launch simulation (central tunnel example)
ros2 launch hunav_gazebo_wrapper central_tunnel_no_robot.launch.py \
  configuration_file:=domenic/central_tunnel/ct_baseline.yaml

# Terminal 2: Record 120 seconds of data
python3 src/analysis/run_recording.py <run_id>

# After all runs: Compare against ground truth
python3 src/analysis/ground_truth_analysis.py \
  --custom-dataset run_1 sim_results/central_tunnel/central_tunnel_baseline/run_1/true_pos_.csv \
  --dt 0.1 --output-dir sim_results/central_tunnel/analysis
```

### 5. Evaluation Metrics

| Metric | Measures | ETH Univ Benchmark |
|--------|----------|-------------------|
| Speed Mean | Average walking velocity (m/s) | 1.46 |
| Collision Rate | Body overlaps per agent/second | 0.0004 |
| Path Efficiency | Direct / actual distance (0–1) | 0.966 |
| Near-Miss Rate | Close passes (<1m) per agent/s | 0.014 |
| Min Inter-Agent Dist | Avg closest neighbor distance (m) | 1.88 |
| Heading Jerk | Smoothness of direction changes | 0.47 |

## Environments

### Café (15m × 20m) — Complete
- 12 agents, 10 goal positions among café tables
- Random-crossing goals → high collision rates by design
- 15 runs complete (3 baseline + 4 per phase × 3 phases)

### Central Tunnel (38.5m × 17.5m) — Ready to Run
- 40 agents: 20 at each end, walking toward opposite end (~30m traversal)
- Bidirectional corridor flow (similar to ETH/UCY corridor datasets)
- Mix: 21 individuals + 5 dyads + 3 triads (group walking via BT)
- Open corridor with minimal obstacles → should yield higher path efficiency
- 19 runs planned (3 baseline + 4 per phase × 4 phases)

## Directory Structure

```
├── CAFE_TESTING_PARAMETERS.txt                  # Café 15-run test plan
├── CENTRAL_TUNNEL_TESTING_PARAMETERS.txt        # Central tunnel 19-run test plan
├── CAPSTONE_GUIDE.md                            # Paper-writing guide
├── METHODOLOGY.md                               # OAT methodology details
│
├── src/analysis/                                # Analysis & recording scripts
│   ├── ground_truth_analysis.py                 # ETH/UCY comparison
│   ├── plot_parameter_variation.py              # Per-agent parameter plots
│   ├── run_recording.py                         # 120s recording script
│   └── verify_metrics.py                        # Metric verification
│
├── src/hunav_gazebo_wrapper/
│   ├── launch/
│   │   ├── cafe_no_robot.launch.py              # Café (no robot)
│   │   └── central_tunnel_no_robot.launch.py    # Central tunnel (no robot)
│   ├── worlds/
│   │   ├── cafe.world
│   │   └── central_tunnel.world
│   ├── scenarios/domenic/
│   │   ├── generate_pure_yamls.py               # Café YAML generator
│   │   ├── generate_central_tunnel_yamls.py     # Central tunnel YAML + BT generator
│   │   ├── cafe/         (7 YAMLs: baseline + 3 phases × low/high)
│   │   └── central_tunnel/ (9 YAMLs: baseline + 4 phases × low/high)
│   └── behavior_trees/   (BT XMLs per agent, flat directory)
│
├── src/hunav_sim/
│   ├── hunav_agent_manager/   # SFM computation, BT execution, YAML loading
│   ├── hunav_evaluator/       # Metrics computation
│   └── hunav_msgs/            # Message definitions (AgentBehavior.msg etc.)
│
├── src/DATASETS/
│   ├── eth/                      # ETH datasets (hotel, univ)
│   └── ucy/                      # UCY datasets (zara, students)
│
└── sim_results/
    ├── cafe/              # 15 runs of café data
    └── central_tunnel/    # Central tunnel outputs
```

## Café Results Summary

| Metric | Baseline | Ph1 Low SF | Ph1 High SF | Ph2 Low GF | Ph2 High GF | Ph3 Slow | Ph3 Fast |
|--------|----------|-----------|-------------|-----------|------------|---------|---------|
| Speed (m/s) | 2.99 | 2.78 | 3.04 | 2.85 | 2.78 | 2.78 | 2.58 |
| Collisions (/ag/s) | 0.661 | 1.104 | 1.032 | 0.798 | **0.644** | **0.413** | 0.984 |
| Path Efficiency | 0.063 | 0.056 | 0.062 | 0.054 | 0.058 | 0.058 | 0.061 |

- **Phase 2 High GF**: Collisions reduced 19% — validates OAT methodology
- **Phase 1**: Both extremes worse than baseline — SF may be pre-optimized
- **Phase 3 Fast**: Paradoxically slower — collision gridlock at high speed
- Path efficiency low across all phases due to café obstacle density

## Ground Truth Datasets

| Dataset | Pedestrians | Duration | Use |
|---------|------------|----------|-----|
| eth_univ | 360 | 773s | Primary comparison |
| eth_hotel | 389 | 722s | Dense indoor |
| ucy_zara01/02/03 | 148–204 | 300–420s | Outdoor walking |
| ucy_univ_s1/s3 | 415–434 | 177–216s | Campus corridors |
