# Pedestrian Simulation Sensitivity Analysis

Social Force Model (SFM) parameter tuning through One-At-A-Time (OAT) sensitivity analysis. Tests 21 configurations (baseline + 6 parameter variations × 3 runs) in a Gazebo simulation with 12 agents in a café environment.

## Quick Start

```bash
# Build
cd ~/sfm_ws_fresh
colcon build --packages-select hunav_gazebo_wrapper hunav_sim hunav_evaluator
source install/setup.bash

# Run a single test  
ros2 launch hunav_gazebo_wrapper cafe_no_robot.launch.py \
  configuration_file:=domenic/cafe/cafe_oat_balanced_baseline.yaml

# Record simulation (120 seconds)
ros2 service call /hunav_start_recording hunav_msgs/srv/StartRecording \
  "{experiment_tag: baseline_oat, run_id: 1}" && sleep 120 && \
ros2 service call /hunav_stop_recording std_srvs/srv/Empty {}

# Compare to ground truth
python3 simulation_analysis/compare_to_ground_truth.py 1
```

## What This Is

A systematic parameter tuning study using empirical testing to improve SFM configuration for realistic pedestrian behavior. The baseline generates unrealistic oscillation patterns:
- **Path Efficiency**: 0.07 (should be 0.96)
- **Collision Rate**: 0.11/ped/s (should be 0.0004)
- **Speed Mean**: 0.83 m/s (should be 1.46)

OAT analysis will isolate which parameters drive these mismatches.

## OAT Test Plan

| Tests | Configuration | Purpose |
|-------|---------------|---------|
| 1-3 | Baseline | Reference (3 repeats for reproducibility) |
| 4-9 | Social Force ± | Person-person collision sensitivity |
| 10-15 | Goal Force ± | Path efficiency/directedness |
| 16-21 | Speed ± | Walking pace variations |

**Duration**: ~2-2.5 hours total (~10-15 min per test)

## Agent Configuration & Normalization

### Scenario Setup
- **Environment**: Café (15m × 20m) with café tables and standing areas
- **Agent Count**: 12 pedestrians with predefined start/goal pairs
- **Duration**: 120 seconds per run (sufficient for pattern detection with ~1,440 observations per run)
- **Recording Frequency**: 10 Hz (trajectory sampled at 10 frames/second)
- **Update Rate**: 100 Hz (SFM physics computed at 100 Hz, downsampled for recording)

### Agent Distribution & Normalization
Agents are distributed uniformly across the scenario:
- **Start Positions**: Grid-distributed to avoid clustered initial conditions
- **Goal Positions**: Evenly spaced around café perimeter to ensure diverse navigation patterns
- **Initial Velocities**: Set to zero (naturalistic cold start)
- **Agent Speeds**: Drawn from realistic distribution (mean ≈ 1.4 m/s, std ≈ 0.4 m/s per ETH benchmark)

### Behavior Normalization
All agents use standardized behavior:
- **Behavior Type**: "Regular" (treat other agents/obstacles like humans would)
- **Goal-Seeking**: Active (agents pursue pre-assigned goals)
- **Collision Avoidance**: Social Force Model (SFM) physics
- **Group Effects**: Disabled (isolated individuals, not group dynamics)

---

## SFM Parameters & How They Vary

### Parameter 1: Social Force (Tests 4-9)
**What it Controls**: Person-to-person collision avoidance strength

**Baseline Value**: 2.0 (balanced)

**Test Levels**:
| Test | Level | Value | Adjustment |
|------|-------|-------|------------|
| 4 | Low | 1.2 | -40% (weak avoidance) |
| 5 | Low-Mid | 1.6 | -20% |
| 6 | Mid | 2.0 | Baseline |
| 7 | Mid-High | 2.4 | +20% |
| 8 | High | 2.8 | +40% |
| 9 | Very High | 3.2 | +60% (strong avoidance) |

**Impact on Metrics**:
- ↑ Social Force → Lower collision rate, tighter spacing, slower speeds
- ↓ Social Force → Higher speeds, larger spacing, more collisions
- **Primary Effect**: Collision Rate, Near-Miss Rate
- **Secondary Effect**: Speed (inverse), Path Efficiency (slight)

---

### Parameter 2: Goal Force (Tests 10-15)
**What it Controls**: Strength of attraction toward navigation goal (directedness)

**Baseline Value**: 2.0 (balanced)

**Test Levels**:
| Test | Level | Value | Adjustment |
|------|-------|-------|------------|
| 10 | Low | 1.0 | -50% (weak goal) |
| 11 | Low-Mid | 1.5 | -25% |
| 12 | Mid | 2.0 | Baseline |
| 13 | Mid-High | 2.5 | +25% |
| 14 | High | 3.0 | +50% |
| 15 | Very High | 3.5 | +75% (strong goal) |

**Impact on Metrics**:
- ↑ Goal Force → Higher path efficiency (direct navigation), faster speeds, less oscillation
- ↓ Goal Force → Lower efficiency (wandering), slower goal convergence
- **Primary Effect**: Path Efficiency, Speed Mean
- **Secondary Effect**: Acceleration (directedness), Collision Rate (less time to react)

---

### Parameter 3: Walking Speed (Tests 16-21)
**What it Controls**: Maximum walking velocity agents can achieve

**Baseline Value**: 1.5 m/s (realistic pedestrian pace)

**Test Levels**:
| Test | Level | Value | Adjustment |
|------|-------|-------|------------|
| 16 | Slow | 0.9 m/s | -40% (elderly/cautious) |
| 17 | Slow-Med | 1.2 m/s | -20% |
| 18 | Med | 1.5 m/s | Baseline |
| 19 | Med-Fast | 1.8 m/s | +20% |
| 20 | Fast | 2.1 m/s | +40% (hurried) |
| 21 | Very Fast | 2.4 m/s | +60% (rushing) |

**Impact on Metrics**:
- ↑ Speed → Faster completion, higher collisions (less time to avoid), higher acceleration variance
- ↓ Speed → Slower completion, lower collisions, smoother motion
- **Primary Effect**: Speed Mean/Std, Collision Rate (inverse)
- **Secondary Effect**: Path Efficiency (slight), Acceleration Jerk (inverse)

---

## Evaluation Metrics Explained

### Speed Metrics

**Speed Mean** (m/s)
- **Definition**: Average velocity across all agents over entire run
- **Calculation**: Sum of |velocity| / number of time steps
- **Why it matters**: Fundamental validation—are agents walking at human pace?
- **Target**: 1.46 m/s (ETH Univ benchmark)
- **Current**: 0.83 m/s (43% too slow)
- **Sensitivity**: Tests 16-21 will directly vary this

**Speed Std** (m/s)
- **Definition**: Standard deviation of velocity across all agents
- **Calculation**: sqrt(variance of speeds)
- **Why it matters**: Detects oscillation or jerky motion
- **Target**: 0.41 m/s (ETH Univ benchmark)
- **Current**: 0.37 m/s (acceptable)
- **Sensitivity**: Sensitive to goal/social force balance

**Speed Variability** (Coefficient of Variation)
- **Definition**: Std / Mean (normalized speed variance)
- **Why it matters**: Independent of absolute speed—detects consistency
- **Target**: 0.163 (ETH Univ)
- **Current**: 0.30 (85% too variable—sign of oscillation)
- **Sensitivity**: Primary indicator of parameter imbalance

---

### Navigation Efficiency Metrics

**Path Efficiency**
- **Definition**: Direct displacement / Arc length traveled
- **Calculation**: Euclidean distance(start, goal) / sum of step distances
- **Range**: 0 (circular loops) to 1 (perfect straight line)
- **Why it matters**: Detects oscillation/backtracking—agents moving but not toward goal
- **Target**: 0.966 (ETH Univ—nearly straight paths)
- **Current**: 0.067 (93% loss—agents oscillate!)
- **Root Cause**: Goal force vs. social force imbalance
- **Sensitivity**: Tests 10-15 (goal force) will fix this

**Acceleration Mean** (m/s²)
- **Definition**: Average magnitude of acceleration
- **Calculation**: Sum of |dv/dt| / number of steps
- **Why it matters**: Smoothness of motion—high values indicate jerky behavior
- **Target**: 0.80 m/s² (ETH Univ)
- **Current**: 0.39 m/s² (acceptable but smooth)
- **Sensitivity**: Moderate sensitivity to goal/social force balance

**Acceleration Jerk** (m/s³)
- **Definition**: Rate of change of acceleration (third derivative of position)
- **Why it matters**: Detects oscillation—jerky acceleration changes
- **Sensitivity**: High sensitivity to parameter imbalance (especially goal/social force)

---

### Safety & Collision Metrics

**Collision Rate** (/ped/s)
- **Definition**: Number of collision events per pedestrian per second
- **Calculation**: (Collision count) / (Agent count × Duration)
- **Why it matters**: Validates avoidance behavior—too high = unsafe model
- **Target**: 0.0004 /ped/s (ETH Univ—near-zero)
- **Current**: 0.11 /ped/s (25,250× too high!)
- **Root Cause**: Weak social force + oscillation prevents avoidance
- **Sensitivity**: Tests 4-9 (social force) will improve this

**Near-Miss Rate** (/ped/s)
- **Definition**: Events where agents pass within 0.5m without collision
- **Calculation**: (Near-miss count) / (Agent count × Duration)
- **Why it matters**: Detects failed avoidance even without collision
- **Target**: 0.0139 /ped/s (ETH Univ)
- **Current**: 0.65 /ped/s (4,600× too high!)
- **Sensitivity**: Tests 4-9 (social force) primary driver

**Intimate/Personal/Social Space Intrusions**
- **Intimate** (0-0.45m): Very close approach
- **Personal** (0.45-1.2m): Personal space violation
- **Social** (1.2-3.6m): Social distance violation
- **Why it matters**: Cultural proxemics—models need realistic spacing behavior
- **Targets**: Near-zero intrusions (realistic crowds maintain distance)
- **Sensitivity**: Tests 4-9 (social force) and 16-21 (speed)

---

### Spacing Metrics

**Min Distance Mean** (m)
- **Definition**: Average of minimum inter-agent distances
- **Calculation**: For each agent, find closest agent; average across all agents
- **Why it matters**: Validates social spacing behavior
- **Target**: 1.88 m (ETH Univ—comfortable spacing)
- **Current**: 1.23 m (acceptable but tight)
- **Sensitivity**: Primary sensitivity to social force

**Min Distance P5** (m)
- **Definition**: 5th percentile of minimum inter-agent distances
- **Why it matters**: Detects rare extreme proximity events
- **Target**: 0.60 m (ETH Univ—acceptable minimum)
- **Current**: 0.46 m (acceptable)
- **Sensitivity**: Tests 4-9 (social force)

---

### Stage 1: Generate Test Scenarios
```bash
python3 generate_oat_scenarios.py
# Creates 7 YAML configuration files (one per parameter level)
# Tests 4-21 read from: src/hunav_gazebo_wrapper/scenarios/domenic/cafe_oat_*.yaml
```

### Stage 2: Run All 21 Tests

Start with tests 1-3 (baseline), then proceed through parameter variations. For each test:

```bash
# Terminal 1: Launch simulator
ros2 launch hunav_gazebo_wrapper cafe_no_robot.launch.py \
  configuration_file:=domenic/cafe/cafe_oat_[CONFIG].yaml

# Terminal 2: Start recording (repeat for each run)
ros2 service call /hunav_start_recording hunav_msgs/srv/StartRecording \
  "{experiment_tag: baseline_oat, run_id: [1-21]}" && \
sleep 120 && \
ros2 service call /hunav_stop_recording std_srvs/srv/Empty {}
```

Results saved to `results/run_[ID]/true_pos_.csv`

### Stage 3: Compare to Ground Truth

After each test completes:
```bash
python3 simulation_analysis/compare_to_ground_truth.py [run_id]
```

After all 21 tests:
```bash
python3 simulation_analysis/compare_to_ground_truth.py
# Creates batch comparison: all 21 runs vs ETH/UCY benchmark
```

### Stage 4: Analyze Results

Review comparison reports:
- **JSON**: `simulation_analysis/comparisons/simulation_vs_ground_truth.json`
- **Text**: `simulation_analysis/reports/comparison_report.txt`

### Stage 5: Generate Overlay Plots (Optional)

Visualize simulation vs ground truth:
```bash
python3 simulation_analysis/overlay_ground_truth.py [run_ids]
# Generates overlay plots in simulation_analysis/plots/
```

Outputs:
- Speed distributions (histogram + cumulative)
- Metrics comparison (6 key metrics side-by-side)
- Ground truth (blue dashed) vs simulation (orange)

### Stage 6: Analyze & Interpret Results

Review comparison data to identify which parameter(s) most improve:
1. **Path Efficiency** (primary target)
2. **Collision Rate** (secondary target)
3. **Speed realism** (tertiary target)

## Directory Structure

```
├── README.md                          # This file
│
├── src/
│   ├── hunav_sim/                    # Core SFM simulation (ROS2)
│   ├── hunav_gazebo_wrapper/         # Gazebo wrapper + scenarios
│   └── people/                       # ROS2 people messages (external)
│
├── simulation_analysis/              # Ground truth comparison
│   ├── compare_to_ground_truth.py    # Comparison script
│   ├── comparisons/                  # Output: JSON data
│   └── reports/                      # Output: Text reports
│
├── results/                          # Simulation outputs
│   └── run_N/
│       └── true_pos_.csv             # Pedestrian trajectories
│
└── src/DATASETS/                     # Ground truth benchmarks
    ├── eth/                          # ETH datasets (2 scenarios)
    ├── ucy/                          # UCY datasets (5 scenarios)
    └── results/
        └── ground_truth_metrics.json # Benchmark metrics
```

## Key Metrics

Each run compares 10 metrics to ETH Univ benchmark (360 pedestrians, 773 seconds):

| Metric | Benchmark | Current | Target |
|--------|-----------|---------|--------|
| Speed Mean | 1.46 m/s | 0.83 m/s | +76% |
| Collision Rate | 0.0004 /ped/s | 0.11 /ped/s | -99.6% |
| Path Efficiency | 0.966 | 0.067 | +1,340% |
| Speed Std | 0.41 m/s | 0.37 m/s | OK |
| Acceleration | 0.80 m/s² | 0.39 m/s² | +105% |
| Near-Miss Rate | 0.014 /ped/s | 0.65 /ped/s | -97.8% |
| Min Distance Mean | 1.88 m | 1.23 m | OK |
| Speed Variability | 0.163 CV | 0.30 CV | -46% |
| Min Distance P5 | 0.60 m | 0.46 m | OK |
| Acceleration Jerk | 0.47 m/s³ | 0.39 m/s³ | OK |

## How to Interpret Results

### After Each Test Run

1. **Check Path Efficiency First**
   - If < 0.5: Parameters favor oscillation—likely bad balance
   - If 0.8-0.96: Good parameter set
   - If > 0.96: Goal force may be too high (unrealistic straight lines)

2. **Check Collision Rate**
   - If > 0.01: Social force too weak—agents colliding
   - If < 0.0001: Social force may be too strong (unrealistic frozen behavior)
   - Target: 0.0004-0.001

3. **Check Speed Mean**
   - If < 1.0 m/s: Model walking too slowly
   - If 1.3-1.6 m/s: Realistic pace
   - If > 1.8 m/s: Unrealistically fast (rushing)

### After All 21 Tests

Compare test results to identify:
- **Best Path Efficiency**: Which parameter set minimizes oscillation?
- **Best Safety**: Which parameters minimize collisions while maintaining realism?
- **Best Speed Realism**: Which speed parameter matches ETH Univ distribution?
- **Overall Winner**: Single parameter combination optimizing all three metrics

## Components

| Component | Purpose |
|-----------|---------|
| [hunav_sim](src/hunav_sim) | Core SFM pedestrian simulation |
| [hunav_gazebo_wrapper](src/hunav_gazebo_wrapper) | Gazebo integration & world generation |
| [hunav_evaluator](src/hunav_sim/hunav_evaluator) | Metrics computation (20+ metrics) |
| [hunav_rviz2_panel](src/hunav_sim/hunav_rviz2_panel) | Interactive agent configuration UI |
| [simulation_analysis](simulation_analysis) | Ground truth comparison framework |

## Ground Truth Datasets

All preserved read-only in `src/DATASETS/`:

1. **eth_univ** ← Default for comparisons (360 ped, 773 s)
2. eth_hotel (389 ped, 722 s)
3. ucy_zara01, ucy_zara02, ucy_zara03 (148-204 ped, 300-420 s)
4. ucy_univ_s1, ucy_univ_s3 (415-434 ped, 177-216 s)

## Data Integrity

✓ Ground truth datasets are **read-only** — never modified by analysis
✓ All outputs go to **simulation_analysis/** — separate workspace
✓ Only pedestrian trajectories needed: **true_pos_.csv** (robot metrics optional, disabled by default)

## Notes

- Ground truth stays clean: `src/DATASETS/` untouched
- Comparisons isolated: `simulation_analysis/` workspace
- Trajectory output: `true_pos_.csv` (ETH/UCY format)
- Robot metrics: Optional, disabled in evaluator by default (`export_robot_metrics: false`)
- Behavior tree definitions: See `src/hunav_gazebo_wrapper/behavior_trees/`

## Next Steps

1. Run `generate_oat_scenarios.py` if not already done
2. Execute tests 1-21 using the workflow above
3. After all 21 complete: `python3 simulation_analysis/compare_to_ground_truth.py`
4. Analyze comparison output to identify optimal parameter values
5. Re-run final optimization tests with identified parameters
6. Document findings for thesis methodology section

## Dependencies

- **ROS2 Humble** with colcon
- **Gazebo 11**
- **Python 3.10+** with numpy, pandas, matplotlib
- **Social Force Model** (lightsfm)

## For More Details

- [Pedestrian Dynamics](src/hunav_sim/README.md) — SFM implementation
- [Gazebo Integration](src/hunav_gazebo_wrapper/README.md) — World generation
- [Metrics Evaluation](src/hunav_sim/hunav_evaluator/README.md) — Metric computation
- [Ground Truth Comparison](simulation_analysis/README.md) — Analysis framework

---

**Reference Dataset Citation:**

Jia, S., Song, C., et al. (2007). Social Force Model for Pedestrian Dynamics. 
Physical Review E, 51(5), 4282-4286.

ETH Dataset: D. Helbing & P. Molnár (1995)
UCY Dataset: A. Solmaz et al. (2012)
