# Domenic Parameter Study: SFM Validation Against Ground Truth

## Overview

This document describes the **parametric validation study** for Social Force Model (SFM) behavior calibration. The goal is to determine if different parameter "profiles" can match realistic crowd dynamics from ETH/UCY datasets across multiple environments.

**Key Features:**
- 5 strategic parameter profiles (not random tuning)
- 4 different environments (cafe, warehouse, house, central_tunnel)
- 3 runs per configuration for statistical confidence
- Hard-saved agent scenarios with validated spawn/goal positions
- Automated comparison against ground truth (ETH/UCY)
- Total: 5 × 4 × 3 = 60 simulation runs

---

## Parameter Profiles

Each profile represents a realistic behavioral strategy with internally varied parameters (normal distributions). Agents within each profile are heterogeneous (different speeds, force factors) but sample from the same distributional profile.

### Profile A: **Balanced** 
*Middle-ground realistic behavior (ETH/UCY baseline reference)*

- **Interpretation:** Represents typical crowd behavior as observed in ground truth datasets
- **Use case:** Baseline for comparison; should be closest to ETH/UCY
- **Parameter distributions:**
  - max_vel: N(1.05, 0.15) m/s
  - social_force_factor: N(12.0, 2.0)
  - goal_force_factor: N(3.0, 0.5)
  - obstacle_force_factor: N(15.0, 3.0)

### Profile B: **Cautious**
*Safe, polite crowds (high social awareness, low speed)*

- **Interpretation:** Conservative behavior with strong avoidance
- **Use case:** Test if SFM can represent courteous crowds (e.g., elderly care facilities)
- **Parameter distributions:**
  - max_vel: N(0.85, 0.12) m/s ← slower
  - social_force_factor: N(16.0, 2.5) ← strong avoidance
  - goal_force_factor: N(2.5, 0.4)
  - obstacle_force_factor: N(20.0, 4.0) ← cautious

### Profile C: **Aggressive**
*Fast, impatient, risky (low safety margins)*

- **Interpretation:** Rushing behavior with minimal personal space
- **Use case:** Test if SFM can represent hurried crowds (e.g., airport, train station rush hour)
- **Parameter distributions:**
  - max_vel: N(1.35, 0.15) m/s ← faster
  - social_force_factor: N(8.0, 2.0) ← weak avoidance
  - goal_force_factor: N(3.5, 0.5)
  - obstacle_force_factor: N(10.0, 3.0) ← risk-tolerant

### Profile D: **Dense-Aware**
*Adapts to crowding (balanced adaptive behavior)*

- **Interpretation:** Agents adjust behavior dynamically (mid-range on all axes)
- **Use case:** Test SFM robustness across densities
- **Parameter distributions:**
  - max_vel: N(0.95, 0.20) m/s ← moderate, variable
  - social_force_factor: N(14.0, 3.0) ← balanced, high variance
  - goal_force_factor: N(3.2, 0.5)
  - obstacle_force_factor: N(18.0, 3.5) ← balanced

### Profile E: **Speed-First**
*Prioritizes goal over safety (goal-driven commute)*

- **Interpretation:** Commuters with time pressure, willing to take risks
- **Use case:** Test if SFM can represent deadlock-prone behavior (e.g., morning commute)
- **Parameter distributions:**
  - max_vel: N(1.40, 0.18) m/s ← fastest
  - social_force_factor: N(10.0, 2.5) ← weak avoidance
  - goal_force_factor: N(4.0, 0.6) ← strong goal pull
  - obstacle_force_factor: N(12.0, 3.5) ← moderate

---

## Scenario Files

Pre-generated YAML configs in: `src/hunav_gazebo_wrapper/scenarios/domenic/`

```
balanced_agents.yaml           # 25 agents, Balanced profile
cautious_agents.yaml           # 25 agents, Cautious profile
aggressive_agents.yaml         # 25 agents, Aggressive profile
dense_aware_agents.yaml        # 25 agents, Dense-Aware profile
speed_first_agents.yaml        # 25 agents, Speed-First profile
```

**Common properties (all profiles):**
- **Agent count:** 25 (median realistic crowd)
- **Spawn validation:** 0.6m minimum separation (no overlaps)
- **Goal placement:** Clustered pool (agents can share destinations)
- **Behavior type:** All Regular (no robot-specific interactions)
- **Randomness:** Spawn positions random, orientations random, parameters sampled per agent

**To regenerate scenarios:**
```bash
python3 src/DATASETS/domenic_analysis/generate_parametric_scenarios.py
```

---

## Experimental Workflow

### Step 1: Create Environment Definitions
**You will handle this:** Create 4 scenario environment YAMLs defining obstacles/geometry

**Expected:** `src/hunav_gazebo_wrapper/scenarios/domenic/env_*.yaml`

Each should define:
- World boundaries
- Static obstacles (walls, furniture)
- Any fixed features (doors, etc.)

---

### Step 2: Run Simulations
**Manual process or bash automation**

```bash
# Option A: Manual (one run at a time)
ros2 launch hunav_gazebo_wrapper cafe_no_robot.launch.py \
  scenario:=scenarios/domenic/balanced_agents.yaml

# In Terminal B
ros2 launch hunav_evaluator hunav_evaluator.launch.py

# In Terminal C (after 5s)
ros2 service call /hunav_start_recording hunav_msgs/srv/StartEvaluation \
  "{experiment_tag: 'balanced_cafe_run1', run_id: 1, robot_goal: {}}"

# After 60s
ros2 service call /hunav_stop_recording std_srvs/srv/Empty "{}"

# Move results
mkdir -p src/DATASETS/domenic_analysis/simulations/balanced/cafe
mv results/run_1 src/DATASETS/domenic_analysis/simulations/balanced/cafe/
```

**Option B: Batch script (run all 60 at once)**
```bash
bash src/DATASETS/domenic_analysis/run_parameter_sweep.sh
```
*(To be created - will automate all launches)*

---

### Step 3: Extract Metrics & Compare

```bash
# Analyze all simulations + overlay with ground truth
python3 src/DATASETS/domenic_analysis/compare_all_runs.py
```

**Output:**
- Comparison table: Profile × Environment × Metric
- Metrics against ETH/UCY ranges
- PNG plots showing simulation vs reality distributions
- Summary: Which profiles match ground truth best

---

## Expected Output Structure

```
src/DATASETS/domenic_analysis/simulations/
├── balanced/
│   ├── cafe/
│   │   ├── run_1/
│   │   │   ├── true_pos_.csv          (raw trajectories)
│   │   │   ├── baseline_run1.txt.csv  (computed metrics)
│   │   │   └── ...
│   │   ├── run_2/
│   │   └── run_3/
│   ├── warehouse/
│   ├── house/
│   └── central_tunnel/
├── cautious/
│   ├── cafe/
│   │   ├── run_1/
│   │   ├── run_2/
│   │   └── run_3/
│   ├── ...
├── aggressive/
├── dense_aware/
└── speed_first/
```

---

## Analysis: What We're Testing

Each comparison answers:

| Question | Profile(s) to Test | Expected If SFM Works |
|----------|---|---|
| Does SFM match realistic crowds? | Balanced | Metrics within ETH/UCY ranges |
| Can SFM represent courtesy? | Cautious | Larger inter-agent distances, fewer collisions |
| Can SFM represent rush hour? | Aggressive | Higher speeds, more intrusions, faster |
| Does SFM adapt to density? | Dense-Aware | Consistent behavior across environments |
| Can SFM handle goal pressure? | Speed-First | Straighter paths, higher speeds, more collisions |

---

## Key Metrics to Track

**Tier 1 (Critical):**
- Mean speed (m/s)
- Speed std dev (m/s)
- Collision rate (events/ped/s)
- Near-miss rate (events/ped/s)

**Tier 2 (Important):**
- Path efficiency
- Acceleration (m/s²)
- Heading jerk (rad/frame³)

**Tier 3 (Useful):**
- Min inter-agent distance (m)
- Min distance 5th percentile (m)

---

## Implementation Notes

### Agent Generation Logic

```python
for profile in PROFILES:
    agents = []
    for i in range(25):
        # Sample parameters from profile distribution
        max_vel = sample_normal(profile.max_vel_mean, profile.max_vel_std)
        social_ff = sample_normal(profile.social_factor_mean, ...)
        # ... etc
        
        # Spawn position: random, validated (0.6m min separation)
        spawn = random_position()
        while any(distance(spawn, other_spawn) < 0.6) for other_spawn in agents:
            spawn = random_position()
        
        # Goal: pick from pool (agents can share)
        goal = random.choice(goal_pool)
        
        agent = Agent(id=i, spawn=spawn, goal=goal, params={...})
        agents.append(agent)
```

### Comparison Logic

```python
# After all runs collected:
for profile in profiles:
    for env in environments:
        for run in runs:
            # Load true_pos_.csv
            sim_metrics = ground_truth_analysis(true_pos_.csv)
            
            # Compare to ETH/UCY ground truth
            for metric in [speed, collision_rate, ...]:
                sim_value = sim_metrics[metric]
                eth_ucy_range = GROUND_TRUTH[metric]
                
                if eth_ucy_range.min <= sim_value <= eth_ucy_range.max:
                    status = "✓ Within range"
                else:
                    status = f"✗ Out of range (expected {eth_ucy_range})"
                
                print(f"{profile}/{env}/run{run}: {metric} = {sim_value} {status}")
```

---

## Timeline Estimate

- **Profile generation:** ✓ Done (5 YAML files)
- **Environment creation:** ~2-3 hours (you will do this)
- **Running 60 simulations:** ~2-3 hours (automated or ~1.5 min per run)
- **Analysis & comparison:** ~30 minutes (automated with `compare_all_runs.py`)
- **Interpretation & reporting:** ~2-4 hours (write-up results)

**Total:** ~1-2 days of active work

---

## Files Reference

| What | Location |
|------|----------|
| Profile YAMLs | `src/hunav_gazebo_wrapper/scenarios/domenic/` |
| Scenario generator | `src/DATASETS/domenic_analysis/generate_parametric_scenarios.py` |
| Comparison tool | `src/DATASETS/domenic_analysis/compare_all_runs.py` |
| Ground truth analyzer | `src/DATASETS/domenic_analysis/ground_truth_analysis.py` |
| Simulation launch | `src/hunav_gazebo_wrapper/launch/domenic_no_robot/cafe_no_robot.launch.py` |
| Results output | `src/DATASETS/domenic_analysis/simulations/` |

### Legacy/Archive

| What | Location |
|------|----------|
| Original scenarios (robot-dependent) | `src/hunav_gazebo_wrapper/scenarios/LEGACY_archived/` |
| Original launch (with robot) | `src/hunav_gazebo_wrapper/launch/LEGACY_archived/` |
| Original analysis scripts | `src/DATASETS/LEGACY_archived/` |

---

## Questions Before You Start

1. **Environments:** Will you create YAMLs for cafe, warehouse, house, central_tunnel? Or use existing maps?
2. **Timeline:** When do you want results?
3. **Extra validation:** Any specific metric comparisons beyond Tier 1?

---

*Study Design: Domenic (2026)*
*Reference: Helbing & Molnár (1995), ETH/UCY Pedestrian Datasets*
