# Domenic Analysis: SFM Parameter Study

Directory for SFM validation study against ETH/UCY datasets for café no-robot environment.

## Contents

### Scripts

- **`generate_cafe_scenarios.py`** — Generates 5 parametric scenario YAML files
  - Samples 25 agents per profile from normal distributions
  - Validates spawn positions (0.6m min separation)
  - Outputs to: `src/hunav_gazebo_wrapper/scenarios/domenic/`
  - Run once (already done)

- **`compare_results.py`** — Analyzes simulation results and compares to ground truth
  - Reads all `true_pos_.csv` files from simulation runs
  - Extracts metrics and computes alignment scores
  - Generates comparison table and summary report
  - Usage: `python3 compare_results.py` from workspace root

- **`run_batch_cafe.sh`** — Batch runner for automating all simulations
  - Cycles through 5 profiles × 3 runs = 15 runs
  - Requires pre-running Terminal 1 (Gazebo) + Terminal 2 (Evaluator)
  - Usage: `bash run_batch_cafe.sh`

### Generated Scenario Files

Located in: `src/hunav_gazebo_wrapper/scenarios/domenic/`

```
cafe_24agents_balanced.yaml      # 25 agents, Balanced profile
cafe_24agents_cautious.yaml      # 25 agents, Cautious profile
cafe_24agents_aggressive.yaml    # 25 agents, Aggressive profile
cafe_24agents_dense_aware.yaml   # 25 agents, Dense-Aware profile
cafe_24agents_speed_first.yaml   # 25 agents, Speed-First profile
```

Each file defines:
- Agent count: 25
- Parameter distributions: sampled normally
- Spawn positions: randomly placed with 0.6m separation
- Goal pairs: randomly selected from pool of 10 waypoints
- Behavior: All Regular (no robot interactions)

### Results Directory (After Runs)

`simulations/` — Created after first simulation run

```
simulations/
├── balanced/cafe/
│   ├── run_1/  → results/run_1/ (moved after trial 1)
│   ├── run_2/  → results/run_2/ (moved after trial 2)
│   └── run_3/  → results/run_3/ (moved after trial 3)
├── cautious/cafe/
│   ├── run_1/
│   ├── run_2/
│   └── run_3/
├── aggressive/cafe/
│   ├── run_1/
│   ├── run_2/
│   └── run_3/
├── dense_aware/cafe/
│   ├── run_1/
│   ├── run_2/
│   └── run_3/
└── speed_first/cafe/
    ├── run_1/
    ├── run_2/
    └── run_3/
```

Each run contains:
- `true_pos_.csv` — Raw trajectories (ETH/UCY format)
- `[experiment_tag].txt.csv` — Computed metrics
- `[experiment_tag]_steps_*.csv` — Per-frame metrics

## Quick Start

### 1. Run One Profile (Manual)

See [../../../CAFE_PARAMETER_STUDY_GUIDE.md](../../../CAFE_PARAMETER_STUDY_GUIDE.md) for detailed steps.

**Short version:**
```bash
# Terminal 1
cd /home/domenic/sfm_ws_fresh
source install/setup.bash
ros2 launch hunav_gazebo_wrapper cafe_no_robot.launch.py \
  configuration_file:=domenic/cafe_24agents_balanced.yaml

# Terminal 2
source install/setup.bash
ros2 launch hunav_evaluator hunav_evaluator.launch.py

# Terminal 3 (after ~5s)
# Start recording
ros2 service call /hunav_start_recording hunav_msgs/srv/StartEvaluation \
  "{experiment_tag: 'balanced_cafe_run1', run_id: 1, robot_goal: {}}"

# After 60s
ros2 service call /hunav_stop_recording std_srvs/srv/Empty "{}"

# Move results
mkdir -p /home/domenic/sfm_ws_fresh/src/DATASETS/domenic_analysis/simulations/balanced/cafe
mv /home/domenic/sfm_ws_fresh/results/run_1 \
   /home/domenic/sfm_ws_fresh/src/DATASETS/domenic_analysis/simulations/balanced/cafe/
```

### 2. Run All 5 Profiles (Batch)

```bash
cd /home/domenic/sfm_ws_fresh
bash src/DATASETS/domenic_analysis/run_batch_cafe.sh
```

### 3. Analyze Results

After all runs complete:

```bash
python3 src/DATASETS/domenic_analysis/compare_results.py
```

Output:
- Console table showing metrics vs. ETH/UCY ranges
- File: `parameter_study_report.txt`

## Profile Definitions

Each profile specifies parameter distributions for 25 agents:

| Profile | max_vel | social_factor | goal_factor | obstacle_factor |
|---------|---------|---------------|-------------|-----------------|
| Balanced | N(1.05, 0.15) | N(12.0, 2.0) | N(3.0, 0.5) | N(15.0, 3.0) |
| Cautious | N(0.85, 0.12) | N(16.0, 2.5) | N(2.5, 0.4) | N(20.0, 4.0) |
| Aggressive | N(1.35, 0.15) | N(8.0, 2.0) | N(3.5, 0.5) | N(10.0, 3.0) |
| Dense-Aware | N(0.95, 0.20) | N(14.0, 3.0) | N(3.2, 0.5) | N(18.0, 3.5) |
| Speed-First | N(1.40, 0.18) | N(10.0, 2.5) | N(4.0, 0.6) | N(12.0, 3.5) |

All parameters are clipped to reasonable bounds (e.g., velocity 0.5-2.0 m/s).

## Expected Workflow

```
1. Setup
   ├─ colcon build
   └─ source install/setup.bash

2. Generate Scenarios (DONE)
   └─ python3 generate_cafe_scenarios.py

3. Run Simulations (15-30 mins)
   ├─ Terminal 1: Launch Gazebo with profile
   ├─ Terminal 2: Launch Evaluator
   └─ Terminal 3: Record 3 runs per profile
       └─ Move results to simulations/[profile]/cafe/

4. Analyze Results (1 min)
   └─ python3 compare_results.py
       └─ Outputs: alignment scores, comparison table

5. Interpret
   ├─ Which profiles match ETH/UCY?
   ├─ Which parameters need tuning?
   └─ Plan next experiments (other environments, variations)
```

## Ground Truth Reference

Expected metric ranges from ETH/UCY pedestrian datasets:

| Metric | Min | Max | Unit |
|--------|-----|-----|------|
| Mean speed | 0.64 | 1.46 | m/s |
| Speed std | 0.34 | 0.53 | m/s |
| Collision rate | 0.0004 | 0.094 | events/ped/s |
| Near-miss rate | 0.009 | 0.229 | events/ped/s |
| Path efficiency | 0.87 | 0.97 | - |
| Acceleration | 0.12 | 0.74 | m/s² |
| Min distance | 0.80 | 1.88 | m |

Goal: Balanced profile should have metrics within these ranges.

## Troubleshooting

### Scenarios not found
```bash
# Regenerate
python3 generate_cafe_scenarios.py
```

### Simulation crashes
```bash
# Rebuild and reinstall
cd /home/domenic/sfm_ws_fresh
colcon build --symlink-install
source install/setup.bash
```

### Recording fails
- Check evaluator is running: `ros2 node list | grep evaluator`
- Check Gazebo is running: `ros2 node list | grep world_generator`

### Results not written
- Check `/home/domenic/sfm_ws_fresh/results/` exists and is writable
- Check hunav_evaluator has write permissions

## Next Steps

After café baseline:

1. **Create profiles for other worlds:**
   - warehouse_24agents_*.yaml
   - house_24agents_*.yaml
   - central_tunnel_24agents_*.yaml

2. **Run extended study:**
   - 5 profiles × 4 environments × 3 runs = 60 total simulations

3. **Calibration:**
   - Identify which profiles produce realistic metrics
   - Refine parameter distributions based on results

4. **With robot:**
   - Test robot-aware behaviors (Surprised, Threatening, Curious)
   - Compare dynamics with vs. without robot

## References

- ETH/UCY Datasets: http://www.vision.ee.ethz.ch/datasets/
- Social Force Model: Helbing & Molnár (1995)
- Parameter Study Design: README_PARAMETER_STUDY.md
- Implementation Guide: CAFE_PARAMETER_STUDY_GUIDE.md

---

*Status: Ready to run*  
*Last updated: March 28, 2026*
