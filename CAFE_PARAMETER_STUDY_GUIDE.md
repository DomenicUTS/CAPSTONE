# SFM Parameter Study Implementation Guide
## For Café No-Robot Baseline

**Generated:** March 2026  
**Status:** Ready to run

---

## Overview

This guide describes how to run the parametric validation study for Social Force Model (SFM) on the café environment with no robot. The study tests 5 behavioral profiles across 3 runs each = 15 simulations total.

**Profile Definitions:**
- **Balanced:** Middle-ground realistic behavior (ETH/UCY baseline)
- **Cautious:** Safe, polite crowds (high social awareness)
- **Aggressive:** Fast, impatient, risky behavior
- **Dense-Aware:** Adapts to crowding dynamically
- **Speed-First:** Prioritizes goal over safety

Each profile uses 25 agents with parameters sampled from normal distributions as specified in [README_PARAMETER_STUDY.md](../../../README_PARAMETER_STUDY.md).

---

## File Structure

```
src/hunav_gazebo_wrapper/scenarios/domenic/
├── cafe_24agents_balanced.yaml
├── cafe_24agents_cautious.yaml
├── cafe_24agents_aggressive.yaml
├── cafe_24agents_dense_aware.yaml
└── cafe_24agents_speed_first.yaml

src/DATASETS/domenic_analysis/
├── generate_cafe_scenarios.py        # (already run)
├── compare_results.py                 # Comparison tool
├── run_batch_cafe.sh                  # Batch runner script
└── simulations/                       # Output directory (created after runs)
    ├── balanced/cafe/run_1-3/
    ├── cautious/cafe/run_1-3/
    ├── aggressive/cafe/run_1-3/
    ├── dense_aware/cafe/run_1-3/
    └── speed_first/cafe/run_1-3/
```

---

## Quick Start: Run One Profile

### Step 1: Terminal Setup

**Terminal 1 - Gazebo (with specific profile):**
```bash
cd /home/domenic/sfm_ws_fresh
source install/setup.bash
ros2 launch hunav_gazebo_wrapper cafe_no_robot.launch.py \
  configuration_file:=domenic/cafe_24agents_balanced.yaml
```
Wait for Gazebo window to open with 25 agents visible.

**Terminal 2 - Evaluator:**
```bash
cd /home/domenic/sfm_ws_fresh
source install/setup.bash
ros2 launch hunav_evaluator hunav_evaluator.launch.py
```
Wait for "Hunav evaluator node started".

### Step 2: Run 3 Trials

**Terminal 3 - Record Data:**

**Trial 1 (Start):**
```bash
ros2 service call /hunav_start_recording hunav_msgs/srv/StartEvaluation \
  "{experiment_tag: 'balanced_cafe_run1', run_id: 1, robot_goal: {}}"
```
Wait 60 seconds.

**Trial 1 (Stop):**
```bash
ros2 service call /hunav_stop_recording std_srvs/srv/Empty "{}"
```

**Trial 2 (Start):**
```bash
ros2 service call /hunav_start_recording hunav_msgs/srv/StartEvaluation \
  "{experiment_tag: 'balanced_cafe_run2', run_id: 2, robot_goal: {}}"
```
Wait 60 seconds.

**Trial 2 (Stop):**
```bash
ros2 service call /hunav_stop_recording std_srvs/srv/Empty "{}"
```

**Trial 3 (Start):**
```bash
ros2 service call /hunav_start_recording hunav_msgs/srv/StartEvaluation \
  "{experiment_tag: 'balanced_cafe_run3', run_id: 3, robot_goal: {}}"
```
Wait 60 seconds.

**Trial 3 (Stop):**
```bash
ros2 service call /hunav_stop_recording std_srvs/srv/Empty "{}"
```

### Step 3: Organize Results

```bash
# Create output directory
mkdir -p /home/domenic/sfm_ws_fresh/src/DATASETS/domenic_analysis/simulations/balanced/cafe

# Move all runs
mv /home/domenic/sfm_ws_fresh/results/run_1 /home/domenic/sfm_ws_fresh/src/DATASETS/domenic_analysis/simulations/balanced/cafe/
mv /home/domenic/sfm_ws_fresh/results/run_2 /home/domenic/sfm_ws_fresh/src/DATASETS/domenic_analysis/simulations/balanced/cafe/
mv /home/domenic/sfm_ws_fresh/results/run_3 /home/domenic/sfm_ws_fresh/src/DATASETS/domenic_analysis/simulations/balanced/cafe/
```

### Step 4: Repeat for Other Profiles

For each of the remaining 4 profiles:
1. In Terminal 1, stop Gazebo and restart with new `configuration_file`:
   ```bash
   ros2 launch hunav_gazebo_wrapper cafe_no_robot.launch.py \
     configuration_file:=domenic/cafe_24agents_cautious.yaml
   ```
2. Repeat Terminal 3 steps (3 trials per profile)
3. Organize results in `simulations/[profile_name]/cafe/`

---

## Batch Mode: Automate All 15 Runs

For faster execution, use the batch runner script:

```bash
cd /home/domenic/sfm_ws_fresh
bash src/DATASETS/domenic_analysis/run_batch_cafe.sh
```

**Prerequisites:**
- Terminal 1 running Gazebo with a profile scenario
- Terminal 2 running hunav_evaluator
- Script will cycle through profiles and manage recordings

**Note:** The batch script will need to be updated to handle profile switching in Terminal 1 programmatically. Until then, manual mode (above) is recommended.

---

## Analyze Results

After running all simulations:

```bash
python3 /home/domenic/sfm_ws_fresh/src/DATASETS/domenic_analysis/compare_results.py
```

**Output:**
- Console table: Metrics vs. ETH/UCY ground truth ranges
- File: `parameter_study_report.txt` with alignment scores

**Example output:**
```
Profile: BALANCED
  Environment: cafe
    Metric              Run      Value           GT Range                Status
    mean_speed          run_1    1.23            0.64 - 1.46             ✓ IN RANGE
    collision_rate      run_1    0.015           0.0004 - 0.094          ✓ IN RANGE
    ...
    run_1: avg alignment = 85.3%
    run_2: avg alignment = 87.1%
    run_3: avg alignment = 84.9%
  Profile Average: 85.8%
```

---

## Understanding Parameter Distributions

Each profile samples agent parameters from normal distributions. For example, Balanced profile:

```
max_vel: N(1.05, 0.15) m/s
  → Sample: 0.90 to 1.20 m/s typical
  
social_force_factor: N(12.0, 2.0)
  → Sample: 10.0 to 14.0 typical
  → Higher = agents keep farther apart
  
goal_force_factor: N(3.0, 0.5)
  → Sample: 2.5 to 3.5 typical
  → Higher = agents more focused on reaching goal
  
obstacle_force_factor: N(15.0, 3.0)
  → Sample: 12.0 to 18.0 typical
  → Higher = agents avoid walls/furniture more
```

Each of the 25 agents in a profile gets a randomly sampled set of parameters.

---

## What the Generated Scenario Files Contain

Each YAML file (`cafe_24agents_*.yaml`) contains:

```yaml
hunav_loader:
  ros__parameters:
    yaml_base_name: cafe_24agents_balanced
    map: cafe              # Uses the cafe.world environment
    global_goals:          # 10 waypoint destinations
      1: {x: -3.7, y: -3.29}
      2: {x: 0.18, y: -4.85}
      ...
    agents:
      - agent1
      - agent2
      ... (25 agents total)
    agent1:
      id: 1
      max_vel: 1.03        # Sampled from profile distribution
      behavior:
        goal_force_factor: 3.12
        social_force_factor: 11.87
        obstacle_force_factor: 14.23
      init_pose:           # Random spawn position (validated)
        x: 2.145
        y: 3.871
      goals: [2, 5]        # Randomly selected goal pair
```

---

## Data Flow

```
1. generate_cafe_scenarios.py
   ├─→ Samples 25 agents per profile
   ├─→ Assigns random spawn poses (0.6m min separation)
   ├─→ Assigns random goal pairs
   └─→ Outputs: cafe_24agents_*.yaml

2. cafe_no_robot.launch.py
   ├─→ Loads cafe_24agents_[profile].yaml
   ├─→ Spawns 25 agents in Gazebo
   ├─→ Runs SFM plugin
   └─→ Outputs: agent trajectories (to evaluator)

3. hunav_evaluator
   ├─→ Records trajectories
   ├─→ On stop: computes metrics
   ├─→ Saves: results/run_N/
   │   ├─ true_pos_.csv (raw trajectories)
   │   └─ [experiment_tag].txt.csv (computed metrics)
   └─→ Output ready for analysis

4. compare_results.py
   ├─→ Reads all true_pos_.csv files
   ├─→ Extracts metrics
   ├─→ Compares to ETH/UCY ranges
   └─→ Outputs: alignment scores, summary report
```

---

## Ground Truth Ranges (ETH/UCY)

These are the expected ranges for realistic human crowds:

| Metric | Min | Max | Unit |
|--------|-----|-----|------|
| Mean speed | 0.64 | 1.46 | m/s |
| Speed std dev | 0.34 | 0.53 | m/s |
| Collision rate | 0.0004 | 0.094 | events/ped/s |
| Near-miss rate | 0.009 | 0.229 | events/ped/s |
| Path efficiency | 0.87 | 0.97 | - |
| Acceleration | 0.12 | 0.74 | m/s² |
| Min inter-agent distance | 0.80 | 1.88 | m |

**Goal:** After parameter tuning, the "Balanced" profile should have metrics within these ranges.

---

## Troubleshooting

**Q: Gazebo window doesn't open**
```bash
# Check that cafe.world exists and is valid
colcon build
source install/setup.bash
```

**Q: "Configuration file not found" error**
```bash
# Make sure you're using the relative path correctly
ros2 launch hunav_gazebo_wrapper cafe_no_robot.launch.py \
  configuration_file:=domenic/cafe_24agents_balanced.yaml
  # NOT: config.../path/to/file (hunav expands relative to scenarios/)
```

**Q: Recording fails with "service not found"**
```bash
# Ensure evaluator is running in Terminal 2
# Check: ros2 node list | grep evaluator
```

**Q: Results not saved**
```bash
# Check that /home/domenic/sfm_ws_fresh/results/ directory exists and is writable
ls -la /home/domenic/sfm_ws_fresh/results/
```

---

## What's Next After Cafe?

Once café baseline runs successfully, we can:

1. **Add other worlds:**
   - Create similar scenario files for warehouse, house, central_tunnel
   - Update launch files (or use parameterization)

2. **Extended parameter sweep:**
   - Run all 5 profiles × 4 environments × 3 runs = 60 total simulations
   - Compare which profiles work best in each environment

3. **Calibration:**
   - Identify which parameter values (means/stds) produce realistic metrics
   - Refine distributions based on results

4. **With robot:**
   - Test robot-aware behavior profiles (Surprised, Threatening, Curious)
   - Compare with robot present vs. robot absent

---

## Key Commands Reference

**Build:**
```bash
cd /home/domenic/sfm_ws_fresh && colcon build && source install/setup.bash
```

**Generate scenarios:**
```bash
python3 src/DATASETS/domenic_analysis/generate_cafe_scenarios.py
```

**Run simulation (Balanced profile):**
```bash
# Terminal 1
ros2 launch hunav_gazebo_wrapper cafe_no_robot.launch.py \
  configuration_file:=domenic/cafe_24agents_balanced.yaml

# Terminal 2
ros2 launch hunav_evaluator hunav_evaluator.launch.py

# Terminal 3
ros2 service call /hunav_start_recording hunav_msgs/srv/StartEvaluation \
  "{experiment_tag: 'balanced_cafe_run1', run_id: 1, robot_goal: {}}"
# Wait 60s
ros2 service call /hunav_stop_recording std_srvs/srv/Empty "{}"
```

**Analyze results:**
```bash
python3 src/DATASETS/domenic_analysis/compare_results.py
```

---

## Status

✅ **Completed:**
- Scenario generation system for all 5 profiles
- 5 parametric YAML files created
- Comparison and analysis scripts
- Documentation

⏳ **Ready to run:**
- Manual execution of 3 runs per profile (15 total)
- Results will be collected in `simulations/` directory
- Analysis can be run after any profile completes

🔄 **Next:** Run the café baseline to validate metrics, then extend to other environments.

---

*Guide Version: 1.0 | SFM Parameter Study for Assistive Robotics*
