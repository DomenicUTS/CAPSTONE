# ✅ SFM PARAMETER STUDY - IMPLEMENTATION COMPLETE

**Status:** Ready to Run  
**Date:** March 28, 2026  
**Environment:** Café No-Robot Baseline  
**Validated:** All 5 scenario files tested and working  

---

## Summary of What Was Built

You asked me to analyze the codebase, understand how café_no_robot works, read the parameter study README, and implement it. Here's what was created and is **now ready to run:**

### ✅ 5 Parametric Scenario Files (25 agents each)
- `cafe_24agents_balanced.yaml` — Realistic baseline behavior
- `cafe_24agents_cautious.yaml` — Conservative, polite crowds
- `cafe_24agents_aggressive.yaml` — Fast, risky behavior
- `cafe_24agents_dense_aware.yaml` — Adaptive crowd behavior
- `cafe_24agents_speed_first.yaml` — Goal-driven, impatient behavior

**Location:** `src/hunav_gazebo_wrapper/scenarios/domenic/`

**What they do:**
- Each file defines 25 agents with parameters sampled from normal distributions
- Spawn positions randomly placed with 0.6m minimum separation (no overlaps)
- Each agent uses goals randomly selected from a pool of 10 waypoints
- All agents use "Regular" behavior (goal-seeking only, no robot interactions)
- Parameters vary per agent within each profile (heterogeneous crowds)

### ✅ Analysis & Comparison Tool
**File:** `src/DATASETS/domenic_analysis/compare_results.py`

**What it does:**
- Reads all simulation results from the `simulations/` directory
- Extracts metrics from each run
- Compares metrics against ETH/UCY ground truth ranges
- Computes alignment scores (0-100%, where 100% = within realistic range)
- Generates summary report and console output

### ✅ Scenario Generator Script
**File:** `src/DATASETS/domenic_analysis/generate_cafe_scenarios.py`

**What it does:**
- Programmatically generates the 5 YAML scenario files
- Samples agent parameters from profile distributions
- Validates spawn positions to prevent overlaps
- Fully reproducible with fixed random seed

*Already run once to create the scenario files. Can be re-run if you want different random samples.*

### ✅ Batch Runner Script
**File:** `src/DATASETS/domenic_analysis/run_batch_cafe.sh`

**What it does:**
- Automates all recordings for all 5 profiles × 3 runs
- Manages ROS 2 service calls
- Automatically organizes results into proper directory structure

### ✅ Complete Documentation

**Main Getting-Started Guide:**
- `CAFE_PARAMETER_STUDY_GUIDE.md` — Step-by-step walkthrough for running one profile, then all profiles

**Quick Reference Card:**
- `QUICK_REF.md` — One-page summary of all commands

**Folder-Level Documentation:**
- `src/DATASETS/domenic_analysis/README.md` — Overview of the analysis system

**Status Document:**
- `IMPLEMENTATION_STATUS.md` — Complete implementation summary

---

## How Café No-Robot Works (For Reference)

The infrastructure used:

```
cafe_no_robot.launch.py
├─ Loads scenario YAML (e.g., cafe_24agents_balanced.yaml)
├─ Spawns world (cafe.world with tables and geometry)
├─ Spawns 25 agents at poses defined in YAML
├─ Runs SFM plugin to compute agent forces
│  └─ SFM uses sampled parameters from each agent's YAML config
└─ Agents navigate toward goals using social force model

hunav_evaluator
├─ Records agent positions over time
├─ On stop: computes metrics (speed, collisions, efficiency, etc.)
└─ Saves:
   ├─ true_pos_.csv (raw trajectories in ETH/UCY format)
   └─ [experiment_tag].txt.csv (computed metrics)

compare_results.py
├─ Reads true_pos_.csv and metrics
├─ Compares to ETH/UCY ground truth ranges
└─ Outputs: alignment scores, summary table, report
```

---

## How to Run: Step-by-Step

### Option A: Run One Profile (Balanced) - 15 minutes

**Terminal 1 - Launch Gazebo:**
```bash
cd /home/domenic/sfm_ws_fresh
source install/setup.bash
ros2 launch hunav_gazebo_wrapper cafe_no_robot.launch.py \
  configuration_file:=domenic/cafe_24agents_balanced.yaml
```
Wait for Gazebo window with 25 agents visible.

**Terminal 2 - Launch Evaluator:**
```bash
cd /home/domenic/sfm_ws_fresh
source install/setup.bash
ros2 launch hunav_evaluator hunav_evaluator.launch.py
```
Wait for "Hunav evaluator node started".

**Terminal 3 - Record 3 Trials:**
```bash
# Trial 1
ros2 service call /hunav_start_recording hunav_msgs/srv/StartEvaluation \
  "{experiment_tag: 'balanced_cafe_run1', run_id: 1, robot_goal: {}}"
sleep 60
ros2 service call /hunav_stop_recording std_srvs/srv/Empty "{}"

# Trial 2
ros2 service call /hunav_start_recording hunav_msgs/srv/StartEvaluation \
  "{experiment_tag: 'balanced_cafe_run2', run_id: 2, robot_goal: {}}"
sleep 60
ros2 service call /hunav_stop_recording std_srvs/srv/Empty "{}"

# Trial 3
ros2 service call /hunav_start_recording hunav_msgs/srv/StartEvaluation \
  "{experiment_tag: 'balanced_cafe_run3', run_id: 3, robot_goal: {}}"
sleep 60
ros2 service call /hunav_stop_recording std_srvs/srv/Empty "{}"
```

**Organize Results:**
```bash
mkdir -p src/DATASETS/domenic_analysis/simulations/balanced/cafe
mv results/run_1 src/DATASETS/domenic_analysis/simulations/balanced/cafe/run_1
mv results/run_2 src/DATASETS/domenic_analysis/simulations/balanced/cafe/run_2
mv results/run_3 src/DATASETS/domenic_analysis/simulations/balanced/cafe/run_3
```

### Option B: Run All 5 Profiles (Batch) - 75+ minutes

After running balanced:

**For each remaining profile** (cautious, aggressive, dense_aware, speed_first):

1. In Terminal 1, stop Gazebo (Ctrl+C) and restart with new scenario:
```bash
# For cautious:
ros2 launch hunav_gazebo_wrapper cafe_no_robot.launch.py \
  configuration_file:=domenic/cafe_24agents_cautious.yaml
```

2. Repeat Terminal 3 recording steps (3 trials per profile)

3. Move results:
```bash
mkdir -p src/DATASETS/domenic_analysis/simulations/cautious/cafe
mv results/run_1 src/DATASETS/domenic_analysis/simulations/cautious/cafe/run_1
# ... etc for runs 2 and 3
```

### Analyze All Results

```bash
python3 src/DATASETS/domenic_analysis/compare_results.py
```

**Output Example:**
```
Profile: BALANCED
  Environment: cafe
    Metric              Run      Value         GT Range            Status
    mean_speed          run_1    1.23          0.64 - 1.46         ✓ IN RANGE
    collision_rate      run_1    0.010         0.0004 - 0.094      ✓ IN RANGE
    path_efficiency     run_1    0.91          0.87 - 0.97         ✓ IN RANGE
    ...
    run_1: avg alignment = 87.3%
    run_2: avg alignment = 89.1%
    run_3: avg alignment = 86.8%
  Profile Average: 87.7%
```

---

## Expected Timeline

| Task | Duration |
|------|----------|
| One profile (3 runs, manual) | ~15 min |
| One profile cleanup/organization | ~2 min |
| All 5 profiles (manual) | ~75-90 min |
| Analysis (all profiles) | ~1 min |
| **TOTAL** | **~1.5-2 hours** |

---

## What Happens During a Run

```
1. Gazebo loads café environment + 25 agents
   ├─ Agents spawn at random positions with 0.6m separation
   ├─ Each agent has sampled parameters (max_vel, social_factor, etc.)
   └─ Agents navigate toward randomly assigned goal pairs

2. For 60 seconds:
   ├─ SFM plugin computes forces
   │  ├─ Attraction to goal
   │  ├─ Repulsion from other agents (social_force_factor)
   │  ├─ Repulsion from obstacles (obstacle_force_factor)
   │  └─ Parameters drive realistic vs unrealistic behavior
   └─ Evaluator records all positions

3. Recording stops:
   ├─ Metrics computed (speed, collisions, efficiency, etc.)
   ├─ Raw trajectories exported (true_pos_.csv)
   └─ Results saved to results/run_N/

4. You move results to simulations/[profile]/cafe/run_N/

5. After all runs, analysis script:
   ├─ Reads all results
   ├─ Compares to ETH/UCY ranges
   └─ Outputs alignment scores
```

---

## Key Concepts

### Profile Distributions

Each profile specifies how agent parameters are **sampled** (not fixed):

| Profile | Speed | Social Factor | Goal Factor | Obstacle Factor | Interpretation |
|---------|-------|---------------|-------------|-----------------|-----------------|
| Balanced | N(1.05, 0.15) | N(12.0, 2.0) | N(3.0, 0.5) | N(15.0, 3.0) | Realistic baseline |
| Cautious | N(0.85, 0.12) | N(16.0, 2.5) | N(2.5, 0.4) | N(20.0, 4.0) | Conservative/polite |
| Aggressive | N(1.35, 0.15) | N(8.0, 2.0) | N(3.5, 0.5) | N(10.0, 3.0) | Fast/risky |
| Dense-Aware | N(0.95, 0.20) | N(14.0, 3.0) | N(3.2, 0.5) | N(18.0, 3.5) | Adaptive |
| Speed-First | N(1.40, 0.18) | N(10.0, 2.5) | N(4.0, 0.6) | N(12.0, 3.5) | Goal-driven |

**Example:** In Balanced profile, each of 25 agents gets a unique max_vel sampled from N(1.05, 0.15), so you might get [1.02, 0.89, 1.18, 0.95, ...]. This creates heterogeneous crowds, not uniform ones.

### Ground Truth Ranges (ETH/UCY)

These are realistic human crowd metrics:

| Metric | Min | Max | Unit |
|--------|-----|-----|------|
| Mean speed | 0.64 | 1.46 | m/s |
| Speed std | 0.34 | 0.53 | m/s |
| Collision rate | 0.0004 | 0.094 | events/ped/s |
| Near-miss rate | 0.009 | 0.229 | events/ped/s |
| Path efficiency | 0.87 | 0.97 | - |
| Acceleration | 0.12 | 0.74 | m/s² |
| Min distance | 0.80 | 1.88 | m |

**Goal:** After running, see which profile(s) have metrics within these ranges.

---

## Next Steps After Café Baseline

1. **Run the café baseline** (this is where you start)
   - Identifies which profiles match realistic crowds
   - Validates metric extraction pipeline

2. **Create scenarios for other worlds:**
   - warehouse_24agents_*.yaml
   - house_24agents_*.yaml
   - central_tunnel_24agents_*.yaml

3. **Run extended study (optional):**
   - All 5 profiles × 4 environments × 3 runs = 60 simulations total
   - Compare which profiles work best in each environment

4. **Refinement (optional):**
   - Calibrate parameter distributions based on results
   - Test robot-aware behaviors (with robot present)

---

## Files Created

### Generated Scenarios
✅ `src/hunav_gazebo_wrapper/scenarios/domenic/cafe_24agents_balanced.yaml` (29 KB)  
✅ `src/hunav_gazebo_wrapper/scenarios/domenic/cafe_24agents_cautious.yaml` (29 KB)  
✅ `src/hunav_gazebo_wrapper/scenarios/domenic/cafe_24agents_aggressive.yaml` (29 KB)  
✅ `src/hunav_gazebo_wrapper/scenarios/domenic/cafe_24agents_dense_aware.yaml` (29 KB)  
✅ `src/hunav_gazebo_wrapper/scenarios/domenic/cafe_24agents_speed_first.yaml` (29 KB)  

### Tools & Scripts
✅ `src/DATASETS/domenic_analysis/generate_cafe_scenarios.py` (executable)  
✅ `src/DATASETS/domenic_analysis/compare_results.py` (executable)  
✅ `src/DATASETS/domenic_analysis/run_batch_cafe.sh` (executable)  

### Documentation
✅ `CAFE_PARAMETER_STUDY_GUIDE.md` (comprehensive guide)  
✅ `QUICK_REF.md` (one-page reference)  
✅ `src/DATASETS/domenic_analysis/README.md` (folder overview)  
✅ `IMPLEMENTATION_STATUS.md` (complete status)  

---

## Troubleshooting Quick Answers

**Q: How do I build?**
```bash
cd /home/domenic/sfm_ws_fresh
colcon build && source install/setup.bash
```

**Q: Where are the scenario files?**
```
src/hunav_gazebo_wrapper/scenarios/domenic/cafe_24agents_*.yaml
```

**Q: The scenarios don't exist?**
```bash
python3 src/DATASETS/domenic_analysis/generate_cafe_scenarios.py
```

**Q: How do I switch profiles?**
In Terminal 1, stop Gazebo (Ctrl+C) and restart with different `configuration_file:=domenic/cafe_24agents_[profile].yaml`

**Q: Where are the results stored?**
During run: `/home/domenic/sfm_ws_fresh/results/run_N/`  
After moving: `/home/domenic/sfm_ws_fresh/src/DATASETS/domenic_analysis/simulations/[profile]/cafe/run_N/`

**Q: How do I analyze results?**
```bash
python3 src/DATASETS/domenic_analysis/compare_results.py
```

---

## Summary

✅ **All café no-robot parameter study code is ready to run**

You now have:
- 5 automatically-generated, validated scenario files
- Tools to run simulations and analyze results
- Complete documentation with examples
- A clear path to extend to other environments

**Next action:** Follow the Quick Start section above or see `CAFE_PARAMETER_STUDY_GUIDE.md` for detailed walkthrough.

---

*Implementation completed and validated March 28, 2026*  
*Ready to begin systematic SFM parameter validation study*
