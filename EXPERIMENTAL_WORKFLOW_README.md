# HuNav Simulation Experimental Workflow

## Overview

This document describes modifications to the HuNav simulation codebase for conducting **systematic parameter calibration and validation** against ETH/UCY pedestrian datasets. The workflow enables:

1. **Robot-free pedestrian simulation** (pure crowd dynamics)
2. **Automated raw trajectory export** (ETH/UCY format)
3. **Organized run tracking** (run_1/, run_2/, etc.)
4. **Batch comparison** of simulation vs. ground truth metrics

---

## Codebase Modifications

### 1. Robot-Free Launch File

**File:** `src/hunav_gazebo_wrapper/launch/cafe_no_robot.launch.py`

**Status:** Already exists in codebase

**Purpose:** Launches Gazebo with only pedestrian agents, no robot. Eliminates robot dependencies (e.g., `pmb2_description`) that caused launch failures.

**Usage:**
```bash
ros2 launch hunav_gazebo_wrapper cafe_no_robot.launch.py
```

---

### 2. Evaluator Node Modifications

**File:** `src/hunav_sim/hunav_evaluator/hunav_evaluator/hunav_evaluator_node.py`

**Changes made:**

#### 2a. Auto-Organized Run Directories
- Added instance variable: `self.current_run_dir`
- Modified `recording_service_start()` to create `run_<run_id>/` folders automatically
- Updated `result_file_path` property to save metrics inside the run directory

**Behavior:**
- When you call `/hunav_start_recording` with `run_id: 1`, creates `/results/run_1/`
- When you call again with `run_id: 2`, creates `/results/run_2/`
- All metrics and trajectories for that run go into that folder

#### 2b. Raw Trajectory Export (ETH/UCY Format)
- Added method: `export_raw_trajectories()`
- Exports data to `run_D/true_pos_.csv` in ETH/UCY format:
  ```
  Row 1: frame_indices (comma-separated)
  Row 2: pedestrian_ids (comma-separated)
  Row 3: x_positions (comma-separated)
  Row 4: y_positions (comma-separated)
  ```
- Automatically called after recording stops, before metrics computation
- Allows reuse of `ground_truth_analysis.py` on simulation data

#### 2c. Updated Metrics Configuration
- Metrics still computed and saved to `run_D/<experiment_tag>.txt.csv`
- Time-series metrics saved to `run_D/<experiment_tag>_steps_<tag>_<run_id>.csv`
- Per-behavior breakdowns in `run_D/<experiment_tag>_beh_*.csv`

**Result structure after one run:**
```
results/
└── run_1/
    ├── baseline_run1.txt.csv                    # Summary metrics
    ├── baseline_run1.txt_steps_baseline_run1_1.csv  # Per-timestep metrics
    ├── baseline_run1.txt_beh_1.csv              # Behavior-filtered metrics
    ├── baseline_run1.txt_beh_1_steps_...csv     # Behavior timesteps
    └── true_pos_.csv                            # Raw trajectories (ETH/UCY format)
```

---

## DATASETS Folder Structure

**Location:** `src/DATASETS/`

### Ground Truth Data (ETH/UCY)
```
DATASETS/
├── eth/
│   ├── hotel/
│   │   ├── true_pos_.csv          # Ground truth trajectories
│   │   └── groups.txt             # Group annotations (optional)
│   └── univ/
│       ├── true_pos_.csv
│       └── groups.txt
├── ucy/
│   ├── zara/
│   │   ├── zara01/
│   │   │   └── true_pos_.csv
│   │   ├── zara02/
│   │   └── zara03/
│   └── univ/
│       ├── students001/
│       │   └── true_pos_.csv
│       └── students003/
│           └── true_pos_.csv
```

### Simulation Output (Recommended Structure)
```
DATASETS/
└── simulations/              # NEW FOLDER (create manually)
    ├── baseline_default/
    │   ├── run_1/
    │   │   └── true_pos_.csv
    │   ├── run_2/
    │   │   └── true_pos_.csv
    │   └── run_3/
    │       └── true_pos_.csv
    ├── social_force_low/
    │   ├── run_1/
    │   │   └── true_pos_.csv
    │   └── ...
    └── obstacle_force_high/
        └── ...
```

**Setup:**
```bash
mkdir -p /home/domenic/sfm_ws/src/DATASETS/simulations
```

**Why organize this way:**
- Ground truth data is immutable (eth/, ucy/ folders)
- Simulation outputs are organized by experiment type and parameter set
- Multiple runs per parameter set for statistical confidence (run_1, run_2, run_3...)
- Easy to configure `ground_truth_analysis.py` to compare both sets

---

## Analysis Scripts in DATASETS

### 1. `ground_truth_analysis.py`

**Purpose:** Analyze trajectory data (ground truth or simulation) and extract benchmark metrics

**Supports:**
- All ETH/UCY datasets
- Simulation output (if in `true_pos_.csv` format)
- Per-dataset analysis or cross-dataset comparison
- Visual plots (if matplotlib installed)

**Usage:**

Analyze all ETH/UCY ground truth:
```bash
cd /home/domenic/sfm_ws
python3 src/DATASETS/ground_truth_analysis.py
```

Analyze specific dataset:
```bash
python3 src/DATASETS/ground_truth_analysis.py --dataset eth_univ
```

Analyze simulation run and compare to ground truth:
```bash
python3 src/DATASETS/ground_truth_analysis.py \
  --sim-json src/DATASETS/simulations/baseline_default/run_1/
```

**Output:**
- `output/eth_univ_metrics.json` — Ground truth metrics in JSON
- `output/simulation_metrics.json` — Simulation metrics (if --sim-json provided)
- Console table comparing both
- PNG plots (if matplotlib available)

### 2. `verify_metrics.py`

**Purpose:** Manual verification of metric computation logic

**Usage:**
```bash
python3 src/DATASETS/verify_metrics.py
```

**What it does:**
- Loads ETH Univ ground truth
- Manually computes 5 key metrics (speed, efficiency, collisions, etc.)
- Prints detailed step-by-step verification
- Ensures metric definitions match expected formulas

---

## Full Experimental Workflow

### Step 0: Prepare for a New Run

```bash
cd /home/domenic/sfm_ws
source install/setup.bash

# Optional: Set up parameterized scenario YAML
# (see next section)
```

### Step 1: Create/Edit Scenario YAML with New Parameters

**Base file:** `src/hunav_gazebo_wrapper/scenarios/agents_cafe.yaml`

**To test a parameter:**
1. Copy the scenario file
2. Edit the force factors or velocity
3. Example:

```bash
cp src/hunav_gazebo_wrapper/scenarios/agents_cafe.yaml \
   src/hunav_gazebo_wrapper/scenarios/agents_cafe_social_low.yaml
```

Then edit `agents_cafe_social_low.yaml` and change:
```yaml
behavior:
  social_force_factor: 5.0  # Changed from 3.0 to 5.0
```

### Step 2: Launch Simulation (Terminal A)

```bash
ros2 launch hunav_gazebo_wrapper cafe_no_robot.launch.py
```

**What to expect:**
- Gazebo window opens
- 3 pedestrians visible, walking around cafe
- Simulation runs (no time limit)

### Step 3: Launch Evaluator (Terminal B)

```bash
ros2 launch hunav_evaluator hunav_evaluator.launch.py
```

**What to expect:**
- Console shows "Hunav evaluator node started"
- Waiting for recording commands

### Step 4: Start Recording (Terminal C)

**For run 1 of baseline:**
```bash
ros2 service call /hunav_start_recording hunav_msgs/srv/StartEvaluation \
  "{experiment_tag: 'baseline_run1', run_id: 1, robot_goal: {}}"
```

**For run 1 of social_low variant:**
```bash
ros2 service call /hunav_start_recording hunav_msgs/srv/StartEvaluation \
  "{experiment_tag: 'social_low_run1', run_id: 1, robot_goal: {}}"
```

**What to expect:**
- Terminal B logs: "Hunav evaluator started recording!"
- Agents continue walking
- Recording starts capturing positions

### Step 5: Wait & Stop Recording (Terminal C/D)

Wait **60 seconds**, then:
```bash
ros2 service call /hunav_stop_recording std_srvs/srv/Empty "{}"
```

**What to expect:**
- Terminal B logs: "Hunav evaluator stopping recording!"
- Metrics computed
- Raw trajectories exported
- Files written to `results/run_1/`

Observe output:
```
[INFO] Raw trajectories exported to /home/domenic/sfm_ws/results/run_1/true_pos_.csv 
       (3 agents, 121 frames)
[INFO] Summary metrics stored in /home/domenic/sfm_ws/results/run_1/baseline_run1.txt.csv
```

### Step 6: Move Results to DATASETS/simulations/

```bash
# Create experiment folder if needed
mkdir -p /home/domenic/sfm_ws/src/DATASETS/simulations/baseline_default

# Move the run folder
mv /home/domenic/sfm_ws/results/run_1 \
   /home/domenic/sfm_ws/src/DATASETS/simulations/baseline_default/
```

### Step 7: Repeat for Multiple Runs (Optional)

For statistical confidence, run 3 times with the same parameters:

```bash
# Run 2
ros2 service call /hunav_start_recording ... "run_id: 2" ...
# Wait 60s
ros2 service call /hunav_stop_recording ...
mv results/run_2 src/DATASETS/simulations/baseline_default/

# Run 3
ros2 service call /hunav_start_recording ... "run_id: 3" ...
# Wait 60s
ros2 service call /hunav_stop_recording ...
mv results/run_3 src/DATASETS/simulations/baseline_default/
```

### Step 8: Analyze Results

Compare baseline simulation against ETH/UCY ground truth:

```bash
python3 src/DATASETS/ground_truth_analysis.py \
  --sim-json src/DATASETS/simulations/baseline_default/run_1/
```

**Output:**
```
COMPARISON: eth_univ (ground truth) vs. simulation
─────────────────────────────────────────────────
Metric                  Ground Truth    Simulation    Difference
Mean speed              1.34 m/s        1.57 m/s      +0.23 (17% higher)
Speed std dev           0.32 m/s        0.28 m/s      -0.04 (13% lower)
Collision rate          2.1%            0.0%          -2.1% (too safe)
Path efficiency         0.89            0.92          +0.03 (too direct)
```

### Step 9: Iterate

If baseline metrics are far from ground truth, adjust parameters:

```bash
# Try lower obstacle force
cp src/hunav_gazebo_wrapper/scenarios/agents_cafe.yaml \
   src/hunav_gazebo_wrapper/scenarios/agents_cafe_obstacle_low.yaml
```

Edit and change:
```yaml
obstacle_force_factor: 5.0  # Default is 10.0
```

Repeat steps 1–8 with the new scenario file.

---

## Parameter Tuning Reference

**Tunable parameters** in scenario YAML:

| Parameter | Range | Effect | Default |
|-----------|-------|--------|---------|
| `max_vel` | 0.0–1.8 m/s | Desired walking speed | 1.5 |
| `radius` | 0.3–0.5 m | Agent size (collision radius) | 0.4 |
| `goal_force_factor` | 2.0–5.0 | Strength of pull toward goal | 2.0 |
| `obstacle_force_factor` | 2.0–50.0 | Strength of avoidance from obstacles | 10.0 |
| `social_force_factor` | 5.0–20.0 | Strength of avoidance from crowds | 5.0 |
| `other_force_factor` | 0–25.0 | Robot repulsion (not used in no-robot mode) | 20.0 |

**Interpretation:**
- **Higher `social_force_factor`** → agents keep farther apart, less crowding
- **Lower `obstacle_force_factor`** → agents pass closer to walls, more risk
- **Higher `max_vel`** → faster walking, less time in scenario
- **Lower `goal_force_factor`** → agents wander, take longer routes

---

## Commands Cheat Sheet

### Build & Setup
```bash
cd /home/domenic/sfm_ws
colcon build
source install/setup.bash
```

### Clean Old Results (Use Wisely)
```bash
rm -rf /home/domenic/sfm_ws/results/*
```

### Run Simulation
```bash
# Terminal A
ros2 launch hunav_gazebo_wrapper cafe_no_robot.launch.py

# Terminal B
ros2 launch hunav_evaluator hunav_evaluator.launch.py

# Terminal C (start after 5s)
ros2 service call /hunav_start_recording hunav_msgs/srv/StartEvaluation \
  "{experiment_tag: 'my_experiment', run_id: 1, robot_goal: {}}"

# Terminal C (after 60s)
ros2 service call /hunav_stop_recording std_srvs/srv/Empty "{}"
```

### Move Results
```bash
mkdir -p src/DATASETS/simulations/my_experiment
mv results/run_1 src/DATASETS/simulations/my_experiment/
```

### Analyze
```bash
# Compare single run to ground truth
python3 src/DATASETS/ground_truth_analysis.py \
  --sim-json src/DATASETS/simulations/my_experiment/run_1/

# Analyze all ground truth datasets
python3 src/DATASETS/ground_truth_analysis.py
```

---

## File Locations Summary

| What | Where |
|------|-------|
| Simulation scenarios | `src/hunav_gazebo_wrapper/scenarios/` |
| Evaluator config | `src/hunav_sim/hunav_evaluator/config/metrics.yaml` |
| No-robot launch | `src/hunav_gazebo_wrapper/launch/cafe_no_robot.launch.py` |
| Analysis scripts | `src/DATASETS/` |
| Ground truth data | `src/DATASETS/eth/`, `src/DATASETS/ucy/` |
| Simulation results (temp) | `/home/domenic/sfm_ws/results/run_D/` |
| Simulation results (archival) | `src/DATASETS/simulations/EXPERIMENT_NAME/run_D/` |

---

## Troubleshooting

### "No module named 'pmb2_description'"
- Use `cafe_no_robot.launch.py` instead of `simulation.launch.py`
- Robot-free mode eliminates this dependency

### Results folder has `.txt.csv` instead of `.csv`
- Expected behavior (legacy naming in evaluator)
- Contains correct CSV data, just missing the extension in base name
- Use `cat baseline_run1.txt.csv` to view

### `true_pos_.csv` not created
- Check Terminal B logs for: `"Raw trajectories exported to ..."`
- If missing, ensure recording was at least 5 seconds long
- Verify agents were actually moving (check Gazebo window)

### Agents not moving in Gazebo
- Check that scenario YAML has `goals:` section
- Ensure agents have valid goal indices defined

---

## Next Steps for Capstone

1. **Run baseline** with default parameters, save to `simulations/baseline/`
2. **Identify worst metrics** using `ground_truth_analysis.py`
3. **Design one-factor sweeps** (vary one parameter at a time)
4. **Document sensitivity** (which parameters affect which metrics)
5. **Decide** whether leader-follower is needed based on group metrics gap
6. **Implement conditionally** only if baseline tuning doesn't close all gaps

---

## References

- ETH/UCY dataset format: https://github.com/soheil-khormuji/eth-ucy-pedestrian-datasets
- Social Force Model: Helbing & Molnár (1995)
- Metric definitions: See capstone report Section 7

