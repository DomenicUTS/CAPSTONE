# Simulation Analysis Workflow

## Overview

**Ground Truth Data** (preserved, read-only):
- Location: `src/DATASETS/eth/`, `src/DATASETS/ucy/`
- Status: ✓ **UNTOUCHED** - never modified by analysis
- Content: 7 ETH/UCY pedestrian datasets with 360+ peds each, 700+ seconds

**Simulation Results** (new location):
- Location: `simulation_analysis/` (new dedicated directory)
- Structure:
  ```
  simulation_analysis/
  ├── compare_to_ground_truth.py      (analysis script)
  ├── comparisons/
  │   └── simulation_vs_ground_truth.json  (structured comparison data)
  └── reports/
      └── comparison_report.txt            (human-readable report)
  ```

---

## Workflow: Compare Your Simulation to Ground Truth

### Step 1: Run Simulation
```bash
Terminal 1:
ros2 launch hunav_gazebo_wrapper cafe_no_robot.launch.py configuration_file:=domenic/cafe/cafe_oat_[CONFIG].yaml

Terminal 2:
ros2 service call /hunav_start_recording hunav_msgs/srv/StartRecording \
  "{experiment_tag: [TAG], run_id: [1-21]}" && \
sleep 120 && \
ros2 service call /hunav_stop_recording std_srvs/srv/Empty {}
```

Results saved to: `results/run_[ID]/true_pos_.csv` + metrics CSVs

### Step 2: Compare to Ground Truth
```bash
# Single run
python3 simulation_analysis/compare_to_ground_truth.py 1

# Multiple runs
python3 simulation_analysis/compare_to_ground_truth.py 1 2 3 4 5

# Auto-discover all runs
python3 simulation_analysis/compare_to_ground_truth.py
```

### Step 3: Review Results

**JSON (Machine-readable):**
```
simulation_analysis/comparisons/simulation_vs_ground_truth.json
```

**Text Report (Human-readable):**
```
simulation_analysis/reports/comparison_report.txt
```

---

## Metrics Compared

Each run compares 10 key metrics against ETH Univ benchmark:

| Metric | Ground Truth Range | Meaning |
|--------|-------------------|---------|
| Speed Mean | 1.46 m/s | Average walking pace |
| Speed Std | 0.41 m/s | Variation in speeds |
| Speed Median | 1.49 m/s | Typical speed |
| Acceleration Mean | 0.80 m/s² | Smoothness of motion |
| Collision Rate | 0.0004 /ped/s | Avoidance effectiveness |
| Near-Miss Rate | 0.0139 /ped/s | Social spacing |
| Path Efficiency | 0.966 | Navigation directness (high is good) |
| Speed Variability | 0.163 CV | Consistency of pace |
| Min Distance Mean | 1.88 m | Average inter-agent spacing |
| Min Distance P5 | 0.60 m | Tightest encounters |

---

## Ground Truth Datasets Available

All preserved in `src/DATASETS/`:

1. **eth_univ** ← Default for comparisons
2. eth_hotel
3. ucy_zara01, ucy_zara02, ucy_zara03
4. ucy_univ_s1, ucy_univ_s3

---

## For OAT Analysis (21 Runs)

After running all 21 test configurations (baseline + 6 variations × 3 runs each):

```bash
# Generate comparison for all 21 runs
python3 simulation_analysis/compare_to_ground_truth.py

# Will create:
# - simulation_analysis/comparisons/simulation_vs_ground_truth.json
#   (contains all 21 runs' metrics + comparisons)
# - simulation_analysis/reports/comparison_report.txt
#   (formatted table for all 21 runs)
```

Then analyze which parameters had the most impact on:
- Collision rate reduction
- Path efficiency improvement
- Speed normalization
- Acceleration smoothing

---

## Data Integrity

✓ **Ground Truth:**
- Original: `src/DATASETS/eth/univ/true_pos_.csv`, etc.
- Status: Read-only, never modified
- Accessed only by: `compare_to_ground_truth.py`

✓ **Simulation Results:**
- Stored: `results/run_N/`
- NOT modified by analysis
- Only read for comparison

✓ **Analysis Output:**
- NEW location: `simulation_analysis/`
- Keeps everything separate
- Can re-run without side effects

---

## For Future Analysis

To add more ground truth datasets or extend comparisons:

1. Edit `GROUND_TRUTH_DATASETS` dict in `compare_to_ground_truth.py`
2. Add new metric computation logic as needed
3. Re-run: `python3 simulation_analysis/compare_to_ground_truth.py`

All results stay in `simulation_analysis/`, never touching original data.
