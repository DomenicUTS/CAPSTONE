# OAT Sensitivity Analysis Guide

## Strategy: One-At-A-Time (OAT) Parameter Testing

**Goal:** Understand which parameters most affect realistic crowd metrics

**Approach:**
1. **Baseline (3 runs):** Balanced profile with typical paremeters
2. **Vary ONE parameter (3 runs each):** Same balanced defaults except for one factor
3. **Analyze results:** See how each parameter affects key metrics
4. **Design final profile:** Create optimized scenario based on findings
5. **Document:** Explain in final report why this profile is realistic

---

## Test Matrix (21 Total Runs)

| Test | Profile | Key Changed Parameter | Purpose |
|------|---------|----------------------|---------|
| 1-3 | **Balanced Baseline** | None (1.05 m/s, social=12, goal=3, obstacle=15) | Benchmark |
| 4-6 | Social Low | social=5 (weak person-to-person avoidance) | Test collision sensitivity |
| 7-9 | Social High | social=20 (strong person-to-person avoidance) | Test personal space |
| 10-12 | Goal Low | goal=1 (weak goal attraction) | Test path efficiency |
| 13-15 | Goal High | goal=5 (strong goal attraction) | Test directedness |
| 16-18 | Speed Slow | max_vel=0.7 (slower walking) | Test speed-collision tradeoff |
| 19-21 | Speed Fast | max_vel=1.5 (faster walking) | Test speed-efficiency tradeoff |

---

## Getting Started

### Prerequisites: Build & Source

```bash
cd ~/sfm_ws_fresh
colcon build --packages-select hunav_gazebo_wrapper hunav_evaluator
source install/setup.bash
```

### 3-Terminal Layout

**Terminal 1:** Gazebo + evaluator + world
```bash
cd ~/sfm_ws_fresh
source install/setup.bash
ros2 launch hunav_gazebo_wrapper cafe_no_robot.launch.py configuration_file:=domenic/cafe_oat_balanced_baseline.yaml
```

**Terminal 2:** Data recording
```bash
cd ~/sfm_ws_fresh
source install/setup.bash
# Wait for Terminal 1 to show "World generator finished"
# Then run this entire loop for each test below
```

**Terminal 3:** Switching profiles (between runs)
- Used for `ros2 service call` commands
- Keep Terminal 1/2 running, restart Terminal 1 with new profile

---

## Run Sequence: Balanced Baseline (Tests 1-3)

### Sanity Check: First Run
**Duration:** ~3-5 min

1. **Terminal 1:** Launch balanced baseline
   ```bash
   ros2 launch hunav_gazebo_wrapper cafe_no_robot.launch.py configuration_file:=domenic/cafe_oat_balanced_baseline.yaml
   ```
   ✅ Wait for: `World generator finished`

2. **Terminal 2:** Wait 10 seconds, then record
   ```bash
   sleep 10 && ros2 service call /hunav_start_recording hunav_msgs/srv/StartRecording "{experiment_tag: baseline_oat, run_id: 1}" && sleep 60 && ros2 service call /hunav_stop_recording std_srvs/srv/Empty {}
   ```
   ✅ Wait 70 seconds for recording + stop

3. **Check results:** Look in Terminal 1 output for metrics computed

4. **Review output directory:**
   ```bash
   # Terminal 2
   find ~/.ros/hunav_ws* -name "true_pos_*.csv" -type f -mmin -2
   ```
   Should see trajectory file created in last 2 minutes

---

## Full OAT Run Automation

### Option A: Manual (Per Test)

For each profile in the table above:

```bash
# Terminal 1: Kill old, launch new profile
ROS_DOMAIN_ID=0 ros2 launch hunav_gazebo_wrapper cafe_no_robot.launch.py configuration_file:=domenic/cafe_oat_[PROFILE_NAME].yaml

# Terminal 2: Wait for "World generator finished" in Terminal 1, then run 3 times:
ros2 service call /hunav_start_recording hunav_msgs/srv/StartRecording "{experiment_tag: [PROFILE_NAME], run_id: 1}" && sleep 60 && ros2 service call /hunav_stop_recording std_srvs/srv/Empty {}
ros2 service call /hunav_start_recording hunav_msgs/srv/StartRecording "{experiment_tag: [PROFILE_NAME], run_id: 2}" && sleep 60 && ros2 service call /hunav_stop_recording std_srvs/srv/Empty {}
ros2 service call /hunav_start_recording hunav_msgs/srv/StartRecording "{experiment_tag: [PROFILE_NAME], run_id: 3}" && sleep 60 && ros2 service call /hunav_stop_recording std_srvs/srv/Empty {}
```

### Option B: Scripted (Batch)

Create `run_oat_batch.sh`:

```bash
#!/bin/bash

PROFILES=("balanced_baseline" "social_low" "social_high" "goal_low" "goal_high" "speed_slow" "speed_fast")
RUNS=3

for PROFILE in "${PROFILES[@]}"; do
    echo "================================"
    echo "Testing: $PROFILE"
    echo "================================"
    
    # Launch: Keep Terminal 1 running, user switches manually
    # This script just shows the commands
    
    echo "[Terminal 1] Run this:"
    echo "ros2 launch hunav_gazebo_wrapper cafe_no_robot.launch.py configuration_file:=domenic/cafe_oat_${PROFILE}.yaml"
    
    echo ""
    echo "[Terminal 2] After 'World generator finished', run in sequence:"
    for RUN in $(seq 1 $RUNS); do
        echo "ros2 service call /hunav_start_recording hunav_msgs/srv/StartRecording \"{experiment_tag: oat_${PROFILE}, run_id: ${RUN}}\" && sleep 60 && ros2 service call /hunav_stop_recording std_srvs/srv/Empty {}"
    done
    
    echo ""
    echo "Then press ENTER to continue to next profile..."
    read
done
```

---

## Expected Metrics from Ground Truth

For reference (ETH/UCY benchmarks):

| Metric | Range |
|--------|-------|
| **Speed (mean)** | 0.64-1.46 m/s |
| **Speed (std)** | 0.34-0.53 m/s |
| **Collision rate** | 0.0004-0.094 events/ped/s |
| **Near-miss rate** | 0.009-0.229 events/ped/s |
| **Path efficiency** | 0.87-0.97 |
| **Acceleration** | 0.12-0.74 m/s² |
| **Min inter-agent dist** | 0.80-1.88 m |

---

## Analysis Script

After all 21 runs complete:

```bash
cd ~/sfm_ws_fresh
python3 src/DATASETS/domenic_analysis/compare_results.py > oat_results_summary.txt
cat oat_results_summary.txt
```

This will show:
- **Alignment scores** for each metric
- **Profile comparisons** table
- **Recommendations** for which parameters matter most

---

## Interpreting Results

**After baseline (tests 1-3):**
- Check if metrics fall in ETH/UCY ranges
- If NO → parameter tuning needed
- If YES → baseline is realistic, proceed to variations

**After each variation (6 profiles × 3 runs):**
- Compare metrics to baseline
- **Higher social → fewer collisions? → social_force_factor matters for safety**
- **Lower goal → worse efficiency? → goal_force_factor matters for path quality**
- **Higher speed → more collisions? → max_vel affects speed-safety tradeoff**
- etc.

**Final profile design:**
- Select parameters from variation that maximize alignment to ground truth
- Document which ones were critical (high sensitivity)
- Use in final report: "Tests 1-3 showed baseline was weak in X, so tests 4-21 identified that parameter Y was most sensitive to metric Z"

---

## Troubleshooting

**Q: Gazebo says "not responding"**
- Normal. Click "Wait". Simulation runs in background.

**Q: No metrics computed**
- Check Terminal 1: Did world finish loading?
- Check permissions: Can you read `~/.ros/hunav_ws*/`?

**Q: Terminal 2 command hangs**
- Gazebo crashed. Restart Terminal 1.
- Check `/tmp` for gazebo log files.

**Q: Results not in expected directory**
- Find them: `find ~/.ros -name "true_pos_*.csv" -type f -mmin -2`
- Update results directory in compare_results.py

---

## Next: Final Profile Design

After analyzing all OAT results:

1. Identify 2-3 parameters that had largest metric sensitivity
2. Create "final_optimized.yaml" combining best values
3. Run 3 validation runs with this profile
4. Document in report: "OAT analysis showed X & Y dominate; final profile optimizes for Z"

This gives you **clear methodology** for your report's methods section.
