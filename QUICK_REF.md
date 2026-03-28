# SFM Parameter Study: Quick Reference Card

## Build
```bash
cd /home/domenic/sfm_ws_fresh
colcon build && source install/setup.bash
```

## Run One Profile (Balanced)

### Terminal 1: Gazebo
```bash
source install/setup.bash
ros2 launch hunav_gazebo_wrapper cafe_no_robot.launch.py \
  configuration_file:=domenic/cafe_24agents_balanced.yaml
```

### Terminal 2: Evaluator
```bash
source install/setup.bash
ros2 launch hunav_evaluator hunav_evaluator.launch.py
```

### Terminal 3: Record Trials
```bash
# Trial 1
ros2 service call /hunav_start_recording hunav_msgs/srv/StartEvaluation \
  "{experiment_tag: 'balanced_cafe_run1', run_id: 1, robot_goal: {}}"
# Wait 60s
ros2 service call /hunav_stop_recording std_srvs/srv/Empty "{}"

# Trial 2
ros2 service call /hunav_start_recording hunav_msgs/srv/StartEvaluation \
  "{experiment_tag: 'balanced_cafe_run2', run_id: 2, robot_goal: {}}"
# Wait 60s
ros2 service call /hunav_stop_recording std_srvs/srv/Empty "{}"

# Trial 3
ros2 service call /hunav_start_recording hunav_msgs/srv/StartEvaluation \
  "{experiment_tag: 'balanced_cafe_run3', run_id: 3, robot_goal: {}}"
# Wait 60s
ros2 service call /hunav_stop_recording std_srvs/srv/Empty "{}"
```

## Organize Results
```bash
mkdir -p src/DATASETS/domenic_analysis/simulations/balanced/cafe
mv results/run_1 src/DATASETS/domenic_analysis/simulations/balanced/cafe/
mv results/run_2 src/DATASETS/domenic_analysis/simulations/balanced/cafe/
mv results/run_3 src/DATASETS/domenic_analysis/simulations/balanced/cafe/
```

## Switch Profiles (Repeat for other profiles)
In Terminal 1, stop and restart with:
```bash
# Cautious
configuration_file:=domenic/cafe_24agents_cautious.yaml

# Aggressive
configuration_file:=domenic/cafe_24agents_aggressive.yaml

# Dense-Aware
configuration_file:=domenic/cafe_24agents_dense_aware.yaml

# Speed-First
configuration_file:=domenic/cafe_24agents_speed_first.yaml
```

## Analyze All Results
```bash
python3 src/DATASETS/domenic_analysis/compare_results.py
```

## Files Generated

✅ **Scenario YAMLs** (25 agents each):
- `src/hunav_gazebo_wrapper/scenarios/domenic/cafe_24agents_balanced.yaml`
- `src/hunav_gazebo_wrapper/scenarios/domenic/cafe_24agents_cautious.yaml`
- `src/hunav_gazebo_wrapper/scenarios/domenic/cafe_24agents_aggressive.yaml`
- `src/hunav_gazebo_wrapper/scenarios/domenic/cafe_24agents_dense_aware.yaml`
- `src/hunav_gazebo_wrapper/scenarios/domenic/cafe_24agents_speed_first.yaml`

✅ **Tools**:
- `src/DATASETS/domenic_analysis/generate_cafe_scenarios.py` (already run)
- `src/DATASETS/domenic_analysis/compare_results.py` (analysis)
- `src/DATASETS/domenic_analysis/run_batch_cafe.sh` (batch runner)

✅ **Documentation**:
- `CAFE_PARAMETER_STUDY_GUIDE.md` (detailed guide)
- `IMPLEMENTATION_STATUS.md` (status overview)
- `src/DATASETS/domenic_analysis/README.md` (folder overview)

## Profile Summaries

| Profile | speed | social | goal | obstacle | Use Case |
|---------|-------|--------|------|----------|----------|
| Balanced | N(1.05, 0.15) | N(12.0, 2.0) | N(3.0, 0.5) | N(15.0, 3.0) | Baseline/benchmark |
| Cautious | N(0.85, 0.12) | N(16.0, 2.5) | N(2.5, 0.4) | N(20.0, 4.0) | Conservative crowds |
| Aggressive | N(1.35, 0.15) | N(8.0, 2.0) | N(3.5, 0.5) | N(10.0, 3.0) | Rush hour/risky |
| Dense-Aware | N(0.95, 0.20) | N(14.0, 3.0) | N(3.2, 0.5) | N(18.0, 3.5) | Adaptive/variable |
| Speed-First | N(1.40, 0.18) | N(10.0, 2.5) | N(4.0, 0.6) | N(12.0, 3.5) | Goal-driven/impatient |

## Expected Results vs ETH/UCY

| Metric | Min | Max |
|--------|-----|-----|
| Mean speed | 0.64 | 1.46 m/s |
| Speed std | 0.34 | 0.53 m/s |
| Collision rate | 0.0004 | 0.094 events/ped/s |
| Near-miss rate | 0.009 | 0.229 events/ped/s |
| Path efficiency | 0.87 | 0.97 |
| Acceleration | 0.12 | 0.74 m/s² |
| Min distance | 0.80 | 1.88 m |

## Time Estimates

- **One profile (3 runs):** ~15 min (60s × 3 + overhead)
- **All 5 profiles (15 runs):** ~75 min
- **Analysis:** ~1 min
- **Total:** ~1.5-2 hours for complete baseline

## Status

✅ Ready to run  
🔄 Café baseline in progress  
⏳ Other environments pending  

See `IMPLEMENTATION_STATUS.md` for full status.

---

Keep this handy! Print it or save to a terminal note.
