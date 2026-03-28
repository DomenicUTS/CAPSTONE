# OAT Quick Reference

## Run Order & Commands

### Tests 1-3: Balanced Baseline

**Terminal 1:**
```bash
cd ~/sfm_ws_fresh && source install/setup.bash
ros2 launch hunav_gazebo_wrapper cafe_no_robot.launch.py configuration_file:=domenic/cafe_oat_balanced_baseline.yaml
```

**Terminal 2:** (after "World generator finished")
```bash
ros2 service call /hunav_start_recording hunav_msgs/srv/StartRecording "{experiment_tag: baseline_oat, run_id: 1}" && sleep 60 && ros2 service call /hunav_stop_recording std_srvs/srv/Empty {}
ros2 service call /hunav_start_recording hunav_msgs/srv/StartRecording "{experiment_tag: baseline_oat, run_id: 2}" && sleep 60 && ros2 service call /hunav_stop_recording std_srvs/srv/Empty {}
ros2 service call /hunav_start_recording hunav_msgs/srv/StartRecording "{experiment_tag: baseline_oat, run_id: 3}" && sleep 60 && ros2 service call /hunav_stop_recording std_srvs/srv/Empty {}
```

---

### Tests 4-6: Social Low (weak avoidance)

**Terminal 1:** (stop old, start new)
```bash
# CTRL+C the old launch
ros2 launch hunav_gazebo_wrapper cafe_no_robot.launch.py configuration_file:=domenic/cafe_oat_social_low.yaml
```

**Terminal 2:**
```bash
ros2 service call /hunav_start_recording hunav_msgs/srv/StartRecording "{experiment_tag: social_low, run_id: 1}" && sleep 60 && ros2 service call /hunav_stop_recording std_srvs/srv/Empty {}
ros2 service call /hunav_start_recording hunav_msgs/srv/StartRecording "{experiment_tag: social_low, run_id: 2}" && sleep 60 && ros2 service call /hunav_stop_recording std_srvs/srv/Empty {}
ros2 service call /hunav_start_recording hunav_msgs/srv/StartRecording "{experiment_tag: social_low, run_id: 3}" && sleep 60 && ros2 service call /hunav_stop_recording std_srvs/srv/Empty {}
```

---

### Tests 7-9: Social High (strong avoidance)

**Terminal 1:**
```bash
ros2 launch hunav_gazebo_wrapper cafe_no_robot.launch.py configuration_file:=domenic/cafe_oat_social_high.yaml
```

**Terminal 2:**
```bash
ros2 service call /hunav_start_recording hunav_msgs/srv/StartRecording "{experiment_tag: social_high, run_id: 1}" && sleep 60 && ros2 service call /hunav_stop_recording std_srvs/srv/Empty {}
ros2 service call /hunav_start_recording hunav_msgs/srv/StartRecording "{experiment_tag: social_high, run_id: 2}" && sleep 60 && ros2 service call /hunav_stop_recording std_srvs/srv/Empty {}
ros2 service call /hunav_start_recording hunav_msgs/srv/StartRecording "{experiment_tag: social_high, run_id: 3}" && sleep 60 && ros2 service call /hunav_stop_recording std_srvs/srv/Empty {}
```

---

### Tests 10-12: Goal Low (weak attraction to goal)

**Terminal 1:**
```bash
ros2 launch hunav_gazebo_wrapper cafe_no_robot.launch.py configuration_file:=domenic/cafe_oat_goal_low.yaml
```

**Terminal 2:**
```bash
ros2 service call /hunav_start_recording hunav_msgs/srv/StartRecording "{experiment_tag: goal_low, run_id: 1}" && sleep 60 && ros2 service call /hunav_stop_recording std_srvs/srv/Empty {}
ros2 service call /hunav_start_recording hunav_msgs/srv/StartRecording "{experiment_tag: goal_low, run_id: 2}" && sleep 60 && ros2 service call /hunav_stop_recording std_srvs/srv/Empty {}
ros2 service call /hunav_start_recording hunav_msgs/srv/StartRecording "{experiment_tag: goal_low, run_id: 3}" && sleep 60 && ros2 service call /hunav_stop_recording std_srvs/srv/Empty {}
```

---

### Tests 13-15: Goal High (strong attraction to goal)

**Terminal 1:**
```bash
ros2 launch hunav_gazebo_wrapper cafe_no_robot.launch.py configuration_file:=domenic/cafe_oat_goal_high.yaml
```

**Terminal 2:**
```bash
ros2 service call /hunav_start_recording hunav_msgs/srv/StartRecording "{experiment_tag: goal_high, run_id: 1}" && sleep 60 && ros2 service call /hunav_stop_recording std_srvs/srv/Empty {}
ros2 service call /hunav_start_recording hunav_msgs/srv/StartRecording "{experiment_tag: goal_high, run_id: 2}" && sleep 60 && ros2 service call /hunav_stop_recording std_srvs/srv/Empty {}
ros2 service call /hunav_start_recording hunav_msgs/srv/StartRecording "{experiment_tag: goal_high, run_id: 3}" && sleep 60 && ros2 service call /hunav_stop_recording std_srvs/srv/Empty {}
```

---

### Tests 16-18: Speed Slow (0.7 m/s baseline)

**Terminal 1:**
```bash
ros2 launch hunav_gazebo_wrapper cafe_no_robot.launch.py configuration_file:=domenic/cafe_oat_speed_slow.yaml
```

**Terminal 2:**
```bash
ros2 service call /hunav_start_recording hunav_msgs/srv/StartRecording "{experiment_tag: speed_slow, run_id: 1}" && sleep 60 && ros2 service call /hunav_stop_recording std_srvs/srv/Empty {}
ros2 service call /hunav_start_recording hunav_msgs/srv/StartRecording "{experiment_tag: speed_slow, run_id: 2}" && sleep 60 && ros2 service call /hunav_stop_recording std_srvs/srv/Empty {}
ros2 service call /hunav_start_recording hunav_msgs/srv/StartRecording "{experiment_tag: speed_slow, run_id: 3}" && sleep 60 && ros2 service call /hunav_stop_recording std_srvs/srv/Empty {}
```

---

### Tests 19-21: Speed Fast (1.5 m/s baseline)

**Terminal 1:**
```bash
ros2 launch hunav_gazebo_wrapper cafe_no_robot.launch.py configuration_file:=domenic/cafe_oat_speed_fast.yaml
```

**Terminal 2:**
```bash
ros2 service call /hunav_start_recording hunav_msgs/srv/StartRecording "{experiment_tag: speed_fast, run_id: 1}" && sleep 60 && ros2 service call /hunav_stop_recording std_srvs/srv/Empty {}
ros2 service call /hunav_start_recording hunav_msgs/srv/StartRecording "{experiment_tag: speed_fast, run_id: 2}" && sleep 60 && ros2 service call /hunav_stop_recording std_srvs/srv/Empty {}
ros2 service call /hunav_start_recording hunav_msgs/srv/StartRecording "{experiment_tag: speed_fast, run_id: 3}" && sleep 60 && ros2 service call /hunav_stop_recording std_srvs/srv/Empty {}
```

---

## Parameter Matrix (for reference during analysis)

```
Baseline:    speed=1.05, social=12.0, goal=3.0, obstacle=15.0

Social Low:  speed=1.05, social=5.0,  goal=3.0, obstacle=15.0  [↓ person avoidance]
Social High: speed=1.05, social=20.0, goal=3.0, obstacle=15.0  [↑ person avoidance]

Goal Low:    speed=1.05, social=12.0, goal=1.0, obstacle=15.0  [↓ goal attraction]
Goal High:   speed=1.05, social=12.0, goal=5.0, obstacle=15.0  [↑ goal attraction]

Speed Slow:  speed=0.7,  social=12.0, goal=3.0, obstacle=15.0  [↓ walking speed]
Speed Fast:  speed=1.5,  social=12.0, goal=3.0, obstacle=15.0  [↑ walking speed]
```

---

## After All 21 Runs

Analyze results:
```bash
cd ~/sfm_ws_fresh
python3 src/DATASETS/domenic_analysis/compare_results.py
```

Look for:
- **Which parameter variation caused biggest metric change?**
- **Which profile(s) matched ETH/UCY ground truth best?**
- **What's the 2-3 parameter combo that optimizes for realism?**

Then design final profile and run 3 validation runs.
