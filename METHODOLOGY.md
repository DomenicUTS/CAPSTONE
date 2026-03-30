# OAT Sensitivity Analysis — Methodology

## Test Configuration Files

### Café Environment
Location: `src/hunav_gazebo_wrapper/scenarios/domenic/cafe/`

| File | Parameter Varied | Scale Factor |
|------|------------------|-------------|
| `cafe_oat_balanced_baseline.yaml` | None | — (reference) |
| `cafe_oat_social_low.yaml` | Social Force | × 0.579 |
| `cafe_oat_social_high.yaml` | Social Force | × 2.371 |
| `cafe_oat_goal_low.yaml` | Goal Force | × 0.469 |
| `cafe_oat_goal_high.yaml` | Goal Force | × 2.033 |
| `cafe_oat_speed_slow.yaml` | Max Velocity | × 0.7 |
| `cafe_oat_speed_fast.yaml` | Max Velocity | × 1.330 |

### Central Tunnel Environment  
Location: `src/hunav_gazebo_wrapper/scenarios/domenic/central_tunnel/`

| File | Parameter Varied | Scale Factor |
|------|------------------|-------------|
| `ct_baseline.yaml` | None | — (reference) |
| `ct_phase1_social_low.yaml` | Social Force | × 0.55 |
| `ct_phase1_social_high.yaml` | Social Force | × 1.70 |
| `ct_phase2_goal_low.yaml` | Goal Force | × 0.60 |
| `ct_phase2_goal_high.yaml` | Goal Force | × 1.60 |
| `ct_phase3_speed_low.yaml` | Max Velocity | × 0.75 |
| `ct_phase3_speed_high.yaml` | Max Velocity | × 1.25 |
| `ct_phase4_obstacle_low.yaml` | Obstacle Force | × 0.50 |
| `ct_phase4_obstacle_high.yaml` | Obstacle Force | × 2.00 |

## Configuration Mode: A Critical Detail

The `behavior.configuration` field in each agent's YAML controls how force factors are loaded:

```
Mode 0 (DEFAULT):        Force factors overwritten with defaults (SF=5, GF=2, OF=10)
Mode 1 (CUSTOM):         YAML values used exactly as written
Mode 2 (RANDOM_NORMAL):  Force factors overwritten with random normal samples
Mode 3 (RANDOM_UNIFORM): Force factors overwritten with random uniform samples
```

**Café tests used mode 2** — the force factor values written in the YAML files were overwritten
by the loader with random normal distributions at each launch. This means:
- Phase 1 (Social Force) scaling → overwritten by random at launch
- Phase 2 (Goal Force) scaling → overwritten by random at launch
- Phase 3 (Speed / max_vel) → **NOT affected** (max_vel is not overwritten by config mode)

The café results are still informative (especially Phase 3), but the force factor phases
(1 and 2) may reflect random variation rather than controlled scaling.

**Central tunnel tests use mode 1** — all YAML values are respected exactly. This makes
the OAT methodology rigorous: each phase varies precisely the intended parameter.

Source: `src/hunav_sim/hunav_agent_manager/src/hunav_loader.cpp` (lines ~120–230)
Constants: `src/hunav_sim/hunav_msgs/msg/AgentBehavior.msg`

## Scaling Implementation

Both `generate_pure_yamls.py` and `generate_central_tunnel_yamls.py` (located in
`src/hunav_gazebo_wrapper/scenarios/domenic/`) follow the same proportional scaling approach:

1. Load baseline YAML with all agents
2. For each agent, read baseline value: `P_baseline[i]`
3. Scale: `P_scaled[i] = clamp(P_baseline[i] × scale_factor, min, max)`
4. All other parameters unchanged
5. Individual variation preserved — if agent 1 has higher SF than agent 2 at baseline,
   this ratio is maintained at all scale levels

### Clamping Ranges (enforced in hunav_loader.cpp when config ≠ 1)

| Parameter | Min | Max | Default |
|-----------|-----|-----|---------|
| social_force_factor | 5.0 | 20.0 | 5.0 |
| goal_force_factor | 2.0 | 5.0 | 2.0 |
| obstacle_force_factor | 2.0 | 50.0 | 10.0 |
| other_force_factor | 0.0 | 25.0 | 20.0 |
| max_vel | 0.0 | 1.8 | 1.0 |

## Run Execution Workflow

### Step 1: Launch simulation

```bash
# Café
ros2 launch hunav_gazebo_wrapper cafe_no_robot.launch.py \
  configuration_file:=domenic/cafe/cafe_oat_balanced_baseline.yaml

# Central Tunnel
ros2 launch hunav_gazebo_wrapper central_tunnel_no_robot.launch.py \
  configuration_file:=domenic/central_tunnel/ct_baseline.yaml
```

Both launch files follow the same pattern:
1. `hunav_loader` reads YAML configuration
2. After 2s: `hunav_gazebo_world_generator` creates world with agents
3. After 2s more: Gazebo server launches with generated world
4. `hunav_agent_manager` runs SFM + behavior trees
5. `hunav_evaluator` computes metrics on recording

### Step 2: Record data (120 seconds)

```bash
python3 src/analysis/run_recording.py <run_id>
```

This script:
1. Updates `metrics.yaml` to set the result file path
2. Calls `/hunav_start_recording` ROS2 service
3. Waits 120 seconds
4. Calls `/hunav_stop_recording` service
5. Metrics and trajectories saved to `sim_results/`

### Step 3: Analyze against ground truth

```bash
python3 src/analysis/ground_truth_analysis.py \
  --custom-dataset run_1 sim_results/central_tunnel/central_tunnel_baseline/run_1/true_pos_.csv \
  --custom-dataset run_2 sim_results/central_tunnel/central_tunnel_baseline/run_2/true_pos_.csv \
  --dt 0.1 --output-dir sim_analysis_ct_baseline
```

## Central Tunnel Design Rationale

### Why 40 Agents?
- Café had 12 agents in a 15×20m space → high density, constant collisions
- Central tunnel: 40 agents in 38.5×17.5m → moderate density, more walking data
- ETH/UCY datasets have 150–430 pedestrians over 3–13 minutes

### Why Bidirectional Flow?
- Agents spawn at opposite ends and walk toward each other
- Creates natural collision opportunities at corridor center
- Matches ETH univ corridor dataset topology
- ~30m traversal per goal cycle → agents walk, not just avoid

### Why Groups?
- Real pedestrians walk in groups (ETH/UCY annotate this)
- 5 dyads (pairs) + 3 triads (trios) + 21 individuals
- Group members: similar speeds, shared goals, group_id linkage
- GroupWalkNode in BT: leader navigates, followers match goals

### Why Configuration Mode 1?
- Ensures YAML values are used exactly → true OAT isolation
- Café's mode 2 randomization undermined force factor scaling
- max_vel unaffected by config mode, so café Phase 3 was valid

## Troubleshooting

### Agents not spawning
- Check `hunav_loader` output for parameter warnings
- Verify YAML agent count matches `agents:` list length
- Ensure `yaml_base_name` matches BT file naming pattern

### BT files not found
The BT system looks for: `<installed>/behavior_trees/{yaml_base_name}__agent_{id}_bt.xml`
- Café: `agents_cafe__agent_1_bt.xml`
- Central Tunnel: `agents_central_tunnel__agent_1_bt.xml`
- After generating, run `colcon build` to install to share directory

### Force factors not taking effect
- Check `configuration` mode — mode 0/2/3 override YAML values
- Look for clamping messages in hunav_loader output
- Use mode 1 (CUSTOM) for deterministic parameter control

### Path efficiency near zero
- Agents oscillating between social force and goal force
- Try increasing goal_force_factor or reducing social_force_factor
