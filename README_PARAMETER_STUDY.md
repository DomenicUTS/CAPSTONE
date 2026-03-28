# SFM Sensitivity Analysis: Parameter Study & Methodology

## Overview

This document describes the **One-At-A-Time (OAT) sensitivity analysis** for Social Force Model (SFM) parameter validation. The goal is to systematically identify which individual SFM parameters drive realistic crowd dynamics matching ETH/UCY ground truth datasets.

**Key Features:**
- Systematic parameter variation (one parameter at a time)
- Baseline-controlled experiments (isolates cause-and-effect)
- Statistical sampling using normal distributions (not hand-picked values)
- 21 total runs across 7 configurations (3 runs per config)
- Automated ETH/UCY ground truth comparison
- Clear methodology for final report/thesis

---

## Why This Approach?

### Problem: Hand-Picking Parameters is Unmotivated

**Old approach (5 diverse profiles):** Create profiles with different "personalities" (balanced, cautious, aggressive, etc.) and hope they bracket the solution space.
- ✗ Doesn't tell us *which parameter* drives *which metric*
- ✗ Hard to justify in thesis: "Why these 5 profiles?"
- ✗ Can't isolate effects when multiple parameters vary simultaneously
- ✗ Final profile design is guesswork

### Solution: One-At-A-Time Sensitivity Analysis

**New approach (OAT):** Start with a realistic baseline, then systematically vary *one parameter at a time* while holding others constant.
- ✓ Clear cause-and-effect: "When social_force increased by X, collisions decreased by Y"
- ✓ Scientifically defensible: Standard experimental design (used in physics, biology, engineering)
- ✓ Easily justifiable: "OAT analysis identified [param] as most sensitive to [metric]"
- ✓ Data-driven final profile: "Based on findings, we optimized for..."

---

## Methodology: Normal Distribution Sampling

### Why Normal Distributions (Not Random or Fixed)?

Each test configuration uses **normal distributions** to assign parameters to individual agents. Here's why:

**Goal:** Simulate heterogeneous crowds (realistic—not all people walk the same speed!)

**Method:**
```python
# Instead of hardcoding: all agents have max_vel = 1.0 m/s
# We sample with randomness:
for agent in agents:
    max_vel = float(np.clip(np.random.normal(mean=1.05, std=0.15), min=0.5, max=2.0))
    social_force = float(np.clip(np.random.normal(mean=12.0, std=2.0), min=3.0, max=25.0))
    # ... etc for other parameters
    agent.params = {max_vel, social_force, ...}
```

**Why normal distributions?**
1. **Biologically grounded:** Human behavior is naturally variable, not constant
2. **Realistic heterogeneity:** Each agent slightly different (like real crowds)
3. **Not uniformly random:** Most agents near mean, fewer at extremes (like real people)
4. **Statistically robust:** Multiple runs with same distribution test consistency

**Clipping bounds** (e.g., `0.5 ≤ max_vel ≤ 2.0`):
- Prevents unrealistic values (people don't walk backward or at 100 m/s)
- Keeps variance within physical boundaries
- Ensures parameter ranges stay interpretable

### Example: Balanced Baseline Configuration

```yaml
# All 25 agents sample from these distributions:
Balanced Baseline:
  max_vel: N(μ=1.05, σ=0.15) → clipped to [0.5, 2.0]         # realistic walking speed
  social_force_factor: N(μ=12.0, σ=2.0) → clipped to [3.0, 25.0]    # person-to-person avoidance
  goal_force_factor: N(μ=3.0, σ=0.5) → clipped to [1.0, 6.0]        # attraction to goal
  obstacle_force_factor: N(μ=15.0, σ=3.0) → clipped to [3.0, 30.0]  # wall avoidance
```

**Result:** 25 unique agents, all plausibly realistic, but with natural variation

---

## OAT Test Matrix (21 Runs Total)

Each configuration uses the same **heterogeneous, normally-distributed approach**, but we systematically vary the *mean* of each distribution to isolate parameter effects.

| Tests | Config | Factor Changed | Mean Value | Purpose |
|-------|--------|----------------|------------|---------|
| 1-3 | **Balanced Baseline** | None | speed=1.05, social=12, goal=3, obstacle=15 | Benchmark |
| 4-6 | Social Low | social_force_factor | social=5 (weak avoidance) | Do lower values → more collisions? |
| 7-9 | Social High | social_force_factor | social=20 (strong avoidance) | Do higher values → fewer collisions? |
| 10-12 | Goal Low | goal_force_factor | goal=1 (weak goal attraction) | Do lower values → worse path efficiency? |
| 13-15 | Goal High | goal_force_factor | goal=5 (strong goal attraction) | Do higher values → better efficiency? |
| 16-18 | Speed Slow | max_vel | speed=0.7 (slow) | How does speed affect everything? |
| 19-21 | Speed Fast | max_vel | speed=1.5 (fast) | Is there a speed-collision tradeoff? |

**Key principle:** Each variation still uses normal distributions (heterogeneity), but with the mean shifted.

---

## Metrics Compared Against Ground Truth

After each test run completes, metrics are automatically extracted and compared to **ETH/UCY pedestrian dataset ranges**:

### Tier 1: Critical (Always Compare)
| Metric | ETH/UCY Range | What It Means |
|--------|---------------|--------------|
| Mean speed | 0.64–1.46 m/s | Typical walking pace |
| Speed std dev | 0.34–0.53 m/s | Variation in walking speed |
| Collision rate | 0.0004–0.094 events/ped/s | How often agents overlap |
| Near-miss rate | 0.009–0.229 events/ped/s | Close encounters |

### Tier 2: Important (Track Trends)
| Metric | ETH/UCY Range |
|--------|---------------|
| Path efficiency | 0.87–0.97 |
| Acceleration | 0.12–0.74 m/s² |
| Min inter-agent distance (5th %ile) | 0.80–1.88 m |

---

## Scenario Files & Generation

All 7 OAT test configurations are pre-generated as YAML files in:
```
src/hunav_gazebo_wrapper/scenarios/domenic/
├── cafe_oat_balanced_baseline.yaml
├── cafe_oat_social_low.yaml
├── cafe_oat_social_high.yaml
├── cafe_oat_goal_low.yaml
├── cafe_oat_goal_high.yaml
├── cafe_oat_speed_slow.yaml
└── cafe_oat_speed_fast.yaml
```

### How They're Generated

Run the generator script (already done, but here's how if you need to regenerate):

```bash
python3 src/DATASETS/domenic_analysis/generate_oat_scenarios.py
```

**Script logic:**
1. Define 7 configurations with different distribution means
2. For each configuration, create 25 agents:
   - Sample `max_vel`, `social_force_factor`, `goal_force_factor`, `obstacle_force_factor` from appropriate normal distribution
   - Clip values to physical bounds
   - Randomly assign spawn positions (validated 0.6m minimum separation)
   - Randomly assign goals from pool
3. Output YAML file

**Common properties (all 7 configs):**
- 25 agents per scenario (realistic crowd size)
- Same spawn region (café environment)
- Same goal pool (10 waypoints)
- All agents use "Regular" behavior (no robot interactions)

---

## Experimental Workflow: Running the OAT Tests

### Prerequisites

```bash
cd ~/sfm_ws_fresh
colcon build --packages-select hunav_gazebo_wrapper hunav_evaluator
source install/setup.bash
```

### 3-Terminal Setup

**Terminal 1: Gazebo + Evaluator + World**
```bash
ros2 launch hunav_gazebo_wrapper cafe_no_robot.launch.py configuration_file:=domenic/cafe_oat_[CONFIG_NAME].yaml
```
Wait for: `✓ World generator finished`

**Terminal 2: Recording Control**
```bash
# Wait 10 seconds for Terminal 1, then run one of these per run:
ros2 service call /hunav_start_recording hunav_msgs/srv/StartRecording \
  "{experiment_tag: [EXPERIMENT_TAG], run_id: [1-3]}" && \
sleep 60 && \
ros2 service call /hunav_stop_recording std_srvs/srv/Empty {}
```

**Terminal 3: Profile Switching**
- Monitor and restart Terminal 1 with new configuration between test sets

### Run Sequence Example: Tests 1-3 (Balanced Baseline)

**Terminal 1 (once at start):**
```bash
ros2 launch hunav_gazebo_wrapper cafe_no_robot.launch.py \
  configuration_file:=domenic/cafe_oat_balanced_baseline.yaml
```

**Terminal 2 (run all 3 in sequence):**
```bash
# Run 1
sleep 10 && ros2 service call /hunav_start_recording hunav_msgs/srv/StartRecording \
  "{experiment_tag: baseline_oat, run_id: 1}" && \
sleep 60 && ros2 service call /hunav_stop_recording std_srvs/srv/Empty {}

# Run 2 (after Run 1 completes, no Terminal 1 restart needed)
ros2 service call /hunav_start_recording hunav_msgs/srv/StartRecording \
  "{experiment_tag: baseline_oat, run_id: 2}" && \
sleep 60 && ros2 service call /hunav_stop_recording std_srvs/srv/Empty {}

# Run 3 (same as Run 2)
ros2 service call /hunav_start_recording hunav_msgs/srv/StartRecording \
  "{experiment_tag: baseline_oat, run_id: 3}" && \
sleep 60 && ros2 service call /hunav_stop_recording std_srvs/srv/Empty {}
```

**Terminal 1 (restart between test sets):**
```bash
# After Tests 1-3, CTRL+C and restart with:
ros2 launch hunav_gazebo_wrapper cafe_no_robot.launch.py \
  configuration_file:=domenic/cafe_oat_social_low.yaml

# Then Terminal 2 runs Tests 4-6 with the same recording loop
```

### Quick Reference for All Commands

See [OAT_QUICK_REF.md](OAT_QUICK_REF.md) for all 21 test commands (copy-paste ready).

---

## Analysis: Interpreting Results

After all 21 runs complete, run:

```bash
python3 src/DATASETS/domenic_analysis/compare_results.py
```

**Output shows:**
- Each test configuration's metrics
- Alignment score vs. ETH/UCY ground truth (0-100%)
- Which parameters showed largest metric changes

**Questions answered:**
- **Tests 1-3:** Is balanced baseline realistic? (If no → all parameters need tuning)
- **Tests 4-9:** Should we raise or lower social force? (Look at collision/near-miss changes)
- **Tests 10-15:** Does goal force affect efficiency? (Look at path efficiency vs. baseline)
- **Tests 16-21:** Is there a speed-collision tradeoff? (Look at speed metrics vs. collision metrics)

### Example Analysis Interpretation

Suppose results show:
```
Baseline (balanced):      speed=1.2 m/s ✓, collisions=0.02/ped/s ✗ (too many)
Social Low:               speed=1.2 m/s ✓, collisions=0.05/ped/s ✗✗ (worse!)
Social High:              speed=1.2 m/s ✓, collisions=0.008/ped/s ✓ (better)
```

**Interpretation:** "Increasing social_force_factor from 12 to 20 reduced collision rate from 0.02 to 0.008 events/ped/s, moving closer to ETH/UCY benchmark (0.0004–0.094). This indicates social force is critical for collision dynamics."

Later in your report: "Based on sensitivity analysis, we increased social_force_factor to 18.0 for the final profile, which achieved metrics within ETH/UCY benchmarks for collision rate, speed, and efficiency."

---

## From OAT Analysis to Final Profile

### Step 1: Identify Sensitive Parameters
Which variations most affected which metrics? Rank them.

### Step 2: Design Final Profile
Combine best findings:
```python
# Example: If analysis showed
# - Social High helped collisions
# - Goal High helped efficiency  
# - Speed Slow had fewer far-reaching effects
# Design:
final_profile = {
    'max_vel': N(1.05, 0.15),           # Keep baseline (not critical)
    'social_force_factor': N(18.0, 2.0), # ← Increased (collision reduction)
    'goal_force_factor': N(3.5, 0.5),   # ← Slightly increased (efficiency)
    'obstacle_force_factor': N(15.0, 3.0) # Keep baseline
}
```

### Step 3: Validate Final Profile
Run 3 more tests with final profile to confirm metrics improved.

### Step 4: Document for Thesis
"OAT sensitivity analysis revealed that [X] and [Y] parameters most affect metric [Z]. By systematically varying each parameter while controlling others, we identified [param] as the primary driver of [metric]. The final profile incorporates these findings..."

---

## Expected Timeline

- **Setup & build:** 5 minutes
- **Tests 1-3 (baseline):** 5 minutes
- **Tests 4-21 (variations):** ~45-60 minutes (3 runs × 60s per run, plus Terminal 1 restarts)
- **Analysis & final profile design:** 30 minutes
- **Total:** ~1.5 hours active time

---

## File Locations

| What | Where |
|------|-------|
| OAT scenario YAMLs | `src/hunav_gazebo_wrapper/scenarios/domenic/cafe_oat_*.yaml` |
| Scenario generator | `src/DATASETS/domenic_analysis/generate_oat_scenarios.py` |
| Metrics analyzer | `src/DATASETS/domenic_analysis/compare_results.py` |
| Ground truth data | `src/DATASETS/eth/`, `src/DATASETS/ucy/` |
| Launch file | `src/hunav_gazebo_wrapper/launch/cafe_no_robot.launch.py` |

---

## Key Concepts Summary

| Concept | Why It Matters |
|---------|----------------|
| **Normal distributions** | Accounts for human heterogeneity (not all people identical) |
| **OAT methodology** | Isolates parameter→metric relationships (causation, not correlation) |
| **3 runs per config** | Statistical confidence (one run could be noise) |
| **Ground truth comparison** | Validates realism against real ETH/UCY crowds |
| **Baseline-controlled** | Changes are due to *one parameter*, nothing else |

---

## Troubleshooting

**Q: Gazebo says "not responding"**
- Normal. Click "Wait". Simulation continues in background.

**Q: Metrics not computed**
- Check Terminal 1: Did "World generator finished" print?
- Check Terminal 2: Did recording commands execute without errors?
- Make sure recording ran ≥5 seconds before stop.

**Q: Results directory not where expected**
- Find results: `find ~/.ros -name "true_pos_*.csv" -mmin -5`
- Metrics should be in the same folder as `true_pos_.csv`.

---

## Next Steps

1. **Run Tests 1-3** (Balanced Baseline) to establish benchmark
2. **Review baseline metrics** — Are they within ETH/UCY ranges?
3. **Run Tests 4-21** (Variations) to identify sensitive parameters
4. **Analyze results** and create final profile
5. **Document methodology** for thesis/report

Good luck! 🚀
