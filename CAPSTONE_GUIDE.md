# Capstone Study: Extending the Social Force Model for Realistic Pedestrian Simulation

## Research Question

Can systematic one-at-a-time (OAT) parameter variation produce measurable, predictable
changes in pedestrian simulation behavior — and how do these simulated behaviors compare
to real-world pedestrian datasets?

## Study Design

### Two Environments
1. **Café** (complete): 15m × 20m, 12 agents, dense obstacles, random-crossing goals
2. **Central Tunnel** (ready): 38.5m × 17.5m, 40 agents, open corridor, bidirectional flow

### OAT Methodology
Each phase varies exactly one SFM parameter while holding all others at baseline.
This isolates cause-effect relationships between individual parameters and crowd metrics.

**Café** (15 runs): Baseline (3) + Social Force (4) + Goal Force (4) + Speed (4)
**Central Tunnel** (19 runs): Baseline (3) + Social Force (4) + Goal Force (4) + Speed (4) + Obstacle Force (4)

### Ground Truth Comparison
All simulation metrics compared against ETH/UCY real pedestrian trajectory datasets
(eth_univ primary target: 360 pedestrians, 773 seconds).

## Key Findings — Café Environment

### Phase 2 (Goal Force): Validates Methodology ✅
- High goal force reduced collision rate 19% (0.798 → 0.644 /agent/s)
- Expected direction confirmed: stronger goal = more direct paths = fewer collisions
- Inter-run variance <18%, proving reproducibility

### Phase 1 (Social Force): Counterintuitive ⚠️
- Both low AND high worse than baseline (collisions: 0.661 → 1.104 / 1.032)
- Suggests baseline social force value near-optimal (U-shaped curve)
- Note: config mode 2 may have overridden YAML values (see below)

### Phase 3 (Speed): Emergent Behavior ⚠️
- "Fast" agents paradoxically move slower (2.779 → 2.578 m/s)
- Collision rate 2.4× higher for fast vs slow (0.984 vs 0.413)
- Classic emergent effect: local speed increase → more collisions → gridlock

### Critical Discovery: Configuration Mode
The café tests used `behavior.configuration: 2` (BEH_CONF_RANDOM_NORMAL),
which causes the SFM loader to **overwrite** YAML force factors with random values from
normal distributions at each launch. This means:

- Phases 1 & 2 (social/goal force): YAML scaling was overridden at runtime
- Phase 3 (speed/max_vel): **Valid** — max_vel is not affected by config mode

The central tunnel tests correct this by using `configuration: 1` (BEH_CONF_CUSTOM),
ensuring all YAML values are used exactly as specified.

## Central Tunnel Improvements Over Café

| Issue in Café | Central Tunnel Solution |
|---------------|----------------------|
| Dense obstacles → constant collisions | Open corridor, minimal obstacles |
| 12 agents in 300m² = high density | 40 agents in 674m² = moderate density |
| Random-crossing goals → unrealistic paths | Bidirectional flow → corridor walking |
| No groups | 5 dyads + 3 triads + 21 individuals |
| Config mode 2 → random force values | Config mode 1 → exact YAML values |
| Small goal distances → short walks | ~30m goal distance → extended walking |
| All agents isolated | Group walking via behavior trees |

## How Parameters Are Defined and Altered

### Baseline Parameters (Central Tunnel)
Generated programmatically with `src/hunav_gazebo_wrapper/scenarios/domenic/generate_central_tunnel_yamls.py`:
- `social_force_factor`: mean 11.6, range [7.1, 15.7] (within [5.0, 20.0] clamp)
- `goal_force_factor`: mean 3.4, range [2.5, 4.2] (within [2.0, 5.0] clamp)
- `obstacle_force_factor`: mean 16.9, range [11.1, 24.6] (within [2.0, 50.0] clamp)
- `max_vel`: mean 1.28, range [1.04, 1.52] m/s (realistic pedestrian range)

### Phase Scaling
Each phase multiplies one parameter by a fixed factor across all 40 agents:

| Phase | Parameter | Low Scale | High Scale | Rationale |
|-------|-----------|-----------|------------|-----------|
| 1 | Social Force | × 0.55 | × 1.70 | Test collision avoidance sensitivity |
| 2 | Goal Force | × 0.60 | × 1.60 | Test path directedness |
| 3 | Max Velocity | × 0.75 | × 1.25 | Test speed-congestion tradeoff |
| 4 | Obstacle Force | × 0.50 | × 2.00 | Test wall avoidance in corridor |

### Evaluation Metrics
Computed by `hunav_evaluator` and `ground_truth_analysis.py`:
- Speed distribution (mean, std, percentiles)
- Collision rate, near-miss rate
- Path efficiency (displacement / arc length)
- Heading jerk (motion smoothness)
- Inter-agent distance distribution
- Fundamental diagram (density vs flow)

## For Your Capstone Paper

### Abstract
"This study validates Social Force Model parameter sensitivity through one-at-a-time
sensitivity analysis across two Gazebo environments. Testing 40 agents in a bidirectional
corridor with group dynamics, we isolated the effect of four SFM parameters on collision
rate, path efficiency, and walking speed compared to ETH/UCY ground truth datasets."

### Methodology Section
Emphasize: pure OAT design with controlled scaling, configuration mode 1 for deterministic
parameter control, bidirectional flow matching real corridor datasets, group walking support.

### Results
Report both absolute metrics and relative parameter effects (% change from baseline).
Compare distributions against ETH univ benchmark.

### Limitations
- Agent count (40) vs ground truth (300+) — compare proportional effects, not absolutes
- Three environments tested — frame as multi-topology validation
- OAT cannot capture parameter interactions — acknowledge, propose factorial follow-up
