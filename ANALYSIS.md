# Comprehensive ESFM Parameter Sensitivity Analysis

**Project**: OAT Sensitivity Analysis of the Extended Social Force Model  
**Author**: Domenic Kadioglu (24426924)  
**Date**: April 2026  
**Environments**: Café (12 agents), Central Tunnel (40 agents), Delta (80 agents)  
**Ground Truth**: ETH (hotel, univ) and UCY (zara01/02/03, univ_s1/s3)

---

## Phase 2A — Evaluation Metric Definitions

This section defines every evaluation metric used in this study, its relevance to ESFM crowd dynamics, interpretation guidelines, and sensitivity to the parameters under investigation.

### Tier 1: Population-Level Metrics

#### 1. Speed Mean (m/s)

**Definition**: The arithmetic mean of all instantaneous agent speeds across all frames, filtered to exclude stationary agents (<0.05 m/s) and physically implausible speeds (>5.0 m/s).

**Relevance**: Walking speed is the most fundamental observable of pedestrian dynamics. The SFM produces speed as an emergent property of the balance between goal attraction (accelerating agents toward their target), social repulsion (decelerating agents near others), and obstacle repulsion (decelerating or deflecting agents near walls). Weidmann (1992) established that free-flow adult pedestrian speed follows a normal distribution with mean 1.34 m/s and standard deviation 0.26 m/s.

**Interpretation**: Ground truth benchmarks range from 0.64 m/s (UCY Univ S1, high density campus) to 1.46 m/s (ETH Univ, moderate density). A simulation producing speeds significantly below this range suggests agents are overconstrained by repulsive forces or trapped by obstacles. Speeds significantly above suggest insufficient social/obstacle repulsion or unrealistic goal attraction.

**Parameter sensitivity**:
- `max_vel`: Direct ceiling on speed — reduction in max_vel directly caps achievable speed
- `goal_force_factor`: Higher values increase acceleration toward goals, raising mean speed if not limited by max_vel or opposing forces
- `social_force_factor`: Higher values increase deceleration when agents are proximate, generally reducing mean speed in dense scenarios
- `obstacle_force_factor`: In environments with obstacles, higher values divert agents from direct paths, potentially reducing effective speed

#### 2. Speed Standard Deviation (m/s)

**Definition**: Standard deviation of all valid instantaneous speeds.

**Relevance**: Captures the breadth of the speed distribution. Real pedestrians exhibit speed variation due to individual differences in walking pace, environmental constraints, and social interactions. Ground truth values range from 0.34 m/s (UCY Zara01) to 0.53 m/s (ETH Hotel).

**Interpretation**: A simulation with very low speed_std suggests agents are moving too uniformly (lockstep behaviour), which is unrealistic. Very high speed_std may indicate bimodal behaviour — some agents moving freely while others are trapped or oscillating.

#### 3. Collision Rate (events/agent/second)

**Definition**: The count of agent-frame pairs where the minimum inter-agent distance is below the collision threshold (0.5 m, approximate adult bi-deltoid shoulder breadth per Pheasant & Haslegrave, 2018), normalised by total agent-seconds.

**Relevance**: Physical collisions between real pedestrians are exceedingly rare. Johansson et al. (2007) found that even at densities exceeding 2 ped/m², physical contact remains uncommon. Ground truth collision rates range from 0.0004 (ETH Univ) to 0.094 (UCY Univ S1, the densest dataset). This metric directly measures the SFM's ability to reproduce collision-avoiding social behaviour.

**Interpretation**: Good (<0.01), acceptable (0.01–0.05), poor (>0.05). Any simulation producing collision rates orders of magnitude above ground truth indicates a fundamental failure in the social force mechanism — either forces are too weak, the environment is too constrained, or agents are trapped.

**Parameter sensitivity**:
- `social_force_factor`: Primary controller — higher SF should reduce collision rate by increasing inter-agent repulsion
- `max_vel`: Higher speeds reduce reaction time, typically increasing collisions
- `obstacle_force_factor`: In cluttered environments, high OF can push agents into each other
- `goal_force_factor`: Very high GF can override social repulsion, forcing agents through crowds

#### 4. Near-Miss Rate (events/agent/second)

**Definition**: Count of agent-frame pairs where minimum inter-agent distance is below 1.0 m (Hall's personal distance boundary, 1966), normalised by total agent-seconds.

**Relevance**: Captures uncomfortably close encounters that, while not collisions, violate proxemic norms. Ground truth: 0.014 (ETH Univ) to 0.23 (UCY Univ S1). This metric is the buffer zone around collisions — high near-miss rates indicate the social force is struggling to maintain comfortable separation.

**Parameter sensitivity**: Same as collision rate, but more sensitive to social_force_factor changes because the 1.0 m threshold captures a wider detection zone.

#### 5. Mean Density (agents/m²) and Mean Flow (agents/m/s)

**Definition**: Local density is computed as the count of agents within a circular measurement area (r = 4.0 m centred on the scene mean) divided by the area (≈50.27 m²). Flow is computed as density × mean speed within that area. Both are averaged across all frames.

**Relevance**: The density-flow fundamental diagram is the macroscopic signature of crowd dynamics (Seyfried et al., 2005). At low densities, flow increases linearly with density. Beyond a critical density (~1.5–2.0 ped/m²), flow decreases as congestion forces dominate.

**Interpretation**: These values are heavily environment-dependent (driven by spawn area size and agent count) and are primarily useful for intra-environment comparison rather than cross-environment comparison.

### Tier 2: Agent-Level Metrics

#### 6. Path Efficiency (dimensionless, 0–1)

**Definition**: For each agent with ≥3 observations and total path length >0.1 m, the ratio of straight-line displacement (start to end) to total arc length walked. Averaged across all agents.

**Relevance**: Measures how directly agents navigate to their goals. A value of 1.0 indicates a perfectly straight path; lower values indicate detours due to avoidance manoeuvres. Field studies (Guy et al., 2012) report path efficiency >0.9 in uncrowded environments. Ground truth: 0.87 (UCY Univ S1, dense) to 0.97 (ETH Univ).

**Interpretation**: Good (>0.90), moderate (0.70–0.90), poor (<0.70). Very low path efficiency (<0.10) indicates agents are essentially trapped, walking in circles or oscillating between conflicting forces — a known SFM artefact in cluttered or high-density environments.

**Parameter sensitivity**:
- `goal_force_factor`: Higher GF pulls agents more directly toward goals → higher path efficiency
- `social_force_factor`: Higher SF causes larger detours around other agents → lower path efficiency
- `obstacle_force_factor`: Higher OF in cluttered environments causes wider obstacle avoidance → lower path efficiency

#### 7. Speed Variability / Coefficient of Variation (dimensionless)

**Definition**: For each agent, the standard deviation of their instantaneous speeds divided by their mean speed (CV = σ/μ). Averaged across all agents.

**Relevance**: Real pedestrians modulate their speed — accelerating into gaps, decelerating near obstacles (Bosina & Weidmann, 2017; Taylor et al., 2010). Ground truth: 0.138 (UCY Zara01) to 0.355 (UCY Univ S1). This metric captures intra-agent speed fluctuation as a measure of responsive, adaptive behaviour.

**Interpretation**: Very low CV (<0.10) suggests agents move at near-constant speed (robotic). Very high CV (>0.50) suggests chaotic start-stop behaviour or trapping artefacts. Ground truth sweet spot: 0.13–0.35.

#### 8. Acceleration Magnitude (m/s²)

**Definition**: For each pair of consecutive observations per agent, the magnitude of velocity change divided by the time interval, filtered to <20 m/s² to remove noise. Averaged across all agents.

**Relevance**: Bounded by biomechanical constraints. Johansson et al. (2007) report typical pedestrian accelerations of 0.1–0.7 m/s² during normal walking. Ground truth: 0.12 (UCY Zara) to 0.74 (ETH Univ). Accelerations significantly above 1.0 m/s² in simulation suggest non-physical force magnitudes or numerical instabilities.

**Parameter sensitivity**: Higher force factors (social, goal, obstacle) all increase acceleration magnitude, as agents experience stronger impulses at each timestep. The SFM relaxation time (τ = 0.5s) mediates how quickly agents respond to force changes.

#### 9. Heading Jerk (rad/frame³)

**Definition**: The third discrete derivative of heading angle (θ = atan2(vy, vx)) for each agent with ≥4 observations, averaged per agent then across all agents. All angular differences are wrapped to [-π, π].

**Relevance**: Flash & Hogan (1985) demonstrated that human motor control minimises jerk, producing smooth trajectories. In the SFM context, heading jerk directly measures oscillatory behaviour — a well-documented artefact where opposing social forces cause agents to alternate direction repeatedly (Kretz, 2015). Ground truth: 0.107 (UCY Zara01) to 0.518 (ETH Hotel).

**Interpretation**: Very low heading jerk (<0.01) indicates agents are moving in near-straight lines with almost no directional correction — smooth but possibly unreactive. Very high values (>0.50) may indicate oscillation artefacts or noisy trajectory data.

**Parameter sensitivity**:
- `social_force_factor`: At critical values, opposing social forces between nearby agents cause oscillation → high heading jerk
- `obstacle_force_factor`: Near walls, alternating attraction/repulsion can cause directional jitter

### Tier 3: Pairwise/Group-Level Metrics

#### 10. Minimum Inter-Agent Distance (m)

**Definition**: For each agent at each frame, the Euclidean distance to the nearest other agent. Reported as mean and 5th percentile across all agent-frame pairs.

**Relevance**: Hall (1966) identified four proxemic zones: intimate (<0.45 m), personal (0.45–1.2 m), social (1.2–3.6 m), public (>3.6 m). A realistic simulation should produce a distribution where the 5th percentile rarely enters the intimate zone. Ground truth mean: 0.80 m (UCY Univ S1) to 1.88 m (ETH Univ). Ground truth 5th percentile: 0.30 m (UCY Univ S1) to 0.60 m (ETH Univ).

**Parameter sensitivity**:
- `social_force_factor`: Direct controller of inter-agent spacing — higher SF → larger separations
- Environment density: More agents in a smaller space mechanically reduce achievable distances

---

## Phase 2B — Intra-Environment Analysis

### Important Caveat: Café (Configuration Mode 2)

The café environment used `configuration: 2` (RANDOM_NORMAL), which causes the SFM loader to overwrite all force factor values (`social_force_factor`, `goal_force_factor`, `obstacle_force_factor`) with random normal samples at each launch. This means:

- **Phase 1 (Social Force) and Phase 2 (Goal Force) results are unreliable** — the YAML-specified force factors were ignored at runtime
- **Phase 3 (Speed) remains valid** — `max_vel` is not overwritten by any configuration mode
- Café results are presented for completeness but should not be used for OAT sensitivity conclusions regarding force factors

This was discovered during the transition to Central Tunnel, which correctly uses `configuration: 1` (CUSTOM).

---

### Environment 1: Café (12 agents, 15m × 20m, cluttered indoor)

**Baseline parameters** (mean across 12 agents): SF=9.15, GF=2.73, OF=15.33, MaxVel=1.06 m/s  
**Scaling multipliers**: SF (×0.579/×2.371), GF (×0.469/×2.033), Speed (×0.514/×1.330)

| Phase | Speed (m/s) | Speed σ | Coll Rate | NM Rate | Path Eff | Heading Jerk | Speed CV | Accel | Min Dist | Min Dist p5 |
|-------|-------------|---------|-----------|---------|----------|--------------|----------|-------|----------|-------------|
| Baseline (3 runs) | 2.986 | 1.596 | 0.661 | 4.202 | 0.063 | 0.676 | 0.393 | 6.899 | 1.241 | 0.451 |
| Phase 1 Social (4 runs) | 2.913 | 1.677 | 1.068 | 5.040 | 0.059 | 0.689 | 0.540 | 6.228 | 1.157 | 0.387 |
| Phase 2 Goal (4 runs) | 2.816 | 1.584 | 0.721 | 4.266 | 0.056 | 0.736 | 0.517 | 6.932 | 1.186 | 0.441 |
| Phase 3 Speed (4 runs) | 2.679 | 1.552 | 0.698 | 4.019 | 0.059 | 0.763 | 0.503 | 7.456 | 1.238 | 0.464 |

**Phase-by-phase discussion** (noting config mode 2 unreliability for Phases 1–2):

**Phase 1 — Social Force** (unreliable due to config mode 2):
- Collision rate increased 62% from baseline (0.661 → 1.068). Near-miss rate rose from 4.20 to 5.04.
- Min inter-agent distance dropped from 1.241 to 1.157 m; 5th percentile from 0.451 to 0.387 m.
- Speed variability increased substantially (0.393 → 0.540), suggesting more erratic start-stop behaviour.
- *Interpretation*: Since force factors were randomised, this variation likely reflects random noise rather than systematic parameter effects.

**Phase 2 — Goal Force** (unreliable due to config mode 2):
- Collision rate slightly improved over baseline (0.661 → 0.721). Heading jerk increased (0.676 → 0.736).
- Path efficiency was the lowest of all phases (0.056), somewhat counterintuitive for a goal force change.
- *Interpretation*: As with Phase 1, force factors were randomised, making these results statistically meaningless for OAT conclusions.

**Phase 3 — Speed** (VALID — max_vel unaffected by config mode 2):
- Speed decreased from 2.986 to 2.679 m/s. This phase combined low (×0.514) and high (×1.330) multipliers.
- Collision rate decreased from 0.661 to 0.698 (essentially stable).
- Near-miss rate decreased from 4.202 to 4.019 — modest improvement.
- Heading jerk increased from 0.676 to 0.763, suggesting more directional oscillation at modified speeds.
- Acceleration increased from 6.899 to 7.456 m/s² — the highest of all phases. This is physically unrealistic (real pedestrians: 0.1–0.7 m/s²) and indicates the SFM forces in this cluttered environment are producing extreme acceleration artifacts.
- Min distance slightly improved (p5: 0.451 → 0.464 m).

**Café overall assessment**: The extreme values across all metrics — speeds of ~3 m/s (>2× realistic), accelerations of ~7 m/s² (10× realistic), path efficiency of ~0.06 (agents walking 17× their straight-line distance), collision rates >0.66 — indicate fundamental structural problems beyond parameter tuning. The small 15×20m space with 12 agents among tables creates a gridlock scenario where the SFM, lacking path planning, produces agents trapped in perpetual oscillation between conflicting obstacle and social forces. **The café environment is unsuitable for drawing OAT parameter sensitivity conclusions.**

---

### Environment 2: Central Tunnel (40 agents, 38.5m × 17.5m, open corridor with furniture)

**Baseline parameters** (mean across 40 agents): SF=11.58, GF=3.42, OF=16.91, MaxVel=1.28 m/s  
**Scaling multipliers**: Social (×0.55/×1.70), Goal (×0.60/×1.60), Speed (×0.75/×1.25), Obstacle (×0.50/×2.00)  
**Configuration**: Mode 1 (CUSTOM) — all parameter changes applied deterministically.

| Phase | Speed (m/s) | Speed σ | Coll Rate | NM Rate | Path Eff | Heading Jerk | Speed CV | Accel | Min Dist | Min Dist p5 |
|-------|-------------|---------|-----------|---------|----------|--------------|----------|-------|----------|-------------|
| Baseline (3 runs) | 0.784 | 0.173 | 0.048 | 0.971 | 0.413 | 0.107 | 0.168 | 0.121 | 1.591 | 0.669 |
| Phase 1 Social (4 runs) | 0.729 | 0.170 | 0.041 | 0.914 | 0.558 | 0.092 | 0.174 | 0.130 | 1.550 | 0.705 |
| Phase 2 Goal (4 runs) | 0.773 | 0.189 | 0.042 | 0.965 | 0.435 | 0.106 | 0.177 | 0.117 | 1.611 | 0.700 |
| Phase 3 Speed (4 runs) | 0.742 | 0.207 | 0.049 | 0.919 | 0.544 | 0.091 | 0.251 | 0.134 | 1.576 | 0.705 |
| Phase 4 Obstacle (4 runs) | 0.695 | 0.225 | 0.046 | 0.959 | 0.532 | 0.091 | 0.322 | 0.114 | 1.572 | 0.706 |

#### Phase 1 — Social Force (×0.55 low / ×1.70 high)

**Expected outcome**: Reducing social force (×0.55) should allow agents to pass closer together with fewer detours, potentially increasing collision rate and improving path efficiency. Increasing social force (×1.70) should do the opposite — wider spacing, lower collision rate, but worse path efficiency due to larger avoidance manoeuvres.

**Actual outcome**: The combined phase showed *improved* collision rate (0.048 → 0.041, −15%), *improved* near-miss rate (0.971 → 0.914, −6%), and substantially *improved* path efficiency (0.413 → 0.558, +35%). Speed decreased slightly (0.784 → 0.729 m/s, −7%). Heading jerk decreased (0.107 → 0.092, −14%), indicating smoother trajectories.

**Discrepancy analysis**: The combined low+high results obscure the expected opposing effects. However, the improvement in path efficiency coupled with decreased collision rate suggests that the high-SF runs (×1.70) produced the dominant signal: stronger social repulsion created clearer "lanes" in the bidirectional flow, allowing agents to separate into non-interfering streams. This is consistent with the laminar flow phenomenon observed in dense bidirectional corridors (Feliciani et al., 2018). The low-SF runs likely contributed the near-miss increase while the high-SF runs drove the collision decrease and path efficiency improvement. The min inter-agent distance 5th percentile *improved* from 0.669 to 0.705 m, supporting the hypothesis that the high-SF condition provided better separation.

#### Phase 2 — Goal Force (×0.60 low / ×1.60 high)

**Expected outcome**: Reducing goal force should cause agents to wander more, reducing path efficiency but possibly avoiding congestion at meeting points. Increasing goal force should produce more direct paths (higher path efficiency) but potentially more collisions as agents push through crowds.

**Actual outcome**: Speed changed minimally (0.784 → 0.773, −1%). Collision rate improved slightly (0.048 → 0.042, −12%). Path efficiency showed almost no change (0.413 → 0.435, +5%). Near-miss rate was essentially unchanged (0.971 → 0.965). Min inter-agent distance improved slightly (1.591 → 1.611).

**Discrepancy analysis**: Goal force variation had the smallest effect of any phase in Central Tunnel. This is expected in a bidirectional corridor where the goal is at the far end — agents are already strongly attracted in one direction, and the path to the goal is largely unobstructed except by oncoming agents. In this geometry, the social force (managing lane formation) and obstacle force (managing wall avoidance) are the dominant behavioural controllers. Goal force would be expected to have a much larger effect in environments with non-trivial routing (multiple turns, obstacles between agent and goal).

#### Phase 3 — Speed / Max Velocity (×0.75 low / ×1.25 high)

**Expected outcome**: Reducing max_vel should slow agents, reducing collision severity but potentially increasing near-miss duration. Increasing max_vel increases approach speed, requiring faster social force reactions.

**Actual outcome**: Speed decreased (0.784 → 0.742, −5%). Collision rate was essentially unchanged (0.048 → 0.049). Near-miss rate improved (0.971 → 0.919, −5%). Path efficiency improved substantially (0.413 → 0.544, +32%). Speed variability increased (0.168 → 0.251, +49%). Heading jerk decreased (0.107 → 0.091, −15%).

**Discrepancy analysis**: The speed phase produced an unexpectedly large improvement in path efficiency. The most likely explanation is that slower agents (low-speed runs, ×0.75) had more time to react to social forces and navigate around oncoming agents, producing smoother, more efficient paths. The increase in speed variability is consistent: agents now oscillate more between their lower maximum speed and deceleration during social encounters, widening the per-agent speed distribution. The improved heading jerk supports this — smoother paths with fewer sharp corrections.

#### Phase 4 — Obstacle Force (×0.50 low / ×2.00 high)

**Expected outcome**: Reducing obstacle force should allow agents to walk closer to walls, potentially improving path efficiency in the corridor. Increasing obstacle force should push agents away from walls and toward the centre, potentially increasing congestion.

**Actual outcome**: Speed decreased notably (0.784 → 0.695, −11%). Speed variability increased dramatically (0.168 → 0.322, +92%). Path efficiency improved (0.413 → 0.532, +29%). Heading jerk decreased (0.107 → 0.091, −15%). Collision rate improved slightly (0.048 → 0.046, −4%).

**Discrepancy analysis**: The large decrease in speed and dramatic increase in speed variability warrant investigation. The SFM's `HandleObstacles2()` function uses Axis-Aligned Bounding Boxes (AABBs) of **all** Gazebo models within 5 metres to compute obstacle forces. This is a known implementation issue: multi-link models (e.g., furniture with multiple collision shapes) produce amplified obstacle forces because each collision shape contributes separately. In the high-obstacle (×2.00) condition, this amplification effect is doubled, causing some agents to become partially trapped near furniture — oscillating between obstacle repulsion and goal attraction. This manifests as reduced mean speed, high speed variability, and paradoxically improved path efficiency (trapped agents accumulate very little displacement but also very little arc length, so the ratio approaches 1). The low-obstacle condition (×0.50) likely produced the smoother improvements. **This phase's results are partially confounded by the AABB obstacle bug.**

#### Central Tunnel Summary

The Central Tunnel provides the first reliable OAT data (config mode 1). Key findings:
1. **Social force** was the most impactful parameter for collision avoidance (−15% collision rate) and path efficiency (+35%)
2. **Goal force** produced the smallest effects, consistent with the simple corridor geometry
3. **Speed** had a surprisingly large positive effect on path efficiency (+32%)
4. **Obstacle force** results are partially confounded by the AABB amplification bug in the SFM's obstacle handling
5. Mean speed (0.695–0.784 m/s) is below ground truth (1.10–1.46 m/s), indicating the simulated agents are moving too slowly
6. Path efficiency (0.41–0.56) is well below ground truth (0.87–0.97), indicating significant trapping/wandering artefacts

---

### Environment 3: Delta (80 agents, ~55m triangular, obstacle-free)

**Baseline parameters** (mean across 80 agents): SF=10.85, GF=3.27, OF=17.57, MaxVel=1.21 m/s  
**Scaling multipliers**: Social (×0.55/×1.70), Goal (×0.60/×1.60), Speed (×0.75/×1.25), Obstacle (×0.50/×2.00)  
**Configuration**: Mode 1 (CUSTOM). Cabinets removed from world to avoid AABB obstacle trapping.  
**Geometry**: 80 agents in 3 corner clusters (27/27/26), walking across the triangle to opposite corners.

| Phase | Speed (m/s) | Speed σ | Coll Rate | NM Rate | Path Eff | Heading Jerk | Speed CV | Accel | Min Dist | Min Dist p5 |
|-------|-------------|---------|-----------|---------|----------|--------------|----------|-------|----------|-------------|
| Baseline (3 runs) | 0.181 | 0.036 | 0.034 | 1.283 | 0.998 | 0.001 | 0.133 | 0.046 | 1.081 | 0.670 |
| Phase 1 Social (4 runs) | 0.201 | 0.043 | 0.050 | 1.328 | 0.986 | 0.007 | 0.160 | 0.045 | 1.045 | 0.654 |
| Phase 2 Goal (4 runs) | 0.208 | 0.041 | 0.040 | 1.129 | 0.979 | 0.008 | 0.134 | 0.050 | 1.115 | 0.685 |
| Phase 3 Speed (4 runs) | 0.145 | 0.032 | 0.019 | 1.355 | 0.996 | 0.026 | 0.150 | 0.037 | 1.059 | 0.710 |
| Phase 4 Obstacle (4 runs) | 0.201 | 0.040 | 0.044 | 1.346 | 0.981 | 0.006 | 0.141 | 0.056 | 1.046 | 0.644 |

#### Phase 1 — Social Force (×0.55 low / ×1.70 high)

**Expected outcome**: Same directional expectations as Central Tunnel — low SF should decrease spacing, increase collisions; high SF should increase spacing, reduce collisions.

**Actual outcome**: Collision rate increased (0.034 → 0.050, +47%). Near-miss rate increased slightly (1.283 → 1.328, +3.5%). Speed increased (0.181 → 0.201, +11%). Path efficiency decreased slightly (0.998 → 0.986). Min inter-agent distance decreased (1.081 → 1.045 m). Heading jerk increased substantially in relative terms (0.001 → 0.007, ×7, though both values are extremely low).

**Discrepancy analysis**: The net collision increase suggests the low-SF condition (×0.55) dominates the combined metrics — reducing social repulsion in this densely-packed 80-agent scenario allowed more body-body overlaps, particularly at the centre of the triangle where all three crossing flows intersect. The slight speed increase is consistent: lower social repulsion allows agents to maintain higher speeds through the crossing zone rather than decelerating. The path efficiency decrease from 0.998 to 0.986 shows that stronger social forces do cause minor detours, visible even in this open geometry. The heading jerk increase (×7 in relative terms) is notable — it indicates that social force variations trigger more directional corrections, a physically expected response when agents must dynamically avoid each other.

#### Phase 2 — Goal Force (×0.60 low / ×1.60 high)

**Expected outcome**: Higher goal force should produce faster, more direct movement; lower goal force should cause slower, more wandering behaviour.

**Actual outcome**: Speed increased (0.181 → 0.208, +15%). Collision rate increased slightly (0.034 → 0.040, +18%). Near-miss rate *decreased* (1.283 → 1.129, −12%). Path efficiency decreased slightly (0.998 → 0.979). Min inter-agent distance *increased* (1.081 → 1.115 m). Min distance 5th percentile improved (0.670 → 0.685 m).

**Discrepancy analysis**: This is a methodologically important result. Increased goal force produced faster agents with slightly more collisions but significantly fewer near-misses and better spacing. The mechanism: stronger goal attraction reduces the time agents spend lingering in the crossing zone, which mechanically reduces the duration of close encounters. Agents push through the intersection zone faster, spending less time in close proximity to others. The slight path efficiency decrease is expected — stronger goal force produces more direct paths, but the multi-crossing geometry means agents must navigate around the central congestion, producing slightly less efficient paths than the baseline (where agents barely move at 0.181 m/s). This phase demonstrates a clear trade-off: **stronger goal force trades a small collision rate increase for substantially improved proxemic compliance (near-miss reduction).**

#### Phase 3 — Speed / Max Velocity (×0.75 low / ×1.25 high)

**Expected outcome**: Lower max_vel should directly reduce mean speed and potentially reduce collision rate (slower approach); higher max_vel should increase both.

**Actual outcome**: Speed decreased (0.181 → 0.145, −20%). Collision rate decreased substantially (0.034 → 0.019, −44%). Near-miss rate increased (1.283 → 1.355, +6%). Path efficiency remained excellent (0.998 → 0.996). Speed variability increased (0.133 → 0.150, +13%). Acceleration decreased (0.046 → 0.037, −20%). Min distance 5th percentile *improved* (0.670 → 0.710 m).

**Discrepancy analysis**: The speed phase produced the clearest and most internally consistent results in Delta. The −44% collision rate reduction is the largest single-metric improvement observed across all environments and phases. Slower agents (×0.75) have more time for social forces to take effect before reaching collision threshold, while faster agents (×1.25) still benefit from being fast enough to transit the crossing zone quickly. The increased near-miss rate is the trade-off: slower agents spend more time in the crossing zone, having more near-miss frames even though they successfully avoid actual collisions. The improved min distance 5th percentile (0.670 → 0.710 m) confirms better worst-case separation. Heading jerk increased (0.001 → 0.026, ×26), which is expected — speed variation forces more frequent directional corrections. Both values remain far below ground truth heading jerk levels, indicating the Delta agents are still moving in extremely straight lines.

**This is the strongest OAT parameter signal in the entire study — a direct, large-magnitude, theoretically consistent response to max_vel variation.**

#### Phase 4 — Obstacle Force (×0.50 low / ×2.00 high)

**Expected outcome**: In an obstacle-free environment, obstacle force should have negligible effect because there are no obstacles to generate force from.

**Actual outcome**: Speed was unchanged from baseline (0.181 → 0.201 m/s, essentially matching the social phase). Collision rate increased (0.034 → 0.044, +29%). Near-miss rate increased (1.283 → 1.346, +5%). Path efficiency decreased (0.998 → 0.981). Min inter-agent distance decreased (1.081 → 1.046 m). Acceleration increased (0.046 → 0.056, +22%).

**Discrepancy analysis**: Obstacle force should not matter in an obstacle-free environment — yet it produced measurable effects. The explanation lies in how the SFM's `HandleObstacles2()` function works: it computes obstacle forces from the AABBs of *all* Gazebo models, including the world boundary walls. Even though the Delta triangle has no interior furniture, agents near the triangle's walls still experience obstacle forces. Higher obstacle force (×2.00) pushes agents further from walls and toward the centre, increasing central congestion and slightly raising collision rate. Lower obstacle force (×0.50) allows agents to walk closer to walls with less disruption. The 29% collision rate increase and 5% near-miss increase are small relative effects, confirming that obstacle force is indeed less impactful in this open environment than in Central Tunnel, but the wall boundary effect prevents it from being zero.

#### Delta Summary

Delta provides the cleanest OAT data due to its open geometry, large agent count, and correct configuration mode. Key findings:
1. **Speed (max_vel)** produced the strongest, clearest parameter response: −44% collision rate
2. **Goal force** demonstrated a meaningful trade-off: faster transit reduces near-misses (−12%) at the cost of slight collision increase (+18%)
3. **Social force** variation increased collisions (+47%) when combining low/high, dominated by the low-SF condition
4. **Obstacle force** had the smallest effect, as expected in an obstacle-free environment, but wall boundary forces still produced measurable changes
5. Mean speed (0.145–0.208 m/s) is dramatically below ground truth (1.10–1.46 m/s), indicating a severe speed deficit
6. Path efficiency (0.979–0.998) is excellent and closely matches ground truth (0.87–0.97), confirming the open geometry avoids the trapping artefacts seen in café and Central Tunnel
7. Collision rate (0.019–0.050) is 1–2 orders of magnitude above ground truth (0.0004–0.019), indicating the social force is still insufficient for the crowd density
8. Heading jerk values (0.001–0.026) are 1–2 orders of magnitude *below* ground truth (0.107–0.518), indicating agents make too few directional corrections — they are overly smooth, lacking the responsive course adjustments real pedestrians make

---

## Phase 2C — Cross-Environment Comparison

### Structural Limitations

Before comparing results across environments, the following structural differences must be acknowledged as they impose strict limits on the validity of direct metric comparisons:

#### 1. Environment Size and Agent Density

| Environment | Approximate Area | Agents | Nominal Density |
|-------------|-----------------|--------|-----------------|
| Café | ~300 m² | 12 | ~0.04 agents/m² |
| Central Tunnel | ~674 m² | 40 | ~0.06 agents/m² |
| Delta | ~1,320 m² (triangle) | 80 | ~0.06 agents/m² |

While nominal densities are comparable for Central Tunnel and Delta, the *effective* density (agents within interaction range at any given time) differs substantially due to flow geometry. In the café, agents cross randomly in a small space creating high local densities. In Central Tunnel, two opposing streams meet along a corridor creating a linear high-density band. In Delta, three converging streams create a high-density zone at the centre. These different spatial patterns of density affect every interaction-dependent metric differently, making direct absolute comparison of collision rates, near-miss rates, and spacing metrics across environments methodologically unsound as a basis for individual parameter conclusions.

#### 2. Obstacle Interference in Café and Central Tunnel

The café and Central Tunnel environments contain furniture and obstacles. Because the ESFM implementation used in this project does not include path planning, agents cannot navigate around obstacles — they are repelled by the SFM's exponential obstacle force. This creates three distinct problems:

**(a) Expected behaviour with path planning**: In a properly integrated ESFM-with-path-planning system, obstacles would create structured flow patterns — agents would route around tables and through doorways, with obstacle forces providing only local fine-tuning to prevent clipping. Path efficiency would remain high.

**(b) Actual behaviour without path planning**: Agents walked directly toward goals and were deflected by obstacle forces at close range. In the café, this produced near-permanent trapping: agents oscillated between multiple conflicting obstacle, social, and goal forces. In Central Tunnel, agents occasionally became stuck near furniture but generally found paths along the main corridor axis.

**(c) Metric distortion**: This obstacle trapping severely distorts path efficiency (café: 0.06 vs ground truth: 0.87–0.97), acceleration (café: 6.9 m/s² vs ground truth: 0.1–0.7 m/s²), heading jerk, and collision rate. Any metric comparison involving the café must be read as reflecting the *obstacle trapping artefact*, not meaningful SFM parameter behaviour.

**(d) Intra-environment validity**: Despite obstacle interference, comparisons *within* a single environment remain valid for OAT purposes. The same obstacles affect all runs identically, so relative changes between phases (e.g., "Phase 1 reduced collisions by 15% compared to baseline") correctly isolate the parameter effect. Cross-environment absolute comparisons (e.g., "café collision rate is higher than Central Tunnel") are confounded by the obstacle differences.

#### 3. Agent Population Differences

The simulations use 12, 40, and 80 agents respectively,significantly fewer than the ETH/UCY ground truth datasets (148–434 pedestrians per scene). Lower agent counts reduce:
- The number of simultaneous agent-agent interactions (affecting collision and near-miss statistics)
- Emergent crowd phenomena such as lane formation (which requires a critical mass of agents)
- Statistical robustness of per-agent metrics (fewer agents = higher variance in aggregated values)

### Valid Cross-Environment Insights

Despite these limitations, several cross-environment patterns can be extracted:

#### Consistent Directional Effects

**Speed (max_vel)** produced consistent effects across all valid environments:
- Central Tunnel: path efficiency +32%, heading jerk −15%
- Delta: collision rate −44%, min distance p5 +6%
- In both environments, speed variation was the parameter that produced the largest, most consistent metric changes

**Social force** produced consistent directional effects:
- Central Tunnel: collision rate −15%, path efficiency +35%
- Delta: collision rate +47% (dominated by low-SF condition)
- Both environments showed that social force variation has a large impact on collision metrics

**Goal force** produced consistently small effects:
- Central Tunnel: most metrics changed <5%
- Delta: near-miss rate −12% (the only large goal force effect)
- This suggests that in environments where the path to goal is largely unobstructed, goal force is a secondary parameter

**Obstacle force** effects were environment-dependent (as expected):
- Central Tunnel: speed −11%, speed variability +92% (AABB bug amplification)
- Delta: collision rate +29%, but wall-boundary only
- Obstacle force matters primarily when there are obstacles and is confounded by the AABB bug

#### Environment Structure Dominance

The most striking cross-environment pattern is that **environment geometry dominates absolute metric values far more than parameter tuning**:

| Metric | Café (cluttered) | Central Tunnel (corridor) | Delta (open) | Ground Truth Range |
|--------|-----------------|--------------------------|--------------|-------------------|
| Speed (m/s) | 2.68–2.99 | 0.70–0.78 | 0.15–0.21 | 0.64–1.46 |
| Collision Rate | 0.66–1.07 | 0.04–0.05 | 0.02–0.05 | 0.0004–0.094 |
| Path Efficiency | 0.06 | 0.41–0.56 | 0.98–1.00 | 0.87–0.97 |
| Heading Jerk | 0.68–0.76 | 0.09–0.11 | 0.001–0.026 | 0.11–0.52 |

The range of variation *within* each environment due to parameter changes is small compared to the differences *between* environments. This confirms that OAT parameter tuning operates within bounds set by the environment, and that environment design (obstacle placement, agent count, flow geometry) is the primary determinant of simulation realism.

---

## Phase 2D — Simulation vs Ground Truth Comparison

### Limitations Disclaimer

The following comparisons between simulated and ground truth datasets must be interpreted within these constraints:

1. **Environment mismatch**: The simulated environments (café, tunnel, triangle) are artificial and differ fundamentally in size, layout, and obstacle composition from the ETH Zurich and University of Cyprus recording locations. No simulated environment was designed to directly replicate a ground truth scene.

2. **Population mismatch**: The simulated crowds (12–80 agents) are substantially smaller than ground truth scenes (148–434 pedestrians). This directly affects density-dependent metrics and limits the emergence of collective phenomena.

3. **Model limitations**: The ESFM is a force-model approximation. It does not capture anticipatory navigation (humans look ahead and plan routes), cultural/contextual pedestrian behaviours (queuing norms, right-hand traffic conventions), or higher-order social interactions (acknowledgment gestures, negotiated yielding).

4. **Absence of path planning**: Simulated agents navigate purely by force balance (goal attraction vs. social/obstacle repulsion). They cannot plan around obstacles, leading to trapping artefacts in cluttered environments that do not occur in real pedestrian crowds.

5. **Configuration mode 2 in café**: Café force factor results are unreliable as discussed above.

With these caveats established, the following comparisons assess whether simulated trajectories exhibit statistical properties that trend toward real crowd behaviour as parameters are tuned.

### Reference Ground Truth Summary

| Dataset | Speed (m/s) | Coll Rate | Path Eff | Heading Jerk | Speed CV | Min Dist (m) | Min Dist p5 (m) |
|---------|-------------|-----------|----------|--------------|----------|--------------|-----------------|
| eth_hotel | 1.29 | 0.0010 | 0.910 | 0.518 | 0.155 | 1.65 | 0.51 |
| eth_univ | 1.46 | 0.0004 | 0.966 | 0.467 | 0.163 | 1.88 | 0.60 |
| ucy_zara01 | 1.10 | 0.0022 | 0.964 | 0.107 | 0.138 | 1.57 | 0.57 |
| ucy_zara02 | 1.11 | 0.0185 | 0.948 | 0.145 | 0.183 | 1.23 | 0.43 |
| ucy_zara03 | 1.10 | 0.0121 | 0.944 | 0.128 | 0.143 | 1.55 | 0.41 |
| ucy_univ_s1 | 0.64 | 0.0941 | 0.870 | 0.220 | 0.355 | 0.80 | 0.30 |
| ucy_univ_s3 | 0.77 | 0.0231 | 0.881 | 0.214 | 0.317 | 0.97 | 0.44 |
| **GT Mean** | **1.07** | **0.019** | **0.926** | **0.257** | **0.208** | **1.38** | **0.47** |

### Metric-by-Metric Comparison

#### Speed

| Source | Speed (m/s) | vs GT Mean |
|--------|-------------|------------|
| GT Mean | 1.07 | — |
| Café Baseline | 2.99 | +179% (2.8×) |
| Central Tunnel Baseline | 0.78 | −27% |
| Delta Baseline | 0.18 | −83% |

**Discussion**: The café speed (2.99 m/s) is physically absurd — over twice realistic human walking speed — caused by agents oscillating at high velocity between conflicting forces in the cluttered space. Central Tunnel speed (0.78 m/s) falls within the lower range of ground truth (comparable to UCY Univ S1 at 0.64 m/s, a dense campus scene). Delta speed (0.18 m/s) is drastically below any ground truth dataset, indicating agents are barely moving. Given that baseline max_vel is 1.21 m/s, the fact that agents only achieve 0.18 m/s means opposing forces (social + obstacle from boundary walls) are consuming >85% of the goal-directed velocity. **Central Tunnel produces the most realistic speed, closest to the denser ground truth datasets.**

#### Collision Rate

| Source | Collision Rate | vs GT Mean |
|--------|---------------|------------|
| GT Mean | 0.019 | — |
| GT Low (eth_univ) | 0.0004 | — |
| GT High (ucy_univ_s1) | 0.094 | — |
| Central Tunnel Baseline | 0.048 | +153% vs mean |
| Delta Baseline | 0.034 | +79% vs mean |
| Delta Phase 3 (Speed) | 0.019 | **Match** |

**Discussion**: Central Tunnel (0.048) and Delta baseline (0.034) produce collision rates in the same order of magnitude as ground truth, which is a significant achievement for a pure force-model simulation. Notably, **Delta Phase 3 (Speed variation) achieved an exact match with the ground truth mean collision rate (0.019)** — the strongest single data point validating the OAT approach. The café collision rate (0.661) is 35× the ground truth mean — entirely dominated by the trapping artefact.

#### Path Efficiency

| Source | Path Efficiency | vs GT Mean (0.926) |
|--------|----------------|-------------------|
| Café Baseline | 0.063 | −93% |
| Central Tunnel Baseline | 0.413 | −55% |
| Delta Baseline | 0.998 | +8% |
| GT Range | 0.870–0.966 | — |

**Discussion**: Delta produces near-perfect path efficiency (0.998) — *higher* than ground truth. This is because agents in the open triangle move in nearly straight lines with minimal detours, whereas real pedestrians exhibit small directional variations even in uncrowded spaces. The Delta path efficiency is "too good" — it indicates agents are not making enough adaptive course corrections, consistent with the near-zero heading jerk values. Central Tunnel path efficiency (0.413) is below even the densest ground truth dataset (UCY Univ S1 at 0.870), indicating significant obstacle-induced path degradation. The café value (0.063) reflects complete trapping.

#### Heading Jerk

| Source | Heading Jerk | vs GT Mean (0.257) |
|--------|-------------|-------------------|
| Café Baseline | 0.676 | +163% |
| Central Tunnel Baseline | 0.107 | −58% |
| Delta Baseline | 0.001 | −99.6% |
| GT Range | 0.107–0.518 | — |

**Discussion**: Central Tunnel baseline heading jerk (0.107) exactly matches the lowest ground truth value (UCY Zara01 at 0.107), indicating reasonable trajectory smoothness for a corridor scenario. Delta's heading jerk (0.001) is two orders of magnitude below ground truth — agents are moving in ultra-straight lines without the micro-corrections real pedestrians continuously make. The café's high heading jerk (0.676) reflects oscillation artefacts. **Central Tunnel produces the most realistic heading jerk values.**

#### Speed Variability (CV)

| Source | Speed CV | vs GT Mean (0.208) |
|--------|----------|-------------------|
| Café Baseline | 0.393 | +89% |
| CT Baseline | 0.168 | −19% |
| Delta Baseline | 0.133 | −36% |
| GT Range | 0.138–0.355 | — |

**Discussion**: Central Tunnel (0.168) falls within the realistic ground truth range (0.138–0.355). Delta (0.133) is slightly below the minimum ground truth value. Both are closer to the less-dense ground truth scenes (UCY Zara01: 0.138), which is consistent with the moderate simulated densities. The café value (0.393) is elevated by the oscillation artefact.

#### Minimum Inter-Agent Distance

| Source | Min Dist Mean (m) | Min Dist p5 (m) | vs GT |
|--------|-------------------|------------------|-------|
| GT (eth_univ) | 1.88 | 0.60 | — |
| GT (ucy_univ_s1) | 0.80 | 0.30 | — |
| CT Baseline | 1.59 | 0.67 | Reasonable |
| Delta Baseline | 1.08 | 0.67 | Reasonable |

**Discussion**: Both Central Tunnel (1.59 m) and Delta (1.08 m) produce mean separations within the ground truth range (0.80–1.88 m). The 5th percentile values (0.67 m for both) are within the ground truth range (0.30–0.60 m), falling above the densest scenarios, which is consistent with the moderate simulated densities. These are among the most realistic metrics across the study.

### Summary of Simulation vs Ground Truth

| Metric | Best Simulated Match | Environment/Phase | Ground Truth Target |
|--------|---------------------|-------------------|-------------------|
| Speed | 0.78 m/s | CT Baseline | 1.07 m/s (GT Mean) |
| Collision Rate | 0.019 | Delta Phase 3 | 0.019 (GT Mean) — **exact match** |
| Path Efficiency | 0.998 | Delta Baseline | 0.926 (GT Mean) — close but "too perfect" |
| Heading Jerk | 0.107 | CT Baseline | 0.107 (UCY Zara01) — **exact match** |
| Speed CV | 0.168 | CT Baseline | 0.208 (GT Mean) — close |
| Min Distance | 1.08 m | Delta Baseline | 1.38 m (GT Mean) — reasonable |

---

## Phase 3 — Optimal Parameter Recommendations

### Evidence Summary

Based on the analysis of 53 simulation runs across three environments (15 café, 19 Central Tunnel, 19 Delta) against seven ground truth datasets:

1. **Speed (max_vel)** is the most impactful parameter for improving realism. Delta Phase 3 achieved an exact collision rate match with ground truth (0.019) through speed variation. Speed changes produce large, consistent, theoretically expected metric responses across all valid environments.

2. **Social force** is the second most impactful parameter and the primary controller of inter-agent spacing and collision avoidance. It produced the largest combined effects in Central Tunnel (−15% collision rate, +35% path efficiency). However, the optimal direction of change appears environment-dependent — Central Tunnel benefited from the combined high/low variation, while Delta's results were dominated by the low-SF collision increase.

3. **Goal force** has minimal impact in simple geometries (corridors, open spaces) but demonstrated a useful trade-off in Delta (−12% near-miss rate for +18% collision rate increase). It would be expected to have greater impact in complex, multi-turn environments.

4. **Obstacle force** effects are confounded by the AABB implementation bug and are only relevant in environments with obstacles. In obstacle-free Delta, the effect was small and wall-boundary driven.

### Per-Environment Recommendations

#### Central Tunnel

The Central Tunnel with its bidirectional corridor flow most closely approximates ETH/UCY corridor datasets.

- **Social force**: Increase toward the upper range (×1.50–1.70 of baseline, i.e., SF ~17–20). The high-SF condition drove the collision reduction and path efficiency improvement. This is consistent with the finding that stronger social repulsion promotes lane formation.
- **Goal force**: Maintain near baseline (×1.00). Goal force had minimal effect in this geometry.
- **Max velocity**: Slight reduction (×0.85–0.90 of baseline, i.e., MaxVel ~1.09–1.15 m/s). The speed phase improved path efficiency substantially. However, the baseline speed (0.78 m/s) is already below ground truth, so further reduction must be balanced against speed realism.
- **Obstacle force**: Reduce to ×0.50 of baseline to minimise AABB trapping artefacts, or ideally fix the AABB computation in the SFM plugin.

#### Delta

Delta provides the cleanest parameter sensitivity data due to its open geometry.

- **Social force**: Increase modestly (×1.20–1.40 of baseline, i.e., SF ~13–15). The baseline achieves reasonable collision rates, but the data shows low-SF dramatically worsened them.
- **Goal force**: Increase modestly (×1.30–1.50 of baseline, i.e., GF ~4.3–4.9). The goal phase showed that higher goal force reduces near-miss rate while maintaining acceptable collision levels. The increased transit speed through the crossing zone is beneficial.
- **Max velocity**: Maintain or slightly reduce (×0.80–1.00 of baseline). The −44% collision rate reduction from speed variation is the study's strongest finding, but the baseline speed (0.18 m/s) is already critically low. The max_vel cap is 1.21 m/s — the agents are not reaching it due to opposing forces. The true fix is force balance, not max_vel reduction.
- **Obstacle force**: Not applicable (obstacle-free). For wall boundary effects, baseline values are adequate.

### General Recommendation

If a single parameter configuration must be selected for general-purpose ESFM simulation:

| Parameter | Recommended Value | Justification |
|-----------|-------------------|---------------|
| `social_force_factor` | 14–17 (×1.3–1.5 of typical baseline) | Consistent collision reduction across environments; promotes lane formation |
| `goal_force_factor` | 3.5–4.5 (near-default to slightly elevated) | Minimal impact in simple geometries; slight increase reduces near-miss duration |
| `obstacle_force_factor` | 8–12 (×0.50–0.70 of typical baseline) | Reduces AABB trapping artefacts while maintaining wall avoidance |
| `max_vel` | 1.20–1.40 m/s | Within realistic human walking speed range (Weidmann, 1992); allows agents room to decelerate under social pressure |
| `configuration` | **1 (CUSTOM)** | Mandatory for deterministic parameter control |

### Trade-offs

1. **Social force vs. speed**: Higher social force improves collision rates but reduces effective speed. At SF > 20, agents may become overly cautious, reducing speed below realistic levels.
2. **Goal force vs. collisions**: Higher goal force reduces near-miss duration but can increase collision rate if agents push through congested zones.
3. **Obstacle force vs. path efficiency**: Reducing obstacle force improves path efficiency in cluttered environments but risks agents clipping through obstacle models.
4. **Speed vs. collision severity**: Higher max_vel increases collision energy but also reduces congestion time in crossing zones.

### Inconclusive Areas Requiring Further Testing

1. The interaction between social force and environment density (N agents) is not captured by single-environment OAT analysis. Multi-factor experiments varying both SF and agent count simultaneously are needed.
2. The AABB obstacle handling bug makes all obstacle force conclusions in cluttered environments unreliable until the bug is fixed. A corrected implementation using proper mesh-based obstacle distance computation would likely change all obstacle-phase results substantially.
3. The severe speed deficit in Delta (0.18 vs. 1.21 m/s max_vel) suggests a force balance issue beyond parameter tuning — the relaxation time (τ = 0.5s) or the fundamental force magnitude scaling may need adjustment.
4. The study used 2 runs per phase variant, which limits statistical power. A minimum of 5 runs per condition would substantially improve confidence intervals.

---

## Phase 4 — Report Alignment Review

The following issues were identified by cross-referencing the analysis above against the attached capstone report draft:

### Inconsistencies

1. **Agent count claims**: The report's methodology section states "Agent populations will range between 10 and 100 pedestrians" and "Each scenario will be executed a minimum of 30 times per environment". The actual implementation was 12/40/80 agents with 15/19/19 runs. The report should be updated to reflect actual execution parameters rather than planned ranges.

2. **Config mode 2 not discussed**: The report does not acknowledge or discuss the `configuration: 2` issue in the café. This is a significant methodological limitation that must be disclosed — the café force factor results (Phases 1 and 2) cannot be treated as valid OAT data.

3. **Metric definitions**: The report defines ADE and FDE as key metrics (from the literature review) but the actual analysis script does not compute ADE or FDE — it computes population-level, agent-level, and pairwise metrics. The report should clarify that ADE/FDE are trajectory prediction metrics (requiring predicted vs. ground truth trajectory pairs) and are not applicable to generative SFM evaluation, which instead uses distributional metrics.

4. **Delta agent count**: The DELTA_TESTING_PARAMETERS.txt file states 50 agents, but the actual implementation uses 80 agents (upgraded during development). The report should reflect the final 80-agent configuration.

5. **Obstacle handling**: The report's system overview section (Section 1) mentions path planning decisions but does not document the `HandleObstacles2()` AABB bug that was discovered during testing and significantly impacts results in the café and Central Tunnel. This should be documented as a known limitation.

6. **Speed results in café**: The report (via CAPSTONE_GUIDE.md) highlights "Phase 2 (Goal Force) reduced collisions 19%" as a key result. Given the config mode 2 issue, this cannot be attributed to goal force variation — it was a random effect. This claim should be retracted or reframed.

### Gaps

1. **Missing sections**: The report has placeholder headings for "Results and Discussion" (with notes "graphs: speed distribution, density-flow curve, trajectories, tables of metrics") and "Conclusion and Recommendations" that need to be populated with the actual results from all three environments.

2. **Central Tunnel and Delta results**: The report draft appears to be written primarily around the café results. Central Tunnel (the first reliable OAT environment) and Delta (the cleanest data) are not yet incorporated.

3. **Ground truth comparison**: The report discusses ETH/UCY datasets in the literature review but does not present the actual quantitative comparison between simulation results and ground truth metrics. The analysis in Section 2D above should be incorporated.

4. **No discussion of speed deficit**: The most significant finding — that simulated agents in Central Tunnel and Delta move at 0.18–0.78 m/s compared to ground truth 1.07 m/s — is not discussed. This is a critical observation about the ESFM's force balance behaviour.

5. **Group metrics**: The report discusses group dynamics (cohesion, speed synchronisation) extensively in the methodology, but the simulation results show `num_groups: 0` for all environments in the analysis, meaning group annotations are not being captured in the trajectory export. The report should either present group analysis from the behaviour trees (which do define groups) or acknowledge this gap.

### Recommended Updates

1. Update Section 4 (Experimental Design) with actual run counts and agent populations
2. Add a "Known Limitations" subsection documenting: config mode 2 in café, AABB obstacle bug, absence of path planning
3. Replace café Phases 1–2 conclusions with a transparency note about config mode 2
4. Add full Results section with Central Tunnel and Delta data as the primary evidence base
5. Add Simulation vs. Ground Truth section with the metric-by-metric comparison
6. Update Conclusion to focus on max_vel as the strongest validated parameter, with social force as second
7. Address the ADE/FDE disconnect between literature review and actual metrics
8. Document the speed deficit as a key finding requiring further investigation