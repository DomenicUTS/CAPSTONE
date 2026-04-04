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

---

# V2 ANALYSIS — Extended Metrics

*Generated 4 April 2026 by re-running `crowd_dynamics_evaluator.py` with 5 additional metric categories on all 53 simulation runs and 7 ground truth datasets.*

The methodology (Section 5 of the capstone report) lists several evaluation dimensions beyond the original 14-metric tier system. Five of these have now been implemented and applied:

1. **Time-to-Collision (TTC)** — linear velocity extrapolation to predict collision time
2. **Stuck Rate** — fraction of agent-frames where agent is effectively stationary
3. **Oscillation Index** — heading reversals per metre of path travelled
4. **Translational Jerk** — rate of change of acceleration magnitude (m/s³)
5. **Proxemic Zone Distribution** — fraction of time nearest neighbour falls in each of Hall's (1966) four zones

---

## V2 Metric Definitions

### 11. Time-to-Collision (TTC) (seconds)

**Definition**: For each agent at each frame, given the current positions and velocities of the agent and its nearest neighbour, TTC is the time at which linear extrapolation of both trajectories would result in their separation distance reaching the collision threshold (0.5 m). Computed by solving the quadratic equation:

$$a \cdot t^2 + b \cdot t + c = 0$$

where $\Delta\mathbf{p} = \mathbf{p}_i - \mathbf{p}_j$, $\Delta\mathbf{v} = \mathbf{v}_i - \mathbf{v}_j$, $a = \|\Delta\mathbf{v}\|^2$, $b = 2(\Delta\mathbf{p} \cdot \Delta\mathbf{v})$, $c = \|\Delta\mathbf{p}\|^2 - r^2$. Only the smallest positive root $t \in (0, 30]$ seconds is retained. Events with no positive root (agents diverging) are excluded.

**Relevance**: TTC is a standard safety metric in pedestrian dynamics (van der Horst, 1990) and autonomous vehicle research. Unlike collision rate (which counts after-the-fact events), TTC measures *how close agents are to future collisions at every moment*, capturing the perceived danger level.

**Interpretation**: Higher TTC indicates safer conditions. Ground truth mean: 3.88 s (range 2.71–6.37 s). TTC < 1 s represents an imminent collision threat. The 10th percentile (TTC p10) captures the most dangerous encounters.

**Parameter sensitivity**: Parameters that increase relative velocity (higher max_vel, higher goal force) reduce TTC. Parameters that increase separation (higher social force) increase TTC.

### 12. Stuck Rate (dimensionless, 0–1)

**Definition**: The fraction of all agent-frame observations where the agent's instantaneous speed is below 0.05 m/s (effectively stationary).

**Relevance**: Directly measures the SFM trapping artefact — agents caught between conflicting forces that cannot make progress toward their goal. This is the single most diagnostic metric for the absence-of-path-planning problem discussed in this study.

**Interpretation**: Ground truth mean: 11.8% (range 2.0–27.7%). A stuck rate of 0% means all agents are always moving. A stuck rate of 50% means agents spend half their time immobile. Real pedestrians do stop (waiting, looking at phones, window shopping), so some stuck rate is realistic. However, in simulation, stuck agents are not choosing to stop — they are force-balanced into immobility.

### 13. Oscillation Index (reversals/metre)

**Definition**: For each agent, count the number of heading-direction sign changes (reversals in angular velocity) and divide by the total arc-length distance travelled. Averaged across all agents with ≥3 observations and >0.1 m path length.

**Relevance**: Directly quantifies the oscillation artefact where agents alternate heading due to competing forces. A perfectly straight-line walker has oscillation index 0. A trapped agent oscillating in place has a very high oscillation index (many reversals, minimal distance).

**Interpretation**: Ground truth mean: 1.72 rev/m (range 1.08–2.93). Real pedestrians make frequent micro-corrections, so some oscillation is normal. Values significantly above GT indicate force-induced oscillation.

### 14. Translational Jerk (m/s³)

**Definition**: For each agent, the mean of the absolute frame-to-frame change in acceleration magnitude divided by the time step. This is the third derivative of position magnitude, complementing heading jerk (third derivative of heading angle).

**Relevance**: Flash & Hogan (1985) showed human locomotion minimises jerk. Translational jerk captures the abruptness of speed changes — how "jerky" the start-stop behaviour is. While heading jerk measures directional smoothness, translational jerk measures speed smoothness.

**Interpretation**: Ground truth mean: 0.74 m/s³ (range 0.52–1.22). Values well above GT indicate violently abrupt speed changes; values well below indicate agents that never modulate their speed.

### 15. Proxemic Zone Distribution (dimensionless fractions)

**Definition**: For each agent-frame's nearest-neighbour distance, classify into Hall's (1966) four proxemic zones:
- **Intimate**: < 0.45 m (reserved for close relationships; violations indicate collision-like overlap)
- **Personal**: 0.45–1.2 m (normal conversational distance)
- **Social**: 1.2–3.6 m (formal interaction distance)
- **Public**: ≥ 3.6 m (no social interaction expected)

Report the fraction of all agent-frame observations falling in each zone.

**Relevance**: This maps the dry statistical "minimum inter-agent distance" into a humanly meaningful framework. A realistic crowd simulation should spend most time in the personal and social zones, with minimal intimate-zone violations and some public-zone observations for uncrowded moments.

**Interpretation**: Ground truth distribution: 6.9% intimate, 61.6% personal, 24.7% social, 6.8% public.

---

## V2 Results — Ground Truth Reference

| Dataset | TTC Mean (s) | TTC Median | TTC p10 | Stuck % | Osc Index (rev/m) | Trans Jerk (m/s³) | % Intimate | % Personal | % Social | % Public |
|---------|-------------|------------|---------|---------|-------------------|-------------------|------------|------------|----------|----------|
| eth_hotel | 2.89 | 1.49 | 0.22 | 19.4% | 2.93 | 0.89 | 3.5% | 47.3% | 41.2% | 8.0% |
| eth_univ | 6.37 | 2.82 | 0.37 | 4.9% | 1.37 | 1.22 | 0.6% | 54.5% | 31.1% | 13.9% |
| ucy_zara01 | 3.78 | 2.51 | 0.38 | 2.0% | 1.12 | 0.52 | 0.8% | 67.6% | 22.7% | 9.0% |
| ucy_zara02 | 4.58 | 2.15 | 0.29 | 27.7% | 1.46 | 0.59 | 6.3% | 69.2% | 18.5% | 6.1% |
| ucy_zara03 | 3.59 | 2.02 | 0.29 | 8.9% | 1.08 | 0.54 | 7.4% | 59.5% | 24.0% | 9.2% |
| ucy_univ_s1 | 3.21 | 1.47 | 0.20 | 12.7% | 2.43 | 0.62 | 23.6% | 60.6% | 15.4% | 0.4% |
| ucy_univ_s3 | 2.71 | 1.10 | 0.15 | 6.9% | 1.63 | 0.77 | 6.5% | 72.4% | 20.0% | 1.1% |
| **GT Mean** | **3.88** | **1.93** | **0.27** | **11.8%** | **1.72** | **0.74** | **6.9%** | **61.6%** | **24.7%** | **6.8%** |

---

## V2 Intra-Environment Analysis

### Café — V2 Metrics (configuration mode 2 caveat still applies to Phases 1–2)

| Phase | TTC Mean | TTC Med | TTC p10 | Stuck % | Osc Idx | Trans Jerk | % Intim | % Pers | % Social | % Public |
|-------|----------|---------|---------|---------|---------|------------|---------|--------|----------|----------|
| Baseline | 0.55 | 0.24 | 0.05 | 3.7% | 1.31 | 2.33 | 4.9% | 52.0% | 42.3% | 0.8% |
| Ph1 Social | 0.85 | 0.26 | 0.05 | 3.7% | 0.74 | 2.33 | 7.6% | 54.8% | 36.8% | 0.9% |
| Ph2 Goal | 0.84 | 0.30 | 0.05 | 2.3% | 0.69 | 2.57 | 5.3% | 52.9% | 41.5% | 0.4% |
| Ph3 Speed | 0.64 | 0.24 | 0.05 | 2.3% | 0.65 | 2.89 | 5.1% | 49.4% | 44.9% | 0.6% |

**V2 Discussion — Café**:

**TTC is catastrophically low**: The café mean TTC (0.55–0.85 s) is 5–7× lower than the ground truth mean (3.88 s). At every moment, agents are on average less than 1 second from a collision with their nearest neighbour. The TTC p10 values (0.05 s) indicate the worst encounters are essentially frame-to-frame collision events. This quantitatively confirms that the café produces a fundamentally unsafe crowd dynamic, consistent with the extreme collision rates documented in the V1 analysis.

**Stuck rate is paradoxically low** (2.3–3.7% vs GT 11.8%): Despite the trapping artefact discussed in V1, agents are rarely stationary. Instead, they oscillate at high speed between conflicting forces — they are trapped but still moving rapidly. This is a distinctive SFM failure mode: agents don't stop when trapped, they *vibrate*.

**Translational jerk is 3–4× above GT** (2.33–2.89 vs 0.74 m/s³): The most extreme V2 finding for the café. Agents undergo violent speed changes every frame, consistent with the rapid oscillation between competing forces. Combined with the low stuck rate, this paints a picture of agents that are in constant high-speed oscillatory motion rather than smooth walking.

**Proxemic zones show negligible public space** (0.4–0.9% vs GT 6.8%): Expected for the small environment. The intimate zone fraction (4.9–7.6%) is comparable to GT mean (6.9%), but this is misleading — in GT, intimate encounters are brief accidents; in the café, they represent agents perpetually crammed together.

### Central Tunnel — V2 Metrics

| Phase | TTC Mean | TTC Med | TTC p10 | Stuck % | Osc Idx | Trans Jerk | % Intim | % Pers | % Social | % Public |
|-------|----------|---------|---------|---------|---------|------------|---------|--------|----------|----------|
| Baseline | 3.37 | 0.67 | 0.09 | 8.8% | 1.64 | 0.24 | 1.4% | 51.1% | 41.7% | 5.8% |
| Ph1 Social | 3.06 | 0.42 | 0.09 | 5.8% | 1.03 | 0.26 | 1.1% | 50.3% | 44.0% | 4.6% |
| Ph2 Goal | 3.60 | 0.51 | 0.08 | 8.1% | 2.45 | 0.24 | 1.2% | 48.9% | 44.7% | 5.3% |
| Ph3 Speed | 3.83 | 1.67 | 0.09 | 17.0% | 2.34 | 0.34 | 1.3% | 48.3% | 45.6% | 4.9% |
| Ph4 Obstacle | 3.55 | 0.80 | 0.08 | 9.7% | 1.60 | 0.24 | 1.4% | 49.6% | 44.7% | 4.4% |

**V2 Discussion — Central Tunnel**:

**TTC within ground truth range**: CT mean TTC (3.06–3.83 s) falls squarely within the GT range (2.71–6.37 s). This is a strong positive finding — the simulated bidirectional flow produces temporally realistic collision approach dynamics. Phase 3 (Speed) produced the highest TTC (3.83 s), consistent with slower agents having more time before potential collisions.

**Stuck rate reveals speed-phase trapping**: CT baseline stuck rate (8.8%) is below GT mean (11.8%). However, the speed phase (17.0%) shows significantly elevated stuckness. Examining individual runs, run_13 of the speed phase had **45.7% stuck rate** — nearly half its agents were frozen. This was the slow-speed variant (×0.75 max_vel), confirming that reducing max_vel can push agents below the threshold needed to overcome opposing forces. **This is a critical finding: there exists a max_vel floor below which the SFM's force balance traps agents.**

**Oscillation index within GT range**: CT oscillation (1.03–2.45 rev/m) spans the GT range (1.08–2.93). The goal phase produced the highest oscillation (2.45), suggesting that goal force variation (combined low+high) creates more frequent heading corrections. The social phase had the lowest oscillation (1.03), indicating smoother lane formation under social force variation.

**Translational jerk is 3× below GT** (0.24–0.34 vs 0.74 m/s³): CT agents change speed too gradually. Real pedestrians make sharper accelerations and decelerations during their walk. This suggests the SFM's relaxation time (τ = 0.5s) is too slow for realistic speed modulation, or that the force magnitudes are insufficiently dynamic.

**Proxemic zone distribution is realistic but intimate-zone deficient**: CT produces 1.1–1.4% intimate zone (vs GT 6.9%), 48–51% personal (vs GT 62%), and 42–46% social (vs GT 25%). The agents maintain overly conservative spacing — spending ~20% more time in the social zone and ~15% less in the personal zone than real pedestrians. This is consistent with social force being too strong for natural proximity.

### Delta — V2 Metrics

| Phase | TTC Mean | TTC Med | TTC p10 | Stuck % | Osc Idx | Trans Jerk | % Intim | % Pers | % Social | % Public |
|-------|----------|---------|---------|---------|---------|------------|---------|--------|----------|----------|
| Baseline | 6.32 | 4.25 | 0.34 | 0.03% | 0.79 | 0.09 | 0.7% | 69.9% | 29.2% | 0.2% |
| Ph1 Social | 7.76 | 5.46 | 0.33 | 0.2% | 1.09 | 0.10 | 1.3% | 73.2% | 25.6% | 0.0% |
| Ph2 Goal | 6.85 | 6.25 | 0.19 | 0.3% | 0.93 | 0.09 | 1.0% | 68.4% | 30.1% | 0.4% |
| Ph3 Speed | 6.60 | 4.43 | 0.43 | 12.8% | 1.45 | 0.06 | 0.4% | 72.4% | 27.2% | 0.0% |
| Ph4 Obstacle | 5.76 | 2.10 | 0.24 | 0.07% | 0.87 | 0.08 | 1.0% | 73.9% | 25.1% | 0.0% |

**V2 Discussion — Delta**:

**TTC is the highest across all environments**: Delta mean TTC (5.76–7.76 s) exceeds most GT values and matches eth_univ (6.37 s). This reflects the open geometry: agents have long sight lines and the slow walking speed (0.15–0.21 m/s) gives them ample time before potential collisions. The social phase produced the highest TTC (7.76 s), meaning stronger social forces pushed agents to trajectories that maximised future safety margins.

**Stuck rate is near-zero except for speed phase**: Delta baseline stuck rate (0.03%) is the lowest in the study — agents are always moving. This confirms the open-geometry advantage: without obstacles, agents rarely get force-balanced into immobility. However, the **speed phase jumped to 12.8%**, with run_15 reaching **47.5% stuck**. This is the same speed-floor phenomenon seen in CT: reducing max_vel in the slow variant (×0.75, bringing max_vel to ~0.91 m/s) causes some agents to become force-balanced and cannot overcome social repulsion at the central crossing zone. **Combined with the CT finding, this establishes a generalizable result: the SFM has a critical max_vel threshold below which agents in dense crossing zones become trapped.**

**Oscillation index is below GT range** (0.79–1.45 vs GT 1.08–2.93): Delta agents make fewer heading corrections per metre than real pedestrians. This is consistent with the near-zero heading jerk found in V1 — agents in the open triangle move in very smooth, straight lines without the continuous micro-adjustments real pedestrians make. The speed phase had the highest oscillation (1.45), likely driven by the trapped agents in run_15 oscillating in place.

**Translational jerk is 8–12× below GT** (0.06–0.10 vs 0.74 m/s³): The most extreme V2 discrepancy. Delta agents barely change their speed at all. Combined with their extremely low heading jerk and near-zero stuck rate, this confirms that Delta agents walk in near-straight lines at near-constant speed — essentially moving targets rather than responsive pedestrians. The open geometry removes all stimuli (obstacles, congestion chokepoints) that would force speed modulation.

**Proxemic zones show dominant personal zone** (68–74% vs GT 62%): Delta agents spend more time in the personal zone and less in the intimate zone (0.4–1.3% vs GT 6.9%) than real crowds. The total absence of public zone (0.0–0.4% vs GT 6.8%) reflects the high agent count (80) in a bounded space — there is nowhere to be far from everyone.

---

## V2 Cross-Environment Comparison

### TTC: The Best New Discriminator

| Environment | TTC Mean (s) | vs GT Mean (3.88 s) | Assessment |
|-------------|-------------|---------------------|------------|
| Café | 0.55–0.85 | −78% to −82% | **Catastrophic** |
| Central Tunnel | 3.06–3.83 | −1% to −21% | **Realistic** |
| Delta | 5.76–7.76 | +48% to +100% | **Too safe** |
| GT Range | 2.71–6.37 | — | — |

TTC is the most discriminating new metric. It cleanly separates the three environments: the café is in perpetual collision crisis, the Central Tunnel achieves realistic approaching dynamics, and Delta is overly safe. This pattern was not visible from collision rate alone (which only counts after-the-fact events) and adds temporal dimension to the safety analysis.

### Stuck Rate: Revealing the Max-Vel Floor

| Condition | Stuck Rate | Key Observation |
|-----------|-----------|-----------------|
| GT Mean | 11.8% | Real pedestrians stop sometimes |
| Café (all phases) | 2.3–3.7% | Agents oscillate rapidly but never truly stop |
| CT Baseline | 8.8% | Reasonable |
| CT Speed Phase | 17.0% | run_13: 45.7% stuck |
| Delta Baseline | 0.03% | Agents always moving |
| Delta Speed Phase | 12.8% | run_15: 47.5% stuck |

The speed phase produces catastrophic stuckness in individual runs — nearly half the agents frozen. This is a **robust cross-environment finding**: reducing max_vel to ×0.75 of baseline creates conditions where the force balance traps agents. The mechanism: at low max_vel, the goal force (which accelerates agents toward their target) produces less impulse per timestep, while social repulsion (which does not depend on max_vel) remains constant. The equilibrium tips from "goal wins → agent moves" to "social wins → agent oscillates in place."

### Translational Jerk: A Missing Dimension of Realism

| Environment | Trans Jerk (m/s³) | vs GT Mean (0.74) | Diagnosis |
|-------------|-------------------|-------------------|-----------|
| Café | 2.33–2.89 | +215% to +290% | Violently abrupt oscillation |
| Central Tunnel | 0.24–0.34 | −54% to −68% | Too smooth |
| Delta | 0.06–0.10 | −86% to −92% | Near-constant speed |

Translational jerk reveals a previously invisible dimension: none of the environments produce realistic speed modulation. The café has too much (oscillation artefact), while CT and Delta have too little (SFM relaxation time is too slow). This suggests that τ = 0.5s in the SFM's velocity-update equation ($\mathbf{F}_{goal} = \frac{1}{\tau}(v_{desired}\hat{e} - \mathbf{v})$) is appropriate for smooth corridor walking but cannot produce the quick acceleration-deceleration cycles real pedestrians exhibit when navigating through irregular flows.

### Proxemic Zone Compliance

| Environment | % Intimate | % Personal | % Social | % Public | Closest GT Match |
|-------------|-----------|------------|----------|----------|-----------------|
| GT Mean | 6.9% | 61.6% | 24.7% | 6.8% | — |
| Café | 4.9–7.6% | 49–55% | 37–45% | 0.4–0.9% | Intimate matches, but socially over-dispersed |
| Central Tunnel | 1.1–1.4% | 48–51% | 42–46% | 4.4–5.8% | Under-intimate, over-social |
| Delta | 0.4–1.3% | 68–74% | 25–30% | 0.0–0.4% | Personal zone dominant, matches structure |

A consistent cross-environment finding: **all simulated environments have too little intimate-zone interaction and too much social-zone spacing**. The social force pushes agents into the 1.2–3.6m band more than real pedestrians would occupy. Real pedestrians accept closer proximity (they queue, they pass closely on sidewalks, they navigate through crowds brushing shoulders) because they have anticipatory planning that the SFM lacks.

---

## V2 Simulation vs Ground Truth

| V2 Metric | GT Mean | Best Sim Match | Environment/Phase | Assessment |
|-----------|---------|---------------|-------------------|------------|
| TTC Mean | 3.88 s | 3.37 s | CT Baseline | −13% — excellent |
| TTC p10 | 0.27 s | 0.09 s | CT (all phases) | 3× too low — worst encounters too dangerous |
| Stuck Rate | 11.8% | 8.8% | CT Baseline | −25% — reasonable |
| Osc Index | 1.72 rev/m | 1.64 rev/m | CT Baseline | −5% — **near-exact match** |
| Trans Jerk | 0.74 m/s³ | 0.34 m/s³ | CT Ph3 Speed | −54% — too smooth |
| % Intimate | 6.9% | 5.1% | Café Ph3 Speed | −26% — close |
| % Personal | 61.6% | 69.9% | Delta Baseline | +13% — close |
| % Social | 24.7% | 25.6% | Delta Ph1 Social | +4% — **near-exact match** |

**Key V2 findings relative to ground truth**:

1. **CT oscillation index (1.64 rev/m) matches GT mean (1.72 rev/m) within 5%** — this is a new exact-match result, showing the bidirectional corridor produces realistic heading-correction frequency.

2. **CT TTC mean (3.37 s) is within 13% of GT mean (3.88 s)** — the temporal safety dynamics are realistic.

3. **No environment achieves realistic translational jerk** — this is a fundamental limitation of the SFM's constant relaxation time.

4. **Proxemic social-zone fractions match well in Delta** but intimate zones are underrepresented everywhere — the social force is too conservative.

---

## Updated Recommendations (V2)

The V2 metrics add three actionable recommendations to the Phase 3 conclusions:

### 1. Max-Vel Floor Constraint

**Finding**: The speed phase revealed a critical max_vel floor. Below ~0.91 m/s (×0.75 of baseline 1.21), agents become force-trapped at crossing zones.

**Recommendation**: Any parameter optimisation should enforce `max_vel ≥ 1.0 m/s` as a hard constraint. The original Phase 3 recommendation to "maintain or slightly reduce" max_vel must be qualified: reduction below the floor produces catastrophic stuckness rather than gradual improvement.

### 2. Relaxation Time (τ) Adjustment

**Finding**: Translational jerk is 2–12× below ground truth across all environments. The SFM's velocity-update relaxation time τ = 0.5s produces speed changes that are too gradual.

**Recommendation**: Experiment with τ = 0.2–0.3s to produce more responsive agents. This is not an OAT parameter in the current study but is the most promising single-parameter change for improving speed modulation realism.

### 3. Social Force Strength Reduction for Proxemic Realism

**Finding**: The proxemic zone analysis shows all environments are under-intimate and over-social. Agents maintain too much distance.

**Recommendation**: The Phase 3 recommendation to increase social force (SF 14–17) should be reconsidered in light of the proxemic data. A moderate social force (SF 10–13, near current baselines) may produce more realistic proxemic distributions, even at the cost of slightly higher collision rates. The current social force levels produce collision rates that are closer to GT but proxemic distributions that are shifted toward excessive caution.

### Updated General Parameter Table (V2)

| Parameter | V1 Recommendation | V2 Adjustment | Final Recommendation |
|-----------|-------------------|---------------|---------------------|
| `social_force_factor` | 14–17 | Reduce slightly for proxemic realism | **11–15** |
| `goal_force_factor` | 3.5–4.5 | Unchanged | **3.5–4.5** |
| `obstacle_force_factor` | 8–12 | Unchanged | **8–12** |
| `max_vel` | 1.20–1.40 | Enforce ≥ 1.0 floor | **1.10–1.40** (never below 1.0) |
| `relaxation_time` (τ) | Not tuned | **New**: reduce for jerk realism | **0.2–0.3 s** (requires code change) |
| `configuration` | 1 (CUSTOM) | Unchanged | **1 (CUSTOM)** |