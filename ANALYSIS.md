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

**Definition**: For each agent at each frame, the Euclidean distance to the nearest other agent. Reported as mean, standard deviation, 5th percentile, and 25th percentile across all agent-frame pairs.

**Relevance**: Hall (1966) identified four proxemic zones: intimate (<0.45 m), personal (0.45–1.2 m), social (1.2–3.6 m), public (>3.6 m). A realistic simulation should produce a distribution where the 5th percentile rarely enters the intimate zone. Ground truth mean: 0.80 m (UCY Univ S1) to 1.88 m (ETH Univ). Ground truth 5th percentile: 0.30 m (UCY Univ S1) to 0.60 m (ETH Univ).

**Parameter sensitivity**:
- `social_force_factor`: Direct controller of inter-agent spacing — higher SF → larger separations
- Environment density: More agents in a smaller space mechanically reduce achievable distances

#### 11. Time-to-Collision (TTC) (seconds)

**Definition**: For each agent at each frame, given the current positions and velocities of the agent and its nearest neighbour, TTC is the time at which linear extrapolation of both trajectories would result in their separation distance reaching the collision threshold (0.5 m). Computed by solving the quadratic equation:

$$a \cdot t^2 + b \cdot t + c = 0$$

where $\Delta\mathbf{p} = \mathbf{p}_i - \mathbf{p}_j$, $\Delta\mathbf{v} = \mathbf{v}_i - \mathbf{v}_j$, $a = \|\Delta\mathbf{v}\|^2$, $b = 2(\Delta\mathbf{p} \cdot \Delta\mathbf{v})$, $c = \|\Delta\mathbf{p}\|^2 - r^2$. Only the smallest positive root $t \in (0, 30]$ seconds is retained. Events with no positive root (agents diverging) are excluded.

**Relevance**: TTC is a standard safety metric in pedestrian dynamics (van der Horst, 1990) and autonomous vehicle research. Unlike collision rate (which counts after-the-fact events), TTC measures *how close agents are to future collisions at every moment*, capturing the perceived danger level.

**Interpretation**: Higher TTC indicates safer conditions. Ground truth mean: 3.88 s (range 2.71–6.37 s). TTC < 1 s represents an imminent collision threat. The 10th percentile (TTC p10) captures the most dangerous encounters.

**Parameter sensitivity**: Parameters that increase relative velocity (higher max_vel, higher goal force) reduce TTC. Parameters that increase separation (higher social force) increase TTC.

#### 12. Stuck Rate (dimensionless, 0–1)

**Definition**: The fraction of all agent-frame observations where the agent's instantaneous speed is below 0.05 m/s (effectively stationary).

**Relevance**: Directly measures the SFM trapping artefact — agents caught between conflicting forces that cannot make progress toward their goal. This is the single most diagnostic metric for the absence-of-path-planning problem discussed in this study.

**Interpretation**: Ground truth mean: 11.8% (range 2.0–27.7%). A stuck rate of 0% means all agents are always moving. A stuck rate of 50% means agents spend half their time immobile. Real pedestrians do stop (waiting, looking at phones, window shopping), so some stuck rate is realistic. However, in simulation, stuck agents are not choosing to stop — they are force-balanced into immobility.

#### 13. Oscillation Index (reversals/metre)

**Definition**: For each agent, count the number of heading-direction sign changes (reversals in angular velocity) and divide by the total arc-length distance travelled. Averaged across all agents with ≥3 observations and >0.1 m path length.

**Relevance**: Directly quantifies the oscillation artefact where agents alternate heading due to competing forces. A perfectly straight-line walker has oscillation index 0. A trapped agent oscillating in place has a very high oscillation index (many reversals, minimal distance).

**Interpretation**: Ground truth mean: 1.72 rev/m (range 1.08–2.93). Real pedestrians make frequent micro-corrections, so some oscillation is normal. Values significantly above GT indicate force-induced oscillation.

#### 14. Translational Jerk (m/s³)

**Definition**: For each agent, the mean of the absolute frame-to-frame change in acceleration magnitude divided by the time step. This is the third derivative of position magnitude, complementing heading jerk (third derivative of heading angle).

**Relevance**: Flash & Hogan (1985) showed human locomotion minimises jerk. Translational jerk captures the abruptness of speed changes — how "jerky" the start-stop behaviour is. While heading jerk measures directional smoothness, translational jerk measures speed smoothness.

**Interpretation**: Ground truth mean: 0.74 m/s³ (range 0.52–1.22). Values well above GT indicate violently abrupt speed changes; values well below indicate agents that never modulate their speed.

#### 15. Proxemic Zone Distribution (dimensionless fractions)

**Definition**: For each agent-frame's nearest-neighbour distance, classify into Hall's (1966) four proxemic zones:
- **Intimate**: < 0.45 m (reserved for close relationships; violations indicate collision-like overlap)
- **Personal**: 0.45–1.2 m (normal conversational distance)
- **Social**: 1.2–3.6 m (formal interaction distance)
- **Public**: ≥ 3.6 m (no social interaction expected)

Report the fraction of all agent-frame observations falling in each zone.

**Relevance**: This maps the dry statistical "minimum inter-agent distance" into a humanly meaningful framework. A realistic crowd simulation should spend most time in the personal and social zones, with minimal intimate-zone violations and some public-zone observations for uncrowded moments.

**Interpretation**: Ground truth distribution: 6.9% intimate, 61.6% personal, 24.7% social, 6.8% public.

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

| Phase | Speed μ | Speed σ | Coll Rate | NM Rate | Path Eff | Hdg Jerk | Spd CV | Accel | Min Dist μ | Min Dist p5 | TTC μ | TTC p10 | Stuck% | Osc Idx | Trans Jerk | %Intim | %Pers | %Social | %Public |
|-------|---------|---------|-----------|---------|----------|----------|--------|-------|------------|-------------|-------|---------|--------|---------|------------|--------|-------|---------|---------|
| Baseline | 2.986 | 1.596 | 0.661 | 4.202 | 0.063 | 0.676 | 0.393 | 6.899 | 1.241 | 0.451 | 0.55 | 0.05 | 3.7% | 1.31 | 2.33 | 4.9% | 52.0% | 42.3% | 0.8% |
| Ph1 Social | 2.913 | 1.677 | 1.068 | 5.040 | 0.059 | 0.689 | 0.540 | 6.228 | 1.157 | 0.387 | 0.85 | 0.05 | 3.7% | 0.74 | 2.33 | 7.6% | 54.8% | 36.8% | 0.9% |
| Ph2 Goal | 2.816 | 1.584 | 0.721 | 4.266 | 0.056 | 0.736 | 0.517 | 6.932 | 1.186 | 0.441 | 0.84 | 0.05 | 2.3% | 0.69 | 2.57 | 5.3% | 52.9% | 41.5% | 0.4% |
| Ph3 Speed | 2.679 | 1.552 | 0.698 | 4.019 | 0.059 | 0.763 | 0.503 | 7.456 | 1.238 | 0.464 | 0.64 | 0.05 | 2.3% | 0.65 | 2.89 | 5.1% | 49.4% | 44.9% | 0.6% |

**Phase-by-phase discussion** (noting config mode 2 unreliability for Phases 1–2):

**Phase 1 — Social Force** (unreliable due to config mode 2):
- Collision rate increased 62% from baseline (0.661 → 1.068). Near-miss rate rose from 4.20 to 5.04. TTC increased slightly (0.55 → 0.85 s) but remains catastrophically below GT (3.88 s).
- Min inter-agent distance dropped (1.241 → 1.157 m; p5: 0.451 → 0.387 m). Intimate zone fraction rose from 4.9% to 7.6%.
- Speed variability increased substantially (0.393 → 0.540), suggesting more erratic start-stop behaviour. Oscillation index dropped (1.31 → 0.74 rev/m).
- *Interpretation*: Since force factors were randomised, this variation likely reflects random noise rather than systematic parameter effects.

**Phase 2 — Goal Force** (unreliable due to config mode 2):
- Collision rate slightly increased (0.661 → 0.721). Heading jerk increased (0.676 → 0.736). Translational jerk rose (2.33 → 2.57 m/s³), indicating more abrupt speed changes.
- Path efficiency was the lowest of all phases (0.056). Stuck rate dropped to 2.3%.
- *Interpretation*: As with Phase 1, force factors were randomised, making these results statistically meaningless for OAT conclusions.

**Phase 3 — Speed** (VALID — max_vel unaffected by config mode 2):
- Speed decreased from 2.986 to 2.679 m/s. Collision rate essentially stable (0.661 → 0.698). Near-miss rate decreased (4.202 → 4.019).
- Heading jerk increased (0.676 → 0.763), suggesting more directional oscillation at modified speeds. Translational jerk was highest at 2.89 m/s³ — 4× above GT mean (0.74), reflecting violent speed oscillation.
- Acceleration increased to 7.456 m/s² — physically unrealistic (real: 0.1–0.7 m/s²).
- TTC dropped to 0.64 s (worst of all phases), meaning modified speeds brought agents closer to constant collision.
- Proxemic distribution shifted slightly toward social zone (44.9%, up from 42.3%), with personal zone decreasing (49.4%, down from 52.0%).

**Café overall assessment**: The extreme values across all metrics — speeds of ~3 m/s (>2× realistic), accelerations of ~7 m/s², path efficiency of ~0.06, collision rates >0.66, TTC <1 s, translational jerk 3–4× GT — indicate fundamental structural problems beyond parameter tuning. The small 15×20m space with 12 agents among tables creates a gridlock scenario where the SFM produces agents trapped in perpetual high-speed oscillation between conflicting forces. The stuck rate is paradoxically low (2–4%) because agents don't stop when trapped — they *vibrate*. **The café environment is unsuitable for drawing OAT parameter sensitivity conclusions.**

---

### Environment 2: Central Tunnel (40 agents, 38.5m × 17.5m, open corridor with furniture)

**Baseline parameters** (mean across 40 agents): SF=11.58, GF=3.42, OF=16.91, MaxVel=1.28 m/s  
**Scaling multipliers**: Social (×0.55/×1.70), Goal (×0.60/×1.60), Speed (×0.75/×1.25), Obstacle (×0.50/×2.00)  
**Configuration**: Mode 1 (CUSTOM) — all parameter changes applied deterministically.

| Phase | Speed μ | Speed σ | Coll Rate | NM Rate | Path Eff | Hdg Jerk | Spd CV | Accel | Min Dist μ | Min Dist p5 | TTC μ | TTC p10 | Stuck% | Osc Idx | Trans Jerk | %Intim | %Pers | %Social | %Public |
|-------|---------|---------|-----------|---------|----------|----------|--------|-------|------------|-------------|-------|---------|--------|---------|------------|--------|-------|---------|---------|
| Baseline | 0.784 | 0.173 | 0.048 | 0.971 | 0.413 | 0.107 | 0.168 | 0.121 | 1.591 | 0.669 | 3.37 | 0.09 | 8.8% | 1.64 | 0.24 | 1.4% | 51.1% | 41.7% | 5.8% |
| Ph1 Social | 0.729 | 0.170 | 0.041 | 0.914 | 0.558 | 0.092 | 0.174 | 0.130 | 1.550 | 0.705 | 3.06 | 0.09 | 5.8% | 1.03 | 0.26 | 1.1% | 50.3% | 44.0% | 4.6% |
| Ph2 Goal | 0.773 | 0.189 | 0.042 | 0.965 | 0.435 | 0.106 | 0.177 | 0.117 | 1.611 | 0.700 | 3.60 | 0.08 | 8.1% | 2.45 | 0.24 | 1.2% | 48.9% | 44.7% | 5.3% |
| Ph3 Speed | 0.742 | 0.207 | 0.049 | 0.919 | 0.544 | 0.091 | 0.251 | 0.134 | 1.576 | 0.705 | 3.83 | 0.09 | 17.0% | 2.34 | 0.34 | 1.3% | 48.3% | 45.6% | 4.9% |
| Ph4 Obstacle | 0.695 | 0.225 | 0.046 | 0.959 | 0.532 | 0.091 | 0.322 | 0.114 | 1.572 | 0.706 | 3.55 | 0.08 | 9.7% | 1.60 | 0.24 | 1.4% | 49.6% | 44.7% | 4.4% |

#### Phase 1 — Social Force (×0.55 low / ×1.70 high)

**Expected outcome**: Reducing social force (×0.55) should allow agents to pass closer together, potentially increasing collision rate. Increasing social force (×1.70) should increase spacing and reduce collisions.

**Actual outcome**: Collision rate *improved* (0.048 → 0.041, −15%). Near-miss rate improved (0.971 → 0.914, −6%). Path efficiency improved substantially (0.413 → 0.558, +35%). Speed decreased slightly (0.784 → 0.729 m/s). Heading jerk decreased (0.107 → 0.092, −14%). Oscillation index dropped significantly (1.64 → 1.03 rev/m), indicating smoother lane formation. Stuck rate improved (8.8% → 5.8%). TTC decreased slightly (3.37 → 3.06 s), remaining within GT range. Proxemic distribution shifted slightly toward social zone (41.7% → 44.0%). Intimate zone was lowest of all phases (1.1%).

**Analysis**: The improvement in path efficiency coupled with decreased collision rate and oscillation suggests the high-SF runs (×1.70) produced the dominant signal: stronger social repulsion created clearer "lanes" in the bidirectional flow, consistent with the laminar flow phenomenon (Feliciani et al., 2018). The min inter-agent distance 5th percentile *improved* (0.669 → 0.705 m), and the low oscillation index confirms smoother, more organised movement.

#### Phase 2 — Goal Force (×0.60 low / ×1.60 high)

**Expected outcome**: Reducing goal force should cause agents to wander more. Increasing goal force should produce more direct paths but potentially more collisions.

**Actual outcome**: Speed essentially unchanged (0.784 → 0.773). Collision rate improved slightly (0.048 → 0.042). Path efficiency barely changed (0.413 → 0.435). TTC improved to 3.60 s (closest to GT mean of 3.88 s). Oscillation index was highest of all phases (2.45 rev/m), suggesting goal force variation creates more heading corrections. Stuck rate barely changed (8.8% → 8.1%). Translational jerk unchanged at 0.24 m/s³. Proxemic distribution showed minimal shift.

**Analysis**: Goal force variation had the smallest effect of any phase in Central Tunnel. This is expected in a bidirectional corridor where agents are already strongly attracted in one direction. The elevated oscillation index suggests combined low+high goal force creates conflicting directional signals.

#### Phase 3 — Speed / Max Velocity (×0.75 low / ×1.25 high)

**Expected outcome**: Reducing max_vel should slow agents, reducing collision severity. Increasing max_vel increases approach speed.

**Actual outcome**: Speed decreased (0.784 → 0.742). Collision rate unchanged (0.048 → 0.049). Path efficiency improved substantially (0.413 → 0.544, +32%). Speed variability increased (0.168 → 0.251, +49%). **Stuck rate jumped to 17.0%** — with run_13 reaching 45.7% stuck. This is the critical max_vel floor discovery: reducing max_vel to ×0.75 pushes agents below the threshold needed to overcome social forces. TTC improved to 3.83 s (essentially matching GT mean of 3.88 s). Translational jerk increased to 0.34 m/s³ (highest in CT, but still 54% below GT). Oscillation index elevated at 2.34 rev/m.

**Analysis**: The speed phase produced an unexpectedly large path efficiency improvement and the study's most important discovery — the max_vel floor. Slower agents have more time for social forces to take effect, producing smoother paths, but below ~0.96 m/s (×0.75 of 1.28), the force balance traps agents. The 17% stuck rate is a diagnostic signal: **there exists a critical max_vel threshold below which the SFM's force balance traps agents in dense zones.**

#### Phase 4 — Obstacle Force (×0.50 low / ×2.00 high)

**Expected outcome**: Reducing obstacle force should allow agents closer to walls. Increasing it should push agents toward centre, increasing congestion.

**Actual outcome**: Speed decreased notably (0.784 → 0.695, −11%). Speed variability increased dramatically (0.168 → 0.322, +92%). Path efficiency improved (0.413 → 0.532). Collision rate improved slightly (0.048 → 0.046). Stuck rate rose slightly (8.8% → 9.7%). TTC improved to 3.55 s. Translational jerk unchanged. Oscillation index essentially unchanged (1.64 → 1.60). Proxemic distribution showed minimal change.

**Analysis**: The large speed decrease and speed variability increase stem from the AABB multi-link obstacle bug: the high-obstacle condition (×2.00) amplifies forces from each furniture collision shape separately, causing partial trapping near furniture. These results are **partially confounded by the AABB obstacle bug**.

#### Central Tunnel Summary

1. **Social force** was the most impactful parameter: −15% collision rate, +35% path efficiency, lowest oscillation index (1.03 rev/m — smoothest trajectories)
2. **Goal force** produced the smallest effects across all metrics
3. **Speed** had a large positive effect on path efficiency (+32%) and produced the best TTC match (3.83 s ≈ GT 3.88 s), but revealed the critical max_vel floor (17% stuck rate)
4. **Obstacle force** results are partially confounded by the AABB bug
5. **TTC within GT range** (3.06–3.83 s vs GT 2.71–6.37 s) — the bidirectional flow produces temporally realistic collision-approach dynamics
6. **Oscillation index within GT range** (1.03–2.45 vs GT 1.08–2.93) — near-exact match for CT baseline (1.64 vs GT mean 1.72)
7. **Translational jerk is 3× below GT** (0.24–0.34 vs 0.74 m/s³) — agents change speed too gradually, suggesting τ = 0.5s is too slow
8. **Proxemic zones show over-social, under-intimate spacing** (1.1–1.4% intimate vs GT 6.9%; 42–46% social vs GT 25%) — social force is too conservative

---

### Environment 3: Delta (80 agents, ~55m triangular, obstacle-free)

**Baseline parameters** (mean across 80 agents): SF=10.85, GF=3.27, OF=17.57, MaxVel=1.21 m/s  
**Scaling multipliers**: Social (×0.55/×1.70), Goal (×0.60/×1.60), Speed (×0.75/×1.25), Obstacle (×0.50/×2.00)  
**Configuration**: Mode 1 (CUSTOM). Cabinets removed from world to avoid AABB obstacle trapping.  
**Geometry**: 80 agents in 3 corner clusters (27/27/26), walking across the triangle to opposite corners.

| Phase | Speed μ | Speed σ | Coll Rate | NM Rate | Path Eff | Hdg Jerk | Spd CV | Accel | Min Dist μ | Min Dist p5 | TTC μ | TTC p10 | Stuck% | Osc Idx | Trans Jerk | %Intim | %Pers | %Social | %Public |
|-------|---------|---------|-----------|---------|----------|----------|--------|-------|------------|-------------|-------|---------|--------|---------|------------|--------|-------|---------|---------|
| Baseline | 0.181 | 0.036 | 0.034 | 1.283 | 0.998 | 0.001 | 0.133 | 0.046 | 1.081 | 0.670 | 6.32 | 0.34 | 0.03% | 0.79 | 0.09 | 0.7% | 69.9% | 29.2% | 0.2% |
| Ph1 Social | 0.201 | 0.043 | 0.050 | 1.328 | 0.986 | 0.007 | 0.160 | 0.045 | 1.045 | 0.654 | 7.76 | 0.33 | 0.2% | 1.09 | 0.10 | 1.3% | 73.2% | 25.6% | 0.0% |
| Ph2 Goal | 0.208 | 0.041 | 0.040 | 1.129 | 0.979 | 0.008 | 0.134 | 0.050 | 1.115 | 0.685 | 6.85 | 0.19 | 0.3% | 0.93 | 0.09 | 1.0% | 68.4% | 30.1% | 0.4% |
| Ph3 Speed | 0.145 | 0.032 | 0.019 | 1.355 | 0.996 | 0.026 | 0.150 | 0.037 | 1.059 | 0.710 | 6.60 | 0.43 | 12.8% | 1.45 | 0.06 | 0.4% | 72.4% | 27.2% | 0.0% |
| Ph4 Obstacle | 0.201 | 0.040 | 0.044 | 1.346 | 0.981 | 0.006 | 0.141 | 0.056 | 1.046 | 0.644 | 5.76 | 0.24 | 0.07% | 0.87 | 0.08 | 1.0% | 73.9% | 25.1% | 0.0% |

#### Phase 1 — Social Force (×0.55 low / ×1.70 high)

**Actual outcome**: Collision rate increased (0.034 → 0.050, +47%). Speed increased (0.181 → 0.201, +11%). Path efficiency decreased slightly (0.998 → 0.986). Heading jerk increased ×7 (0.001 → 0.007). TTC was highest of all phases (7.76 s) — the high-SF runs produced maximum future safety margins. Oscillation index increased (0.79 → 1.09 rev/m), entering the GT range (1.08–2.93). Intimate zone doubled from 0.7% to 1.3%. Personal zone expanded (69.9% → 73.2%).

**Analysis**: The collision increase suggests the low-SF condition (×0.55) dominates — reducing social repulsion in the 80-agent crossing scenario allows more overlaps at the intersection centre. The high TTC shows that the high-SF condition pushes agents to safer future trajectories. The oscillation index entering GT range (1.09) indicates more realistic heading-correction frequency under social force variation.

#### Phase 2 — Goal Force (×0.60 low / ×1.60 high)

**Actual outcome**: Speed increased (0.181 → 0.208, +15%). Near-miss rate *decreased* (1.283 → 1.129, −12%). Min inter-agent distance *increased* (1.081 → 1.115 m). TTC increased (6.32 → 6.85 s). TTC p10 decreased (0.34 → 0.19 s), meaning the worst encounters got more dangerous even as average safety improved. Oscillation index stayed moderate (0.93 rev/m).

**Analysis**: This demonstrates a clear trade-off: **stronger goal force trades a small collision rate increase (+18%) for substantially improved proxemic compliance (−12% near-miss rate)**. Agents push through the crossing zone faster, spending less time in close proximity.

#### Phase 3 — Speed / Max Velocity (×0.75 low / ×1.25 high)

**Actual outcome**: Collision rate decreased substantially (0.034 → 0.019, **−44%** — the largest single-metric improvement in the study). **Stuck rate jumped to 12.8%**, with run_15 reaching 47.5% stuck — the same max_vel floor phenomenon as CT. Speed decreased (0.181 → 0.145, −20%). Min distance p5 *improved* (0.670 → 0.710 m — best worst-case separation). TTC p10 improved (0.34 → 0.43 s — the safest worst-case encounters). Translational jerk dropped to 0.06 m/s³ (lowest in study, 12× below GT). Oscillation index highest of all Delta phases (1.45 rev/m), likely driven by trapped agents in run_15.

**Analysis**: **This is the strongest OAT parameter signal in the entire study.** The −44% collision rate reduction (matching GT mean 0.019 exactly) and improved worst-case spacing demonstrate a direct, large-magnitude, theoretically consistent response to max_vel variation. The stuck rate confirms the generalizable max_vel floor: reducing max_vel to ×0.75 of baseline (~0.91 m/s) creates force-balanced trapping at crossing zones, the same mechanism found in CT.

#### Phase 4 — Obstacle Force (×0.50 low / ×2.00 high)

**Actual outcome**: Speed unchanged (0.201 m/s). Collision rate increased (0.034 → 0.044, +29%). TTC was lowest (5.76 s), still exceeding GT mean. Oscillation index unchanged. Proxemic distribution shifted slightly toward personal zone (73.9%). Stuck rate remained near zero (0.07%).

**Analysis**: Obstacle force should be irrelevant in an obstacle-free environment, yet produced measurable effects from boundary wall forces. The SFM's `HandleObstacles2()` computes forces from all Gazebo model AABBs, including boundary walls. Higher OF pushes agents toward the centre, increasing congestion.

#### Delta Summary

1. **Speed (max_vel)** produced the strongest signal: −44% collision rate matching GT mean exactly (0.019), but revealed the max_vel floor (12.8% stuck)
2. **Goal force** demonstrated a meaningful near-miss/collision trade-off: −12% near-miss rate for +18% collision rate
3. **Social force** variation increased collisions (+47%), dominated by low-SF condition
4. **Obstacle force** had the smallest effect, as expected
5. **TTC is the highest across all environments** (5.76–7.76 s), reflecting open geometry and slow walking
6. **Stuck rate is near-zero except speed phase** (0.03–0.3% baseline → 12.8% speed phase) — confirms the max_vel floor is a generalizable phenomenon
7. **Oscillation index below GT range** (0.79–1.45 vs GT 1.08–2.93) — agents make too few heading corrections
8. **Translational jerk is 8–12× below GT** (0.06–0.10 vs 0.74 m/s³) — agents change speed too smoothly, lacking responsive modulation
9. **Proxemic zones show dominant personal zone** (68–74%) with near-zero intimate (0.4–1.3%) and public (0–0.4%) — spacing is realistic but overly conservative

---

## Phase 2C — Cross-Environment Comparison

### Structural Limitations

Before comparing results across environments, the following structural differences must be acknowledged:

#### 1. Environment Size and Agent Density

| Environment | Approximate Area | Agents | Nominal Density |
|-------------|-----------------|--------|-----------------|
| Café | ~300 m² | 12 | ~0.04 agents/m² |
| Central Tunnel | ~674 m² | 40 | ~0.06 agents/m² |
| Delta | ~1,320 m² (triangle) | 80 | ~0.06 agents/m² |

While nominal densities are comparable for CT and Delta, the *effective* density differs due to flow geometry: random crossing in café, linear opposing streams in CT, three converging streams in Delta.

#### 2. Obstacle Interference in Café and Central Tunnel

The café and Central Tunnel contain furniture and obstacles. Because the ESFM implementation lacks path planning, agents cannot navigate around obstacles — they are repelled by the SFM's exponential obstacle force. This creates trapping: agents oscillate between conflicting obstacle, social, and goal forces. The AABB multi-link bug amplifies this in environments with furniture. Despite this, comparisons *within* a single environment remain valid for OAT purposes.

#### 3. Agent Population Differences

The simulations use 12, 40, and 80 agents respectively — significantly fewer than GT datasets (148–434 pedestrians). Lower counts reduce simultaneous interactions, emergent phenomena, and statistical robustness.

### Cross-Environment Metric Comparison

| Metric | Café (cluttered) | Central Tunnel (corridor) | Delta (open) | GT Range |
|--------|-----------------|--------------------------|--------------|----------|
| Speed (m/s) | 2.68–2.99 | 0.70–0.78 | 0.15–0.21 | 0.64–1.46 |
| Collision Rate | 0.66–1.07 | 0.04–0.05 | 0.02–0.05 | 0.0004–0.094 |
| Path Efficiency | 0.06 | 0.41–0.56 | 0.98–1.00 | 0.87–0.97 |
| Heading Jerk | 0.68–0.76 | 0.09–0.11 | 0.001–0.026 | 0.11–0.52 |
| TTC Mean (s) | 0.55–0.85 | 3.06–3.83 | 5.76–7.76 | 2.71–6.37 |
| Stuck Rate | 2.3–3.7% | 5.8–17.0% | 0.03–12.8% | 2.0–27.7% |
| Osc Index (rev/m) | 0.65–1.31 | 1.03–2.45 | 0.79–1.45 | 1.08–2.93 |
| Trans Jerk (m/s³) | 2.33–2.89 | 0.24–0.34 | 0.06–0.10 | 0.52–1.22 |
| % Intimate | 4.9–7.6% | 1.1–1.4% | 0.4–1.3% | 0.6–23.6% |
| % Personal | 49–55% | 48–51% | 68–74% | 47–72% |
| % Social | 37–45% | 42–46% | 25–30% | 15–41% |

### Consistent Directional Effects

**Speed (max_vel)** produced consistent effects across all valid environments:
- Central Tunnel: path efficiency +32%, heading jerk −15%, TTC improved to 3.83 s (≈ GT mean)
- Delta: collision rate −44% (matching GT mean exactly), min distance p5 improved
- In both environments, speed also revealed the max_vel floor (stuck rate jumping to 17% and 12.8% respectively)

**Social force** produced consistent directional effects:
- Central Tunnel: collision rate −15%, path efficiency +35%, oscillation index lowest (1.03 rev/m)
- Delta: collision rate +47% (dominated by low-SF), TTC highest (7.76 s)
- Both show social force is the primary controller of inter-agent spacing and lane formation

**Goal force** produced consistently small effects:
- Central Tunnel: most metrics changed <5%
- Delta: near-miss rate −12% (the only large goal force effect)
- Goal force matters less in simple geometries where paths are largely unobstructed

**Obstacle force** effects were environment-dependent:
- Central Tunnel: speed −11%, speed variability +92% (AABB bug)
- Delta: collision rate +29%, small wall-boundary effect

### TTC as the Best Discriminator

TTC cleanly separates the three environments where collision rate alone cannot:

| Environment | TTC Mean (s) | vs GT Mean (3.88 s) | Assessment |
|-------------|-------------|---------------------|------------|
| Café | 0.55–0.85 | −78% to −82% | **Catastrophic** — perpetual collision crisis |
| Central Tunnel | 3.06–3.83 | −1% to −21% | **Realistic** — within GT range |
| Delta | 5.76–7.76 | +48% to +100% | **Too safe** — overly conservative |

### Translational Jerk Gap

| Environment | Trans Jerk (m/s³) | vs GT Mean (0.74) | Diagnosis |
|-------------|-------------------|-------------------|-----------|
| Café | 2.33–2.89 | +215% to +290% | Violently abrupt oscillation |
| Central Tunnel | 0.24–0.34 | −54% to −68% | Too smooth |
| Delta | 0.06–0.10 | −86% to −92% | Near-constant speed |

No environment produces realistic speed modulation. This suggests τ = 0.5s in the SFM's velocity-update equation is too slow for the acceleration-deceleration cycles real pedestrians exhibit.

### Proxemic Zone Compliance

A consistent cross-environment finding: **all simulated environments have too little intimate-zone interaction and too much social-zone spacing**. The social force pushes agents into the 1.2–3.6m social band more than real pedestrians occupy. Real pedestrians accept closer proximity because they have anticipatory planning that the SFM lacks.

### Environment Structure Dominance

The range of variation *within* each environment due to parameter changes is small compared to differences *between* environments. This confirms that environment design (obstacle placement, agent count, flow geometry) is the primary determinant of simulation realism, with OAT parameter tuning operating within those bounds.

---

## Phase 2D — Simulation vs Ground Truth Comparison

### Limitations Disclaimer

1. **Environment mismatch**: The simulated environments differ fundamentally from ETH/UCY recording locations
2. **Population mismatch**: Simulated crowds (12–80 agents) are substantially smaller than GT scenes (148–434 pedestrians)
3. **Model limitations**: ESFM lacks anticipatory navigation, cultural behaviours, and higher-order social interactions
4. **No path planning**: Agents navigate purely by force balance, leading to trapping artefacts
5. **Config mode 2 in café**: Café force factor results are unreliable

### Reference Ground Truth Summary

| Dataset | Speed μ | Coll Rate | Path Eff | Hdg Jerk | Spd CV | Min Dist μ | Min Dist p5 | TTC μ | TTC p10 | Stuck% | Osc Idx | Trans Jerk | %Intim | %Pers | %Social | %Public |
|---------|---------|-----------|----------|----------|--------|------------|-------------|-------|---------|--------|---------|------------|--------|-------|---------|---------|
| eth_hotel | 1.29 | 0.0010 | 0.910 | 0.518 | 0.155 | 1.65 | 0.51 | 2.89 | 0.22 | 19.4% | 2.93 | 0.89 | 3.5% | 47.3% | 41.2% | 8.0% |
| eth_univ | 1.46 | 0.0004 | 0.966 | 0.467 | 0.163 | 1.88 | 0.60 | 6.37 | 0.37 | 4.9% | 1.37 | 1.22 | 0.6% | 54.5% | 31.1% | 13.9% |
| ucy_zara01 | 1.10 | 0.0022 | 0.964 | 0.107 | 0.138 | 1.57 | 0.57 | 3.78 | 0.38 | 2.0% | 1.12 | 0.52 | 0.8% | 67.6% | 22.7% | 9.0% |
| ucy_zara02 | 1.11 | 0.0185 | 0.948 | 0.145 | 0.183 | 1.23 | 0.43 | 4.58 | 0.29 | 27.7% | 1.46 | 0.59 | 6.3% | 69.2% | 18.5% | 6.1% |
| ucy_zara03 | 1.10 | 0.0121 | 0.944 | 0.128 | 0.143 | 1.55 | 0.41 | 3.59 | 0.29 | 8.9% | 1.08 | 0.54 | 7.4% | 59.5% | 24.0% | 9.2% |
| ucy_univ_s1 | 0.64 | 0.0941 | 0.870 | 0.220 | 0.355 | 0.80 | 0.30 | 3.21 | 0.20 | 12.7% | 2.43 | 0.62 | 23.6% | 60.6% | 15.4% | 0.4% |
| ucy_univ_s3 | 0.77 | 0.0231 | 0.881 | 0.214 | 0.317 | 0.97 | 0.44 | 2.71 | 0.15 | 6.9% | 1.63 | 0.77 | 6.5% | 72.4% | 20.0% | 1.1% |
| **GT Mean** | **1.07** | **0.019** | **0.926** | **0.257** | **0.208** | **1.38** | **0.47** | **3.88** | **0.27** | **11.8%** | **1.72** | **0.74** | **6.9%** | **61.6%** | **24.7%** | **6.8%** |

### Metric-by-Metric Comparison

#### Speed

| Source | Speed (m/s) | vs GT Mean |
|--------|-------------|------------|
| GT Mean | 1.07 | — |
| Café Baseline | 2.99 | +179% (2.8×) |
| CT Baseline | 0.78 | −27% |
| Delta Baseline | 0.18 | −83% |

The café speed (2.99 m/s) is physically absurd — agents oscillating at high velocity between conflicting forces. Central Tunnel (0.78 m/s) falls within the lower range of GT. Delta (0.18 m/s) is drastically below any GT dataset — opposing forces consume >85% of goal-directed velocity. **CT produces the most realistic speed.**

#### Collision Rate

| Source | Collision Rate | vs GT Mean |
|--------|---------------|------------|
| GT Mean | 0.019 | — |
| CT Baseline | 0.048 | +153% |
| Delta Baseline | 0.034 | +79% |
| Delta Phase 3 (Speed) | 0.019 | **Exact match** |

**Delta Phase 3 achieved an exact match with GT mean collision rate (0.019)** — the strongest single validation of the OAT approach.

#### TTC

| Source | TTC Mean (s) | vs GT Mean (3.88 s) |
|--------|-------------|---------------------|
| GT Mean | 3.88 | — |
| Café Baseline | 0.55 | −86% |
| CT Baseline | 3.37 | −13% — **excellent** |
| CT Phase 3 | 3.83 | −1% — **near-exact match** |
| Delta Baseline | 6.32 | +63% |

CT TTC (3.37 s) is within 13% of GT mean. CT Phase 3 Speed achieved 3.83 s — essentially matching GT mean. This demonstrates realistic temporal safety dynamics in the bidirectional corridor.

#### Path Efficiency

| Source | Path Efficiency | vs GT Mean (0.926) |
|--------|----------------|-------------------|
| Café Baseline | 0.063 | −93% |
| CT Baseline | 0.413 | −55% |
| Delta Baseline | 0.998 | +8% |

Delta is "too perfect" — agents walk in near-straight lines without adaptive corrections. CT is below even the densest GT dataset (0.870), indicating obstacle-induced path degradation.

#### Heading Jerk and Oscillation Index

| Source | Heading Jerk | Osc Index (rev/m) | Assessment |
|--------|-------------|-------------------|------------|
| GT Mean | 0.257 | 1.72 | — |
| CT Baseline | 0.107 | 1.64 | Jerk matches UCY Zara01 exactly; Osc within 5% of GT mean |
| Delta Baseline | 0.001 | 0.79 | Both far below GT — overly smooth |
| Café | 0.676 | 1.31 | Jerk elevated by oscillation artefact |

CT produces the most realistic directional dynamics. The oscillation index near-exact match (1.64 vs GT 1.72, −5%) confirms realistic heading-correction frequency.

#### Translational Jerk

| Source | Trans Jerk (m/s³) | vs GT Mean (0.74) |
|--------|-------------------|-------------------|
| GT Mean | 0.74 | — |
| Café | 2.33 | +215% |
| CT (best: Ph3) | 0.34 | −54% |
| Delta | 0.06–0.10 | −86 to −92% |

**No environment achieves realistic translational jerk.** This is a fundamental limitation of the SFM's constant relaxation time τ = 0.5s.

#### Speed Variability (CV) and Stuck Rate

| Source | Speed CV | Stuck Rate | Assessment |
|--------|----------|-----------|------------|
| GT Mean | 0.208 | 11.8% | — |
| CT Baseline | 0.168 | 8.8% | Both within realistic range |
| Delta Baseline | 0.133 | 0.03% | CV slightly low; agents never stop |

CT speed CV (0.168) falls within GT range (0.138–0.355). CT stuck rate (8.8%) is below GT mean but reasonable.

#### Minimum Inter-Agent Distance and Proxemic Zones

| Source | Min Dist μ (m) | Min Dist p5 (m) | %Intimate | %Personal | %Social |
|--------|---------------|-----------------|-----------|-----------|---------|
| GT Mean | 1.38 | 0.47 | 6.9% | 61.6% | 24.7% |
| CT Baseline | 1.59 | 0.67 | 1.4% | 51.1% | 41.7% |
| Delta Baseline | 1.08 | 0.67 | 0.7% | 69.9% | 29.2% |

Both CT and Delta produce mean separations within GT range. All environments are under-intimate and over-social — the social force is too conservative for natural proxemic compliance.

### Summary of Best Simulation Matches

| Metric | GT Target | Best Match | Environment/Phase | Error |
|--------|-----------|-----------|-------------------|-------|
| Speed | 1.07 m/s | 0.78 m/s | CT Baseline | −27% |
| Collision Rate | 0.019 | 0.019 | Delta Phase 3 | **0% — exact** |
| TTC Mean | 3.88 s | 3.83 s | CT Phase 3 | **−1% — near-exact** |
| Path Efficiency | 0.926 | 0.998 | Delta Baseline | +8% (too perfect) |
| Heading Jerk | 0.257 | 0.107 | CT Baseline | −58% (matches UCY Zara01 exactly) |
| Osc Index | 1.72 rev/m | 1.64 rev/m | CT Baseline | **−5% — near-exact** |
| Speed CV | 0.208 | 0.168 | CT Baseline | −19% |
| Stuck Rate | 11.8% | 8.8% | CT Baseline | −25% |
| Trans Jerk | 0.74 m/s³ | 0.34 m/s³ | CT Ph3 Speed | −54% — too smooth |
| Min Distance | 1.38 m | 1.08 m | Delta Baseline | −22% |
| % Intimate | 6.9% | 5.1% | Café Ph3 | −26% |
| % Personal | 61.6% | 69.9% | Delta Baseline | +13% |
| % Social | 24.7% | 25.6% | Delta Ph1 | **+4% — near-exact** |

---

## Phase 3 — Optimal Parameter Recommendations

### Evidence Summary

Based on 53 simulation runs across three environments (15 café, 19 Central Tunnel, 19 Delta) against seven ground truth datasets with all 15 evaluation metrics:

1. **Speed (max_vel)** is the most impactful parameter. Delta Phase 3 achieved an exact collision rate match (0.019) and CT Phase 3 achieved a near-exact TTC match (3.83 vs 3.88 s). However, the max_vel floor (stuck rate jumping to 12.8–17.0% at ×0.75 of baseline) imposes a hard constraint: **max_vel must remain ≥ 1.0 m/s**.

2. **Social force** is the second most impactful parameter and the primary controller of inter-agent spacing, lane formation, and collision avoidance. It produced the largest combined effects in CT (−15% collision rate, +35% path efficiency, lowest oscillation index). However, the proxemic zone analysis shows social force is already too conservative — agents maintain too much distance, spending excessive time in the social zone (42–46%) and too little in the intimate zone (1–1.4% vs GT 6.9%).

3. **Goal force** has minimal impact in simple geometries but demonstrated a useful trade-off in Delta (−12% near-miss rate for +18% collision rate increase).

4. **Obstacle force** effects are confounded by the AABB implementation bug and are only relevant in environments with obstacles.

5. **Translational jerk** is 2–12× below GT across all environments. The SFM's relaxation time τ = 0.5s produces speed changes that are too gradual. Experiment with τ = 0.2–0.3s to produce more responsive agents.

### Per-Environment Recommendations

#### Central Tunnel

- **Social force**: Moderate increase (×1.20–1.40, i.e., SF ~14–16). Not as high as V1 recommendation (SF 14–17) because proxemic data shows agents are already over-social.
- **Goal force**: Maintain near baseline (×1.00).
- **Max velocity**: Maintain ≥ 1.10 m/s. Baseline (1.28) produces speed below GT (0.78 vs 1.07 m/s), so do not reduce further.
- **Obstacle force**: Reduce to ×0.50 to minimise AABB trapping, or fix the bug.

#### Delta

- **Social force**: Increase modestly (×1.20–1.40, i.e., SF ~13–15). Low-SF dramatically worsened collisions.
- **Goal force**: Increase modestly (×1.30–1.50, i.e., GF ~4.3–4.9) for near-miss reduction.
- **Max velocity**: Maintain ≥ 1.0 m/s. The baseline speed (0.18 m/s) is critically low — the max_vel cap (1.21 m/s) isn't being reached due to opposing forces. The true fix is force balance, not max_vel reduction.
- **Obstacle force**: Not applicable (obstacle-free).

### General Parameter Recommendations

| Parameter | Recommended Value | Justification |
|-----------|-------------------|---------------|
| `social_force_factor` | 11–15 | Balances collision avoidance with proxemic realism; avoids over-conservative spacing |
| `goal_force_factor` | 3.5–4.5 | Minimal impact in simple geometries; slight increase reduces near-miss duration |
| `obstacle_force_factor` | 8–12 (×0.50–0.70 of baseline) | Reduces AABB trapping artefacts while maintaining wall avoidance |
| `max_vel` | 1.10–1.40 m/s (never below 1.0) | Within realistic human walking speed; enforces floor to prevent force trapping |
| `relaxation_time` (τ) | 0.2–0.3 s (requires code change) | Current τ=0.5s produces translational jerk 2–12× below GT; faster response needed |
| `configuration` | **1 (CUSTOM)** | Mandatory for deterministic parameter control |

### Trade-offs

1. **Social force vs. speed**: Higher SF improves collision rates but reduces speed. At SF > 20, agents become overly cautious.
2. **Social force vs. proxemics**: Higher SF reduces collisions but shifts proxemic distribution toward over-social spacing (>1.2m). The GT intimate-zone fraction (6.9%) requires allowing closer approaches than current SF levels permit.
3. **Goal force vs. collisions**: Higher GF reduces near-miss duration but can increase collision rate.
4. **Speed vs. stuckness**: Reducing max_vel below ~1.0 m/s causes catastrophic force trapping (12–47% stuck rate in individual runs).

### Inconclusive Areas Requiring Further Testing

1. The interaction between social force and environment density is not captured by single-environment OAT analysis.
2. The AABB obstacle handling bug makes all obstacle force conclusions in cluttered environments unreliable.
3. The severe speed deficit in Delta (0.18 vs 1.21 m/s max_vel) suggests a force balance issue — τ or fundamental force magnitude scaling may need adjustment.
4. The study used 2 runs per phase variant; a minimum of 5 runs per condition would improve confidence intervals.
5. No environment achieves realistic translational jerk — this is the most significant unresolved realism gap.

---

## Phase 4 — Report Alignment Review

The following issues were identified by cross-referencing the analysis above against the attached capstone report draft:

### Inconsistencies

1. **Agent count claims**: The report states "10–100 pedestrians" and "30 runs per environment". Actual: 12/40/80 agents with 15/19/19 runs.
2. **Config mode 2 not discussed**: The café force factor issue must be disclosed.
3. **Metric definitions**: The report defines ADE/FDE (trajectory prediction metrics) but the analysis uses distributional population/agent/pairwise metrics. The report should clarify ADE/FDE are not applicable to generative SFM evaluation.
4. **Delta agent count**: Documentation says 50, actual is 80.
5. **Obstacle handling**: The `HandleObstacles2()` AABB bug is not documented.
6. **Speed results in café**: The report highlights "Phase 2 reduced collisions 19%" — this was a random effect due to config mode 2. Must be retracted.

### Gaps

1. **Missing Results sections**: Placeholder headings need populating with CT and Delta data.
2. **Central Tunnel and Delta data**: The report draft is primarily written around café results. CT (first reliable OAT environment) and Delta (cleanest data) are not yet incorporated.
3. **GT comparison**: The quantitative sim-vs-GT comparison in Phase 2D should be incorporated.
4. **Speed deficit**: The most significant finding (0.18–0.78 m/s vs GT 1.07 m/s) is not discussed.
5. **Group metrics**: Report discusses group dynamics extensively but `num_groups: 0` for all environments.
6. **TTC and extended metrics**: The report's methodology lists TTC, stuck rate, oscillation index, translational jerk, and proxemic zones as evaluation dimensions. The results section should present these alongside the original metrics.

### Recommended Updates

1. Update Experimental Design with actual run counts and agent populations
2. Add "Known Limitations" subsection: config mode 2, AABB bug, no path planning, max_vel floor
3. Replace café Phases 1–2 conclusions with transparency note about config mode 2
4. Add full Results section with CT and Delta as primary evidence, including all 15 metrics
5. Add Sim vs GT section with comprehensive metric-by-metric comparison
6. Update Conclusion to focus on max_vel as strongest validated parameter, social force as second
7. Address ADE/FDE disconnect between literature review and actual metrics
8. Document speed deficit and translational jerk gap as key findings
9. Include TTC, proxemic zone, stuck rate, and oscillation index results throughout (not as an appendix)
