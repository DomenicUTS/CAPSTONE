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

| Variant | Speed μ | Speed σ | Coll Rate | NM Rate | Path Eff | Hdg Jerk | Spd CV | Accel | Min Dist μ | Min Dist p5 | TTC μ | TTC p10 | Stuck% | Osc Idx | Trans Jerk | %Intim | %Pers | %Social | %Public |
|---------|---------|---------|-----------|---------|----------|----------|--------|-------|------------|-------------|-------|---------|--------|---------|------------|--------|-------|---------|---------|
| Baseline (3 runs) | 2.986 | 1.596 | 0.661 | 4.202 | 0.063 | 0.676 | 0.393 | 6.899 | 1.241 | 0.451 | 0.137 | 0.012 | 0.6% | 1.315 | 33.706 | 4.9% | 52.0% | 42.3% | 0.8% |
| Ph1 Social LOW (×0.579) | 2.784 | 1.760 | 1.104 | 5.152 | 0.056 | 0.684 | 0.581 | 5.951 | 1.161 | 0.382 | 0.294 | 0.012 | 0.8% | 0.770 | 29.782 | 7.8% | 54.6% | 36.4% | 1.2% |
| Ph1 Social HIGH (×2.371) | 3.042 | 1.593 | 1.032 | 4.927 | 0.062 | 0.694 | 0.499 | 6.505 | 1.153 | 0.393 | 0.142 | 0.014 | 0.7% | 0.714 | 31.904 | 7.3% | 55.0% | 37.2% | 0.5% |
| Ph2 Goal LOW (×0.469) | 2.853 | 1.561 | 0.798 | 4.205 | 0.054 | 0.756 | 0.476 | 7.035 | 1.187 | 0.422 | 0.144 | 0.012 | 0.1% | 0.622 | 34.174 | 5.9% | 52.0% | 41.7% | 0.4% |
| Ph2 Goal HIGH (×2.033) | 2.779 | 1.606 | 0.644 | 4.328 | 0.058 | 0.716 | 0.558 | 6.829 | 1.184 | 0.459 | 0.406 | 0.015 | 0.8% | 0.752 | 32.764 | 4.6% | 53.8% | 41.2% | 0.3% |
| Ph3 Speed LOW (×0.514) | 2.779 | 1.589 | 0.413 | 3.725 | 0.058 | 0.689 | 0.539 | 7.058 | 1.276 | 0.518 | 0.228 | 0.015 | 0.4% | 0.710 | 32.306 | 3.1% | 48.9% | 47.4% | 0.6% |
| Ph3 Speed HIGH (×1.330) | 2.578 | 1.514 | 0.984 | 4.314 | 0.061 | 0.836 | 0.466 | 7.854 | 1.199 | 0.409 | 0.090 | 0.008 | 0.2% | 0.580 | 34.802 | 7.1% | 49.9% | 42.5% | 0.5% |

**Phase-by-phase discussion** (noting config mode 2 unreliability for Phases 1–2):

**Phase 1 — Social Force** (unreliable — config mode 2 randomises force factors):

The HIGH/LOW breakdown reveals that both variants increased collision rate substantially above baseline:
- **LOW (×0.579)**: Collision rate 1.104 (+67% from baseline 0.661). Near-miss rate 5.152 (+23%). Min distance p5 decreased (0.451→0.382 m). TTC increased slightly (0.137→0.294 s). Speed CV jumped (0.393→0.581), indicating very erratic motion.
- **HIGH (×2.371)**: Collision rate 1.032 (+56%). Near-miss rate 4.927 (+17%). TTC was near-baseline (0.142 s). Speed CV elevated (0.499). Translational jerk slightly lower (31.904 vs 33.706).

Both LOW and HIGH increased collisions, which is expected given config mode 2 randomised the actual force factors, making both conditions effectively uncontrolled. The slight advantage of HIGH (lower collision rate than LOW) cannot be attributed to the intentional scaling. The proxemic distribution shifted in both conditions: intimate zone roughly doubled (4.9%→7.3–7.8%), personal zone expanded (52.0%→54.6–55.0%), and social zone contracted (42.3%→36.4–37.2%).

**Phase 2 — Goal Force** (unreliable — config mode 2 randomises force factors):

The HIGH/LOW breakdown reveals an interesting pattern despite randomised forces:
- **LOW (×0.469)**: Collision rate 0.798 (+21%). Heading jerk elevated (0.756). Acceleration 7.035 m/s² (highest of all café variants). Stuck rate was lowest (0.1%).
- **HIGH (×2.033)**: Collision rate 0.644 (−3% from baseline — the best of any café variant). TTC highest of all café variants (0.406 s, still critically low). Min distance p5 improved (0.451→0.459 m).

The HIGH condition's lower collision rate was the only directionally consistent result in the café force phases, but this cannot be attributed to goal force variation because the actual goal force was randomised by config mode 2.

**Phase 3 — Speed** (VALID — max_vel is not overwritten by config mode 2):

The HIGH/LOW breakdown reveals a **clearly directional, consistent result** — the only reliable OAT finding in the café:
- **LOW (×0.514, max_vel ~0.55 m/s)**: Collision rate **0.413 (−38% from baseline)** — the largest collision reduction in the café. Near-miss rate dropped to 3.725 (−11%). Min distance p5 improved substantially (0.451→0.518 m). TTC improved (0.137→0.228 s). Translational jerk decreased slightly (33.706→32.306). Intimate zone was lowest of all phases (3.1%). Social zone was highest (47.4%).
- **HIGH (×1.330, max_vel ~1.41 m/s)**: Collision rate **0.984 (+49%)** — the second-worst after Phase 1 LOW. Near-miss rate increased (4.314). Min distance p5 worsened (0.451→0.409 m). TTC decreased to 0.090 s (worst of all café variants). Heading jerk highest (0.836). Acceleration highest (7.854 m/s²). Intimate zone doubled (4.9%→7.1%).

The directional effect is unambiguous: **reducing max_vel reduced collisions (−38%), increasing max_vel increased collisions (+49%)**. Slower agents oscillate at lower velocities, reducing the energy of each oscillation cycle. The mechanism is specific to the café's trapping pathology — slower oscillation produces less violent collisions, not fewer encounters per se.

**Café overall assessment**: The HIGH/LOW analysis confirms the café as a diagnostic environment rather than a calibration environment. The extreme values across all metrics — speeds of 2.6–3.0 m/s (2–3× max_vel, indicating SFM force-driven oscillation overriding velocity clamping), accelerations of 6–8 m/s² (>8× realistic), translational jerk 30–35 m/s³ (>40× GT mean 0.74), path efficiency ~0.06, TTC <0.41 s — all reflect fundamental force trapping, not parameter-tunable behaviour. The café demonstrates the SFM's complete failure mode in cluttered environments lacking path planning.

---

### Environment 2: Central Tunnel (40 agents, 38.5m × 17.5m, open corridor with furniture)

**Baseline parameters** (mean across 40 agents): SF=11.58, GF=3.42, OF=16.91, MaxVel=1.28 m/s  
**Scaling multipliers**: Social (×0.55/×1.70), Goal (×0.60/×1.60), Speed (×0.75/×1.25), Obstacle (×0.50/×2.00)  
**Configuration**: Mode 1 (CUSTOM) — all parameter changes applied deterministically.

| Variant | Speed μ | Speed σ | Coll Rate | NM Rate | Path Eff | Hdg Jerk | Spd CV | Accel | Min Dist μ | Min Dist p5 | TTC μ | TTC p10 | Stuck% | Osc Idx | Trans Jerk | %Intim | %Pers | %Social | %Public |
|---------|---------|---------|-----------|---------|----------|----------|--------|-------|------------|-------------|-------|---------|--------|---------|------------|--------|-------|---------|---------|
| Baseline (3 runs) | 2.987 | 0.940 | 0.193 | 3.884 | 0.413 | 0.107 | 0.238 | 1.514 | 1.591 | 0.669 | 1.116 | 0.023 | 4.0% | 1.642 | 9.691 | 1.4% | 51.1% | 41.6% | 5.8% |
| Ph1 Social LOW (×0.55) | 2.751 | 0.777 | 0.232 | 4.978 | 0.596 | 0.083 | 0.207 | 1.737 | 1.474 | 0.624 | 0.932 | 0.026 | 2.5% | 1.128 | 12.144 | 1.6% | 55.3% | 38.5% | 4.6% |
| Ph1 Social HIGH (×1.70) | 2.863 | 0.936 | 0.094 | 2.334 | 0.520 | 0.100 | 0.246 | 1.774 | 1.627 | 0.786 | 1.566 | 0.024 | 1.6% | 0.935 | 11.942 | 0.6% | 45.4% | 49.4% | 4.6% |
| Ph2 Goal LOW (×0.60) | 2.839 | 0.980 | 0.107 | 3.663 | 0.470 | 0.101 | 0.249 | 1.651 | 1.627 | 0.737 | 1.489 | 0.023 | 1.6% | 1.321 | 10.777 | 0.7% | 48.0% | 46.1% | 5.2% |
| Ph2 Goal HIGH (×1.60) | 3.033 | 1.005 | 0.231 | 4.056 | 0.399 | 0.112 | 0.218 | 1.289 | 1.595 | 0.663 | 1.116 | 0.019 | 4.7% | 3.579 | 8.475 | 1.6% | 49.8% | 43.2% | 5.4% |
| Ph3 Speed LOW (×0.75) | 1.694 | 0.864 | 0.183 | 3.683 | 0.832 | 0.051 | 0.544 | 1.038 | 1.524 | 0.731 | 3.027 | 0.040 | 4.9% | 2.535 | 6.606 | 1.2% | 50.2% | 44.1% | 4.5% |
| Ph3 Speed HIGH (×1.25) | 3.590 | 1.204 | 0.208 | 3.671 | 0.255 | 0.132 | 0.245 | 1.812 | 1.629 | 0.679 | 0.628 | 0.010 | 4.0% | 2.150 | 12.437 | 1.3% | 46.3% | 47.1% | 5.3% |
| Ph4 Obstacle LOW (×0.50) | 2.972 | 0.946 | 0.196 | 3.832 | 0.427 | 0.108 | 0.235 | 1.314 | 1.588 | 0.710 | 1.222 | 0.018 | 4.1% | 2.243 | 9.172 | 1.4% | 49.2% | 44.2% | 5.2% |
| Ph4 Obstacle HIGH (×2.00) | 2.295 | 1.175 | 0.172 | 3.836 | 0.637 | 0.075 | 0.575 | 1.620 | 1.556 | 0.702 | 1.055 | 0.025 | 3.4% | 0.963 | 10.091 | 1.4% | 49.9% | 45.1% | 3.6% |

#### Phase 1 — Social Force (×0.55 low / ×1.70 high)

**Expected outcome**: Reducing social force (×0.55) should allow agents to pass closer together, potentially increasing collision rate. Increasing social force (×1.70) should increase spacing and reduce collisions.

The HIGH/LOW breakdown reveals a **strong directional effect** — the most impactful CT finding:
- **LOW (×0.55)**: Collision rate increased to 0.232 (+20% from baseline 0.193). Near-miss rate jumped to 4.978 (+28%). Min distance p5 worsened (0.669→0.624 m). TTC decreased (1.116→0.932 s). Stuck rate improved to 2.5% (half baseline). Path efficiency improved substantially (0.413→0.596, +44%). Oscillation index decreased (1.642→1.128), indicating smoother paths despite more collisions.
- **HIGH (×1.70)**: Collision rate **0.094 (−51% from baseline)** — the largest collision reduction in the CT environment. Near-miss rate dropped to 2.334 (−40%). Min distance p5 improved substantially (0.669→0.786 m — best worst-case spacing in CT). TTC improved (1.116→1.566 s, +40%). Stuck rate lowest of all CT variants (1.6%). Oscillation index decreased (1.642→0.935). Intimate zone dropped to just 0.6%. Social zone expanded to 49.4%.

**Analysis**: The HIGH/LOW disaggregation completely clarifies this phase: stronger social repulsion (×1.70) produced dramatically better collision avoidance (−51%) while weaker repulsion (×0.55) worsened it (+20%). The HIGH condition created clearer "lanes" in the bidirectional flow, consistent with the laminar flow phenomenon (Feliciani et al., 2018). The combined phase mean (−15%) previously reported masked the true magnitude of the HIGH condition's −51% collision reduction. Both variants improved path efficiency over baseline, with LOW making slightly greater gains (+44% vs +26%), suggesting that even reduced social force allows more fluid navigation than the conflicted baseline.

#### Phase 2 — Goal Force (×0.60 low / ×1.60 high)

**Expected outcome**: Reducing goal force should cause agents to wander more. Increasing goal force should produce more direct paths but potentially more collisions.

The HIGH/LOW breakdown reveals a **reversed pattern** — LOW improves safety while HIGH degrades it:
- **LOW (×0.60)**: Collision rate improved to 0.107 (−45% from baseline). Near-miss rate 3.663 (−6%). Min distance p5 improved (0.669→0.737 m). TTC improved (1.116→1.489 s, +33%). Stuck rate improved (4.0%→1.6%). Path efficiency improved (0.413→0.470). Oscillation index was 1.321 (moderate).
- **HIGH (×1.60)**: Collision rate **0.231 (+20% from baseline)** — the highest collision rate in the CT environment. Near-miss rate increased to 4.056 (+4%). TTC unchanged (1.116 s). Stuck rate slightly elevated (4.7%). Oscillation index was **3.579** (highest of all CT variants by a large margin), indicating extreme heading oscillation. Acceleration was the lowest of all variants (1.289 m/s²).

**Analysis**: The directional effect is opposite to expectation: *reducing* goal force improved safety, while *increasing* it degraded safety. Lower goal force allows social forces more relative influence, producing better collision avoidance. Higher goal force overrides social repulsion, driving agents directly toward their goals through other agents. The extreme oscillation index at HIGH (3.579) suggests agents rapidly alternate between strong goal attraction and social repulsion. The combined phase mean previously reported would have shown ~0.169 collision rate — masking that LOW achieved 0.107 while HIGH achieved 0.231.

#### Phase 3 — Speed / Max Velocity (×0.75 low / ×1.25 high)

**Expected outcome**: Reducing max_vel should slow agents, reducing collision severity. Increasing max_vel increases approach speed.

The HIGH/LOW breakdown reveals **clearly separated speed-safety regimes**:
- **LOW (×0.75, max_vel ~0.96 m/s)**: Speed dropped dramatically to 1.694 m/s (−43%). Path efficiency improved to **0.832** — the highest of any CT variant, approaching GT mean (0.926). Heading jerk lowest of all CT variants (0.051). TTC improved to **3.027 s** — approaching GT mean (3.88 s). Speed CV increased (0.238→0.544), reflecting varied force-trapped vs. free-moving agents. Stuck rate 4.9% (essentially baseline). Translational jerk was lowest of all CT variants (6.606 m/s³).
- **HIGH (×1.25, max_vel ~1.60 m/s)**: Speed increased to 3.590 m/s (+20%). Path efficiency dropped to **0.255** — the worst of any CT variant. TTC plummeted to **0.628 s** (worst in CT, catastrophically low). Heading jerk highest of all CT variants (0.132). Translational jerk highest (12.437 m/s³). Collision rate increased slightly (0.193→0.208). Stuck rate 4.0% (near-baseline).

**Analysis**: The speed phase produced the widest metric spread of any CT phase. The LOW condition achieved the best path efficiency in CT (0.832) and the best TTC (3.027 s, −22% vs GT mean), confirming that slower bidirectional flow allows social forces to organise lane formation effectively. The HIGH condition produced the worst path efficiency (0.255) and worst TTC (0.628 s), demonstrating that faster agents create more turbulent flow with less time to react. Critically, the stuck rate was similar for both variants (~4–5%), indicating the max_vel floor is not the dominant effect at ×0.75 in CT — rather, the benefit is in flow organisation. The combined phase mean previously obscured that LOW produced 3.027 s TTC while HIGH produced 0.628 s — a 4.8× difference.

#### Phase 4 — Obstacle Force (×0.50 low / ×2.00 high)

**Expected outcome**: Reducing obstacle force should allow agents closer to walls. Increasing it should push agents toward centre, increasing congestion.

The HIGH/LOW breakdown reveals that **HIGH drives most of the phase variation**:
- **LOW (×0.50)**: Nearly identical to baseline across all metrics. Speed 2.972, collision 0.196, TTC 1.222, path efficiency 0.427. Oscillation index 2.243. This confirms that halving obstacle force has minimal impact in CT.
- **HIGH (×2.00)**: Speed decreased substantially (2.987→2.295, −23%). Path efficiency improved dramatically (0.413→0.637, +54%). Speed CV more than doubled (0.238→0.575). Oscillation index dropped to 0.963 (lowest of all CT variants). Heading jerk decreased (0.107→0.075). Collision rate improved slightly (0.193→0.172). Stuck rate improved (4.0%→3.4%).

**Analysis**: Doubling the obstacle force produced surprisingly positive effects: agents were pushed away from furniture, reducing furniture-trapping interactions and allowing smoother paths through the corridor's central channel. The improved path efficiency (0.637) and reduced oscillation (0.963) suggest that stronger obstacle repulsion effectively creates clear corridors. These results are still **partially confounded by the AABB multi-link obstacle bug** — the improvements may reflect stronger AABB-avoidance rather than realistic wall-avoidance.

#### Central Tunnel Summary

1. **Social force HIGH (×1.70)** produced the strongest collision reduction: **−51% collision rate** (0.094), best worst-case spacing (p5=0.786 m), and best TTC improvement (+40%). This was masked by the combined phase mean (−15%).
2. **Goal force** showed a **reversed directional effect**: LOW (×0.60) reduced collisions by 45%, while HIGH (×1.60) increased them by 20% with extreme oscillation (3.579 rev/m). Lower goal force gives social forces more relative influence.
3. **Speed LOW (×0.75)** produced the best path efficiency (0.832), best TTC (3.027 s), and lowest heading jerk (0.051). Speed HIGH (×1.25) produced the worst TTC (0.628 s) and path efficiency (0.255).
4. **Obstacle force HIGH** unexpectedly improved path efficiency (+54%) by pushing agents away from furniture traps.
5. **Baseline speed ~3.0 m/s** (well above max_vel 1.28 m/s) indicates SFM force-driven oscillation overriding velocity clamping, similar to the café. Path efficiency 0.413 confirms significant oscillation in the baseline.
6. **TTC range** 0.628–3.027 s — the LOW speed condition approaches GT mean (3.88 s), while HIGH speed is catastrophically low.
7. **Translational jerk** 6.6–12.4 m/s³ — all variants above GT mean (0.74 m/s³), reflecting force-driven acceleration artefacts.
8. **Proxemic zones show under-intimate spacing** (0.6–1.6% intimate vs GT 6.9%) — social force is too conservative even at baseline.

---

### Environment 3: Delta (80 agents, ~55m triangular, obstacle-free)

**Baseline parameters** (mean across 80 agents): SF=10.85, GF=3.27, OF=17.57, MaxVel=1.21 m/s  
**Scaling multipliers**: Social (×0.55/×1.70), Goal (×0.60/×1.60), Speed (×0.75/×1.25), Obstacle (×0.50/×2.00)  
**Configuration**: Mode 1 (CUSTOM). Cabinets removed from world to avoid AABB obstacle trapping.  
**Geometry**: 80 agents in 3 corner clusters (27/27/26), walking across the triangle to opposite corners.

| Variant | Speed μ | Speed σ | Coll Rate | NM Rate | Path Eff | Hdg Jerk | Spd CV | Accel | Min Dist μ | Min Dist p5 | TTC μ | TTC p10 | Stuck% | Osc Idx | Trans Jerk | %Intim | %Pers | %Social | %Public |
|---------|---------|---------|-----------|---------|----------|----------|--------|-------|------------|-------------|-------|---------|--------|---------|------------|--------|-------|---------|---------|
| Baseline (3 runs) | 0.723 | 0.142 | 0.136 | 5.130 | 0.998 | 0.001 | 0.133 | 0.733 | 1.081 | 0.670 | 5.898 | 0.129 | 0.0% | 0.792 | 5.634 | 0.7% | 69.9% | 29.2% | 0.2% |
| Ph1 Social LOW (×0.55) | 0.813 | 0.166 | 0.283 | 6.328 | 0.984 | 0.006 | 0.146 | 0.771 | 0.990 | 0.599 | 3.130 | 0.069 | 0.0% | 1.219 | 6.533 | 1.9% | 76.5% | 21.6% | 0.0% |
| Ph1 Social HIGH (×1.70) | 0.788 | 0.185 | 0.117 | 4.296 | 0.988 | 0.008 | 0.182 | 0.681 | 1.100 | 0.710 | 9.413 | 2.103 | 0.0% | 0.966 | 6.554 | 0.7% | 69.8% | 29.5% | 0.0% |
| Ph2 Goal LOW (×0.60) | 0.811 | 0.174 | 0.042 | 2.966 | 0.984 | 0.006 | 0.147 | 0.891 | 1.229 | 0.790 | 9.972 | 0.125 | 0.0% | 0.894 | 5.417 | 0.2% | 59.9% | 39.1% | 0.9% |
| Ph2 Goal HIGH (×1.60) | 0.846 | 0.162 | 0.274 | 6.067 | 0.974 | 0.009 | 0.129 | 0.720 | 1.000 | 0.579 | 2.623 | 0.060 | 0.0% | 0.973 | 6.081 | 1.8% | 76.9% | 21.2% | 0.0% |
| Ph3 Speed LOW (×0.75) | 0.596 | 0.129 | 0.108 | 5.561 | 0.996 | 0.001 | 0.139 | 0.653 | 1.054 | 0.696 | 8.242 | 1.307 | 0.0% | 1.065 | 3.960 | 0.6% | 73.1% | 26.4% | 0.0% |
| Ph3 Speed HIGH (×1.25) | 0.548 | 0.148 | 0.047 | 5.282 | 0.996 | 0.050 | 0.211 | 0.546 | 1.064 | 0.723 | 6.963 | 0.229 | 22.6% | 1.834 | 3.552 | 0.2% | 71.7% | 28.1% | 0.0% |
| Ph4 Obstacle LOW (×0.50) | 0.810 | 0.160 | 0.183 | 5.376 | 0.975 | 0.007 | 0.140 | 0.848 | 1.044 | 0.643 | 5.508 | 0.088 | 0.0% | 0.896 | 5.318 | 1.1% | 74.0% | 24.8% | 0.0% |
| Ph4 Obstacle HIGH (×2.00) | 0.797 | 0.160 | 0.164 | 5.390 | 0.987 | 0.005 | 0.145 | 0.931 | 1.047 | 0.646 | 5.657 | 0.091 | 0.0% | 0.848 | 5.247 | 0.9% | 73.7% | 25.3% | 0.0% |

#### Phase 1 — Social Force (×0.55 low / ×1.70 high)

The HIGH/LOW breakdown reveals a **strong directional effect**, consistent with CT:
- **LOW (×0.55)**: Collision rate **more than doubled** to 0.283 (+108% from baseline 0.136). Near-miss rate jumped to 6.328 (+23%). Min distance μ dropped (1.081→0.990 m), p5 worsened (0.670→0.599 m). TTC plummeted (5.898→3.130 s, −47%). Personal zone expanded (69.9%→76.5%) despite more collisions — agents cluster closer overall. Intimate zone nearly tripled (0.7%→1.9%).
- **HIGH (×1.70)**: Collision rate **improved to 0.117 (−14% from baseline)**. Near-miss rate decreased (5.130→4.296, −16%). Min distance p5 improved substantially (0.670→0.710 m). TTC improved to **9.413 s** — the highest of any variant in the entire study. TTC p10 jumped dramatically (0.129→2.103 s), meaning even worst-case encounters maintained >2 s safety margin. Proxemic distribution nearly identical to baseline.

**Analysis**: The directional effect is unambiguous and matches CT: stronger social repulsion (HIGH) reduces collisions while weaker repulsion (LOW) increases them. The combined phase mean previously reported (+47% collision increase) was dominated by the LOW condition, masking the HIGH condition's −14% improvement. The most striking finding is the TTC: HIGH produced 9.413 s mean TTC with 2.103 s p10 — exceptional safety margins that far exceed GT mean (3.88 s). This confirms social force as the primary controller of collision-approach dynamics in open geometries.

#### Phase 2 — Goal Force (×0.60 low / ×1.60 high)

The HIGH/LOW breakdown reveals a **dramatic reversal** — LOW massively improves safety while HIGH dramatically worsens it:
- **LOW (×0.60)**: Collision rate **dropped to 0.042 (−69% from baseline)** — the largest collision reduction in Delta and one of the largest in the study. Near-miss rate dropped to 2.966 (−42%). Min distance improved substantially (1.081→1.229 m; p5: 0.670→0.790 m — the best worst-case spacing in Delta). TTC improved to 9.972 s (second-highest in the study). Translational jerk slightly decreased (5.634→5.417). Proxemic distribution shifted strongly: social zone expanded (29.2%→39.1%), personal zone contracted (69.9%→59.9%), public zone appeared (0.9%).
- **HIGH (×1.60)**: Collision rate **doubled to 0.274 (+101% from baseline)**. Near-miss rate increased to 6.067 (+18%). Min distance worsened (1.081→1.000 m; p5: 0.670→0.579 m). TTC collapsed to 2.623 s (−56%, below GT mean). Personal zone expanded (69.9%→76.9%). Intimate zone rose (0.7%→1.8%).

**Analysis**: The reversal is counterintuitive but mechanistically clear. Reducing goal force gives social forces more relative influence over agent behaviour in the convergence zone — exactly the same mechanism found in CT Phase 2. With less goal-directed urgency (×0.60), agents yield more to social repulsion, maintaining spacing and avoiding the central pile-up. With stronger goal force (×1.60), agents charge through social barriers, producing more collisions. The combined phase mean previously obscured this: the mean would have shown collision rate ~0.158 (+16%), hiding that LOW achieved 0.042 (−69%) while HIGH produced 0.274 (+101%). **This is the most dramatic example of why HIGH/LOW disaggregation is essential for OAT analysis.**

#### Phase 3 — Speed / Max Velocity (×0.75 low / ×1.25 high)

The HIGH/LOW breakdown reveals a **counterintuitive result** driven by a catastrophic outlier:
- **LOW (×0.75, max_vel ~0.91 m/s)**: Speed 0.596 m/s (−18% from baseline). Collision rate 0.108 (−21%). Near-miss rate 5.561 (+8%). Path efficiency maintained (0.996). Stuck rate 0.0%. TTC improved to 8.242 s. TTC p10 improved dramatically (0.129→1.307 s). Translational jerk decreased (5.634→3.960 — lowest in Delta).
- **HIGH (×1.25, max_vel ~1.51 m/s)**: Speed paradoxically **lower** than LOW: 0.548 m/s (−24% from baseline). Collision rate 0.047 (−65%). Heading jerk jumped ×50 (0.001→0.050). **Stuck rate 22.6%** — entirely driven by run_15 (45.2% stuck, oscillation index 2.709) while run_14 was normal (0% stuck, osc 0.958). TTC 6.963 s. Translational jerk lowest in study (3.552 m/s³).

**Analysis**: The HIGH condition's anomalous results are dominated by run_15, where 45.2% of agents became trapped at the convergence centre. With higher max_vel (×1.25), agents approach the convergence zone faster, creating a more intense simultaneous pile-up. If the stochastic initial conditions create a particularly dense convergence (as in run_15), more agents become force-balanced and trapped. Run_14 with the same parameters was completely normal (0% stuck), demonstrating high sensitivity to initial conditions. This reveals that **higher max_vel in convergence geometries can paradoxically increase trapping risk** by creating more intense pile-ups. The previous phase-averaged analysis reported −44% collision rate and 12.8% stuck rate but attributed the stuck rate to LOW conditions — the HIGH/LOW disaggregation reveals the stuck rate came entirely from the HIGH condition.

#### Phase 4 — Obstacle Force (×0.50 low / ×2.00 high)

The HIGH/LOW breakdown shows **minimal differentiation**, as expected in an obstacle-free environment:
- **LOW (×0.50)**: Collision rate 0.183 (+35% from baseline). Near-miss rate 5.376. Min distance 1.044 m. TTC 5.508 s. Stuck rate 0.0%.
- **HIGH (×2.00)**: Collision rate 0.164 (+21%). Near-miss rate 5.390. Min distance 1.047 m. TTC 5.657 s. Stuck rate 0.0%.

**Analysis**: Both variants increased collision rate above baseline, with minimal difference between them (0.183 vs 0.164). The effects come from boundary wall forces computed by `HandleObstacles2()` on Gazebo AABBs. The LOW condition fares slightly worse because reduced wall repulsion allows agents to spread slightly more, reducing coordination at the convergence zone. The practical difference is negligible.

#### Delta Summary

1. **Goal force LOW (×0.60)** produced the largest collision reduction in Delta: **−69%** (0.042 vs baseline 0.136). Goal force HIGH (×1.60) produced the largest increase: **+101%** (0.274). This dramatic reversal was completely hidden by phase-averaged analysis.
2. **Social force** showed a clear directional effect: HIGH (×1.70) reduced collisions −14% with TTC 9.413 s (highest in study); LOW (×0.55) increased collisions +108%.
3. **Speed HIGH (×1.25)** produced a counterintuitive 22.6% stuck rate from a single catastrophic run (run_15 at 45.2% stuck). Speed LOW (×0.75) was completely stable (0% stuck) — reversing the previous interpretation.
4. **Obstacle force** produced minimal effects, as expected in an obstacle-free environment.
5. **Baseline speed 0.723 m/s** (within GT range 0.64–1.46 m/s) — Delta is the only environment producing realistic walking speeds.
6. **Path efficiency near-perfect** (0.974–0.998) across all variants — agents walk in near-straight lines in the open geometry.
7. **TTC range** 2.623–9.972 s — the widest range of any environment, spanning from below GT mean to far above it.
8. **Translational jerk** 3.6–6.6 m/s³ — above GT mean (0.74 m/s³), reflecting force-driven acceleration artefacts.
9. **Proxemic zones show dominant personal zone** (60–77%) with near-zero intimate (0.2–1.9%) and public (0.0–0.9%) — spacing is realistic but overly conservative.

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

Ranges below reflect the minimum and maximum across all HIGH/LOW variants (not just phase means).

| Metric | Café (cluttered) | Central Tunnel (corridor) | Delta (open) | GT Range |
|--------|-----------------|--------------------------|--------------|----------|
| Speed (m/s) | 2.58–3.04 | 1.69–3.59 | 0.55–0.85 | 0.64–1.46 |
| Collision Rate | 0.41–1.10 | 0.09–0.23 | 0.04–0.28 | 0.0004–0.094 |
| Path Efficiency | 0.054–0.063 | 0.255–0.832 | 0.974–0.998 | 0.87–0.97 |
| Heading Jerk | 0.676–0.836 | 0.051–0.132 | 0.001–0.050 | 0.11–0.52 |
| TTC Mean (s) | 0.090–0.406 | 0.628–3.027 | 2.623–9.972 | 2.71–6.37 |
| Stuck Rate | 0.1–0.8% | 1.6–4.9% | 0.0–22.6% | 2.0–27.7% |
| Osc Index (rev/m) | 0.58–1.32 | 0.94–3.58 | 0.79–1.83 | 1.08–2.93 |
| Trans Jerk (m/s³) | 29.8–34.8 | 6.6–12.4 | 3.6–6.6 | 0.52–1.22 |
| % Intimate | 3.1–7.8% | 0.6–1.6% | 0.2–1.9% | 0.6–23.6% |
| % Personal | 49–55% | 45–55% | 60–77% | 47–72% |
| % Social | 36–47% | 38–49% | 21–39% | 15–41% |

### Consistent Directional Effects (from HIGH/LOW disaggregation)

**Social force HIGH** produced consistent collision reduction across CT and Delta:
- Central Tunnel: collision rate −51% (0.094), best worst-case spacing (p5=0.786 m), TTC +40% (1.566 s)
- Delta: collision rate −14% (0.117), TTC highest in study (9.413 s), TTC p10 highest (2.103 s)
- Social force LOW consistently worsened collisions: CT +20% (0.232), Delta +108% (0.283)
- **Conclusion**: Increasing social force is the most reliable way to reduce collisions in the ESFM.

**Goal force LOW** produced better safety than HIGH in both CT and Delta — a consistent reversal:
- Central Tunnel: LOW collision rate −45% (0.107); HIGH +20% (0.231)
- Delta: LOW collision rate −69% (0.042); HIGH +101% (0.274)
- **Mechanism**: Reducing goal force gives social forces more relative influence, improving collision avoidance. This is the opposite of the naïve expectation.

**Speed LOW** produced improved path efficiency and TTC across CT and Delta:
- Central Tunnel: LOW path efficiency 0.832 (best in CT), TTC 3.027 s; HIGH PE 0.255 (worst), TTC 0.628 s
- Delta: LOW collision rate 0.108, stuck 0%; HIGH collision 0.047 but stuck 22.6% (run_15 catastrophe)
- Café: LOW collision 0.413 (−38%, best café result); HIGH collision 0.984 (+49%)
- **Speed (max_vel) remains the most impactful parameter**, but the HIGH/LOW breakdown reveals the mechanism is flow organisation (LOW allows smoother lane formation) rather than simply reduced kinetic energy.

**Obstacle force** effects remained environment-dependent:
- Central Tunnel: HIGH (×2.00) unexpectedly improved path efficiency +54% by pushing agents away from furniture traps
- Delta: minimal differentiation between LOW and HIGH (both slightly worse than baseline)

### TTC as the Best Discriminator

TTC cleanly separates the three environments where collision rate alone cannot:

| Environment | TTC Range (s) | vs GT Mean (3.88 s) | Assessment |
|-------------|--------------|---------------------|------------|
| Café | 0.090–0.406 | −90% to −98% | **Catastrophic** — perpetual collision crisis |
| Central Tunnel | 0.628–3.027 | −22% to −84% | **Partially realistic** — LOW speed approaches GT |
| Delta | 2.623–9.972 | −32% to +157% | **Wide range** — spans from below to far above GT |

The HIGH/LOW breakdown substantially widens the TTC range for all environments compared to phase-averaged analysis, revealing that parameter variants can push the same environment from catastrophic to near-realistic TTC.

### Translational Jerk Gap

| Environment | Trans Jerk Range (m/s³) | vs GT Mean (0.74) | Diagnosis |
|-------------|------------------------|-------------------|-----------|
| Café | 29.8–34.8 | +3,930% to +4,600% | Extreme oscillation artefact |
| Central Tunnel | 6.6–12.4 | +790% to +1,580% | Force-driven acceleration artefact |
| Delta | 3.6–6.6 | +380% to +790% | Force-driven but less extreme |

**All environments produce translational jerk far above GT** — the opposite of what the previous phase-averaged analysis reported. This reflects SFM force-driven acceleration artefacts: agents experience high-magnitude force fluctuations that produce violent speed changes. The magnitude scales with obstacle density (café > CT > Delta), confirming that force trapping drives the jerk excess.

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
| Café Baseline | 2.986 | +179% (2.8×) |
| CT Baseline | 2.987 | +179% (2.8×) |
| Delta Baseline | 0.723 | −32% |
| Delta Best (Ph3 Speed LOW) | 0.596 | −44% |
| CT Best (Ph3 Speed LOW) | 1.694 | +58% |

The café (2.986 m/s) and CT (2.987 m/s) produce near-identical speeds — both far above realistic walking speed — reflecting SFM force-driven oscillation that overrides velocity clamping (both environments have max_vel ~1.1–1.3 m/s). The measured speed captures instantaneous oscillatory velocity between conflicting forces, not forward walking speed. Path efficiency distinguishes them: café PE=0.063 (near-total trapping) vs CT PE=0.413 (partial forward progress). **Delta produces the only realistic speed (0.723 m/s)**, falling within the GT range (0.64–1.46 m/s) and matching ucy_univ_s1 (0.64 m/s). CT Ph3 Speed LOW (1.694 m/s) is the closest approach to GT from CT, though still elevated.

#### Collision Rate

| Source | Collision Rate | vs GT Mean |
|--------|---------------|------------|
| GT Mean | 0.019 | — |
| CT Baseline | 0.193 | +916% |
| Delta Baseline | 0.136 | +616% |
| Delta Best (Ph2 Goal LOW) | 0.042 | +121% |
| CT Best (Ph1 Social HIGH) | 0.094 | +395% |
| Delta Ph3 Speed HIGH | 0.047 | +147% |

All simulation collision rates are substantially above GT. The best results come from Delta Goal LOW (0.042, closest to GT) and Delta Speed HIGH (0.047). The HIGH/LOW disaggregation reveals that the previously reported "exact match" of 0.019 was a phase average that does not hold when examining individual variants.

#### TTC

| Source | TTC Mean (s) | vs GT Mean (3.88 s) |
|--------|-------------|---------------------|
| GT Mean | 3.88 | — |
| Café Baseline | 0.137 | −96% |
| CT Baseline | 1.116 | −71% |
| CT Ph3 Speed LOW | 3.027 | −22% — **best CT match** |
| Delta Baseline | 5.898 | +52% |
| Delta Ph1 Social HIGH | 9.413 | +143% |
| Delta Ph2 Goal LOW | 9.972 | +157% |

CT Ph3 Speed LOW (3.027 s) is the closest approach to GT mean TTC. Delta baseline (5.898 s) is within GT range (2.71–6.37 s). The café TTC (0.137 s) is catastrophically low — agents are in near-constant collision. The HIGH/LOW disaggregation reveals that Delta social and goal force variants push TTC far above GT (9.4–10.0 s), creating an overly safe environment.

#### Path Efficiency

| Source | Path Efficiency | vs GT Mean (0.926) |
|--------|----------------|-------------------|
| Café Baseline | 0.063 | −93% |
| CT Baseline | 0.413 | −55% |
| CT Best (Ph3 Speed LOW) | 0.832 | −10% — **closest to GT** |
| Delta Baseline | 0.998 | +8% |

CT Ph3 Speed LOW (0.832) is the closest to GT, approaching the lower GT bound (0.870). Delta is "too perfect" — agents walk in near-straight lines without adaptive corrections. The café is in complete path failure.

#### Heading Jerk and Oscillation Index

| Source | Heading Jerk | Osc Index (rev/m) | Assessment |
|--------|-------------|-------------------|------------|
| GT Mean | 0.257 | 1.72 | — |
| CT Baseline | 0.107 | 1.642 | Jerk matches UCY Zara01 exactly (0.107); Osc within 5% of GT mean |
| Delta Baseline | 0.001 | 0.792 | Both far below GT — overly smooth |
| Café Baseline | 0.676 | 1.315 | Jerk elevated by oscillation artefact |

CT produces the most realistic directional dynamics. The oscillation index near-exact match (1.642 vs GT 1.72, −5%) confirms realistic heading-correction frequency.

#### Translational Jerk

| Source | Trans Jerk (m/s³) | vs GT Mean (0.74) |
|--------|-------------------|-------------------|
| GT Mean | 0.74 | — |
| Café Baseline | 33.706 | +4,453% |
| CT Baseline | 9.691 | +1,210% |
| Delta Baseline | 5.634 | +661% |
| Delta Best (Ph3 Speed HIGH) | 3.552 | +380% |

**All environments produce translational jerk far above GT** — reflecting SFM force-driven acceleration artefacts. The magnitude scales with obstacle density and oscillation severity: café (34×) > CT (13×) > Delta (8×). This is a fundamental SFM limitation where large-magnitude force fluctuations produce violent speed changes that real pedestrians do not exhibit. The relaxation time τ = 0.5s is insufficient to filter out these high-frequency force oscillations. Reducing τ would likely increase jerk further; the root cause is force magnitudes, not response time.

#### Speed Variability (CV) and Stuck Rate

| Source | Speed CV | Stuck Rate | Assessment |
|--------|----------|-----------|------------|
| GT Mean | 0.208 | 11.8% | — |
| CT Baseline | 0.238 | 4.0% | CV within range; stuck below GT |
| Delta Baseline | 0.133 | 0.0% | CV slightly low; agents never stop |
| Delta Ph3 Speed HIGH | 0.211 | 22.6% | CV matches GT; stuck from run_15 outlier |

CT speed CV (0.238) falls within GT range (0.138–0.355). CT stuck rate (4.0%) is below GT mean. Delta Speed HIGH (0.211) matches GT CV almost exactly, though the 22.6% stuck rate reflects a single catastrophic run.

#### Minimum Inter-Agent Distance and Proxemic Zones

| Source | Min Dist μ (m) | Min Dist p5 (m) | %Intimate | %Personal | %Social |
|--------|---------------|-----------------|-----------|-----------|---------|
| GT Mean | 1.38 | 0.47 | 6.9% | 61.6% | 24.7% |
| CT Baseline | 1.591 | 0.669 | 1.4% | 51.1% | 41.6% |
| Delta Baseline | 1.081 | 0.670 | 0.7% | 69.9% | 29.2% |
| Delta Ph2 Goal LOW | 1.229 | 0.790 | 0.2% | 59.9% | 39.1% |

Both CT and Delta produce mean separations within or near GT range. All environments are under-intimate and over-social — the social force is too conservative for natural proxemic compliance.

### Summary of Best Simulation Matches (HIGH/LOW disaggregated)

| Metric | GT Target | Best Match | Environment / Variant | Error |
|--------|-----------|-----------|----------------------|-------|
| Speed | 1.07 m/s | 0.723 m/s | Delta Baseline | −32% |
| Collision Rate | 0.019 | 0.042 | Delta Ph2 Goal LOW | +121% |
| TTC Mean | 3.88 s | 3.027 s | CT Ph3 Speed LOW | **−22%** |
| Path Efficiency | 0.926 | 0.832 | CT Ph3 Speed LOW | −10% |
| Heading Jerk | 0.257 | 0.107 | CT Baseline | −58% (matches UCY Zara01) |
| Osc Index | 1.72 rev/m | 1.642 rev/m | CT Baseline | **−5% — near-exact** |
| Speed CV | 0.208 | 0.211 | Delta Ph3 Speed HIGH | **+1% — near-exact** |
| Stuck Rate | 11.8% | 4.0% | CT Baseline | −66% |
| Trans Jerk | 0.74 m/s³ | 3.552 m/s³ | Delta Ph3 Speed HIGH | +380% — all above GT |
| Min Distance | 1.38 m | 1.081 m | Delta Baseline | −22% |
| % Intimate | 6.9% | 7.1% | Café Ph3 Speed HIGH | **+3% — near-exact** |
| % Personal | 61.6% | 69.9% | Delta Baseline | +13% |
| % Social | 24.7% | 29.2% | Delta Baseline | +18% |

---

## Phase 3 — Optimal Parameter Recommendations

### Evidence Summary

Based on 53 simulation runs across three environments (15 café, 19 Central Tunnel, 19 Delta) against seven ground truth datasets with all 15+ evaluation metrics, with HIGH/LOW variant disaggregation:

1. **Speed (max_vel)** is the most impactful parameter with consistent directional effects across all environments. Café Phase 3 LOW reduced collisions by 38% (0.413), CT Phase 3 LOW achieved the best path efficiency in CT (0.832) and best TTC (3.027 s). Delta Phase 3 showed a counterintuitive result: HIGH (×1.25) produced 22.6% stuck rate in a catastrophic outlier run, while LOW (×0.75) was stable (0% stuck).

2. **Social force HIGH** is the most reliable collision reducer: CT Phase 1 HIGH produced **−51% collision rate** (0.094), the largest collision improvement in any CT variant. Delta Phase 1 HIGH reduced collisions by 14% with the highest TTC in the study (9.413 s). Social force LOW consistently worsened outcomes.

3. **Goal force** produced a consistent **reversed directional effect**: LOW improved safety in both CT (−45% collision) and Delta (−69% collision), while HIGH degraded safety. Reducing goal force gives social forces more relative influence, improving collision avoidance. This is the most important mechanistic insight from the HIGH/LOW disaggregation.

4. **Obstacle force** effects are partially confounded by the AABB implementation bug. CT HIGH (×2.00) unexpectedly improved path efficiency by 54%.

5. **Translational jerk** is above GT across all environments (3.6–34.8 vs GT 0.74 m/s³), reflecting SFM force-driven acceleration artefacts. This is the most significant systemic realism gap.

### Per-Environment Recommendations

#### Central Tunnel

- **Social force**: Increase to ×1.50–1.70 (SF ~17–20). The HIGH condition (×1.70) produced the best collision results (−51%), and the proxemic analysis shows agents are already under-intimate (0.6% vs GT 6.9%), suggesting further increase is safe.
- **Goal force**: **Reduce** to ×0.60–0.80 (GF ~2.1–2.7). The reversed directional effect shows LOW improves safety. Do not increase beyond baseline.
- **Max velocity**: Maintain at baseline (1.28 m/s) or increase slightly. The LOW condition (×0.75) improved flow organisation but the baseline speed (~3 m/s) reflects oscillation, not actual walking speed.
- **Obstacle force**: Increase to ×1.50–2.00 if AABB bug persists (pushes agents away from furniture traps). Reduce to ×0.50 if bug is fixed.

#### Delta

- **Social force**: Increase to ×1.50–1.70 (SF ~16–18). HIGH (×1.70) reduced collisions −14% and achieved TTC 9.413 s. LOW doubled collisions.
- **Goal force**: **Reduce** to ×0.60–0.80 (GF ~2.0–2.6). LOW (×0.60) produced the largest collision reduction in Delta (−69%, 0.042). **Do not increase** — HIGH (×1.60) nearly tripled collisions.
- **Max velocity**: Maintain ≥ 1.0 m/s. The baseline speed (0.723 m/s) is realistic but well below max_vel (1.21 m/s) due to social force opposition. Delta Ph3 Speed HIGH revealed catastrophic trapping sensitivity (45.2% stuck in run_15).
- **Obstacle force**: Not relevant (obstacle-free).

### General Parameter Recommendations

| Parameter | Recommended Value | Justification |
|-----------|-------------------|---------------|
| `social_force_factor` | 16–20 | HIGH (×1.70) produced best collision rates across CT and Delta; current baseline ~11 is under-conservative |
| `goal_force_factor` | 2.0–2.7 (×0.60–0.80 of baseline) | LOW improved safety consistently; HIGH worsened it in both CT and Delta |
| `obstacle_force_factor` | 8–12 (×0.50–0.70 of baseline), or ×1.50–2.00 if AABB bug persists | Depends on bug status — increase keeps agents from furniture traps |
| `max_vel` | 1.10–1.40 m/s (never below 1.0) | Within realistic human walking speed; enforces floor to prevent force trapping |
| `relaxation_time` (τ) | Investigate cautiously (requires code change) | Current τ=0.5s contributes to force oscillation artefacts; reducing τ may increase rather than decrease jerk |
| `configuration` | **1 (CUSTOM)** | Mandatory for deterministic parameter control |

### Trade-offs

1. **Social force vs. proxemics**: Higher SF reduces collisions but shifts proxemic distribution toward over-social spacing (>1.2m). The GT intimate-zone fraction (6.9%) requires allowing closer approaches than current SF levels permit. However, the data shows even HIGH SF (×1.70) only reaches 49% social zone — still within GT range (15–41%).
2. **Goal force vs. path directness**: Lower GF improves safety but may reduce path efficiency. CT data shows LOW GF improved PE (0.470 vs baseline 0.413), so this trade-off is less sharp than expected.
3. **Speed vs. flow quality**: Lower max_vel improves TTC and path efficiency (CT LOW: PE 0.832, TTC 3.027 s) but the measured speed already reflects oscillation artefacts rather than actual walking speed.
4. **Speed vs. convergence trapping**: In convergence geometries (Delta), higher max_vel can create more intense pile-ups, paradoxically increasing trapping risk (Delta run_15: 45.2% stuck with ×1.25 max_vel).

### Inconclusive Areas Requiring Further Testing

1. The interaction between social force and goal force is not captured by OAT — the reversed goal force effect may depend on social force magnitude.
2. The AABB obstacle handling bug makes all obstacle force conclusions in cluttered environments unreliable.
3. Delta speed 0.723 m/s (realistic) vs CT/café speed ~3.0 m/s (oscillation artefact) suggests the speed metric needs reinterpretation — net displacement speed would be more informative.
4. The study used 2 runs per phase variant; Delta run_15 (45.2% stuck) shows high sensitivity to initial conditions. A minimum of 5 runs per condition would improve confidence intervals.
5. All environments produce translational jerk far above GT (3.6–34.8 vs 0.74 m/s³) — the most significant unresolved realism gap, likely requiring force magnitude reduction rather than τ adjustment.

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

1. **HIGH/LOW disaggregation**: The report's results sections must present HIGH and LOW variant data separately, not combined phase means. The combined means masked the most important findings (e.g., Delta Goal LOW −69% collision hidden by mean +16%).
2. **Corrected numerical data**: Multiple values in previous report versions used incorrect data. All numerical claims must match the verified JSON source data used in this analysis.
3. **Translational jerk direction**: Previous versions may have claimed jerk was below GT. The corrected data shows all environments produce jerk far above GT (3.6–34.8 vs 0.74 m/s³).
4. **Speed interpretation**: CT and café speeds (~3 m/s) reflect oscillation artefacts, not walking speed. This must be explained explicitly.
5. **Delta stuck rate source**: Previous analysis attributed stuck rate to LOW speed conditions. The corrected data shows the 22.6% stuck rate came from the HIGH condition (run_15 catastrophe).
6. **Goal force reversed effect**: The finding that reducing goal force improves safety (opposite of naïve expectation) must be prominently discussed.

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
