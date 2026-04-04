#!/usr/bin/env python3
"""
Ground Truth Crowd Dynamics Analyzer
=====================================
Extracts benchmark metrics from ETH/UCY pedestrian datasets for comparison
against HuNavSim simulation runs.

Datasets supported (all true_pos_.csv format):
  - ETH (Hotel, Univ)
  - UCY (Zara01/02/03, Students001/003)

Strategy
--------
We produce three tiers of metrics:

1. POPULATION-LEVEL (one number per scene)
   - Mean/std speed, density-flow (fundamental diagram), collision rate
   - These reveal whether the *distribution* of crowd behaviour matches reality.

2. AGENT-LEVEL (per-pedestrian trajectory)
   - Path efficiency, speed variability, heading jerk (smoothness)
   - These reveal whether *individual* agent motion is realistic.

3. PAIRWISE / GROUP-LEVEL
   - Minimum inter-agent distance distribution, group cohesion,
     relative speed between neighbours
   - These reveal whether *social forces* (avoidance, grouping) are realistic.

Output
------
  - JSON file with all scalar metrics per dataset (machine-readable)
  - PNG plots for visual comparison (distributions, fundamental diagram)
  - Console summary table

Usage
-----
  python3 crowd_dynamics_evaluator.py                       # analyze all datasets
  python3 crowd_dynamics_evaluator.py --dataset eth_univ    # analyze one dataset
  python3 crowd_dynamics_evaluator.py --sim-json <path>     # compare against simulation
"""

import argparse
import json
import os
import sys
from dataclasses import dataclass, field, asdict
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    HAS_MPL = True
except ImportError:
    HAS_MPL = False
    print("[WARN] matplotlib not found – plots will be skipped. Install with: pip3 install matplotlib")

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
SCRIPT_DIR = Path(__file__).resolve().parent
DATASETS_DIR = SCRIPT_DIR.parent / "DATASETS"

# Each entry: (label, relative_path_from_DATASETS_DIR, dt_seconds)
DATASET_REGISTRY = [
    ("eth_hotel",  "eth/hotel/true_pos_.csv",             0.4),
    ("eth_univ",   "eth/univ/true_pos_.csv",              0.4),
    ("ucy_zara01", "ucy/zara/zara01/true_pos_.csv",       0.4),
    ("ucy_zara02", "ucy/zara/zara02/true_pos_.csv",       0.4),
    ("ucy_zara03", "ucy/zara/zara03/true_pos_.csv",       0.4),
    ("ucy_univ_s1","ucy/univ/students001/true_pos_.csv",  0.4),
    ("ucy_univ_s3","ucy/univ/students003/true_pos_.csv",  0.4),
]

# Group annotations
GROUP_FILES = {
    "eth_univ":   "eth/univ/groups.txt",
    "eth_hotel":  "eth/hotel/groups.txt",
}

# Collision / near-miss thresholds (meters)
COLLISION_RADIUS = 0.5      # body-body overlap (shoulder width ~0.45m)
NEAR_MISS_RADIUS = 1.0      # uncomfortably close

# Fundamental diagram measurement area radius (meters)
FD_AREA_RADIUS = 4.0

# ---------------------------------------------------------------------------
# Data loading
# ---------------------------------------------------------------------------

@dataclass
class TrajectoryData:
    """Unified trajectory representation."""
    frames: np.ndarray       # (N,) int – frame indices
    ped_ids: np.ndarray      # (N,) int – pedestrian IDs
    positions: np.ndarray    # (N, 2) float – x, y in meters
    velocities: np.ndarray   # (N, 2) float – vx, vy in m/s  (computed if not given)
    dt: float                # seconds between consecutive frames
    groups: List[List[int]] = field(default_factory=list)  # group membership


def load_truepos(filepath: str, dt: float) -> TrajectoryData:
    """Load true_pos_.csv format:
    Row 0 = frame numbers (integers as floats)
    Row 1 = pedestrian IDs
    Row 2 = x positions (meters)
    Row 3 = y positions (meters)
    Each column is one observation.
    """
    with open(filepath) as f:
        import csv
        rows = list(csv.reader(f))

    n = len(rows[0])
    frames = np.array([int(float(x)) for x in rows[0]])
    ped_ids = np.array([int(float(x)) for x in rows[1]])
    x = np.array([float(v) for v in rows[2]])
    y = np.array([float(v) for v in rows[3]])
    positions = np.column_stack([x, y])

    # Compute velocities via finite differences per pedestrian.
    # The frame-step varies by dataset , so we detect
    # it from the data rather than hardcoding.
    unique_frames = np.unique(frames)
    median_frame_step = float(np.median(np.diff(unique_frames))) if len(unique_frames) > 1 else 1.0

    velocities = np.zeros_like(positions)
    for pid in np.unique(ped_ids):
        mask = ped_ids == pid
        idx = np.where(mask)[0]
        if len(idx) < 2:
            continue
        # sort by frame
        order = np.argsort(frames[idx])
        sorted_idx = idx[order]
        sorted_frames = frames[sorted_idx]
        sorted_pos = positions[sorted_idx]

        for k in range(1, len(sorted_idx)):
            df = sorted_frames[k] - sorted_frames[k - 1]
            if df == 0:
                df = 1
            # Convert frame diff to seconds: (frame_diff / median_step) * dt
            actual_dt = df * dt / median_frame_step
            # fallback: if that gives weird dt, use raw dt
            if actual_dt <= 0 or actual_dt > 10:
                actual_dt = dt
            velocities[sorted_idx[k]] = (sorted_pos[k] - sorted_pos[k - 1]) / actual_dt
        # first point gets same velocity as second
        if len(sorted_idx) > 1:
            velocities[sorted_idx[0]] = velocities[sorted_idx[1]]

    return TrajectoryData(frames=frames, ped_ids=ped_ids,
                          positions=positions, velocities=velocities, dt=dt)


def load_groups(filepath: str) -> List[List[int]]:
    """Parse EWAP groups.txt – each line is a space-separated list of ped IDs."""
    groups = []
    with open(filepath) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            ids = [int(x) for x in line.split()]
            if len(ids) >= 2:
                groups.append(ids)
    return groups


# ---------------------------------------------------------------------------
# Metric computation
# ---------------------------------------------------------------------------

@dataclass
class CrowdMetrics:
    """All computed metrics for one dataset.

    Metrics are organized into three tiers that test different aspects of
    crowd-simulation realism:

    ═══════════════════════════════════════════════════════════════════════
    TIER 1 — POPULATION-LEVEL  (one scalar per scene)
    ═══════════════════════════════════════════════════════════════════════
    These describe the *macroscopic* distribution of motion across the
    entire recording.  They answer: "Does the simulated crowd *look like*
    a real crowd overall?"

    speed_mean / speed_std / speed_median / speed_p25 / speed_p75 / speed_p95
        Instantaneous walking speed ||v|| for every observation where the
        agent is actually moving (0.05 < ||v|| < 5.0 m/s).
        • Level: population (all observations pooled)
        • Unit: m/s
        • Typical real-world range: mean 1.0–1.5 m/s, std 0.2–0.5 m/s
        • What mismatch means:
            - Mean too high → SFM desired_velocity or max_vel too large
            - Std too low → agents are too homogeneous (all same speed)
            - Shape (histogram) is usually unimodal; bimodal hints at
              two distinct populations (e.g. tourists vs commuters)

    collision_rate / near_miss_rate / total_collisions / total_near_misses
        A *collision* is defined as any agent whose nearest neighbour at
        that frame is closer than COLLISION_RADIUS (0.5 m, roughly
        shoulder width).  A *near-miss* uses NEAR_MISS_RADIUS (1.0 m,
        personal-space bubble in Western cultures).
        Counting: At each frame, for each agent, we check the minimum
        distance to any other agent.  Each such agent-frame pair
        contributes at most one event.
        • Level: population (aggregated over all frames)
        • Unit: events / agent / second
        • Typical range: collisions ≈ 0 in uncrowded, up to ~0.09 in
          very dense (ucy_univ_s1); near-misses 0.01–0.23
        • What mismatch means:
            - Too many collisions → social repulsion force is too weak
            - Too few near-misses → agents keep too much distance
              (over-tuned repulsion)

    mean_density / mean_flow
        Measured inside a circle of radius FD_AREA_RADIUS (4 m) centred
        on the scene mean position.
        density = count_in_area / π·r²   [agents/m²]
        flow    = density × mean_speed   [agents/m/s]
        Together they form the *fundamental diagram* of pedestrian traffic
        (Seyfried et al., 2005).
        • Level: population (one sample per frame)
        • Unit: agents/m²; agents/m/s
        • What mismatch means:
            - Flow should rise linearly at low density, then saturate and
              drop at high density.  If sim flow never drops, congestion
              behaviour is unrealistic.

    ═══════════════════════════════════════════════════════════════════════
    TIER 2 — AGENT-LEVEL  (one value per pedestrian, then aggregated)
    ═══════════════════════════════════════════════════════════════════════
    These characterise the quality of *individual* trajectories.  They
    answer: "Does each simulated agent move like a real person?"

    path_efficiency_mean / path_efficiency_std
        Ratio of straight-line displacement to total arc length:
        PE = ||pos_end − pos_start|| / Σ ||Δpos||
        • Level: per agent → mean/std over all agents with ≥3 obs and
          total path > 0.1 m
        • Unit: dimensionless, ∈ (0, 1]
        • Typical range: 0.87–0.97
        • What mismatch means:
            - Too low (< 0.85) → agents wander, oscillate, or take
              large detours (poor goal navigation or unstable SFM)
            - Too high (> 0.98) → agents walk in perfectly straight
              lines ignoring obstacles

    heading_jerk_mean / heading_jerk_std
        Third discrete derivative of the heading angle θ = atan2(vy, vx).
        Computed as: Δθ → ΔΔθ → ΔΔΔθ, each wrapped to [−π, π].
        Measures *smoothness* of direction changes.
        • Level: per agent (mean |ΔΔΔθ| along trajectory) → aggregated
        • Unit: rad/frame³ (proxy for rad/s³; multiply by 1/dt³ for SI)
        • Typical range: 0.1–0.5 rad/frame³
        • What mismatch means:
            - Too high → agents oscillate or zig-zag (classic SFM
              artefact when opposing forces are poorly balanced)
            - Too low → agents never change direction (no avoidance)
        • Requires ≥4 observations per agent.

    speed_variability_mean / speed_variability_std
        Coefficient of variation (CV) of speed along each agent's path:
        CV = std(speed) / mean(speed)
        • Level: per agent → aggregated
        • Unit: dimensionless
        • Typical range: 0.1–0.35
        • What mismatch means:
            - Too low → robotic constant-speed (no acceleration/braking)
            - Too high → jerky start-stop behaviour

    acceleration_mean / acceleration_std
        Magnitude of velocity change between consecutive frames:
        a = ||v(t+1) − v(t)|| / Δt
        Filtered to < 20 m/s² to remove noise.
        • Level: per transition → pooled over all agents
        • Unit: m/s²
        • Typical range: mean 0.1–0.7 m/s²
        • What mismatch means:
            - Too high → abrupt speed/direction changes (damping issue)

    ═══════════════════════════════════════════════════════════════════════
    TIER 3 — PAIRWISE / GROUP-LEVEL
    ═══════════════════════════════════════════════════════════════════════
    These test *social interactions*: avoidance, grouping, spatial
    awareness.  They answer: "Do simulated agents interact like real
    people?"

    min_inter_agent_dist_mean / _std / _p5 / _p25
        For each agent at each frame, the Euclidean distance to the
        nearest other agent.  These are pooled across all agent-frame
        pairs where ≥2 agents coexist.  The 5th percentile captures the
        *tightest* encounters.
        • Level: pairwise (per agent-frame)
        • Unit: meters
        • Typical range: mean 0.8–1.9 m; p5 0.3–0.6 m
        • What mismatch means:
            - Mean too low → agents don't keep enough distance
            - P5 too low → near-collisions are too frequent
            - Too high → over-conservative avoidance

    num_groups / group_size_mean
        Number of walking groups and their average size.
        Groups are defined by annotation files (EWAP groups.txt).
        • Level: scene metadata
        • Unit: count / persons per group

    group_cohesion_mean / group_cohesion_std
        Mean pairwise Euclidean distance between members of the same
        group, measured at each frame where ≥2 members are present.
        • Level: per group per frame → aggregated
        • Unit: meters
        • Typical range: 0.7–1.1 m (normal walking abreast)
        • What mismatch means:
            - Too tight (< 0.5 m) → group attraction force too strong
            - Too loose (> 1.5 m) → group force too weak or absent

    group_speed_std_mean
        Standard deviation of speeds among group members at each frame.
        • Level: per group per frame → averaged
        • Unit: m/s
        • Typical range: 0.04–0.09 m/s
        • What mismatch means:
            - Too high → group members don't synchronise speed
    """
    dataset_name: str = ""

    # ── Population-level ──
    num_pedestrians: int = 0
    num_frames: int = 0
    duration_seconds: float = 0.0

    speed_mean: float = 0.0          # m/s – mean of ||v|| over all valid observations
    speed_std: float = 0.0           # m/s – std dev of speed distribution
    speed_median: float = 0.0        # m/s – median speed
    speed_p25: float = 0.0           # m/s – 25th percentile of speed
    speed_p75: float = 0.0           # m/s – 75th percentile  
    speed_p95: float = 0.0           # m/s – 95th percentile (tail behaviour)

    collision_rate: float = 0.0      # collisions per agent per second
    near_miss_rate: float = 0.0      # near-misses per agent per second
    total_collisions: int = 0        # raw count of agent-frame collision events
    total_near_misses: int = 0       # raw count of agent-frame near-miss events

    mean_density: float = 0.0        # agents/m² in measurement area (r=4m)
    mean_flow: float = 0.0           # agents/m/s — density × mean speed

    # ── Agent-level (aggregated) ──
    path_efficiency_mean: float = 0.0   # dimensionless ∈ (0,1] — displacement / arc length
    path_efficiency_std: float = 0.0

    heading_jerk_mean: float = 0.0      # rad/frame³ — directional smoothness
    heading_jerk_std: float = 0.0

    speed_variability_mean: float = 0.0  # dimensionless — CV(speed) per agent
    speed_variability_std: float = 0.0

    acceleration_mean: float = 0.0       # m/s² — magnitude of velocity change
    acceleration_std: float = 0.0

    # ── Pairwise / Group-level ──
    min_inter_agent_dist_mean: float = 0.0   # meters — nearest-neighbour distance
    min_inter_agent_dist_std: float = 0.0
    min_inter_agent_dist_p5: float = 0.0     # meters — 5th percentile (closest encounters)
    min_inter_agent_dist_p25: float = 0.0    # meters — 25th percentile

    # Group metrics (if group data available)
    num_groups: int = 0                  # number of annotated walking groups
    group_size_mean: float = 0.0         # persons per group
    group_cohesion_mean: float = 0.0     # meters — mean intra-group pairwise distance
    group_cohesion_std: float = 0.0
    group_speed_std_mean: float = 0.0    # m/s — speed variance within groups

    # ── V2 Metrics (methodology success criteria) ──
    # Time-to-Collision (TTC) — linear extrapolation of current velocities
    ttc_mean: float = 0.0               # seconds — mean TTC across all agent-pair-frame events
    ttc_median: float = 0.0             # seconds — median TTC
    ttc_p10: float = 0.0               # seconds — 10th percentile (most dangerous encounters)

    # Stuck rate — fraction of agent-frames where agent is effectively stationary
    stuck_rate: float = 0.0             # dimensionless ∈ [0,1] — agent-frames with speed < 0.05 m/s

    # Oscillation index — heading reversals per meter of path traveled
    oscillation_index_mean: float = 0.0  # reversals/m — mean across agents
    oscillation_index_std: float = 0.0   # reversals/m — std across agents

    # Translational jerk — third derivative of position magnitude
    translational_jerk_mean: float = 0.0  # m/s³ — smoothness of speed changes
    translational_jerk_std: float = 0.0   # m/s³

    # Proxemic zone distribution — fraction of agent-frames in each Hall (1966) zone
    proxemic_intimate_fraction: float = 0.0   # min_dist < 0.45 m
    proxemic_personal_fraction: float = 0.0   # 0.45 ≤ min_dist < 1.2 m
    proxemic_social_fraction: float = 0.0     # 1.2 ≤ min_dist < 3.6 m
    proxemic_public_fraction: float = 0.0     # min_dist ≥ 3.6 m

    # ── Distributions (stored for plotting, not in JSON summary) ──
    speed_distribution: list = field(default_factory=list)
    min_distance_distribution: list = field(default_factory=list)
    density_flow_points: list = field(default_factory=list)   # [(density, flow), ...]
    acceleration_distribution: list = field(default_factory=list)


def compute_metrics(td: TrajectoryData, dataset_name: str) -> CrowdMetrics:
    """Compute all crowd dynamics metrics from trajectory data."""
    m = CrowdMetrics(dataset_name=dataset_name)

    unique_peds = np.unique(td.ped_ids)
    unique_frames = np.unique(td.frames)
    m.num_pedestrians = len(unique_peds)
    m.num_frames = len(unique_frames)
    m.duration_seconds = (unique_frames.max() - unique_frames.min()) * td.dt / _frame_step(unique_frames)

    # --- Speed distribution (all observations) ---
    speeds = np.linalg.norm(td.velocities, axis=1)
    # Filter out unrealistic speeds (> 5 m/s indicates noise)
    valid_speed_mask = (speeds > 0.05) & (speeds < 5.0)
    valid_speeds = speeds[valid_speed_mask]

    if len(valid_speeds) > 0:
        m.speed_mean = float(np.mean(valid_speeds))
        m.speed_std = float(np.std(valid_speeds))
        m.speed_median = float(np.median(valid_speeds))
        m.speed_p25 = float(np.percentile(valid_speeds, 25))
        m.speed_p75 = float(np.percentile(valid_speeds, 75))
        m.speed_p95 = float(np.percentile(valid_speeds, 95))
        m.speed_distribution = valid_speeds.tolist()

    # --- Per-agent metrics ---
    path_efficiencies = []
    heading_jerks = []
    speed_cvs = []
    accels_all = []
    oscillation_indices = []      # V2: heading reversals per meter
    translational_jerks = []      # V2: m/s³ per agent
    total_obs = 0                 # V2: total observations for stuck rate
    stuck_obs = 0                 # V2: observations with speed < 0.05 m/s

    for pid in unique_peds:
        mask = td.ped_ids == pid
        idx = np.where(mask)[0]
        if len(idx) < 3:
            # Still count for stuck rate even with few observations
            total_obs += len(idx)
            pid_spd = np.linalg.norm(td.velocities[idx], axis=1)
            stuck_obs += int(np.sum(pid_spd < 0.05))
            continue

        order = np.argsort(td.frames[idx])
        sorted_idx = idx[order]
        pos = td.positions[sorted_idx]
        vel = td.velocities[sorted_idx]
        spd = np.linalg.norm(vel, axis=1)
        frm = td.frames[sorted_idx]

        # V2: Stuck rate accumulation
        total_obs += len(sorted_idx)
        stuck_obs += int(np.sum(spd < 0.05))

        # Path efficiency
        straight_line = np.linalg.norm(pos[-1] - pos[0])
        segment_lengths = np.linalg.norm(np.diff(pos, axis=0), axis=1)
        actual_path = np.sum(segment_lengths)
        if actual_path > 0.1:
            path_efficiencies.append(straight_line / actual_path)

        # Speed variability (coefficient of variation)
        valid_spd = spd[(spd > 0.05) & (spd < 5.0)]
        if len(valid_spd) > 2 and np.mean(valid_spd) > 0.05:
            speed_cvs.append(float(np.std(valid_spd) / np.mean(valid_spd)))

        # Acceleration
        agent_accels = []
        for k in range(1, len(sorted_idx)):
            df = frm[k] - frm[k - 1]
            if df <= 0:
                continue
            local_dt = df * td.dt / _frame_step(unique_frames)
            if local_dt <= 0 or local_dt > 10:
                local_dt = td.dt
            acc = np.linalg.norm(vel[k] - vel[k - 1]) / local_dt
            if acc < 20:  # filter noise
                accels_all.append(acc)
                agent_accels.append(acc)

        # V2: Translational jerk (d(acceleration)/dt)
        if len(agent_accels) >= 2:
            agent_accels_arr = np.array(agent_accels)
            # Jerk = change in acceleration magnitude / dt
            frame_dt = td.dt  # approximate uniform dt
            d_acc = np.abs(np.diff(agent_accels_arr)) / frame_dt
            d_acc = d_acc[d_acc < 100]  # filter extreme outliers
            if len(d_acc) > 0:
                translational_jerks.append(float(np.mean(d_acc)))

        # Heading jerk (smoothness of direction changes)
        headings = np.arctan2(vel[:, 1], vel[:, 0])
        if len(headings) >= 4:
            # angular velocity
            dh = np.diff(headings)
            # wrap to [-pi, pi]
            dh = (dh + np.pi) % (2 * np.pi) - np.pi
            # angular acceleration
            ddh = np.diff(dh)
            ddh = (ddh + np.pi) % (2 * np.pi) - np.pi
            # angular jerk
            dddh = np.diff(ddh)
            dddh = (dddh + np.pi) % (2 * np.pi) - np.pi
            valid_jerk = np.abs(dddh)
            valid_jerk = valid_jerk[valid_jerk < 50]  # filter outliers
            if len(valid_jerk) > 0:
                heading_jerks.append(float(np.mean(valid_jerk)))

        # V2: Oscillation index (heading reversals per meter)
        if len(headings) >= 3 and actual_path > 0.1:
            dh = np.diff(headings)
            dh = (dh + np.pi) % (2 * np.pi) - np.pi
            # A reversal occurs when the sign of angular velocity changes
            sign_changes = np.sum(np.abs(np.diff(np.sign(dh))) > 0)
            oscillation_indices.append(float(sign_changes / actual_path))

    if path_efficiencies:
        m.path_efficiency_mean = float(np.mean(path_efficiencies))
        m.path_efficiency_std = float(np.std(path_efficiencies))
    if heading_jerks:
        m.heading_jerk_mean = float(np.mean(heading_jerks))
        m.heading_jerk_std = float(np.std(heading_jerks))
    if speed_cvs:
        m.speed_variability_mean = float(np.mean(speed_cvs))
        m.speed_variability_std = float(np.std(speed_cvs))
    if accels_all:
        m.acceleration_mean = float(np.mean(accels_all))
        m.acceleration_std = float(np.std(accels_all))
        m.acceleration_distribution = accels_all

    # V2: Stuck rate
    if total_obs > 0:
        m.stuck_rate = float(stuck_obs / total_obs)

    # V2: Oscillation index
    if oscillation_indices:
        m.oscillation_index_mean = float(np.mean(oscillation_indices))
        m.oscillation_index_std = float(np.std(oscillation_indices))

    # V2: Translational jerk
    if translational_jerks:
        m.translational_jerk_mean = float(np.mean(translational_jerks))
        m.translational_jerk_std = float(np.std(translational_jerks))

    # --- Pairwise: minimum inter-agent distances, TTC, proxemic zones ---
    min_dists_all = []
    collisions = 0
    near_misses = 0
    total_pair_checks = 0
    ttc_values = []             # V2: all valid TTC values
    prox_intimate = 0           # V2: proxemic zone counters
    prox_personal = 0
    prox_social = 0
    prox_public = 0

    for frame in unique_frames:
        mask = td.frames == frame
        pos_f = td.positions[mask]
        vel_f = td.velocities[mask]
        n = len(pos_f)
        if n < 2:
            continue
        for i in range(n):
            dists = np.linalg.norm(pos_f[i] - pos_f, axis=1)
            dists[i] = np.inf  # exclude self
            min_d = np.min(dists)
            nearest_j = np.argmin(dists)
            min_dists_all.append(min_d)
            total_pair_checks += 1
            if min_d < COLLISION_RADIUS:
                collisions += 1
            if min_d < NEAR_MISS_RADIUS:
                near_misses += 1

            # V2: Proxemic zone classification (Hall, 1966)
            if min_d < 0.45:
                prox_intimate += 1
            elif min_d < 1.2:
                prox_personal += 1
            elif min_d < 3.6:
                prox_social += 1
            else:
                prox_public += 1

            # V2: Time-to-Collision (linear extrapolation)
            # TTC = time until ||p_i + v_i*t - (p_j + v_j*t)|| = COLLISION_RADIUS
            # Solved as quadratic in t: a*t² + b*t + c = 0
            # where dp = p_i - p_j, dv = v_i - v_j
            dp = pos_f[i] - pos_f[nearest_j]
            dv = vel_f[i] - vel_f[nearest_j]
            a = np.dot(dv, dv)
            b = 2.0 * np.dot(dp, dv)
            c = np.dot(dp, dp) - COLLISION_RADIUS ** 2
            if a > 1e-12:  # agents have relative motion
                discriminant = b * b - 4.0 * a * c
                if discriminant >= 0:
                    sqrt_disc = np.sqrt(discriminant)
                    t1 = (-b - sqrt_disc) / (2.0 * a)
                    t2 = (-b + sqrt_disc) / (2.0 * a)
                    # We want the smallest positive t (time until collision)
                    candidates = [t for t in [t1, t2] if 0 < t < 30.0]
                    if candidates:
                        ttc_values.append(min(candidates))

    if min_dists_all:
        min_dists_arr = np.array(min_dists_all)
        m.min_inter_agent_dist_mean = float(np.mean(min_dists_arr))
        m.min_inter_agent_dist_std = float(np.std(min_dists_arr))
        m.min_inter_agent_dist_p5 = float(np.percentile(min_dists_arr, 5))
        m.min_inter_agent_dist_p25 = float(np.percentile(min_dists_arr, 25))
        m.min_distance_distribution = min_dists_arr.tolist()

    m.total_collisions = collisions
    m.total_near_misses = near_misses
    if m.num_pedestrians > 0 and m.duration_seconds > 0:
        m.collision_rate = collisions / m.num_pedestrians / m.duration_seconds
        m.near_miss_rate = near_misses / m.num_pedestrians / m.duration_seconds

    # V2: TTC aggregation
    if ttc_values:
        ttc_arr = np.array(ttc_values)
        m.ttc_mean = float(np.mean(ttc_arr))
        m.ttc_median = float(np.median(ttc_arr))
        m.ttc_p10 = float(np.percentile(ttc_arr, 10))

    # V2: Proxemic zone fractions
    if total_pair_checks > 0:
        m.proxemic_intimate_fraction = float(prox_intimate / total_pair_checks)
        m.proxemic_personal_fraction = float(prox_personal / total_pair_checks)
        m.proxemic_social_fraction = float(prox_social / total_pair_checks)
        m.proxemic_public_fraction = float(prox_public / total_pair_checks)

    # --- Density-Flow fundamental diagram ---
    # Measure in a circle around the scene center
    center = np.mean(td.positions, axis=0)
    density_flow_pts = []
    for frame in unique_frames:
        mask = td.frames == frame
        pos_f = td.positions[mask]
        vel_f = td.velocities[mask]

        # agents within measurement area
        dists_to_center = np.linalg.norm(pos_f - center, axis=1)
        in_area = dists_to_center < FD_AREA_RADIUS
        n_in = np.sum(in_area)
        area = np.pi * FD_AREA_RADIUS ** 2

        density = n_in / area  # agents/m²
        if n_in > 0:
            mean_speed = float(np.mean(np.linalg.norm(vel_f[in_area], axis=1)))
            flow = density * mean_speed  # agents/m/s
        else:
            mean_speed = 0.0
            flow = 0.0

        density_flow_pts.append((float(density), float(flow), float(mean_speed)))

    if density_flow_pts:
        df_arr = np.array(density_flow_pts)
        m.mean_density = float(np.mean(df_arr[:, 0]))
        m.mean_flow = float(np.mean(df_arr[:, 1]))
        m.density_flow_points = [(d, f) for d, f, _ in density_flow_pts]

    # --- Group metrics ---
    if td.groups:
        m.num_groups = len(td.groups)
        m.group_size_mean = float(np.mean([len(g) for g in td.groups]))

        cohesions = []
        group_speed_stds = []
        for group in td.groups:
            # For each frame, compute intra-group distance & speed variance
            for frame in unique_frames[::5]:  # sample every 5th frame for speed
                mask = td.frames == frame
                fids = td.ped_ids[mask]
                fpos = td.positions[mask]
                fvel = td.velocities[mask]

                members_in_frame = [pid for pid in group if pid in fids]
                if len(members_in_frame) < 2:
                    continue

                member_pos = []
                member_spd = []
                for pid in members_in_frame:
                    idx = np.where((fids == pid))[0]
                    if len(idx) > 0:
                        member_pos.append(fpos[idx[0]])
                        member_spd.append(np.linalg.norm(fvel[idx[0]]))

                if len(member_pos) >= 2:
                    member_pos = np.array(member_pos)
                    # mean pairwise distance
                    dists = []
                    for i in range(len(member_pos)):
                        for j in range(i + 1, len(member_pos)):
                            dists.append(np.linalg.norm(member_pos[i] - member_pos[j]))
                    cohesions.append(np.mean(dists))
                    group_speed_stds.append(np.std(member_spd))

        if cohesions:
            m.group_cohesion_mean = float(np.mean(cohesions))
            m.group_cohesion_std = float(np.std(cohesions))
        if group_speed_stds:
            m.group_speed_std_mean = float(np.mean(group_speed_stds))

    return m


def _frame_step(unique_frames: np.ndarray) -> float:
    """Estimate the frame step (e.g. 6 for ETH univ, 10 for UCY)."""
    if len(unique_frames) < 2:
        return 1.0
    diffs = np.diff(unique_frames)
    diffs = diffs[diffs > 0]
    if len(diffs) == 0:
        return 1.0
    return float(np.median(diffs))


# ---------------------------------------------------------------------------
# Comparison against simulation
# ---------------------------------------------------------------------------

def compare_metrics(gt: Dict[str, CrowdMetrics], sim: Dict[str, CrowdMetrics]) -> dict:
    """
    Produce a comparison report. Returns a dict of relative errors for scalar metrics.
    """
    comparison = {}
    scalar_keys = [
        "speed_mean", "speed_std", "speed_median",
        "collision_rate", "near_miss_rate",
        "path_efficiency_mean", "heading_jerk_mean",
        "speed_variability_mean", "acceleration_mean",
        "min_inter_agent_dist_mean", "min_inter_agent_dist_p5",
        "mean_density", "mean_flow",
        "group_cohesion_mean",
        # V2 metrics
        "ttc_mean", "ttc_median", "ttc_p10",
        "stuck_rate",
        "oscillation_index_mean",
        "translational_jerk_mean",
        "proxemic_intimate_fraction", "proxemic_personal_fraction",
        "proxemic_social_fraction", "proxemic_public_fraction",
    ]

    for ds_name in sim:
        if ds_name not in gt:
            continue
        gt_m = gt[ds_name]
        sim_m = sim[ds_name]
        ds_cmp = {}
        for key in scalar_keys:
            gt_val = getattr(gt_m, key, 0.0)
            sim_val = getattr(sim_m, key, 0.0)
            if abs(gt_val) > 1e-9:
                rel_err = (sim_val - gt_val) / abs(gt_val)
                ds_cmp[key] = {
                    "ground_truth": round(gt_val, 4),
                    "simulation": round(sim_val, 4),
                    "relative_error": round(rel_err, 4),
                    "percent_error": round(abs(rel_err) * 100, 1),
                }
            else:
                ds_cmp[key] = {
                    "ground_truth": round(gt_val, 4),
                    "simulation": round(sim_val, 4),
                    "relative_error": None,
                    "percent_error": None,
                }
        comparison[ds_name] = ds_cmp

    return comparison


# ---------------------------------------------------------------------------
# Plotting
# ---------------------------------------------------------------------------

# Consistent colour palette
_ENV_COLORS = {"cafe": "#E07A5F", "central_tunnel": "#3D405B", "delta": "#81B29A"}
_PHASE_COLORS = {
    "baseline": "#4A90D9",
    "phase1_social": "#E07A5F",
    "phase2_goal": "#F2CC8F",
    "phase3_speed": "#81B29A",
    "phase4_obstacle": "#6D597A",
}
_GT_COLOR = "#3D405B"
_SIM_COLOR = "#E07A5F"

# Hall (1966) proxemic zone boundaries and colors
_PROX_BOUNDS = [0, 0.45, 1.2, 3.6]
_PROX_LABELS = ["Intimate\n(<0.45m)", "Personal\n(0.45–1.2m)", "Social\n(1.2–3.6m)", "Public\n(≥3.6m)"]
_PROX_COLORS = ["#E07A5F", "#F2CC8F", "#81B29A", "#3D405B"]


def plot_metrics(all_metrics: Dict[str, CrowdMetrics], output_dir: Path,
                 sim_metrics: Optional[Dict[str, CrowdMetrics]] = None):
    """Generate per-phase plots: one scalar dashboard + one distribution dashboard.

    Replaces the previous 4 separate plots with 2 consolidated figures that
    cover all V1 and V2 metrics.
    """
    if not HAS_MPL:
        return

    output_dir.mkdir(parents=True, exist_ok=True)
    names = list(all_metrics.keys())
    n = len(names)
    if n == 0:
        return

    # ── Figure 1: Complete Scalar Metrics Dashboard (6×6 = 36 panels) ────
    metric_specs = [
        # Row 1 — Speed
        ("speed_mean",                 "Speed Mean (m/s)"),
        ("speed_std",                  "Speed Std (m/s)"),
        ("speed_median",               "Speed Median (m/s)"),
        ("speed_p25",                  "Speed p25 (m/s)"),
        ("speed_p75",                  "Speed p75 (m/s)"),
        ("speed_p95",                  "Speed p95 (m/s)"),
        # Row 2 — Safety
        ("collision_rate",             "Collision Rate"),
        ("near_miss_rate",             "Near-Miss Rate"),
        ("total_collisions",           "Total Collisions"),
        ("total_near_misses",          "Total Near-Misses"),
        ("ttc_mean",                   "TTC Mean (s)"),
        ("ttc_p10",                    "TTC p10 (s)"),
        # Row 3 — Navigation
        ("path_efficiency_mean",       "Path Efficiency μ"),
        ("path_efficiency_std",        "Path Efficiency σ"),
        ("speed_variability_mean",     "Speed CV μ"),
        ("speed_variability_std",      "Speed CV σ"),
        ("stuck_rate",                 "Stuck Rate"),
        ("ttc_median",                 "TTC Median (s)"),
        # Row 4 — Dynamics
        ("acceleration_mean",          "Accel μ (m/s²)"),
        ("acceleration_std",           "Accel σ (m/s²)"),
        ("heading_jerk_mean",          "Heading Jerk μ"),
        ("heading_jerk_std",           "Heading Jerk σ"),
        ("translational_jerk_mean",    "Trans Jerk μ (m/s³)"),
        ("translational_jerk_std",     "Trans Jerk σ (m/s³)"),
        # Row 5 — Spacing
        ("min_inter_agent_dist_mean",  "Min Dist μ (m)"),
        ("min_inter_agent_dist_std",   "Min Dist σ (m)"),
        ("min_inter_agent_dist_p5",    "Min Dist p5 (m)"),
        ("min_inter_agent_dist_p25",   "Min Dist p25 (m)"),
        ("oscillation_index_mean",     "Osc Index μ (rev/m)"),
        ("oscillation_index_std",      "Osc Index σ (rev/m)"),
        # Row 6 — Macro / Proxemic
        ("mean_density",               "Density (ag/m²)"),
        ("mean_flow",                  "Flow (ag/m/s)"),
        ("proxemic_intimate_fraction", "% Intimate (<0.45m)"),
        ("proxemic_personal_fraction", "% Personal (0.45–1.2m)"),
        ("proxemic_social_fraction",   "% Social (1.2–3.6m)"),
        ("proxemic_public_fraction",   "% Public (≥3.6m)"),
    ]

    # Keys that should display as percentages
    _pct_keys = {"stuck_rate", "proxemic_intimate_fraction", "proxemic_personal_fraction",
                 "proxemic_social_fraction", "proxemic_public_fraction"}

    fig, axes = plt.subplots(6, 6, figsize=(28, 24))
    fig.suptitle("Complete Metrics Dashboard — All Runs", fontsize=18, fontweight="bold", y=0.995)
    axes = axes.flatten()
    x = np.arange(n)

    for idx, (key, label) in enumerate(metric_specs):
        ax = axes[idx]
        raw_vals = [getattr(all_metrics[nm], key, 0) for nm in names]
        display_vals = [v * 100 if key in _pct_keys else v for v in raw_vals]
        bars = ax.bar(x, display_vals, color=_GT_COLOR, alpha=0.85,
                      edgecolor="white", linewidth=0.5)
        if sim_metrics:
            sim_raw = [getattr(sim_metrics.get(nm, CrowdMetrics()), key, 0) for nm in names]
            sim_display = [v * 100 if key in _pct_keys else v for v in sim_raw]
            ax.bar(x + 0.35, sim_display, width=0.35, color=_SIM_COLOR, alpha=0.85)
        ylabel = label + " (%)" if key in _pct_keys and "%" not in label else label
        ax.set_ylabel(ylabel, fontsize=7)
        ax.set_xticks(x)
        ax.set_xticklabels([nm.replace("_", "\n") for nm in names],
                           fontsize=6, rotation=45, ha="right")
        ax.tick_params(axis="y", labelsize=6)
        for bar_obj, v in zip(bars, display_vals):
            if v != 0:
                fmt = f"{v:.1f}%" if key in _pct_keys else (
                    f"{v:.3f}" if abs(v) < 1 else (f"{v:.1f}" if abs(v) >= 100 else f"{v:.2f}"))
                ax.text(bar_obj.get_x() + bar_obj.get_width()/2, bar_obj.get_height(),
                        fmt, ha="center", va="bottom", fontsize=5, color="#333")

    plt.tight_layout(rect=[0, 0, 1, 0.97])
    plt.savefig(output_dir / "metrics_dashboard.png", dpi=150, bbox_inches="tight")
    plt.close()

    # ── Figure 2: Distributions Dashboard (2×2) ──────────────────────────
    fig, axes = plt.subplots(2, 2, figsize=(16, 12))
    fig.suptitle("Distribution Analysis — All Runs", fontsize=14, fontweight="bold")
    cmap = plt.cm.Set2

    # (a) Speed distributions overlaid
    ax = axes[0, 0]
    for i, (nm, m) in enumerate(all_metrics.items()):
        if m.speed_distribution:
            ax.hist(m.speed_distribution, bins=40, density=True, alpha=0.45,
                    color=cmap(i % 8), label=nm[:15], histtype="stepfilled")
    ax.set_xlabel("Speed (m/s)")
    ax.set_ylabel("Probability Density")
    ax.set_title("Speed Distributions")
    ax.set_xlim(0, 4)
    ax.legend(fontsize=7, ncol=2)

    # (b) Min-distance distributions with proxemic zone shading
    ax = axes[0, 1]
    # Draw proxemic zone backgrounds
    ax.axvspan(0, 0.45, alpha=0.15, color=_PROX_COLORS[0], label="Intimate")
    ax.axvspan(0.45, 1.2, alpha=0.15, color=_PROX_COLORS[1], label="Personal")
    ax.axvspan(1.2, 3.6, alpha=0.15, color=_PROX_COLORS[2], label="Social")
    ax.axvspan(3.6, 8.0, alpha=0.15, color=_PROX_COLORS[3], label="Public")
    for i, (nm, m) in enumerate(all_metrics.items()):
        if m.min_distance_distribution:
            data = [d for d in m.min_distance_distribution if d < 8]
            ax.hist(data, bins=60, density=True, alpha=0.4,
                    color=cmap(i % 8), label=nm[:15], histtype="stepfilled")
    ax.set_xlabel("Min Inter-Agent Distance (m)")
    ax.set_ylabel("Probability Density")
    ax.set_title("Min Distance + Proxemic Zones")
    ax.set_xlim(0, 8)
    ax.legend(fontsize=6, ncol=2)

    # (c) Fundamental diagram
    ax = axes[1, 0]
    for i, (nm, m) in enumerate(all_metrics.items()):
        if m.density_flow_points:
            densities = [p[0] for p in m.density_flow_points]
            flows = [p[1] for p in m.density_flow_points]
            ax.scatter(densities, flows, alpha=0.3, s=8, color=cmap(i % 8), label=nm[:15])
    ax.set_xlabel("Density (agents/m²)")
    ax.set_ylabel("Flow (agents/m/s)")
    ax.set_title("Fundamental Diagram: Density vs Flow")
    ax.legend(fontsize=7, ncol=2)

    # (d) Proxemic zone stacked bar chart per run
    ax = axes[1, 1]
    prox_keys = ["proxemic_intimate_fraction", "proxemic_personal_fraction",
                 "proxemic_social_fraction", "proxemic_public_fraction"]
    bottom = np.zeros(n)
    for k_idx, pk in enumerate(prox_keys):
        vals = [getattr(all_metrics[nm], pk, 0) * 100 for nm in names]
        ax.bar(x, vals, bottom=bottom, color=_PROX_COLORS[k_idx],
               label=_PROX_LABELS[k_idx].replace("\n", " "), edgecolor="white", linewidth=0.5)
        bottom += np.array(vals)
    ax.set_ylabel("% of Agent-Frame Observations")
    ax.set_title("Proxemic Zone Distribution")
    ax.set_xticks(x)
    ax.set_xticklabels([nm.replace("_", "\n")[:12] for nm in names],
                       fontsize=7, rotation=45, ha="right")
    ax.legend(fontsize=7, loc="upper right")
    ax.set_ylim(0, 105)

    plt.tight_layout()
    plt.savefig(output_dir / "distributions_dashboard.png", dpi=150, bbox_inches="tight")
    plt.close()

    print(f"  Plots saved to {output_dir}/")


def plot_environment_summary(env_name: str, phase_data: Dict[str, Dict[str, CrowdMetrics]],
                             output_dir: Path):
    """Generate a cross-phase comparison figure for one environment.

    Args:
        env_name: e.g. "central_tunnel"
        phase_data: {phase_label: {run_name: CrowdMetrics, ...}, ...}
        output_dir: where to save the PNG
    """
    if not HAS_MPL:
        return

    output_dir.mkdir(parents=True, exist_ok=True)

    # Compute phase averages
    phase_labels = list(phase_data.keys())
    phase_avgs = {}
    for phase, runs in phase_data.items():
        avg = {}
        for key in CrowdMetrics.__dataclass_fields__:
            vals = [getattr(m, key, 0) for m in runs.values()
                    if isinstance(getattr(m, key, None), (int, float))]
            avg[key] = float(np.mean(vals)) if vals else 0.0
        phase_avgs[phase] = avg

    # ── Figure: Cross-Phase Comparison (5 rows × 4 cols = 20 panels) ─────
    metric_specs = [
        # Row 1
        ("speed_mean",                 "Speed Mean (m/s)"),
        ("speed_std",                  "Speed Std (m/s)"),
        ("collision_rate",             "Collision Rate"),
        ("near_miss_rate",             "Near-Miss Rate"),
        # Row 2
        ("path_efficiency_mean",       "Path Efficiency"),
        ("speed_variability_mean",     "Speed CV"),
        ("heading_jerk_mean",          "Heading Jerk"),
        ("acceleration_mean",          "Accel (m/s²)"),
        # Row 3
        ("min_inter_agent_dist_mean",  "Min Dist μ (m)"),
        ("min_inter_agent_dist_p5",    "Min Dist p5 (m)"),
        ("ttc_mean",                   "TTC Mean (s)"),
        ("ttc_p10",                    "TTC p10 (s)"),
        # Row 4
        ("stuck_rate",                 "Stuck Rate"),
        ("oscillation_index_mean",     "Osc Index (rev/m)"),
        ("translational_jerk_mean",    "Trans Jerk (m/s³)"),
        ("ttc_median",                 "TTC Median (s)"),
        # Row 5
        ("mean_density",               "Density (ag/m²)"),
        ("mean_flow",                  "Flow (ag/m/s)"),
        ("total_collisions",           "Total Collisions"),
        ("total_near_misses",          "Total Near-Misses"),
    ]

    n_phases = len(phase_labels)
    x = np.arange(n_phases)
    short_labels = []
    colors = []
    for pl in phase_labels:
        # Extract the short phase name from e.g. "cafe_baseline" -> "baseline"
        parts = pl.split("_")
        # Find env-specific prefix length, then take the rest
        short = pl.replace(env_name + "_", "")
        short_labels.append(short.replace("_", "\n"))
        # Match to phase color
        matched = False
        for pkey, pcol in _PHASE_COLORS.items():
            if pkey in pl:
                colors.append(pcol)
                matched = True
                break
        if not matched:
            colors.append("#888888")

    fig, axes = plt.subplots(5, 4, figsize=(20, 20))
    display_name = env_name.replace("_", " ").title()
    fig.suptitle(f"{display_name} — OAT Phase Comparison", fontsize=16, fontweight="bold", y=0.98)
    axes = axes.flatten()

    for idx, (key, label) in enumerate(metric_specs):
        ax = axes[idx]
        vals = [phase_avgs[pl].get(key, 0) for pl in phase_labels]
        bars = ax.bar(x, vals, color=colors, edgecolor="white", linewidth=0.8, alpha=0.9)
        ax.set_ylabel(label, fontsize=9)
        ax.set_xticks(x)
        ax.set_xticklabels(short_labels, fontsize=7, rotation=30, ha="right")
        # Value labels
        for bar_obj, v in zip(bars, vals):
            if v != 0:
                fmt = f"{v:.4f}" if abs(v) < 0.01 else (f"{v:.3f}" if abs(v) < 1 else f"{v:.2f}")
                ax.text(bar_obj.get_x() + bar_obj.get_width()/2, bar_obj.get_height(),
                        fmt, ha="center", va="bottom", fontsize=6.5, color="#333")
        # Highlight baseline with a dashed line
        if vals:
            ax.axhline(y=vals[0], color="#cccccc", linestyle="--", linewidth=0.8, alpha=0.6)

    plt.tight_layout(rect=[0, 0, 1, 0.96])
    outpath = output_dir / f"{env_name}_phase_comparison.png"
    plt.savefig(outpath, dpi=150, bbox_inches="tight")
    plt.close()
    print(f"  Phase comparison saved to {outpath}")

    # ── Proxemic Zone Comparison (stacked bar, one per phase) ────────────
    fig, ax = plt.subplots(figsize=(10, 6))
    fig.suptitle(f"{display_name} — Proxemic Zone Distribution by Phase",
                 fontsize=14, fontweight="bold")
    prox_keys = ["proxemic_intimate_fraction", "proxemic_personal_fraction",
                 "proxemic_social_fraction", "proxemic_public_fraction"]
    bottom = np.zeros(n_phases)
    for k_idx, pk in enumerate(prox_keys):
        vals = [phase_avgs[pl].get(pk, 0) * 100 for pl in phase_labels]
        ax.bar(x, vals, bottom=bottom, color=_PROX_COLORS[k_idx],
               label=_PROX_LABELS[k_idx].replace("\n", " "), edgecolor="white", linewidth=0.5)
        # Add percentage text in each segment if > 3%
        for xi, v in enumerate(vals):
            if v > 3:
                ax.text(xi, bottom[xi] + v/2, f"{v:.1f}%", ha="center", va="center",
                        fontsize=7, color="white" if k_idx in (0, 3) else "#333",
                        fontweight="bold")
        bottom += np.array(vals)
    ax.set_ylabel("% of Agent-Frame Observations", fontsize=10)
    ax.set_xticks(x)
    ax.set_xticklabels(short_labels, fontsize=9, rotation=30, ha="right")
    ax.legend(fontsize=9, loc="upper right")
    ax.set_ylim(0, 105)
    plt.tight_layout()
    outpath = output_dir / f"{env_name}_proxemic_comparison.png"
    plt.savefig(outpath, dpi=150, bbox_inches="tight")
    plt.close()
    print(f"  Proxemic comparison saved to {outpath}")


def plot_cross_environment(env_data: Dict[str, Dict[str, float]],
                           gt_data: Dict[str, Dict[str, float]],
                           output_dir: Path):
    """Generate a cross-environment + ground truth comparison figure.

    Args:
        env_data: {env_name: {metric: avg_baseline_value, ...}}
        gt_data: {dataset_name: {metric: value, ...}}
        output_dir: where to save
    """
    if not HAS_MPL:
        return

    output_dir.mkdir(parents=True, exist_ok=True)

    # ── Figure: Sim Baselines vs GT (4×4 = 16 panels) ──────────────────
    metric_specs = [
        ("speed_mean",                 "Speed\n(m/s)"),
        ("speed_std",                  "Speed σ\n(m/s)"),
        ("collision_rate",             "Collision\nRate"),
        ("near_miss_rate",             "Near-Miss\nRate"),
        ("path_efficiency_mean",       "Path\nEfficiency"),
        ("speed_variability_mean",     "Speed\nCV"),
        ("heading_jerk_mean",          "Heading\nJerk"),
        ("acceleration_mean",          "Accel\n(m/s²)"),
        ("min_inter_agent_dist_mean",  "Min Dist\n(m)"),
        ("ttc_mean",                   "TTC Mean\n(s)"),
        ("ttc_p10",                    "TTC p10\n(s)"),
        ("stuck_rate",                 "Stuck\nRate"),
        ("oscillation_index_mean",     "Osc\nIndex"),
        ("translational_jerk_mean",    "Trans\nJerk"),
        ("mean_density",               "Density\n(ag/m²)"),
        ("mean_flow",                  "Flow\n(ag/m/s)"),
    ]

    sources = list(env_data.keys())  # sim environments
    gt_names = list(gt_data.keys())  # GT datasets

    # Compute GT mean for reference line (all numeric keys, not just metric_specs)
    gt_mean = {}
    all_gt_keys = set()
    for ds in gt_names:
        all_gt_keys.update(gt_data[ds].keys())
    for key in all_gt_keys:
        gt_vals = [gt_data[ds].get(key, 0) for ds in gt_names
                   if isinstance(gt_data[ds].get(key), (int, float)) and gt_data[ds].get(key, 0) != 0]
        gt_mean[key] = float(np.mean(gt_vals)) if gt_vals else 0

    fig, axes = plt.subplots(4, 4, figsize=(24, 16))
    fig.suptitle("Cross-Environment Baseline Comparison vs Ground Truth",
                 fontsize=16, fontweight="bold", y=0.98)
    axes = axes.flatten()

    for idx, (key, label) in enumerate(metric_specs):
        ax = axes[idx]

        # GT datasets as individual bars
        gt_vals = [gt_data[ds].get(key, 0) for ds in gt_names]
        gt_x = np.arange(len(gt_names))
        ax.bar(gt_x, gt_vals, color="#90A4AE", alpha=0.7, width=0.7, edgecolor="white")

        # Sim baselines as coloured bars after GT
        sim_x = np.arange(len(gt_names), len(gt_names) + len(sources))
        sim_vals = [env_data[s].get(key, 0) for s in sources]
        sim_colors = [_ENV_COLORS.get(s, "#888") for s in sources]
        bars = ax.bar(sim_x, sim_vals, color=sim_colors, alpha=0.9, width=0.7, edgecolor="white")

        # GT mean reference line
        if gt_mean[key] > 0:
            ax.axhline(y=gt_mean[key], color="#FF6F00", linestyle="--", linewidth=1.5,
                       alpha=0.7, label=f"GT Mean: {gt_mean[key]:.3f}")
            ax.legend(fontsize=6, loc="best")

        all_labels = [ds.replace("_", "\n")[:10] for ds in gt_names] + \
                     [s.replace("_", "\n") for s in sources]
        ax.set_xticks(np.arange(len(all_labels)))
        ax.set_xticklabels(all_labels, fontsize=5.5, rotation=45, ha="right")
        ax.set_title(label, fontsize=9, fontweight="bold")
        ax.tick_params(axis="y", labelsize=7)

    plt.tight_layout(rect=[0, 0, 1, 0.95])
    outpath = output_dir / "cross_environment_vs_gt.png"
    plt.savefig(outpath, dpi=150, bbox_inches="tight")
    plt.close()
    print(f"  Cross-environment comparison saved to {outpath}")

    # ── Proxemic Zone Comparison: GT mean vs all 3 sim baselines ─────────
    fig, ax = plt.subplots(figsize=(10, 6))
    fig.suptitle("Proxemic Zone Distribution — Ground Truth vs Simulated Baselines",
                 fontsize=14, fontweight="bold")
    prox_keys = ["proxemic_intimate_fraction", "proxemic_personal_fraction",
                 "proxemic_social_fraction", "proxemic_public_fraction"]

    bar_labels = ["GT Mean"] + [s.replace("_", " ").title() for s in sources]
    n_bars = len(bar_labels)
    x = np.arange(n_bars)
    bottom = np.zeros(n_bars)
    for k_idx, pk in enumerate(prox_keys):
        gt_val = gt_mean.get(pk, 0) * 100
        sim_vals = [env_data[s].get(pk, 0) * 100 for s in sources]
        vals = [gt_val] + sim_vals
        ax.bar(x, vals, bottom=bottom, color=_PROX_COLORS[k_idx],
               label=_PROX_LABELS[k_idx].replace("\n", " "), edgecolor="white", linewidth=0.5)
        for xi, v in enumerate(vals):
            if v > 3:
                ax.text(xi, bottom[xi] + v/2, f"{v:.1f}%", ha="center", va="center",
                        fontsize=8, color="white" if k_idx in (0, 3) else "#333",
                        fontweight="bold")
        bottom += np.array(vals)

    ax.set_xticks(x)
    ax.set_xticklabels(bar_labels, fontsize=10)
    ax.set_ylabel("% of Agent-Frame Observations", fontsize=11)
    ax.legend(fontsize=10, loc="upper right")
    ax.set_ylim(0, 105)
    plt.tight_layout()
    outpath = output_dir / "proxemic_gt_vs_sim.png"
    plt.savefig(outpath, dpi=150, bbox_inches="tight")
    plt.close()
    print(f"  Proxemic GT vs Sim saved to {outpath}")


# ---------------------------------------------------------------------------
# Console output
# ---------------------------------------------------------------------------

def print_summary(all_metrics: Dict[str, CrowdMetrics]):
    """Print a formatted summary table."""
    print("\n" + "=" * 120)
    print("GROUND TRUTH CROWD DYNAMICS METRICS")
    print("=" * 120)

    header = f"{'Dataset':<16} {'#Peds':>6} {'Dur(s)':>7} {'Spd μ':>6} {'Spd σ':>6} " \
             f"{'PathEff':>8} {'SpdCV':>6} {'MinD μ':>7} {'MinD p5':>8} " \
             f"{'CollR':>7} {'NMissR':>7} {'AccMu':>6} {'HdJerk':>7} {'Dens':>6} {'Flow':>6}"
    print(header)
    print("-" * 120)

    for name, m in all_metrics.items():
        row = f"{name:<16} {m.num_pedestrians:>6} {m.duration_seconds:>7.1f} " \
              f"{m.speed_mean:>6.2f} {m.speed_std:>6.2f} " \
              f"{m.path_efficiency_mean:>8.3f} {m.speed_variability_mean:>6.3f} " \
              f"{m.min_inter_agent_dist_mean:>7.2f} {m.min_inter_agent_dist_p5:>8.2f} " \
              f"{m.collision_rate:>7.4f} {m.near_miss_rate:>7.4f} " \
              f"{m.acceleration_mean:>6.2f} {m.heading_jerk_mean:>7.3f} " \
              f"{m.mean_density:>6.3f} {m.mean_flow:>6.3f}"
        print(row)

    print("-" * 120)

    # V2 Metrics table
    print("\n" + "=" * 120)
    print("V2 METRICS (Extended)")
    print("=" * 120)
    v2_header = f"{'Dataset':<16} {'TTC μ':>7} {'TTC med':>8} {'TTC p10':>8} " \
                f"{'Stuck%':>7} {'OscIdx':>7} {'TrJerk':>7} " \
                f"{'%Intim':>7} {'%Pers':>7} {'%Social':>8} {'%Public':>8}"
    print(v2_header)
    print("-" * 120)

    for name, m in all_metrics.items():
        row = f"{name:<16} {m.ttc_mean:>7.2f} {m.ttc_median:>8.2f} {m.ttc_p10:>8.2f} " \
              f"{m.stuck_rate * 100:>6.1f}% {m.oscillation_index_mean:>7.3f} " \
              f"{m.translational_jerk_mean:>7.2f} " \
              f"{m.proxemic_intimate_fraction * 100:>6.1f}% " \
              f"{m.proxemic_personal_fraction * 100:>6.1f}% " \
              f"{m.proxemic_social_fraction * 100:>7.1f}% " \
              f"{m.proxemic_public_fraction * 100:>7.1f}%"
        print(row)

    print("-" * 120)

    # Group info
    for name, m in all_metrics.items():
        if m.num_groups > 0:
            print(f"  {name}: {m.num_groups} groups, avg size {m.group_size_mean:.1f}, "
                  f"cohesion {m.group_cohesion_mean:.2f}±{m.group_cohesion_std:.2f}m, "
                  f"speed σ within groups {m.group_speed_std_mean:.3f} m/s")

    print()


def print_interpretation():
    """Print guidance on interpreting the metrics."""
    print("=" * 120)
    print("INTERPRETATION GUIDE")
    print("=" * 120)
    print("""
WHAT EACH METRIC REVEALS ABOUT REALISM:

┌─────────────────────────────────────┬──────────────────────────────────────────────────────────┐
│ Metric                              │ What a mismatch tells you                                │
├─────────────────────────────────────┼──────────────────────────────────────────────────────────┤
│ Speed mean/std                      │ SFM desired_velocity or max_vel params are wrong         │
│ Speed distribution shape            │ Need mixture of slow/fast walkers (heterogeneity)        │
│ Path efficiency                     │ Too low → agents oscillate/detour excessively             │
│                                     │ Too high → agents ignore obstacles, walk straight lines   │
│ Speed variability (CV)              │ Too low → robotic constant-speed; too high → jerky motion │
│ Heading jerk                        │ Too high → oscillation in direction changes (SFM tuning)  │
│ Min inter-agent distance            │ Too small → social force too weak; too large → too strong │
│ Collision rate                      │ Should be ~0 in reality; high → forces insufficient       │
│ Near-miss rate                      │ Higher in dense scenes; calibrate social force factor     │
│ Density vs Flow (fund. diagram)     │ Classic test: flow peaks then drops as density increases  │
│ Acceleration                        │ Too high → abrupt speed changes (damping issue)           │
│ Group cohesion                      │ Too tight → group force over-tuned; too loose → under    │
│ Group speed variance                │ Should be low: group members walk at similar speeds       │
└─────────────────────────────────────┴──────────────────────────────────────────────────────────┘

RECOMMENDED COMPARISON STRATEGY:
  1. Match speed distribution shape (histogram overlay) — most impactful
  2. Match collision/near-miss rates — validates avoidance behavior
  3. Match path efficiency — validates obstacle/goal navigation
  4. Match fundamental diagram shape — validates density-dependent behavior
  5. Match group cohesion — validates group dynamics
  6. Check heading jerk — diagnose oscillation problems
""")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="Ground truth crowd dynamics analysis")
    parser.add_argument("--dataset", type=str, default=None,
                        help="Analyze only this dataset (e.g. eth_univ, ucy_zara01)")
    parser.add_argument("--custom-dataset", type=str, nargs=2, action="append",
                        metavar=("NAME", "PATH"),
                        help="Analyze a custom true_pos_.csv file (repeatable). Example: --custom-dataset run_1 /path/to/true_pos_.csv")
    parser.add_argument("--sim-json", type=str, default=None,
                        help="Path to simulation metrics JSON for comparison")
    parser.add_argument("--output-dir", type=str, default=None,
                        help="Output directory for results (default: DATASETS/results or ./analysis_output)")
    parser.add_argument("--no-plots", action="store_true",
                        help="Skip plot generation")
    parser.add_argument("--dt", type=float, default=0.4,
                        help="Time delta (seconds) for custom datasets (default: 0.4)")
    parser.add_argument("--env-summary", type=str, default=None,
                        help="Generate cross-phase comparison for an environment. "
                             "Value is the sim_results/<env> directory path.")
    parser.add_argument("--cross-env", type=str, default=None,
                        help="Generate cross-environment comparison. "
                             "Value is the sim_results/ root directory path.")
    args = parser.parse_args()

    # Determine output directory
    if args.custom_dataset and not args.output_dir:
        # For custom datasets, default to ./analysis_output
        output_dir = Path("./analysis_output")
    elif args.output_dir:
        output_dir = Path(args.output_dir)
    else:
        # For GT datasets, use DATASETS/results
        output_dir = DATASETS_DIR / "results"
    output_dir.mkdir(parents=True, exist_ok=True)

    # Load and analyze datasets
    all_metrics: Dict[str, CrowdMetrics] = {}

    # If custom datasets provided, analyze them (skip registry)
    if args.custom_dataset:
        print(f"\n[CUSTOM ANALYSIS] Using {len(args.custom_dataset)} custom dataset(s)")
        for label, filepath in args.custom_dataset:
            filepath = Path(filepath)
            if not filepath.exists():
                print(f"  [SKIP] {label}: file not found at {filepath}")
                continue

            print(f"  [LOAD] {label} ← {filepath}")
            td = load_truepos(str(filepath), args.dt)

            print(f"         {len(np.unique(td.ped_ids))} pedestrians, "
                  f"{len(np.unique(td.frames))} frames")

            metrics = compute_metrics(td, label)
            all_metrics[label] = metrics
    else:
        # Original registry-based analysis
        for label, rel_path, dt in DATASET_REGISTRY:
            if args.dataset and args.dataset != label:
                continue

            filepath = DATASETS_DIR / rel_path
            if not filepath.exists():
                print(f"  [SKIP] {label}: file not found at {filepath}")
                continue

            print(f"  [LOAD] {label} ← {rel_path}")
            td = load_truepos(str(filepath), dt)

            # Load groups if available
            if label in GROUP_FILES:
                gpath = DATASETS_DIR / GROUP_FILES[label]
                if gpath.exists():
                    td.groups = load_groups(str(gpath))
                    print(f"         loaded {len(td.groups)} groups")

            print(f"         {len(np.unique(td.ped_ids))} pedestrians, "
                  f"{len(np.unique(td.frames))} frames")

            metrics = compute_metrics(td, label)
            all_metrics[label] = metrics

    if not all_metrics:
        print("No datasets found. Check paths.")
        sys.exit(1)

    # Print summary
    print_summary(all_metrics)
    if not args.custom_dataset:
        # Only print interpretation for GT analysis
        print_interpretation()

    # Save JSON (without large distribution arrays)
    json_out = {}
    for name, m in all_metrics.items():
        d = asdict(m)
        # Remove large arrays from JSON (keep them for plots)
        d.pop("speed_distribution", None)
        d.pop("min_distance_distribution", None)
        d.pop("density_flow_points", None)
        d.pop("acceleration_distribution", None)
        json_out[name] = d

    json_path = output_dir / "crowd_metrics.json"
    with open(json_path, "w") as f:
        json.dump(json_out, f, indent=2)
    print(f"  Metrics saved to {json_path}")

    # Load simulation data for comparison if provided
    sim_metrics = None
    if args.sim_json:
        with open(args.sim_json) as f:
            sim_data = json.load(f)
        sim_metrics = {}
        for name, d in sim_data.items():
            m = CrowdMetrics(**{k: v for k, v in d.items()
                               if k in CrowdMetrics.__dataclass_fields__})
            sim_metrics[name] = m

        comparison = compare_metrics(all_metrics, sim_metrics)
        cmp_path = output_dir / "comparison_report.json"
        with open(cmp_path, "w") as f:
            json.dump(comparison, f, indent=2)
        print(f"  Comparison report saved to {cmp_path}")

        # Print comparison highlights
        print("\n" + "=" * 80)
        print("SIMULATION vs GROUND TRUTH COMPARISON")
        print("=" * 80)
        for ds_name, cmp in comparison.items():
            print(f"\n  {ds_name}:")
            for key, vals in cmp.items():
                if vals["percent_error"] is not None:
                    status = "✓" if vals["percent_error"] < 15 else "✗"
                    print(f"    {status} {key:<35} GT={vals['ground_truth']:.4f}  "
                          f"SIM={vals['simulation']:.4f}  err={vals['percent_error']:.1f}%")

    # Generate plots
    if not args.no_plots and not args.env_summary and not args.cross_env:
        plot_metrics(all_metrics, output_dir, sim_metrics)

    # ── Environment-level cross-phase summary ─────────────────────────────
    if args.env_summary:
        env_dir = Path(args.env_summary)
        env_name = env_dir.name  # e.g. "cafe"
        phase_data: Dict[str, Dict[str, CrowdMetrics]] = {}
        for phase_dir in sorted(env_dir.iterdir()):
            if not phase_dir.is_dir():
                continue
            json_path = phase_dir / "analysis" / "crowd_metrics.json"
            if not json_path.exists():
                continue
            with open(json_path) as f:
                data = json.load(f)
            phase_metrics = {}
            for run_name, d in data.items():
                m = CrowdMetrics(**{k: v for k, v in d.items()
                                    if k in CrowdMetrics.__dataclass_fields__})
                phase_metrics[run_name] = m
            phase_data[phase_dir.name] = phase_metrics
            print(f"  Loaded {phase_dir.name}: {len(phase_metrics)} runs")

        if phase_data:
            summary_dir = env_dir / "summary"
            plot_environment_summary(env_name, phase_data, summary_dir)
        else:
            print(f"  No phase data found in {env_dir}")

    # ── Cross-environment + GT comparison ─────────────────────────────────
    if args.cross_env:
        sim_root = Path(args.cross_env)
        env_baselines: Dict[str, Dict[str, float]] = {}

        for env_dir in sorted(sim_root.iterdir()):
            if not env_dir.is_dir():
                continue
            env_name = env_dir.name
            # Find baseline directory
            baseline_dir = None
            for sub in env_dir.iterdir():
                if sub.is_dir() and "baseline" in sub.name:
                    baseline_dir = sub
                    break
            if baseline_dir is None:
                continue
            json_path = baseline_dir / "analysis" / "crowd_metrics.json"
            if not json_path.exists():
                continue
            with open(json_path) as f:
                data = json.load(f)
            # Average across runs
            avg = {}
            for key in CrowdMetrics.__dataclass_fields__:
                vals = [d.get(key, 0) for d in data.values()
                        if isinstance(d.get(key), (int, float))]
                avg[key] = float(np.mean(vals)) if vals else 0.0
            env_baselines[env_name] = avg
            print(f"  Loaded baseline for {env_name}: {len(data)} runs")

        # Load GT data
        gt_json = DATASETS_DIR / "results" / "crowd_metrics.json"
        gt_data: Dict[str, Dict[str, float]] = {}
        if gt_json.exists():
            with open(gt_json) as f:
                raw_gt = json.load(f)
            for ds_name, d in raw_gt.items():
                gt_data[ds_name] = {k: v for k, v in d.items()
                                    if isinstance(v, (int, float))}
            print(f"  Loaded GT: {len(gt_data)} datasets")

        if env_baselines and gt_data:
            summary_dir = sim_root / "summary"
            plot_cross_environment(env_baselines, gt_data, summary_dir)
        elif env_baselines:
            print("  Warning: GT data not found, skipping cross-environment plot")
        else:
            print(f"  No baseline data found in {sim_root}")

    print("\nDone.")
    print(f"Results saved to: {output_dir.resolve()}")


if __name__ == "__main__":
    main()
