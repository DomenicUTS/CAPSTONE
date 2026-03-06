#!/usr/bin/env python3
"""
Ground Truth Crowd Dynamics Analyzer
=====================================
Extracts benchmark metrics from ETH/UCY pedestrian datasets for comparison
against HuNavSim simulation runs.

Datasets supported:
  - EWAP (ETH/Hotel): obsmat.txt format with velocities
  - UCY (Zara01/02/03, Students001/003, Uni): true_pos_.csv format

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
  python3 ground_truth_analysis.py                       # analyze all datasets
  python3 ground_truth_analysis.py --dataset ewap_eth    # analyze one dataset
  python3 ground_truth_analysis.py --sim-json <path>     # compare against simulation
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

# Each entry: (label, relative_path, format, dt_seconds)
DATASET_REGISTRY = [
    ("ewap_eth",   "ewap_dataset/seq_eth/obsmat.txt",   "obsmat", 0.4),
    ("ewap_hotel", "ewap_dataset/seq_hotel/obsmat.txt",  "obsmat", 0.4),
    ("eth_hotel",  "eth/hotel/true_pos_.csv",             "truepos", 0.4),
    ("eth_univ",   "eth/univ/true_pos_.csv",              "truepos", 0.4),
    ("ucy_zara01", "ucy/zara/zara01/true_pos_.csv",       "truepos", 0.4),
    ("ucy_zara02", "ucy/zara/zara02/true_pos_.csv",       "truepos", 0.4),
    ("ucy_zara03", "ucy/zara/zara03/true_pos_.csv",       "truepos", 0.4),
    ("ucy_univ_s1","ucy/univ/students001/true_pos_.csv",  "truepos", 0.4),
    ("ucy_univ_s3","ucy/univ/students003/true_pos_.csv",  "truepos", 0.4),
]

# Group annotations (EWAP only)
GROUP_FILES = {
    "ewap_eth":   "ewap_dataset/seq_eth/groups.txt",
    "ewap_hotel": "ewap_dataset/seq_hotel/groups.txt",
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


def load_obsmat(filepath: str, dt: float) -> TrajectoryData:
    """Load EWAP obsmat.txt format:
    [frame_number ped_ID pos_x pos_z pos_y v_x v_z v_y]
    pos_z and v_z are unused (perpendicular to ground).
    """
    data = np.loadtxt(filepath)
    frames = data[:, 0].astype(int)
    ped_ids = data[:, 1].astype(int)
    positions = data[:, [2, 4]]   # pos_x, pos_y
    velocities = data[:, [5, 7]]  # v_x, v_y
    return TrajectoryData(frames=frames, ped_ids=ped_ids,
                          positions=positions, velocities=velocities, dt=dt)


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
    # The frame-step varies by dataset (6 for ETH, 10 for UCY), so we detect
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

    for pid in unique_peds:
        mask = td.ped_ids == pid
        idx = np.where(mask)[0]
        if len(idx) < 3:
            continue

        order = np.argsort(td.frames[idx])
        sorted_idx = idx[order]
        pos = td.positions[sorted_idx]
        vel = td.velocities[sorted_idx]
        spd = np.linalg.norm(vel, axis=1)
        frm = td.frames[sorted_idx]

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

    # --- Pairwise: minimum inter-agent distances ---
    min_dists_all = []
    collisions = 0
    near_misses = 0
    total_pair_checks = 0

    for frame in unique_frames:
        mask = td.frames == frame
        pos_f = td.positions[mask]
        n = len(pos_f)
        if n < 2:
            continue
        for i in range(n):
            dists = np.linalg.norm(pos_f[i] - pos_f, axis=1)
            dists[i] = np.inf  # exclude self
            min_d = np.min(dists)
            min_dists_all.append(min_d)
            total_pair_checks += 1
            if min_d < COLLISION_RADIUS:
                collisions += 1
            if min_d < NEAR_MISS_RADIUS:
                near_misses += 1

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
    """Estimate the frame step (e.g. 6 for EWAP, 10 for UCY)."""
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

def plot_metrics(all_metrics: Dict[str, CrowdMetrics], output_dir: Path,
                 sim_metrics: Optional[Dict[str, CrowdMetrics]] = None):
    """Generate comparison plots."""
    if not HAS_MPL:
        return

    output_dir.mkdir(parents=True, exist_ok=True)

    # 1. Speed distributions
    fig, axes = plt.subplots(2, 3, figsize=(18, 10))
    fig.suptitle("Speed Distributions (Ground Truth vs Simulation)", fontsize=14)
    axes = axes.flatten()
    for i, (name, metrics) in enumerate(all_metrics.items()):
        if i >= len(axes):
            break
        ax = axes[i]
        if metrics.speed_distribution:
            ax.hist(metrics.speed_distribution, bins=50, density=True,
                    alpha=0.6, color="steelblue", label="Ground Truth")
        if sim_metrics and name in sim_metrics and sim_metrics[name].speed_distribution:
            ax.hist(sim_metrics[name].speed_distribution, bins=50, density=True,
                    alpha=0.6, color="coral", label="Simulation")
        ax.set_title(name, fontsize=10)
        ax.set_xlabel("Speed (m/s)")
        ax.set_ylabel("Density")
        ax.legend(fontsize=8)
        ax.set_xlim(0, 4)
    # hide unused axes
    for j in range(i + 1, len(axes)):
        axes[j].set_visible(False)
    plt.tight_layout()
    plt.savefig(output_dir / "speed_distributions.png", dpi=150)
    plt.close()

    # 2. Minimum inter-agent distance distributions
    fig, axes = plt.subplots(2, 3, figsize=(18, 10))
    fig.suptitle("Minimum Inter-Agent Distance Distributions", fontsize=14)
    axes = axes.flatten()
    for i, (name, metrics) in enumerate(all_metrics.items()):
        if i >= len(axes):
            break
        ax = axes[i]
        if metrics.min_distance_distribution:
            data = [d for d in metrics.min_distance_distribution if d < 10]
            ax.hist(data, bins=60, density=True, alpha=0.6,
                    color="steelblue", label="Ground Truth")
        if sim_metrics and name in sim_metrics and sim_metrics[name].min_distance_distribution:
            data = [d for d in sim_metrics[name].min_distance_distribution if d < 10]
            ax.hist(data, bins=60, density=True, alpha=0.6,
                    color="coral", label="Simulation")
        ax.axvline(x=COLLISION_RADIUS, color="red", linestyle="--", alpha=0.7, label=f"Collision ({COLLISION_RADIUS}m)")
        ax.axvline(x=NEAR_MISS_RADIUS, color="orange", linestyle="--", alpha=0.7, label=f"Near-miss ({NEAR_MISS_RADIUS}m)")
        ax.set_title(name, fontsize=10)
        ax.set_xlabel("Min distance to nearest agent (m)")
        ax.set_ylabel("Density")
        ax.legend(fontsize=7)
        ax.set_xlim(0, 10)
    for j in range(i + 1, len(axes)):
        axes[j].set_visible(False)
    plt.tight_layout()
    plt.savefig(output_dir / "min_distance_distributions.png", dpi=150)
    plt.close()

    # 3. Fundamental diagram (density vs flow)
    fig, axes = plt.subplots(2, 3, figsize=(18, 10))
    fig.suptitle("Fundamental Diagram: Density vs Flow", fontsize=14)
    axes = axes.flatten()
    for i, (name, metrics) in enumerate(all_metrics.items()):
        if i >= len(axes):
            break
        ax = axes[i]
        if metrics.density_flow_points:
            densities = [p[0] for p in metrics.density_flow_points]
            flows = [p[1] for p in metrics.density_flow_points]
            ax.scatter(densities, flows, alpha=0.3, s=5, c="steelblue", label="Ground Truth")
        if sim_metrics and name in sim_metrics and sim_metrics[name].density_flow_points:
            densities = [p[0] for p in sim_metrics[name].density_flow_points]
            flows = [p[1] for p in sim_metrics[name].density_flow_points]
            ax.scatter(densities, flows, alpha=0.3, s=5, c="coral", label="Simulation")
        ax.set_title(name, fontsize=10)
        ax.set_xlabel("Density (agents/m²)")
        ax.set_ylabel("Flow (agents/m/s)")
        ax.legend(fontsize=8)
    for j in range(i + 1, len(axes)):
        axes[j].set_visible(False)
    plt.tight_layout()
    plt.savefig(output_dir / "fundamental_diagram.png", dpi=150)
    plt.close()

    # 4. Summary bar chart comparing key metrics across datasets
    fig, axes = plt.subplots(2, 3, figsize=(18, 10))
    fig.suptitle("Key Metrics Comparison Across Datasets", fontsize=14)
    metric_labels = [
        ("speed_mean", "Mean Speed (m/s)"),
        ("path_efficiency_mean", "Path Efficiency"),
        ("min_inter_agent_dist_mean", "Mean Min Distance (m)"),
        ("collision_rate", "Collision Rate (/agent/s)"),
        ("speed_variability_mean", "Speed CV"),
        ("heading_jerk_mean", "Heading Jerk (smoothness)"),
    ]
    axes = axes.flatten()
    names = list(all_metrics.keys())
    for i, (key, label) in enumerate(metric_labels):
        ax = axes[i]
        gt_vals = [getattr(all_metrics[n], key, 0) for n in names]
        x = np.arange(len(names))
        width = 0.35
        ax.bar(x - width/2, gt_vals, width, label="Ground Truth", color="steelblue")
        if sim_metrics:
            sim_vals = [getattr(sim_metrics.get(n, CrowdMetrics()), key, 0) for n in names]
            ax.bar(x + width/2, sim_vals, width, label="Simulation", color="coral")
        ax.set_ylabel(label)
        ax.set_xticks(x)
        ax.set_xticklabels([n.replace("_", "\n") for n in names], fontsize=7, rotation=45)
        ax.legend(fontsize=8)
    plt.tight_layout()
    plt.savefig(output_dir / "metrics_comparison.png", dpi=150)
    plt.close()

    print(f"  Plots saved to {output_dir}/")


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
                        help="Analyze only this dataset (e.g. ewap_eth, ucy_zara01)")
    parser.add_argument("--sim-json", type=str, default=None,
                        help="Path to simulation metrics JSON for comparison")
    parser.add_argument("--output-dir", type=str, default=None,
                        help="Output directory for results (default: DATASETS/results)")
    parser.add_argument("--no-plots", action="store_true",
                        help="Skip plot generation")
    args = parser.parse_args()

    output_dir = Path(args.output_dir) if args.output_dir else SCRIPT_DIR / "results"
    output_dir.mkdir(parents=True, exist_ok=True)

    # Load and analyze datasets
    all_metrics: Dict[str, CrowdMetrics] = {}

    for label, rel_path, fmt, dt in DATASET_REGISTRY:
        if args.dataset and args.dataset != label:
            continue

        filepath = SCRIPT_DIR / rel_path
        if not filepath.exists():
            print(f"  [SKIP] {label}: file not found at {filepath}")
            continue

        print(f"  [LOAD] {label} ← {rel_path}")
        if fmt == "obsmat":
            td = load_obsmat(str(filepath), dt)
        else:
            td = load_truepos(str(filepath), dt)

        # Load groups if available
        if label in GROUP_FILES:
            gpath = SCRIPT_DIR / GROUP_FILES[label]
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

    json_path = output_dir / "ground_truth_metrics.json"
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
    if not args.no_plots:
        plot_metrics(all_metrics, output_dir, sim_metrics)

    print("\nDone.")


if __name__ == "__main__":
    main()
