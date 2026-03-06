#!/usr/bin/env python3
"""
Verification Script for Ground Truth Metrics
=============================================
Manually computes metrics step-by-step on the EWAP ETH dataset (obsmat.txt)
and cross-checks against the main analysis script's output.

This dataset is ideal for verification because:
  - It has explicit velocities (no finite-difference estimation needed)
  - The format is well-documented in the README
  - It has group annotations
"""

import json
import math
import numpy as np
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
DT = 0.4  # seconds per frame step (2.5 fps)

# ═══════════════════════════════════════════════════════════════════════════
# LOAD DATA
# ═══════════════════════════════════════════════════════════════════════════
print("=" * 80)
print("VERIFICATION: EWAP ETH (obsmat.txt)")
print("=" * 80)

data = np.loadtxt(SCRIPT_DIR / "ewap_dataset/seq_eth/obsmat.txt")
# Format: [frame_number, ped_ID, pos_x, pos_z, pos_y, v_x, v_z, v_y]
# pos_z and v_z are unused (perpendicular to ground)

frames   = data[:, 0].astype(int)
ped_ids  = data[:, 1].astype(int)
pos_x    = data[:, 2]
pos_y    = data[:, 4]  # NOTE: column 4, not 3 (pos_z is unused)
vel_x    = data[:, 5]
vel_y    = data[:, 7]  # NOTE: column 7, not 6 (v_z is unused)

unique_peds = np.unique(ped_ids)
unique_frames = np.unique(frames)
frame_step = float(np.median(np.diff(unique_frames)))

print(f"\nRaw data shape: {data.shape}")
print(f"Columns: frame, ped_id, pos_x, pos_z(unused), pos_y, v_x, v_z(unused), v_y")
print(f"Total observations: {len(frames)}")
print(f"Unique pedestrians: {len(unique_peds)}")
print(f"Unique frames: {len(unique_frames)}")
print(f"Frame range: {frames.min()} → {frames.max()}")
print(f"Frame step (median): {frame_step}")
print(f"Duration: ({frames.max()} - {frames.min()}) * {DT} / {frame_step} = "
      f"{(frames.max() - frames.min()) * DT / frame_step:.1f} seconds")

# ═══════════════════════════════════════════════════════════════════════════
# VERIFY METRIC 1: Speed Distribution
# ═══════════════════════════════════════════════════════════════════════════
print("\n" + "─" * 80)
print("METRIC 1: SPEED DISTRIBUTION")
print("─" * 80)
print("Definition: Speed = ||velocity|| for each observation.")
print("We compute speed = sqrt(v_x² + v_y²) at every recorded data point.")
print("We filter out v < 0.05 m/s (standing still) and v > 5.0 m/s (noise).\n")

speeds = np.sqrt(vel_x**2 + vel_y**2)
print(f"All speeds: {len(speeds)} observations")
print(f"  Raw range: {speeds.min():.4f} to {speeds.max():.4f} m/s")

valid_mask = (speeds > 0.05) & (speeds < 5.0)
valid_speeds = speeds[valid_mask]
print(f"  After filtering: {len(valid_speeds)} observations "
      f"(removed {len(speeds) - len(valid_speeds)} static/noisy)")

mean_spd = np.mean(valid_speeds)
std_spd = np.std(valid_speeds)
med_spd = np.median(valid_speeds)
p25_spd = np.percentile(valid_speeds, 25)
p75_spd = np.percentile(valid_speeds, 75)

print(f"\n  ► Mean speed:   {mean_spd:.4f} m/s")
print(f"  ► Std speed:    {std_spd:.4f} m/s")
print(f"  ► Median speed: {med_spd:.4f} m/s")
print(f"  ► P25 speed:    {p25_spd:.4f} m/s")
print(f"  ► P75 speed:    {p75_spd:.4f} m/s")

# Manual spot-check: first 5 observations
print(f"\n  Spot-check first 5 observations:")
for i in range(5):
    s = math.sqrt(vel_x[i]**2 + vel_y[i]**2)
    print(f"    obs[{i}]: frame={frames[i]}, ped={ped_ids[i]}, "
          f"v=({vel_x[i]:.4f}, {vel_y[i]:.4f}), speed={s:.4f} m/s")

# ═══════════════════════════════════════════════════════════════════════════
# VERIFY METRIC 2: Path Efficiency (one specific pedestrian)
# ═══════════════════════════════════════════════════════════════════════════
print("\n" + "─" * 80)
print("METRIC 2: PATH EFFICIENCY (per agent)")
print("─" * 80)
print("Definition: straight_line_distance(start→end) / total_path_length")
print("  1.0 = perfectly straight path")
print("  <1.0 = detours or oscillation")
print("  Computed per pedestrian, then averaged.\n")

# Pick pedestrian 1 as example
test_pid = 1
mask = ped_ids == test_pid
idx = np.where(mask)[0]
order = np.argsort(frames[idx])
sorted_idx = idx[order]
pos = np.column_stack([pos_x[sorted_idx], pos_y[sorted_idx]])
frm = frames[sorted_idx]

print(f"  Pedestrian {test_pid}: {len(sorted_idx)} observations")
print(f"    Start: ({pos[0, 0]:.3f}, {pos[0, 1]:.3f}) at frame {frm[0]}")
print(f"    End:   ({pos[-1, 0]:.3f}, {pos[-1, 1]:.3f}) at frame {frm[-1]}")

straight = np.linalg.norm(pos[-1] - pos[0])
segments = np.linalg.norm(np.diff(pos, axis=0), axis=1)
total_path = np.sum(segments)
efficiency = straight / total_path if total_path > 0 else 0

print(f"    Straight-line distance: {straight:.4f} m")
print(f"    Total path walked:      {total_path:.4f} m")
print(f"    ► Path efficiency:      {efficiency:.4f}")
print(f"    (First 5 segment lengths: {segments[:5].round(4).tolist()})")

# Compute for ALL pedestrians
all_eff = []
for pid in unique_peds:
    m = ped_ids == pid
    ix = np.where(m)[0]
    if len(ix) < 3:
        continue
    o = np.argsort(frames[ix])
    si = ix[o]
    p = np.column_stack([pos_x[si], pos_y[si]])
    sl = np.linalg.norm(p[-1] - p[0])
    tp = np.sum(np.linalg.norm(np.diff(p, axis=0), axis=1))
    if tp > 0.1:
        all_eff.append(sl / tp)

print(f"\n  All pedestrians: {len(all_eff)} valid trajectories")
print(f"  ► Mean path efficiency: {np.mean(all_eff):.4f}")
print(f"  ► Std path efficiency:  {np.std(all_eff):.4f}")

# ═══════════════════════════════════════════════════════════════════════════
# VERIFY METRIC 3: Minimum Inter-Agent Distance (one specific frame)
# ═══════════════════════════════════════════════════════════════════════════
print("\n" + "─" * 80)
print("METRIC 3: MINIMUM INTER-AGENT DISTANCE (pairwise)")
print("─" * 80)
print("Definition: For each agent at each frame, the distance to their")
print("  nearest neighbour. Computed as Euclidean distance in meters.")
print("  Low values = agents are close together (tight avoidance).\n")

# Pick a frame with multiple agents
test_frame = None
for f in unique_frames:
    n = np.sum(frames == f)
    if n >= 5:
        test_frame = f
        break

if test_frame is not None:
    mask_f = frames == test_frame
    pos_f = np.column_stack([pos_x[mask_f], pos_y[mask_f]])
    pids_f = ped_ids[mask_f]
    n_agents = len(pos_f)

    print(f"  Frame {test_frame}: {n_agents} agents present")
    print(f"  Agent positions:")
    for k in range(n_agents):
        print(f"    ped {pids_f[k]}: ({pos_f[k, 0]:.3f}, {pos_f[k, 1]:.3f})")

    print(f"\n  Pairwise distances:")
    min_dists = []
    for k in range(n_agents):
        dists = np.linalg.norm(pos_f[k] - pos_f, axis=1)
        dists[k] = np.inf
        min_d = np.min(dists)
        nearest = np.argmin(dists)
        min_dists.append(min_d)
        print(f"    ped {pids_f[k]} → nearest is ped {pids_f[nearest]}: {min_d:.4f} m")

    print(f"\n  ► Mean min distance this frame: {np.mean(min_dists):.4f} m")

# ═══════════════════════════════════════════════════════════════════════════
# VERIFY METRIC 4: Collision & Near-Miss Counts
# ═══════════════════════════════════════════════════════════════════════════
print("\n" + "─" * 80)
print("METRIC 4: COLLISION & NEAR-MISS RATE")
print("─" * 80)
print("Definition:")
print("  Collision: min inter-agent distance < 0.5m (body overlap)")
print("  Near-miss: min inter-agent distance < 1.0m (uncomfortable)")
print("  Rate = total events / num_pedestrians / duration_seconds\n")

collisions = 0
near_misses = 0
total_checks = 0

for f in unique_frames:
    mask_f = frames == f
    pos_f = np.column_stack([pos_x[mask_f], pos_y[mask_f]])
    n = len(pos_f)
    if n < 2:
        continue
    for k in range(n):
        dists = np.linalg.norm(pos_f[k] - pos_f, axis=1)
        dists[k] = np.inf
        min_d = np.min(dists)
        total_checks += 1
        if min_d < 0.5:
            collisions += 1
        if min_d < 1.0:
            near_misses += 1

duration = (frames.max() - frames.min()) * DT / frame_step
n_peds = len(unique_peds)
coll_rate = collisions / n_peds / duration
nm_rate = near_misses / n_peds / duration

print(f"  Total agent-frame checks: {total_checks}")
print(f"  Collisions (< 0.5m):  {collisions}")
print(f"  Near-misses (< 1.0m): {near_misses}")
print(f"  Duration: {duration:.1f}s, Pedestrians: {n_peds}")
print(f"  ► Collision rate:  {coll_rate:.6f} per agent per second")
print(f"  ► Near-miss rate:  {nm_rate:.6f} per agent per second")

# ═══════════════════════════════════════════════════════════════════════════
# VERIFY METRIC 5: Speed Variability (CV per agent)
# ═══════════════════════════════════════════════════════════════════════════
print("\n" + "─" * 80)
print("METRIC 5: SPEED VARIABILITY (per agent)")
print("─" * 80)
print("Definition: Coefficient of Variation (CV) = std(speed) / mean(speed)")
print("  For each pedestrian along their trajectory.")
print("  Low CV = consistent speed (robotic); High CV = erratic (jerky).\n")

# Show for pedestrian 1
mask = ped_ids == test_pid
spds = np.sqrt(vel_x[mask]**2 + vel_y[mask]**2)
valid = spds[(spds > 0.05) & (spds < 5.0)]
cv = np.std(valid) / np.mean(valid) if np.mean(valid) > 0.05 else 0

print(f"  Pedestrian {test_pid}: {len(valid)} valid speed observations")
print(f"    Speeds: {valid[:8].round(3).tolist()}...")
print(f"    Mean: {np.mean(valid):.4f}, Std: {np.std(valid):.4f}")
print(f"    ► CV = {cv:.4f}")

# All pedestrians
all_cvs = []
for pid in unique_peds:
    m = ped_ids == pid
    s = np.sqrt(vel_x[m]**2 + vel_y[m]**2)
    v = s[(s > 0.05) & (s < 5.0)]
    if len(v) > 2 and np.mean(v) > 0.05:
        all_cvs.append(np.std(v) / np.mean(v))

print(f"\n  All pedestrians: {len(all_cvs)} valid")
print(f"  ► Mean CV: {np.mean(all_cvs):.4f}")
print(f"  ► Std CV:  {np.std(all_cvs):.4f}")

# ═══════════════════════════════════════════════════════════════════════════
# VERIFY METRIC 6: Group Cohesion
# ═══════════════════════════════════════════════════════════════════════════
print("\n" + "─" * 80)
print("METRIC 6: GROUP COHESION")
print("─" * 80)
print("Definition: Mean pairwise distance between members of the same group,")
print("  measured at each frame. Low = tight group; High = loose/splitting.\n")

# Load groups
groups = []
with open(SCRIPT_DIR / "ewap_dataset/seq_eth/groups.txt") as f:
    for line in f:
        ids = [int(x) for x in line.strip().split() if x]
        if len(ids) >= 2:
            groups.append(ids)

print(f"  Total groups: {len(groups)}")
print(f"  First 5 groups: {groups[:5]}")
print(f"  Group sizes: {[len(g) for g in groups[:10]]}")

# Verify one group in detail
test_group = groups[0]
print(f"\n  Verifying group {test_group}:")
cohesion_samples = []
checked_frames = 0
for f in unique_frames[::5]:
    mask_f = frames == f
    pids_f = ped_ids[mask_f]
    pos_f = np.column_stack([pos_x[mask_f], pos_y[mask_f]])
    
    members = [pid for pid in test_group if pid in pids_f]
    if len(members) < 2:
        continue
    checked_frames += 1
    
    member_pos = []
    for pid in members:
        ix = np.where(pids_f == pid)[0][0]
        member_pos.append(pos_f[ix])
    member_pos = np.array(member_pos)
    
    dists = []
    for i in range(len(member_pos)):
        for j in range(i+1, len(member_pos)):
            dists.append(np.linalg.norm(member_pos[i] - member_pos[j]))
    
    cohesion_samples.append(np.mean(dists))
    if checked_frames <= 3:
        print(f"    Frame {f}: members present = {members}, "
              f"positions = {member_pos.round(2).tolist()}, "
              f"pairwise dists = {[round(d, 3) for d in dists]}, "
              f"cohesion = {np.mean(dists):.3f}m")

if cohesion_samples:
    print(f"  Frames checked: {checked_frames}")
    print(f"  ► Group cohesion mean: {np.mean(cohesion_samples):.4f} m")
    print(f"  ► Group cohesion std:  {np.std(cohesion_samples):.4f} m")

# ═══════════════════════════════════════════════════════════════════════════
# CROSS-CHECK AGAINST MAIN SCRIPT OUTPUT
# ═══════════════════════════════════════════════════════════════════════════
print("\n" + "═" * 80)
print("CROSS-CHECK vs MAIN SCRIPT (ground_truth_metrics.json)")
print("═" * 80)

json_path = SCRIPT_DIR / "results" / "ground_truth_metrics.json"
if json_path.exists():
    with open(json_path) as f:
        saved = json.load(f)
    
    if "ewap_eth" in saved:
        s = saved["ewap_eth"]
        checks = [
            ("speed_mean",              mean_spd,                    s.get("speed_mean", 0)),
            ("speed_std",               std_spd,                     s.get("speed_std", 0)),
            ("speed_median",            med_spd,                     s.get("speed_median", 0)),
            ("num_pedestrians",         float(n_peds),               float(s.get("num_pedestrians", 0))),
            ("collision_rate",          coll_rate,                   s.get("collision_rate", 0)),
            ("near_miss_rate",          nm_rate,                     s.get("near_miss_rate", 0)),
            ("total_collisions",        float(collisions),           float(s.get("total_collisions", 0))),
            ("total_near_misses",       float(near_misses),          float(s.get("total_near_misses", 0))),
            ("path_efficiency_mean",    np.mean(all_eff),            s.get("path_efficiency_mean", 0)),
            ("speed_variability_mean",  np.mean(all_cvs),            s.get("speed_variability_mean", 0)),
        ]
        
        all_pass = True
        for name, verified, saved_val in checks:
            match = abs(verified - saved_val) < 0.001
            symbol = "✓" if match else "✗"
            if not match:
                all_pass = False
            print(f"  {symbol} {name:<28} verified={verified:<12.6f} saved={saved_val:<12.6f} "
                  f"{'MATCH' if match else f'DIFF={abs(verified-saved_val):.6f}'}")
        
        print(f"\n  {'ALL CHECKS PASSED ✓' if all_pass else 'SOME CHECKS FAILED ✗'}")
    else:
        print("  ewap_eth not found in JSON")
else:
    print(f"  JSON file not found at {json_path}")

print()
