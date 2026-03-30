#!/usr/bin/env python3
"""
Verification Script for Ground Truth Metrics
=============================================
Manually computes metrics step-by-step on the ETH Univ dataset (true_pos_.csv)
and cross-checks against the main analysis script's output.

This verifies that:
  - Velocity computation via finite differences is correct
  - Speed, path efficiency, collision, and other metrics match
  - Group annotations (from EWAP groups.txt) are loaded correctly
"""

import json
import csv
import numpy as np
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
DT = 0.4  # seconds per frame step (2.5 fps)

# ═══════════════════════════════════════════════════════════════════════════
# LOAD DATA
# ═══════════════════════════════════════════════════════════════════════════
print("=" * 80)
print("VERIFICATION: ETH Univ (true_pos_.csv)")
print("=" * 80)

with open(SCRIPT_DIR / "eth/univ/true_pos_.csv") as f:
    rows = list(csv.reader(f))

frames   = np.array([int(float(x)) for x in rows[0]])
ped_ids  = np.array([int(float(x)) for x in rows[1]])
pos_x    = np.array([float(v) for v in rows[2]])
pos_y    = np.array([float(v) for v in rows[3]])
positions = np.column_stack([pos_x, pos_y])

unique_peds = np.unique(ped_ids)
unique_frames = np.unique(frames)
frame_step = float(np.median(np.diff(unique_frames)))

print(f"\nRaw data: {len(frames)} observations")
print(f"Unique pedestrians: {len(unique_peds)}")
print(f"Unique frames: {len(unique_frames)}")
print(f"Frame range: {frames.min()} → {frames.max()}")
print(f"Frame step (median): {frame_step}")
print(f"Duration: ({frames.max()} - {frames.min()}) * {DT} / {frame_step} = "
      f"{(frames.max() - frames.min()) * DT / frame_step:.1f} seconds")

# ═══════════════════════════════════════════════════════════════════════════
# COMPUTE VELOCITIES (finite differences — same logic as ground_truth_analysis.py)
# ═══════════════════════════════════════════════════════════════════════════
print("\n" + "─" * 80)
print("VELOCITY COMPUTATION (finite differences)")
print("─" * 80)
print(f"Method: v(t) = (pos(t) - pos(t-1)) / actual_dt")
print(f"  actual_dt = frame_diff * {DT} / {frame_step}")

velocities = np.zeros_like(positions)
for pid in unique_peds:
    mask = ped_ids == pid
    idx = np.where(mask)[0]
    if len(idx) < 2:
        continue
    order = np.argsort(frames[idx])
    sorted_idx = idx[order]
    sorted_frames = frames[sorted_idx]
    sorted_pos = positions[sorted_idx]

    for k in range(1, len(sorted_idx)):
        df = sorted_frames[k] - sorted_frames[k - 1]
        if df == 0:
            df = 1
        actual_dt = df * DT / frame_step
        if actual_dt <= 0 or actual_dt > 10:
            actual_dt = DT
        velocities[sorted_idx[k]] = (sorted_pos[k] - sorted_pos[k - 1]) / actual_dt
    if len(sorted_idx) > 1:
        velocities[sorted_idx[0]] = velocities[sorted_idx[1]]

# Spot-check first pedestrian
test_pid = unique_peds[0]
mask_tp = ped_ids == test_pid
idx_tp = np.where(mask_tp)[0]
order_tp = np.argsort(frames[idx_tp])
si_tp = idx_tp[order_tp]
print(f"\nSpot-check pedestrian {test_pid} ({len(si_tp)} obs):")
for k in range(min(5, len(si_tp))):
    spd = np.linalg.norm(velocities[si_tp[k]])
    print(f"  frame={frames[si_tp[k]]}, pos=({pos_x[si_tp[k]]:.4f}, {pos_y[si_tp[k]]:.4f}), "
          f"vel=({velocities[si_tp[k],0]:.4f}, {velocities[si_tp[k],1]:.4f}), speed={spd:.4f} m/s")

# ═══════════════════════════════════════════════════════════════════════════
# VERIFY METRIC 1: Speed Distribution
# ═══════════════════════════════════════════════════════════════════════════
print("\n" + "─" * 80)
print("METRIC 1: SPEED DISTRIBUTION")
print("─" * 80)

speeds = np.linalg.norm(velocities, axis=1)
valid_mask = (speeds > 0.05) & (speeds < 5.0)
valid_speeds = speeds[valid_mask]

mean_spd = np.mean(valid_speeds)
std_spd = np.std(valid_speeds)
med_spd = np.median(valid_speeds)

print(f"  All speeds: {len(speeds)} observations")
print(f"  After filtering: {len(valid_speeds)} valid "
      f"(removed {len(speeds) - len(valid_speeds)} static/noisy)")
print(f"  ► Mean speed:   {mean_spd:.4f} m/s")
print(f"  ► Std speed:    {std_spd:.4f} m/s")
print(f"  ► Median speed: {med_spd:.4f} m/s")

# ═══════════════════════════════════════════════════════════════════════════
# VERIFY METRIC 2: Path Efficiency
# ═══════════════════════════════════════════════════════════════════════════
print("\n" + "─" * 80)
print("METRIC 2: PATH EFFICIENCY")
print("─" * 80)

# Show one pedestrian in detail
mask = ped_ids == test_pid
idx = np.where(mask)[0]
order = np.argsort(frames[idx])
sorted_idx = idx[order]
pos = positions[sorted_idx]
frm = frames[sorted_idx]

straight = np.linalg.norm(pos[-1] - pos[0])
segments = np.linalg.norm(np.diff(pos, axis=0), axis=1)
total_path = np.sum(segments)
efficiency = straight / total_path if total_path > 0 else 0

print(f"  Pedestrian {test_pid}: {len(sorted_idx)} observations")
print(f"    Start: ({pos[0, 0]:.3f}, {pos[0, 1]:.3f}) → End: ({pos[-1, 0]:.3f}, {pos[-1, 1]:.3f})")
print(f"    Straight-line: {straight:.4f} m, Total path: {total_path:.4f} m")
print(f"    ► Path efficiency: {efficiency:.4f}")

# All pedestrians
all_eff = []
for pid in unique_peds:
    m = ped_ids == pid
    ix = np.where(m)[0]
    if len(ix) < 3:
        continue
    o = np.argsort(frames[ix])
    si = ix[o]
    p = positions[si]
    sl = np.linalg.norm(p[-1] - p[0])
    tp = np.sum(np.linalg.norm(np.diff(p, axis=0), axis=1))
    if tp > 0.1:
        all_eff.append(sl / tp)

print(f"\n  All pedestrians: {len(all_eff)} valid")
print(f"  ► Mean path efficiency: {np.mean(all_eff):.4f}")

# ═══════════════════════════════════════════════════════════════════════════
# VERIFY METRIC 3: Collision & Near-Miss Rates
# ═══════════════════════════════════════════════════════════════════════════
print("\n" + "─" * 80)
print("METRIC 3: COLLISION & NEAR-MISS RATE")
print("─" * 80)

collisions = 0
near_misses = 0

for f in unique_frames:
    mask_f = frames == f
    pos_f = positions[mask_f]
    n = len(pos_f)
    if n < 2:
        continue
    for k in range(n):
        dists = np.linalg.norm(pos_f[k] - pos_f, axis=1)
        dists[k] = np.inf
        min_d = np.min(dists)
        if min_d < 0.5:
            collisions += 1
        if min_d < 1.0:
            near_misses += 1

duration = (frames.max() - frames.min()) * DT / frame_step
n_peds = len(unique_peds)
coll_rate = collisions / n_peds / duration
nm_rate = near_misses / n_peds / duration

print(f"  Collisions (< 0.5m):  {collisions}")
print(f"  Near-misses (< 1.0m): {near_misses}")
print(f"  Duration: {duration:.1f}s, Pedestrians: {n_peds}")
print(f"  ► Collision rate:  {coll_rate:.6f} per agent per second")
print(f"  ► Near-miss rate:  {nm_rate:.6f} per agent per second")

# ═══════════════════════════════════════════════════════════════════════════
# VERIFY METRIC 4: Speed Variability (CV per agent)
# ═══════════════════════════════════════════════════════════════════════════
print("\n" + "─" * 80)
print("METRIC 4: SPEED VARIABILITY (per agent)")
print("─" * 80)

all_cvs = []
for pid in unique_peds:
    m = ped_ids == pid
    s = np.linalg.norm(velocities[m], axis=1)
    v = s[(s > 0.05) & (s < 5.0)]
    if len(v) > 2 and np.mean(v) > 0.05:
        all_cvs.append(np.std(v) / np.mean(v))

print(f"  Valid pedestrians: {len(all_cvs)}")
print(f"  ► Mean CV: {np.mean(all_cvs):.4f}")

# ═══════════════════════════════════════════════════════════════════════════
# VERIFY METRIC 5: Group Cohesion
# ═══════════════════════════════════════════════════════════════════════════
print("\n" + "─" * 80)
print("METRIC 5: GROUP COHESION")
print("─" * 80)

groups = []
gpath = SCRIPT_DIR / "eth/univ/groups.txt"
if gpath.exists():
    with open(gpath) as gf:
        for line in gf:
            ids = [int(x) for x in line.strip().split() if x]
            if len(ids) >= 2:
                groups.append(ids)
    print(f"  Loaded {len(groups)} groups from EWAP annotations")

    test_group = groups[0]
    cohesion_samples = []
    for f in unique_frames[::5]:
        mask_f = frames == f
        pids_f = ped_ids[mask_f]
        pos_f = positions[mask_f]
        members = [pid for pid in test_group if pid in pids_f]
        if len(members) < 2:
            continue
        member_pos = np.array([pos_f[np.where(pids_f == pid)[0][0]] for pid in members])
        dists = [np.linalg.norm(member_pos[i] - member_pos[j])
                 for i in range(len(member_pos)) for j in range(i+1, len(member_pos))]
        cohesion_samples.append(np.mean(dists))

    if cohesion_samples:
        print(f"  Group {test_group}: cohesion = {np.mean(cohesion_samples):.4f}m")
else:
    print("  No group file found")

# ═══════════════════════════════════════════════════════════════════════════
# CROSS-CHECK AGAINST MAIN SCRIPT OUTPUT
# ═══════════════════════════════════════════════════════════════════════════
print("\n" + "═" * 80)
print("CROSS-CHECK vs MAIN SCRIPT (ground_truth_metrics.json)")
print("═" * 80)

json_path = SCRIPT_DIR / "results" / "ground_truth_metrics.json"
if json_path.exists():
    with open(json_path) as jf:
        saved = json.load(jf)
    
    if "eth_univ" in saved:
        s = saved["eth_univ"]
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
        print("  eth_univ not found in JSON")
else:
    print(f"  JSON file not found at {json_path}")

print()
