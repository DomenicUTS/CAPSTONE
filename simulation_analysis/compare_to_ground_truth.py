#!/usr/bin/env python3
"""
Compare Simulated Results to Ground Truth (ETH/UCY Datasets)

This script:
1. READS ground truth from src/DATASETS/ (never modifies)
2. READS simulation results from results/run_N/
3. WRITES comparisons to simulation_analysis/comparisons/

Usage:
    python3 simulation_analysis/compare_to_ground_truth.py [run_id]
    python3 simulation_analysis/compare_to_ground_truth.py 1
    python3 simulation_analysis/compare_to_ground_truth.py 1 2 3  # Multiple runs
"""

import json
import csv
import numpy as np
from pathlib import Path
from dataclasses import dataclass
import sys

# Constants
SCRIPT_DIR = Path(__file__).resolve().parent
WORKSPACE_DIR = SCRIPT_DIR.parent
DATASETS_DIR = WORKSPACE_DIR / "src" / "DATASETS"
RESULTS_DIR = WORKSPACE_DIR / "results"
COMPARISONS_DIR = SCRIPT_DIR / "comparisons"
REPORTS_DIR = SCRIPT_DIR / "reports"

# Ground truth dataset info
GROUND_TRUTH_DATASETS = {
    "eth_hotel": {"file": "eth/hotel/true_pos_.csv", "dt": 0.4},
    "eth_univ": {"file": "eth/univ/true_pos_.csv", "dt": 0.4},
    "ucy_zara01": {"file": "ucy/zara/zara01/true_pos_.csv", "dt": 0.4},
    "ucy_zara02": {"file": "ucy/zara/zara02/true_pos_.csv", "dt": 0.4},
    "ucy_zara03": {"file": "ucy/zara/zara03/true_pos_.csv", "dt": 0.4},
    "ucy_univ_s1": {"file": "ucy/univ/students001/true_pos_.csv", "dt": 0.4},
    "ucy_univ_s3": {"file": "ucy/univ/students003/true_pos_.csv", "dt": 0.4},
}

# Collision/near-miss thresholds
COLLISION_RADIUS = 0.5
NEAR_MISS_RADIUS = 1.0


@dataclass
class CrowdMetrics:
    """Crowd dynamics metrics"""
    dataset_name: str = ""
    num_agents: int = 0
    duration: float = 0.0
    
    speed_mean: float = 0.0
    speed_std: float = 0.0
    speed_median: float = 0.0
    
    collision_rate: float = 0.0
    near_miss_rate: float = 0.0
    
    path_efficiency_mean: float = 0.0
    acceleration_mean: float = 0.0
    speed_variability_mean: float = 0.0
    
    min_inter_agent_dist_mean: float = 0.0
    min_inter_agent_dist_p5: float = 0.0


def load_true_pos(csv_file: Path) -> tuple:
    """Load true_pos_.csv format trajectory data"""
    with open(csv_file) as f:
        rows = list(csv.reader(f))
    
    frames = np.array([int(float(x)) for x in rows[0]])
    ped_ids = np.array([int(float(x)) for x in rows[1]])
    pos_x = np.array([float(v) for v in rows[2]])
    pos_y = np.array([float(v) for v in rows[3]])
    positions = np.column_stack([pos_x, pos_y])
    
    return frames, ped_ids, positions


def compute_metrics(frames, ped_ids, positions, duration, dataset_name="simulation"):
    """Compute crowd metrics from trajectory data"""
    unique_peds = np.unique(ped_ids)
    unique_frames = np.unique(frames)
    
    metrics = CrowdMetrics(
        dataset_name=dataset_name,
        num_agents=len(unique_peds),
        duration=duration
    )
    
    # Compute velocities
    velocities = np.zeros_like(positions)
    frame_step = float(np.median(np.diff(unique_frames))) if len(unique_frames) > 1 else 1.0
    
    for pid in unique_peds:
        mask = ped_ids == pid
        idx = np.where(mask)[0]
        if len(idx) < 2:
            continue
        order = np.argsort(frames[idx])
        sorted_idx = idx[order]
        sorted_frames = frames[sorted_idx]
        sorted_pos = positions[sorted_idx]
        
        time_per_frame = duration / (unique_frames.max() - unique_frames.min()) if unique_frames.max() > unique_frames.min() else 1.0
        
        for k in range(1, len(sorted_idx)):
            df = sorted_frames[k] - sorted_frames[k - 1]
            if df == 0:
                df = 1
            actual_dt = df * time_per_frame
            velocities[sorted_idx[k]] = (sorted_pos[k] - sorted_pos[k - 1]) / actual_dt if actual_dt > 0 else 0
        if len(sorted_idx) > 1:
            velocities[sorted_idx[0]] = velocities[sorted_idx[1]]
    
    # Speed distribution
    speeds = np.linalg.norm(velocities, axis=1)
    valid_speed_mask = (speeds > 0.05) & (speeds < 5.0)
    valid_speeds = speeds[valid_speed_mask]
    
    if len(valid_speeds) > 0:
        metrics.speed_mean = float(np.mean(valid_speeds))
        metrics.speed_std = float(np.std(valid_speeds))
        metrics.speed_median = float(np.median(valid_speeds))
    
    # Path efficiency
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
    
    if all_eff:
        metrics.path_efficiency_mean = float(np.mean(all_eff))
    
    # Collision & near-miss rates
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
            if min_d < COLLISION_RADIUS:
                collisions += 1
            if min_d < NEAR_MISS_RADIUS:
                near_misses += 1
    
    if metrics.num_agents > 0 and duration > 0:
        metrics.collision_rate = collisions / metrics.num_agents / duration
        metrics.near_miss_rate = near_misses / metrics.num_agents / duration
    
    # Acceleration
    accels_all = []
    for pid in unique_peds:
        m = ped_ids == pid
        idx = np.where(m)[0]
        if len(idx) < 2:
            continue
        o = np.argsort(frames[idx])
        si = idx[o]
        frm = frames[si]
        vel = velocities[si]
        
        time_per_frame = duration / (unique_frames.max() - unique_frames.min()) if unique_frames.max() > unique_frames.min() else 1.0
        
        for k in range(1, len(si)):
            df = frm[k] - frm[k - 1]
            if df > 0:
                local_dt = df * time_per_frame
                acc = np.linalg.norm(vel[k] - vel[k - 1]) / local_dt if local_dt > 0 else 0
                if 0 < acc < 20:
                    accels_all.append(acc)
    
    if accels_all:
        metrics.acceleration_mean = float(np.mean(accels_all))
    
    # Speed variability (CV per agent)
    all_cvs = []
    for pid in unique_peds:
        m = ped_ids == pid
        s = np.linalg.norm(velocities[m], axis=1)
        v = s[(s > 0.05) & (s < 5.0)]
        if len(v) > 2 and np.mean(v) > 0.05:
            all_cvs.append(np.std(v) / np.mean(v))
    
    if all_cvs:
        metrics.speed_variability_mean = float(np.mean(all_cvs))
    
    # Min inter-agent distance
    min_dists_all = []
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
            if min_d < np.inf:
                min_dists_all.append(min_d)
    
    if min_dists_all:
        min_dists_arr = np.array(min_dists_all)
        metrics.min_inter_agent_dist_mean = float(np.mean(min_dists_arr))
        metrics.min_inter_agent_dist_p5 = float(np.percentile(min_dists_arr, 5))
    
    return metrics


def load_ground_truth_metrics(dataset_name: str) -> CrowdMetrics:
    """Load and compute ground truth metrics from ETH/UCY data"""
    if dataset_name not in GROUND_TRUTH_DATASETS:
        raise ValueError(f"Unknown dataset: {dataset_name}")
    
    info = GROUND_TRUTH_DATASETS[dataset_name]
    csv_file = DATASETS_DIR / info["file"]
    
    if not csv_file.exists():
        raise FileNotFoundError(f"Ground truth file not found: {csv_file}")
    
    print(f"  Loading ground truth: {dataset_name}")
    frames, ped_ids, positions = load_true_pos(csv_file)
    
    # Compute actual duration from frame data first
    unique_frames = np.unique(frames)
    frame_step = float(np.median(np.diff(unique_frames))) if len(unique_frames) > 1 else 1.0
    duration = (unique_frames.max() - unique_frames.min()) * info["dt"] / frame_step
    
    metrics = compute_metrics(frames, ped_ids, positions, 
                             duration=duration, dataset_name=dataset_name)
    
    return metrics


def load_simulation_metrics(run_id: int) -> CrowdMetrics:
    """Load metrics from simulation run"""
    run_dir = RESULTS_DIR / f"run_{run_id}"
    true_pos_file = run_dir / "true_pos_.csv"
    
    if not true_pos_file.exists():
        raise FileNotFoundError(f"Simulation data not found: {true_pos_file}")
    
    # Get actual duration from metrics file
    metrics_file = run_dir / f"baseline_run1.txt_steps_baseline_oat_{run_id}.csv"
    duration = 120  # default
    
    if metrics_file.exists():
        with open(metrics_file) as f:
            reader = csv.reader(f)
            header = next(reader)
            rows = list(reader)
            if rows:
                times = [float(row[0]) for row in rows]
                duration = times[-1] - times[0] if len(times) > 1 else 120
    
    print(f"  Loading simulation: run_{run_id} (duration: {duration:.1f}s)")
    frames, ped_ids, positions = load_true_pos(true_pos_file)
    metrics = compute_metrics(frames, ped_ids, positions, 
                             duration=duration, dataset_name=f"sim_run_{run_id}")
    
    return metrics


def compare_metrics(sim: CrowdMetrics, gt: CrowdMetrics) -> dict:
    """Compare simulation to ground truth"""
    comparison = {}
    
    metrics_to_compare = [
        ("speed_mean", "Speed Mean", "m/s"),
        ("speed_std", "Speed Std", "m/s"),
        ("speed_median", "Speed Median", "m/s"),
        ("acceleration_mean", "Acceleration Mean", "m/s²"),
        ("collision_rate", "Collision Rate", "events/ped/s"),
        ("near_miss_rate", "Near-Miss Rate", "events/ped/s"),
        ("path_efficiency_mean", "Path Efficiency", ""),
        ("speed_variability_mean", "Speed Variability", "CV"),
        ("min_inter_agent_dist_mean", "Min Distance Mean", "m"),
        ("min_inter_agent_dist_p5", "Min Distance P5", "m"),
    ]
    
    for attr, label, unit in metrics_to_compare:
        sim_val = getattr(sim, attr, 0.0)
        gt_val = getattr(gt, attr, 0.0)
        
        if gt_val != 0:
            rel_err = (sim_val - gt_val) / abs(gt_val)
            pct_err = rel_err * 100
        else:
            rel_err = 0 if sim_val == 0 else float('inf')
            pct_err = 0
        
        comparison[label] = {
            "simulation": round(sim_val, 4),
            "ground_truth": round(gt_val, 4),
            "relative_error": round(rel_err, 4),
            "percent_error": round(pct_err, 1),
            "unit": unit,
        }
    
    return comparison


def main():
    COMPARISONS_DIR.mkdir(parents=True, exist_ok=True)
    REPORTS_DIR.mkdir(parents=True, exist_ok=True)
    
    # Get run IDs from command line
    if len(sys.argv) > 1:
        run_ids = [int(x) for x in sys.argv[1:]]
    else:
        # Default: find all runs
        run_ids = sorted([int(d.name.split('_')[1]) for d in RESULTS_DIR.glob('run_*')])
    
    # Load ETH Univ as reference (most cited)
    print("\n" + "=" * 100)
    print("LOADING GROUND TRUTH BENCHMARKS")
    print("=" * 100)
    
    try:
        gt_eth_univ = load_ground_truth_metrics("eth_univ")
        print(f"✓ Loaded eth_univ: {gt_eth_univ.num_agents} agents × {gt_eth_univ.duration:.1f}s")
    except Exception as e:
        print(f"✗ Error loading ground truth: {e}")
        return
    
    # Compare each simulation run
    print("\n" + "=" * 100)
    print("COMPARING SIMULATION RESULTS TO GROUND TRUTH")
    print("=" * 100)
    
    all_comparisons = {}
    
    for run_id in run_ids:
        print(f"\nRun {run_id}:")
        try:
            sim_metrics = load_simulation_metrics(run_id)
            comparison = compare_metrics(sim_metrics, gt_eth_univ)
            all_comparisons[f"run_{run_id}"] = {
                "simulation": sim_metrics.__dict__,
                "ground_truth": gt_eth_univ.__dict__,
                "comparison": comparison,
            }
            
            # Print summary
            print(f"\n{'Metric':<25} {'Simulation':<15} {'GT (ETH Univ)':<15} {'Deviation':<15}")
            print("─" * 70)
            for metric_name, comp_data in comparison.items():
                print(f"{metric_name:<25} {comp_data['simulation']:<14} {comp_data['ground_truth']:<14} {comp_data['percent_error']:>+6.1f}%")
        
        except Exception as e:
            print(f"✗ Error: {e}")
    
    # Save comparison results
    output_file = COMPARISONS_DIR / "simulation_vs_ground_truth.json"
    with open(output_file, 'w') as f:
        json.dump(all_comparisons, f, indent=2, default=str)
    print(f"\n✓ Results saved to {output_file}")
    
    # Create comparison report
    report_file = REPORTS_DIR / "comparison_report.txt"
    with open(report_file, 'w') as f:
        f.write("=" * 100 + "\n")
        f.write("SIMULATION vs GROUND TRUTH COMPARISON REPORT\n")
        f.write("=" * 100 + "\n\n")
        
        for run_name, data in all_comparisons.items():
            f.write(f"\n{run_name.upper()}\n")
            f.write("─" * 100 + "\n")
            f.write(f"Duration: {data['simulation']['duration']:.1f}s (simulation) vs {data['ground_truth']['duration']:.1f}s (GT)\n")
            f.write(f"Agents: {data['simulation']['num_agents']} (simulation) vs {data['ground_truth']['num_agents']} (GT)\n\n")
            
            f.write(f"{'Metric':<30} {'Simulation':<15} {'Ground Truth':<15} {'Deviation':<15}\n")
            f.write("─" * 100 + "\n")
            
            for metric_name, comp_data in data['comparison'].items():
                f.write(f"{metric_name:<30} {comp_data['simulation']:<14} {comp_data['ground_truth']:<14} {comp_data['percent_error']:>+6.1f}%\n")
    
    print(f"✓ Report saved to {report_file}")


if __name__ == "__main__":
    main()
