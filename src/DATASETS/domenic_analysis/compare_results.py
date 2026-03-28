#!/usr/bin/env python3
"""
Comparison and analysis script for SFM parameter study.
Compares simulation metrics against ETH/UCY ground truth ranges.
"""

import json
import pandas as pd
import numpy as np
from pathlib import Path
import csv

# Ground truth benchmark ranges (from ETH/UCY datasets)
GROUND_TRUTH_RANGES = {
    'mean_speed': {'min': 0.64, 'max': 1.46, 'unit': 'm/s'},
    'speed_std': {'min': 0.34, 'max': 0.53, 'unit': 'm/s'},
    'collision_rate': {'min': 0.0004, 'max': 0.094, 'unit': 'events/ped/s'},
    'near_miss_rate': {'min': 0.009, 'max': 0.229, 'unit': 'events/ped/s'},
    'path_efficiency': {'min': 0.87, 'max': 0.97, 'unit': '-'},
    'acceleration_mean': {'min': 0.12, 'max': 0.74, 'unit': 'm/s²'},
    'min_distance': {'min': 0.80, 'max': 1.88, 'unit': 'm'},
}


def extract_metrics_from_csv(csv_file):
    """Extract key metrics from hunav evaluator CSV output."""
    try:
        df = pd.read_csv(str(csv_file))
        metrics = {}
        
        # Map CSV columns to our metric names
        # This depends on actual hunav evaluator output format
        # For now, we'll check what columns exist
        
        if 'mean_speed' in df.columns:
            metrics['mean_speed'] = df['mean_speed'].iloc[0]
        if 'speed_std' in df.columns:
            metrics['speed_std'] = df['speed_std'].iloc[0]
        if 'collision_rate' in df.columns:
            metrics['collision_rate'] = df['collision_rate'].iloc[0]
        if 'near_miss_rate' in df.columns:
            metrics['near_miss_rate'] = df['near_miss_rate'].iloc[0]
        if 'path_efficiency' in df.columns:
            metrics['path_efficiency'] = df['path_efficiency'].iloc[0]
        if 'acceleration_mean' in df.columns:
            metrics['acceleration_mean'] = df['acceleration_mean'].iloc[0]
        if 'min_distance' in df.columns:
            metrics['min_distance'] = df['min_distance'].iloc[0]
        
        return metrics
    except Exception as e:
        print(f"  Error reading {csv_file}: {e}")
        return {}


def compute_alignment_score(metric_value, gt_range):
    """
    Compute how well a metric aligns with ground truth (0-100 score).
    100 = perfect (within range)
    0 = completely off
    """
    if metric_value is None or pd.isna(metric_value):
        return 0
    
    if gt_range['min'] <= metric_value <= gt_range['max']:
        return 100  # Within range
    elif metric_value < gt_range['min']:
        # How far below?
        pct_below = (gt_range['min'] - metric_value) / gt_range['min']
        return max(0, 100 - pct_below * 200)
    else:
        # How far above?
        pct_above = (metric_value - gt_range['max']) / gt_range['max']
        return max(0, 100 - pct_above * 200)


def analyze_runs(base_dir):
    """Analyze all simulation runs in a directory structure."""
    base_path = Path(base_dir)
    
    if not base_path.exists():
        print(f"Directory not found: {base_dir}")
        return {}
    
    results = {}
    
    # Expected structure: simulations/[profile]/[env]/run_N/
    for profile_dir in base_path.glob('*/'):
        if not profile_dir.is_dir():
            continue
        
        profile_name = profile_dir.name
        results[profile_name] = {}
        
        for env_dir in profile_dir.glob('*/'):
            if not env_dir.is_dir():
                continue
            
            env_name = env_dir.name
            results[profile_name][env_name] = {}
            
            run_dirs = sorted(env_dir.glob('run_*/'))
            
            for run_dir in run_dirs:
                run_num = run_dir.name.replace('run_', '')
                
                # Look for metric CSVs
                metric_files = list(run_dir.glob('*.txt.csv'))
                if not metric_files:
                    continue
                
                csv_file = metric_files[0]
                metrics = extract_metrics_from_csv(csv_file)
                
                # Compute alignment scores
                scores = {}
                for metric_name, metric_value in metrics.items():
                    if metric_name in GROUND_TRUTH_RANGES:
                        score = compute_alignment_score(metric_value, GROUND_TRUTH_RANGES[metric_name])
                        scores[metric_name] = score
                
                results[profile_name][env_name][f'run_{run_num}'] = {
                    'metrics': metrics,
                    'scores': scores,
                    'csv_file': str(csv_file),
                }
    
    return results


def print_comparison_table(results):
    """Pretty-print comparison table."""
    print("\n" + "=" * 100)
    print("SFM PARAMETER STUDY: GROUND TRUTH COMPARISON")
    print("=" * 100)
    
    for profile_name in sorted(results.keys()):
        print(f"\n{'Profile: ' + profile_name.upper():^100}")
        print("-" * 100)
        
        profile_data = results[profile_name]
        
        for env_name in sorted(profile_data.keys()):
            env_data = profile_data[env_name]
            
            print(f"\n  Environment: {env_name}")
            print(f"  {'-' * 96}")
            print(f"  {'Metric':<25} {'Run':<8} {'Value':<15} {'GT Range':<20} {'Status':<15}")
            print(f"  {'-' * 96}")
            
            for run_name in sorted(env_data.keys()):
                run_data = env_data[run_name]
                metrics = run_data['metrics']
                scores = run_data['scores']
                
                first_metric = True
                for metric_name in sorted(metrics.keys()):
                    metric_value = metrics[metric_name]
                    score = scores.get(metric_name, 0)
                    gt_range = GROUND_TRUTH_RANGES.get(metric_name, {})
                    
                    if gt_range:
                        status = "✓ IN RANGE" if score == 100 else f"✗ {score:.0f}%"
                        range_str = f"{gt_range['min']:.3f} - {gt_range['max']:.3f}"
                    else:
                        status = "?"
                        range_str = "N/A"
                    
                    metric_prefix = metric_name if first_metric else ""
                    run_prefix = run_name if first_metric else ""
                    
                    print(f"  {metric_prefix:<25} {run_prefix:<8} {metric_value:<15.4f} {range_str:<20} {status:<15}")
                    first_metric = False


def generate_summary_report(results, output_file='parameter_study_report.txt'):
    """Generate a summary report."""
    with open(output_file, 'w') as f:
        f.write("=" * 100 + "\n")
        f.write("SFM PARAMETER STUDY: SUMMARY REPORT\n")
        f.write("=" * 100 + "\n\n")
        
        for profile_name in sorted(results.keys()):
            f.write(f"PROFILE: {profile_name.upper()}\n")
            f.write("-" * 100 + "\n")
            
            profile_data = results[profile_name]
            profile_scores = []
            
            for env_name in sorted(profile_data.keys()):
                f.write(f"\n  Environment: {env_name}\n")
                env_data = profile_data[env_name]
                
                for run_name in sorted(env_data.keys()):
                    run_data = env_data[run_name]
                    scores = run_data['scores']
                    mean_score = np.mean(list(scores.values())) if scores else 0
                    profile_scores.append(mean_score)
                    
                    f.write(f"    {run_name}: avg alignment = {mean_score:.1f}%\n")
            
            avg_profile_score = np.mean(profile_scores) if profile_scores else 0
            f.write(f"\n  Profile Average: {avg_profile_score:.1f}%\n")
            f.write("-" * 100 + "\n\n")
    
    print(f"Report saved to: {output_file}")


def main():
    import sys
    
    # Default: look in src/DATASETS/domenic_analysis/simulations/
    base_dir = Path(__file__).parent / 'simulations'
    
    if len(sys.argv) > 1:
        base_dir = Path(sys.argv[1])
    
    print(f"Analyzing simulations in: {base_dir}")
    
    results = analyze_runs(str(base_dir))
    
    if not results:
        print("No simulation results found. Have you run any simulations yet?")
        print(f"Expected structure: {base_dir}/[profile]/[env]/run_N/*.txt.csv")
        return
    
    print_comparison_table(results)
    generate_summary_report(results)


if __name__ == '__main__':
    main()
