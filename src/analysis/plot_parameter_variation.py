#!/usr/bin/env python3
"""
Plot per-agent SFM parameter variation across scenario conditions.

Works with any environment (cafe, central_tunnel, warehouse, etc.).
Auto-discovers YAML files, detects agent count, and generates a 2×2 plot.

Usage:
  # Auto-discover all YAMLs in a scenario directory
  python3 plot_parameter_variation.py --scenario-dir src/hunav_gazebo_wrapper/scenarios/domenic/central_tunnel

  # Cafe example
  python3 plot_parameter_variation.py --scenario-dir src/hunav_gazebo_wrapper/scenarios/domenic/cafe

  # Specify output path
  python3 plot_parameter_variation.py --scenario-dir <dir> --output results/my_plot.png

  # Manually list YAMLs in order (label=file pairs)
  python3 plot_parameter_variation.py --scenario-dir <dir> \\
      --conditions "Baseline=ct_baseline.yaml" "Phase1 Low SF=ct_phase1_social_low.yaml" ...

  # Set a custom title
  python3 plot_parameter_variation.py --scenario-dir <dir> --title "My Environment"
"""

import argparse
import re
import sys
from pathlib import Path

import numpy as np
import yaml

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
except ImportError:
    print("ERROR: matplotlib required.  pip install matplotlib")
    sys.exit(1)

# ---------- SFM parameters to plot (2×2 grid) ----------
PARAMS = [
    ("max_vel",              "Maximum Velocity (m/s)",  lambda a: a["max_vel"]),
    ("goal_force_factor",    "Goal Force Factor",       lambda a: a["behavior"]["goal_force_factor"]),
    ("social_force_factor",  "Social Force Factor",     lambda a: a["behavior"]["social_force_factor"]),
    ("obstacle_force_factor","Obstacle Force Factor",   lambda a: a["behavior"]["obstacle_force_factor"]),
]

# ---------- Condition ordering priority (for auto-discovery) ----------
SORT_KEYS = {
    "baseline": 0,
    "social_low":  10, "social_high":  11,
    "goal_low":    20, "goal_high":    21,
    "speed_low":   30, "speed_slow":   30, "speed_high":  31, "speed_fast": 31,
    "obstacle_low":40, "obstacle_high":41,
}


def sort_key_for_yaml(name: str) -> int:
    """Return an ordering integer so phases are plotted in logical order."""
    lower = name.lower().replace(".yaml", "")
    for pattern, priority in SORT_KEYS.items():
        if pattern in lower:
            return priority
    return 99  # unknown → end


def label_from_filename(name: str) -> str:
    """Derive a human-readable x-tick label from a YAML filename."""
    stem = Path(name).stem
    # Strip common prefixes like ct_, cafe_oat_, etc.
    stem = re.sub(r'^(ct_|cafe_oat_|cafe_|warehouse_|wh_)', '', stem)
    # Replace underscores and prettify
    parts = stem.split("_")
    mapping = {
        "baseline": "Baseline",
        "social": "Social", "goal": "Goal", "speed": "Speed", "obstacle": "Obstacle",
        "low": "Low", "high": "High", "slow": "Slow", "fast": "Fast",
        "phase1": "Phase1", "phase2": "Phase2", "phase3": "Phase3", "phase4": "Phase4",
        "sf": "SF", "gf": "GF", "of": "OF",
    }
    pretty = [mapping.get(p.lower(), p.capitalize()) for p in parts]
    # Group into two lines if more than 2 words
    if len(pretty) > 2:
        return " ".join(pretty[:len(pretty)//2]) + "\n" + " ".join(pretty[len(pretty)//2:])
    return " ".join(pretty)


def detect_agents(ros_params: dict) -> int:
    """Count agent keys in a YAML's ros__parameters."""
    return sum(1 for k in ros_params if re.match(r'^agent\d+$', k))


def load_conditions(scenario_dir: Path, manual_conditions=None):
    """Return list of (label, yaml_path) in display order."""
    if manual_conditions:
        result = []
        for spec in manual_conditions:
            label, fname = spec.split("=", 1)
            result.append((label, scenario_dir / fname))
        return result

    # Auto-discover
    yamls = sorted(scenario_dir.glob("*.yaml"), key=lambda p: sort_key_for_yaml(p.name))
    if not yamls:
        print(f"ERROR: No .yaml files found in {scenario_dir}")
        sys.exit(1)
    return [(label_from_filename(y.name), y) for y in yamls]


def main():
    parser = argparse.ArgumentParser(
        description="Plot per-agent SFM parameter variation across scenario conditions.")
    parser.add_argument("--scenario-dir", required=True,
                        help="Directory containing scenario YAML files")
    parser.add_argument("--conditions", nargs="+", default=None,
                        help="Ordered label=filename pairs (auto-discovered if omitted)")
    parser.add_argument("--output", default=None,
                        help="Output PNG path (default: results/<env>/parameter_variation_individual_agents.png)")
    parser.add_argument("--title", default=None,
                        help="Plot title (auto-detected from directory name if omitted)")
    parser.add_argument("--dpi", type=int, default=200, help="Output DPI (default: 200)")
    args = parser.parse_args()

    scenario_dir = Path(args.scenario_dir)
    if not scenario_dir.is_dir():
        print(f"ERROR: {scenario_dir} is not a directory")
        sys.exit(1)

    conditions = load_conditions(scenario_dir, args.conditions)
    env_name = scenario_dir.name.replace("_", " ").title()
    title = args.title or f"{env_name}: Parameter Variation Across Agents and Conditions"

    print(f"Environment: {env_name}")
    print(f"Conditions ({len(conditions)}):")
    for label, fpath in conditions:
        print(f"  {label.replace(chr(10), ' '):30s} ← {fpath.name}")

    # ---------- Detect agent count from first YAML ----------
    with open(conditions[0][1]) as f:
        first_cfg = yaml.safe_load(f)
    num_agents = detect_agents(first_cfg["hunav_loader"]["ros__parameters"])
    print(f"Agents: {num_agents}")

    # ---------- Load data ----------
    data = {key: np.zeros((num_agents, len(conditions))) for key, _, _ in PARAMS}

    for ci, (label, fpath) in enumerate(conditions):
        with open(fpath) as f:
            cfg = yaml.safe_load(f)
        ros_params = cfg["hunav_loader"]["ros__parameters"]
        for ai in range(num_agents):
            agent = ros_params[f"agent{ai + 1}"]
            for key, _, extract in PARAMS:
                data[key][ai, ci] = extract(agent)

    # ---------- Build colors ----------
    if num_agents <= 12:
        # Small agent count: distinct colors per agent
        cmap = plt.cm.tab20(np.linspace(0, 1, max(num_agents, 1)))
        colors = cmap[:num_agents]
        legend_entries = [(0, f"Agent 1"), (num_agents - 1, f"Agent {num_agents}")]
    else:
        # Large agent count: split into two halves (e.g. west/east)
        half = num_agents // 2
        c1 = plt.cm.Blues(np.linspace(0.3, 0.9, half))
        c2 = plt.cm.Oranges(np.linspace(0.3, 0.9, num_agents - half))
        colors = np.vstack([c1, c2])
        legend_entries = [(0, f"Agents 1-{half}"), (half, f"Agents {half+1}-{num_agents}")]

    # ---------- Plot ----------
    fig, axes = plt.subplots(2, 2, figsize=(18, 12))
    fig.suptitle(title, fontsize=16, fontweight="bold", y=0.98)

    x = np.arange(len(conditions))
    xlabels = [label for label, _ in conditions]

    for idx, (key, ptitle, _) in enumerate(PARAMS):
        ax = axes[idx // 2][idx % 2]
        vals = data[key]

        for ai in range(num_agents):
            lbl = None
            for li, lt in legend_entries:
                if ai == li:
                    lbl = lt
            ax.plot(x, vals[ai], color=colors[ai], marker="o",
                    markersize=max(1.5, 4 - num_agents * 0.05),
                    linewidth=max(0.5, 1.2 - num_agents * 0.01),
                    alpha=0.7, label=lbl)

        baseline_avg = np.mean(vals[:, 0])
        ax.axhline(baseline_avg, color="red", linestyle="--", linewidth=1.5,
                   label=f"Baseline Avg ({baseline_avg:.2f})")

        ax.set_title(ptitle, fontsize=13, fontweight="bold")
        ax.set_xticks(x)
        ax.set_xticklabels(xlabels, fontsize=max(6, 9 - len(conditions) * 0.3))
        ax.set_ylabel(ptitle.split("(")[0].strip(), fontsize=10)
        ax.legend(fontsize=8, loc="best")
        ax.grid(True, alpha=0.3)

    plt.tight_layout(rect=[0, 0, 1, 0.95])

    if args.output:
        out = Path(args.output)
    else:
        out = Path(f"sim_results/{scenario_dir.name}/parameter_variation_individual_agents.png")
    out.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(out, dpi=args.dpi, bbox_inches="tight")
    print(f"\nSaved → {out}")
    plt.close()


if __name__ == "__main__":
    main()
