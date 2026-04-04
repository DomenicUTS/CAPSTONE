#!/usr/bin/env bash
# ============================================================================
# Regenerate All Plots — V2 Metrics Dashboard
# ============================================================================
# Regenerates per-phase dashboards, per-environment summaries, and the
# cross-environment comparison figure.
#
# Usage: bash regenerate_all_plots.sh
# ============================================================================

set -euo pipefail

SCRIPT="src/analysis/crowd_dynamics_evaluator.py"
SIM_ROOT="sim_results"

# Environments and their phase directories
declare -A ENV_PHASES
ENV_PHASES[cafe]="cafe_baseline cafe_phase1_social cafe_phase2_goal cafe_phase3_speed"
ENV_PHASES[central_tunnel]="central_tunnel_baseline central_tunnel_phase1_social central_tunnel_phase2_goal central_tunnel_phase3_speed central_tunnel_phase4_obstacle"
ENV_PHASES[delta]="delta_baseline delta_phase1_social delta_phase2_goal delta_phase3_speed delta_phase4_obstacle"

echo "============================================================"
echo "  Step 1: Regenerate Ground Truth plots"
echo "============================================================"
python3 "$SCRIPT"
echo ""

echo "============================================================"
echo "  Step 2: Regenerate per-phase dashboards"
echo "============================================================"
for env in cafe central_tunnel delta; do
    for phase in ${ENV_PHASES[$env]}; do
        phase_dir="$SIM_ROOT/$env/$phase"
        analysis_dir="$phase_dir/analysis"

        if [[ ! -d "$phase_dir" ]]; then
            echo "  [SKIP] $phase_dir not found"
            continue
        fi

        echo "  [PLOT] $phase"

        # Collect all run CSVs
        CUSTOM_ARGS=""
        for csv in $(find "$phase_dir" -name "true_pos_.csv" -type f | sort); do
            run_name=$(basename "$(dirname "$csv")")
            CUSTOM_ARGS="$CUSTOM_ARGS --custom-dataset $run_name $csv"
        done

        if [[ -z "$CUSTOM_ARGS" ]]; then
            echo "    No CSVs found, skipping"
            continue
        fi

        python3 "$SCRIPT" $CUSTOM_ARGS --output-dir "$analysis_dir" --dt 0.1
    done
done
echo ""

echo "============================================================"
echo "  Step 3: Generate per-environment phase comparisons"
echo "============================================================"
for env in cafe central_tunnel delta; do
    echo "  [SUMMARY] $env"
    python3 "$SCRIPT" --env-summary "$SIM_ROOT/$env" --no-plots
done
echo ""

echo "============================================================"
echo "  Step 4: Generate cross-environment comparison"
echo "============================================================"
python3 "$SCRIPT" --cross-env "$SIM_ROOT" --no-plots
echo ""

echo "============================================================"
echo "  All plots regenerated!"
echo "============================================================"

# Count outputs
echo ""
echo "Output inventory:"
find "$SIM_ROOT" -name "*.png" -newer "$SCRIPT" | wc -l | xargs -I{} echo "  {} new PNG files in sim_results/"
find "src/DATASETS/results" -name "*.png" -newer "$SCRIPT" 2>/dev/null | wc -l | xargs -I{} echo "  {} new PNG files in DATASETS/results/"
