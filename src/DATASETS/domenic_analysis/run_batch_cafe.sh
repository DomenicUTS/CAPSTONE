#!/bin/bash
# Batch runner for cafe_no_robot parameter study
# This script runs all 5 profiles × 3 runs = 15 simulations for cafe_no_robot
# It requires terminals to be pre-setup:
#   Terminal 1: ros2 launch hunav_gazebo_wrapper cafe_no_robot.launch.py configuration_file:=domenic/cafe_24agents_balanced.yaml
#   Terminal 2: ros2 launch hunav_evaluator hunav_evaluator.launch.py

set -e

WORKSPACE="/home/domenic/sfm_ws_fresh"
SCENARIOS_DIR="$WORKSPACE/src/hunav_gazebo_wrapper/scenarios/domenic"
RESULTS_DIR="$WORKSPACE/results"
SIM_OUTPUT_DIR="$WORKSPACE/src/DATASETS/domenic_analysis/simulations"

# Profile configurations
PROFILES=(
    "cafe_24agents_balanced"
    "cafe_24agents_cautious"
    "cafe_24agents_aggressive"
    "cafe_24agents_dense_aware"
    "cafe_24agents_speed_first"
)

NUM_RUNS=3
RECORD_DURATION=60  # seconds

echo "=========================================="
echo "SFM Parameter Study: Batch Runner"
echo "=========================================="
echo "Profiles: ${#PROFILES[@]}"
echo "Runs per profile: $NUM_RUNS"
echo "Total simulations: $((${#PROFILES[@]} * $NUM_RUNS))"
echo "=========================================="

# Check prerequisites
if [ ! -d "$SCENARIOS_DIR" ]; then
    echo "ERROR: Scenarios directory not found: $SCENARIOS_DIR"
    exit 1
fi

if [ ! -f "$SCENARIOS_DIR/cafe_24agents_balanced.yaml" ]; then
    echo "ERROR: Generated scenario files not found. Run generate_cafe_scenarios.py first."
    exit 1
fi

echo ""
echo "Starting batch run..."
echo "IMPORTANT: Make sure these terminals are running:"
echo "  Terminal 1: ros2 launch hunav_gazebo_wrapper cafe_no_robot.launch.py configuration_file:=..."
echo "  Terminal 2: ros2 launch hunav_evaluator hunav_evaluator.launch.py"
echo ""
read -p "Press ENTER to continue..."

# Main loop
run_count=0
for profile in "${PROFILES[@]}"; do
    profile_name=$(echo "$profile" | sed 's/cafe_24agents_//')
    profile_dir="$SIM_OUTPUT_DIR/$profile_name/cafe"
    mkdir -p "$profile_dir"
    
    echo ""
    echo "========== PROFILE: $profile_name =========="
    
    for run_num in $(seq 1 $NUM_RUNS); do
        run_count=$((run_count + 1))
        run_id=$run_num
        experiment_tag="${profile_name}_run${run_num}"
        
        echo ""
        echo "[$run_count/$((${#PROFILES[@]} * $NUM_RUNS))] Running: $profile_name (Run $run_num)"
        echo "  Scenario: $profile.yaml"
        echo "  Experiment tag: $experiment_tag"
        
        # Start recording
        echo "  Starting recording..."
        ros2 service call /hunav_start_recording hunav_msgs/srv/StartEvaluation \
            "{experiment_tag: '$experiment_tag', run_id: $run_id, robot_goal: {}}" 2>/dev/null || {
            echo "  ERROR: Failed to start recording. Check evaluator is running."
            continue
        }
        
        # Wait for duration
        echo "  Recording... (${RECORD_DURATION}s)"
        sleep "$RECORD_DURATION"
        
        # Stop recording
        echo "  Stopping recording..."
        ros2 service call /hunav_stop_recording std_srvs/srv/Empty "{}" 2>/dev/null || {
            echo "  ERROR: Failed to stop recording."
            continue
        }
        
        sleep 2
        
        # Move results
        if [ -d "$RESULTS_DIR/run_$run_id" ]; then
            echo "  Moving results to $profile_dir/run_$run_num/"
            mv "$RESULTS_DIR/run_$run_id" "$profile_dir/run_$run_num"
            echo "  ✓ Complete"
        else
            echo "  ERROR: Results not found in $RESULTS_DIR/run_$run_id"
        fi
    done
done

echo ""
echo "=========================================="
echo "Batch run complete!"
echo "Results stored in: $SIM_OUTPUT_DIR/"
echo ""
echo "To analyze results, run:"
echo "  python3 $WORKSPACE/src/DATASETS/domenic_analysis/compare_results.py"
echo "=========================================="
