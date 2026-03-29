#!/bin/bash

source install/setup.bash

echo "=== Checking available services ==="
ros2 service list | grep -E "(compute_agents|start_evaluation|hunav)" || echo "No hunav services found!"

echo ""
echo "=== Checking available nodes ==="
ros2 node list | grep -E "(hunav|agent)" || echo "No hunav nodes found!"

echo ""
echo "=== Checking service types ==="
ros2 service type /hunav_start_evaluation 2>/dev/null || echo "hunav_start_evaluation not found"
ros2 service type /compute_agents 2>/dev/null || echo "compute_agents not found"
