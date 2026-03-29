#!/usr/bin/env python3
"""
Script to run a recording test with proper service calls.
Automatically updates the metrics.yaml result_file for each run.
"""

import rclpy
from rclpy.node import Node
from hunav_msgs.srv import StartEvaluation
from std_srvs.srv import Empty
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
import time
import sys
import os
import subprocess

def update_metrics_config(run_id):
    """Update metrics.yaml to save results in the correct run directory"""
    # Update the installed config (what ROS actually uses)
    config_file = os.path.expanduser('~/sfm_ws_fresh/install/hunav_evaluator/share/hunav_evaluator/config/metrics.yaml')
    result_dir = os.path.expanduser(f'~/sfm_ws_fresh/results/run_{run_id}')
    
    # Ensure the run directory exists
    os.makedirs(result_dir, exist_ok=True)
    
    # Update only the result_file line (preserve formatting/comments)
    with open(config_file, 'r') as f:
        lines = f.readlines()
    
    # Find and replace the result_file line
    for i, line in enumerate(lines):
        if 'result_file:' in line:
            lines[i] = f"    result_file: '{result_dir}/metrics'\n"
            break
    
    with open(config_file, 'w') as f:
        f.writelines(lines)
    
    print(f"Updated installed metrics.yaml: result_file = {result_dir}/metrics")

class RecordingTest(Node):
    def __init__(self):
        super().__init__('recording_test')
        self.client_start = self.create_client(StartEvaluation, '/hunav_start_recording')
        self.client_stop = self.create_client(Empty, '/hunav_stop_recording')
        
    def wait_for_service(self, client, service_name, timeout=10):
        """Wait for service to be available"""
        start = time.time()
        while not client.wait_for_service(timeout_sec=1.0):
            if time.time() - start > timeout:
                self.get_logger().error(f'Service {service_name} not available after {timeout}s')
                return False
            self.get_logger().info(f'Waiting for {service_name} service...')
        self.get_logger().info(f'Service {service_name} is available!')
        return True
    
    def start_recording(self, run_id=1, experiment_tag='baseline_oat'):
        """Call start recording service"""
        if not self.wait_for_service(self.client_start, '/hunav_start_recording'):
            return False
            
        # Create the request
        request = StartEvaluation.Request()
        
        # Fill in robot_goal (PoseStamped)
        header = Header()
        header.frame_id = 'map'
        header.stamp = self.get_clock().now().to_msg()
        
        pose = Pose()
        pose.position = Point(x=0.0, y=0.0, z=0.0)
        pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        
        request.robot_goal = PoseStamped()
        request.robot_goal.header = header
        request.robot_goal.pose = pose
        
        request.experiment_tag = experiment_tag
        request.run_id = run_id
        
        self.get_logger().info(f'Calling /hunav_start_recording with run_id={run_id}, experiment_tag={experiment_tag}')
        
        future = self.client_start.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            self.get_logger().info(f'[OK] Recording started! success={future.result().success}')
            return True
        else:
            self.get_logger().error('Failed to start recording')
            return False
    
    def stop_recording(self):
        """Call stop recording service"""
        if not self.wait_for_service(self.client_stop, '/hunav_stop_recording'):
            return False
            
        request = Empty.Request()
        self.get_logger().info('Calling /hunav_stop_recording...')
        
        future = self.client_stop.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            self.get_logger().info('[OK] Recording stopped!')
            return True
        else:
            self.get_logger().error('Failed to stop recording')
            return False

def main():
    # Get run_id from command line argument (default to 1)
    run_id = 1
    if len(sys.argv) > 1:
        try:
            run_id = int(sys.argv[1])
        except ValueError:
            print(f'Error: run_id must be an integer, got "{sys.argv[1]}"')
            sys.exit(1)
    
    # Update the metrics config before starting ROS
    print(f'Setting up run {run_id}...')
    update_metrics_config(run_id)
    
    rclpy.init()
    
    node = RecordingTest()
    
    try:
        # Start recording
        if not node.start_recording(run_id=run_id, experiment_tag='baseline_oat'):
            sys.exit(1)
        
        # Record for 120 seconds
        node.get_logger().info('Recording for 120 seconds...')
        time.sleep(120)
        
        # Stop recording
        if not node.stop_recording():
            sys.exit(1)
        
        node.get_logger().info('Recording test completed successfully!')
        
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
