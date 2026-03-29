#!/usr/bin/env python3
"""
Script to run a recording test with proper service calls.
"""

import rclpy
from rclpy.node import Node
from hunav_msgs.srv import StartEvaluation
from std_srvs.srv import Empty
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
import time
import sys

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
    rclpy.init()
    
    node = RecordingTest()
    
    try:
        # Start recording
        if not node.start_recording(run_id=1, experiment_tag='baseline_oat'):
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
