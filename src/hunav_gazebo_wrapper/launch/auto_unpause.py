#!/usr/bin/env python3
"""
Auto-unpause physics for headless Gazebo mode.
Waits for /gazebo/unpause_physics service and calls it.
"""

import subprocess
import time
import sys
import signal

def timeout_handler(signum, frame):
    print("ERROR: Timeout waiting for unpause_physics service (30s) - service may not exist")
    sys.exit(1)

if __name__ == '__main__':
    try:
        # Wait up to 30 seconds for the service to be available
        print("Waiting for /gazebo/unpause_physics service (max 30 seconds)...")
        signal.signal(signal.SIGALRM, timeout_handler)
        signal.alarm(30)
        
        result = subprocess.run(
            ['ros2', 'service', 'call', '/gazebo/unpause_physics', 'std_srvs/srv/Empty', '{}'],
            timeout=35,
            capture_output=True,
            text=True
        )
        
        signal.alarm(0)  # Cancel the alarm
        
        if result.returncode == 0:
            print("[SUCCESS] Physics unpaused!")
            print(result.stdout)
        else:
            print(f"[ERROR] Failed to unpause physics: {result.stderr}")
            sys.exit(1)
            
    except subprocess.TimeoutExpired:
        print("[ERROR] Service call timed out")
        sys.exit(1)
    except Exception as e:
        print(f"[ERROR] {str(e)}")
        sys.exit(1)
