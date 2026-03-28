#!/usr/bin/env python3
"""
Generate parametric scenarios for SFM validation study.
Creates 5 profile YAML files (Balanced, Cautious, Aggressive, Dense-Aware, Speed-First)
for cafe_no_robot with 25 agents each, sampling parameters from specified distributions.
"""

import yaml
import numpy as np
import os
from pathlib import Path

# Random seed for reproducibility
np.random.seed(42)

# Profile definitions: specify mean and std for each force factor
PROFILES = {
    'balanced': {
        'description': 'Middle-ground realistic behavior (ETH/UCY baseline reference)',
        'max_vel': {'mean': 1.05, 'std': 0.15},
        'social_force_factor': {'mean': 12.0, 'std': 2.0},
        'goal_force_factor': {'mean': 3.0, 'std': 0.5},
        'obstacle_force_factor': {'mean': 15.0, 'std': 3.0},
    },
    'cautious': {
        'description': 'Safe, polite crowds (high social awareness, low speed)',
        'max_vel': {'mean': 0.85, 'std': 0.12},
        'social_force_factor': {'mean': 16.0, 'std': 2.5},
        'goal_force_factor': {'mean': 2.5, 'std': 0.4},
        'obstacle_force_factor': {'mean': 20.0, 'std': 4.0},
    },
    'aggressive': {
        'description': 'Fast, impatient, risky (low safety margins)',
        'max_vel': {'mean': 1.35, 'std': 0.15},
        'social_force_factor': {'mean': 8.0, 'std': 2.0},
        'goal_force_factor': {'mean': 3.5, 'std': 0.5},
        'obstacle_force_factor': {'mean': 10.0, 'std': 3.0},
    },
    'dense_aware': {
        'description': 'Adapts to crowding (balanced adaptive behavior)',
        'max_vel': {'mean': 0.95, 'std': 0.20},
        'social_force_factor': {'mean': 14.0, 'std': 3.0},
        'goal_force_factor': {'mean': 3.2, 'std': 0.5},
        'obstacle_force_factor': {'mean': 18.0, 'std': 3.5},
    },
    'speed_first': {
        'description': 'Prioritizes goal over safety (goal-driven commute)',
        'max_vel': {'mean': 1.40, 'std': 0.18},
        'social_force_factor': {'mean': 10.0, 'std': 2.5},
        'goal_force_factor': {'mean': 4.0, 'std': 0.6},
        'obstacle_force_factor': {'mean': 12.0, 'std': 3.5},
    },
}

# Global goals pool for cafe
GLOBAL_GOALS = {
    1: {'x': -3.700, 'y': -3.290},
    2: {'x': 0.184, 'y': -4.850},
    3: {'x': 0.269, 'y': -3.204},
    4: {'x': 2.963, 'y': 0.113},
    5: {'x': -2.880, 'y': 0.028},
    6: {'x': 2.000, 'y': 4.226},
    7: {'x': 0.411, 'y': -6.039},
    8: {'x': -1.500, 'y': 2.500},
    9: {'x': 1.500, 'y': 1.500},
    10: {'x': -0.500, 'y': -2.000},
}

# Spawn region for cafe (approximate bounds from cafe.world)
SPAWN_REGION = {
    'x_min': -5.0, 'x_max': 3.5,
    'y_min': -9.0, 'y_max': 6.0,
}

MIN_SPAWN_SEPARATION = 0.6  # meters
NUM_AGENTS = 25
RADIUS = 0.4  # agent collision radius


def sample_valid_spawn(existing_spawns, region, min_separation, radius):
    """Sample a spawn position that doesn't overlap with existing agents."""
    max_attempts = 100
    for attempt in range(max_attempts):
        x = np.random.uniform(region['x_min'], region['x_max'])
        y = np.random.uniform(region['y_min'], region['y_max'])
        
        # Check minimum separation
        valid = True
        for ex_x, ex_y in existing_spawns:
            dist = np.sqrt((x - ex_x)**2 + (y - ex_y)**2)
            if dist < (min_separation + 2 * radius):
                valid = False
                break
        
        if valid:
            return (x, y)
    
    # Fallback if max attempts exceeded
    print(f"  Warning: Could not achieve perfect separation on attempt {max_attempts}. Using best effort.")
    return (x, y)


def generate_scenario(profile_name, profile_spec, num_agents=NUM_AGENTS, output_dir=None):
    """Generate a parametric scenario YAML for the given profile."""
    
    if output_dir is None:
        output_dir = Path(__file__).parent.parent.parent / 'hunav_gazebo_wrapper' / 'scenarios' / 'domenic'
    
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    
    print(f"\nGenerating {profile_name.upper()} scenario ({num_agents} agents)...")
    
    # Initialize structure
    scenario = {
        'hunav_loader': {
            'ros__parameters': {
                'yaml_base_name': f'cafe_24agents_{profile_name}',
                'simulator': 'Gazebo Classic',
                'map': 'cafe',
                'publish_people': True,
                'global_goals': GLOBAL_GOALS,
                'agents': [],
            }
        }
    }
    
    # Generate spawn positions
    spawns = []
    while len(spawns) < num_agents:
        spawn = sample_valid_spawn(spawns, SPAWN_REGION, MIN_SPAWN_SEPARATION, RADIUS)
        spawns.append(spawn)
    
    # Generate agents
    for agent_id in range(1, num_agents + 1):
        # Sample parameters from profile distributions (convert numpy to Python float)
        max_vel = float(np.clip(np.random.normal(
            profile_spec['max_vel']['mean'],
            profile_spec['max_vel']['std']
        ), 0.5, 2.0))
        
        social_force_factor = float(np.clip(np.random.normal(
            profile_spec['social_force_factor']['mean'],
            profile_spec['social_force_factor']['std']
        ), 3.0, 25.0))
        
        goal_force_factor = float(np.clip(np.random.normal(
            profile_spec['goal_force_factor']['mean'],
            profile_spec['goal_force_factor']['std']
        ), 1.0, 6.0))
        
        obstacle_force_factor = float(np.clip(np.random.normal(
            profile_spec['obstacle_force_factor']['mean'],
            profile_spec['obstacle_force_factor']['std']
        ), 3.0, 30.0))
        
        # Spawn position and orientation (convert numpy to Python float)
        x, y = spawns[agent_id - 1]
        h = float(np.random.uniform(-np.pi, np.pi))
        
        # Goal pair (randomly select 2 from goal pool, convert to Python ints)
        goal_ids = [int(gid) for gid in np.random.choice(list(GLOBAL_GOALS.keys()), size=2, replace=False)]
        
        # Skin/appearance (cycle through 0-2 for variety)
        skin = (agent_id - 1) % 3
        
        agent_key = f'agent{agent_id}'
        scenario['hunav_loader']['ros__parameters']['agents'].append(agent_key)
        
        scenario['hunav_loader']['ros__parameters'][agent_key] = {
            'id': agent_id,
            'group_id': -1,
            'skin': skin,
            'max_vel': round(max_vel, 4),
            'radius': RADIUS,
            'goal_radius': 0.3,
            'cyclic_goals': True,
            'init_pose': {
                'x': round(x, 3),
                'y': round(y, 3),
                'z': 1.250,
                'h': round(h, 3),
            },
            'behavior': {
                'type': 'Regular',
                'configuration': 2,
                'goal_force_factor': round(goal_force_factor, 2),
                'obstacle_force_factor': round(obstacle_force_factor, 2),
                'social_force_factor': round(social_force_factor, 2),
                'other_force_factor': 20.0,
            },
            'goals': goal_ids,
        }
    
    # Save to YAML
    output_file = output_dir / f'cafe_24agents_{profile_name}.yaml'
    with open(output_file, 'w') as f:
        yaml.dump(scenario, f, default_flow_style=False, sort_keys=False, allow_unicode=True)
    
    print(f"  ✓ Saved to: {output_file}")
    return output_file


def main():
    """Generate all profile scenarios."""
    print("=" * 70)
    print("SFM Parameter Study: Cafe Scenario Generator")
    print("=" * 70)
    
    output_files = []
    for profile_name, profile_spec in PROFILES.items():
        output_file = generate_scenario(profile_name, profile_spec)
        output_files.append(output_file)
    
    print("\n" + "=" * 70)
    print(f"Generated {len(output_files)} scenario files:")
    for f in output_files:
        print(f"  - {f.name}")
    print("\nTo use a scenario, launch with:")
    print("  ros2 launch hunav_gazebo_wrapper cafe_no_robot.launch.py \\")
    print("    configuration_file:=domenic/cafe_24agents_balanced.yaml")
    print("=" * 70)


if __name__ == '__main__':
    main()
