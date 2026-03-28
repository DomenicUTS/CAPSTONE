#!/usr/bin/env python3
"""
Generate One-At-A-Time (OAT) sensitivity test scenarios.
Based on Balanced profile, vary one parameter at a time.
"""

import yaml
import numpy as np
from pathlib import Path

np.random.seed(42)

# Baseline (Balanced) parameters
BASELINE = {
    'max_vel': {'mean': 1.05, 'std': 0.15},
    'social_force_factor': {'mean': 12.0, 'std': 2.0},
    'goal_force_factor': {'mean': 3.0, 'std': 0.5},
    'obstacle_force_factor': {'mean': 15.0, 'std': 3.0},
}

# OAT variations (vary ONE parameter, keep others at baseline means)
OAT_VARIATIONS = {
    'balanced_baseline': BASELINE,
    
    # Vary social force
    'social_low': {
        'max_vel': {'mean': 1.05, 'std': 0.15},
        'social_force_factor': {'mean': 5.0, 'std': 1.0},  # ← VARIED
        'goal_force_factor': {'mean': 3.0, 'std': 0.5},
        'obstacle_force_factor': {'mean': 15.0, 'std': 3.0},
    },
    'social_high': {
        'max_vel': {'mean': 1.05, 'std': 0.15},
        'social_force_factor': {'mean': 20.0, 'std': 2.0},  # ← VARIED
        'goal_force_factor': {'mean': 3.0, 'std': 0.5},
        'obstacle_force_factor': {'mean': 15.0, 'std': 3.0},
    },
    
    # Vary goal force
    'goal_low': {
        'max_vel': {'mean': 1.05, 'std': 0.15},
        'social_force_factor': {'mean': 12.0, 'std': 2.0},
        'goal_force_factor': {'mean': 1.0, 'std': 0.3},  # ← VARIED
        'obstacle_force_factor': {'mean': 15.0, 'std': 3.0},
    },
    'goal_high': {
        'max_vel': {'mean': 1.05, 'std': 0.15},
        'social_force_factor': {'mean': 12.0, 'std': 2.0},
        'goal_force_factor': {'mean': 5.0, 'std': 0.8},  # ← VARIED
        'obstacle_force_factor': {'mean': 15.0, 'std': 3.0},
    },
    
    # Vary speed
    'speed_slow': {
        'max_vel': {'mean': 0.7, 'std': 0.12},  # ← VARIED
        'social_force_factor': {'mean': 12.0, 'std': 2.0},
        'goal_force_factor': {'mean': 3.0, 'std': 0.5},
        'obstacle_force_factor': {'mean': 15.0, 'std': 3.0},
    },
    'speed_fast': {
        'max_vel': {'mean': 1.5, 'std': 0.15},  # ← VARIED
        'social_force_factor': {'mean': 12.0, 'std': 2.0},
        'goal_force_factor': {'mean': 3.0, 'std': 0.5},
        'obstacle_force_factor': {'mean': 15.0, 'std': 3.0},
    },
}

# Same as before
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

SPAWN_REGION = {
    'x_min': -5.0, 'x_max': 3.5,
    'y_min': -9.0, 'y_max': 6.0,
}

MIN_SPAWN_SEPARATION = 0.6
NUM_AGENTS = 25
RADIUS = 0.4


def sample_valid_spawn(existing_spawns, region, min_separation, radius):
    """Sample a spawn position that doesn't overlap with existing agents."""
    max_attempts = 100
    for attempt in range(max_attempts):
        x = np.random.uniform(region['x_min'], region['x_max'])
        y = np.random.uniform(region['y_min'], region['y_max'])
        
        valid = True
        for ex_x, ex_y in existing_spawns:
            dist = np.sqrt((x - ex_x)**2 + (y - ex_y)**2)
            if dist < (min_separation + 2 * radius):
                valid = False
                break
        
        if valid:
            return (x, y)
    
    return (x, y)


def generate_oat_scenario(variation_name, variation_spec, num_agents=NUM_AGENTS, output_dir=None):
    """Generate a scenario for OAT parameter variation."""
    
    if output_dir is None:
        output_dir = Path(__file__).parent.parent.parent / 'hunav_gazebo_wrapper' / 'scenarios' / 'domenic'
    
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    
    print(f"  Generating OAT: {variation_name}...", end=" ")
    
    scenario = {
        'hunav_loader': {
            'ros__parameters': {
                'yaml_base_name': f'cafe_oat_{variation_name}',
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
        max_vel = float(np.clip(np.random.normal(
            variation_spec['max_vel']['mean'],
            variation_spec['max_vel']['std']
        ), 0.5, 2.0))
        
        social_force_factor = float(np.clip(np.random.normal(
            variation_spec['social_force_factor']['mean'],
            variation_spec['social_force_factor']['std']
        ), 3.0, 25.0))
        
        goal_force_factor = float(np.clip(np.random.normal(
            variation_spec['goal_force_factor']['mean'],
            variation_spec['goal_force_factor']['std']
        ), 1.0, 6.0))
        
        obstacle_force_factor = float(np.clip(np.random.normal(
            variation_spec['obstacle_force_factor']['mean'],
            variation_spec['obstacle_force_factor']['std']
        ), 3.0, 30.0))
        
        x, y = spawns[agent_id - 1]
        h = float(np.random.uniform(-np.pi, np.pi))
        goal_ids = [int(gid) for gid in np.random.choice(list(GLOBAL_GOALS.keys()), size=2, replace=False)]
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
    
    output_file = output_dir / f'cafe_oat_{variation_name}.yaml'
    with open(output_file, 'w') as f:
        yaml.dump(scenario, f, default_flow_style=False, sort_keys=False, allow_unicode=True)
    
    print(f"✓ {output_file.name}")
    return output_file


def main():
    print("\n" + "=" * 70)
    print("OAT SENSITIVITY ANALYSIS: Scenario Generator")
    print("=" * 70)
    
    output_files = []
    for variation_name, variation_spec in OAT_VARIATIONS.items():
        output_file = generate_oat_scenario(variation_name, variation_spec)
        output_files.append(output_file)
    
    print("\n" + "=" * 70)
    print(f"Generated {len(output_files)} OAT scenario files")
    print("\nRun order (recommended):")
    print("  1. cafe_oat_balanced_baseline.yaml (3 runs)")
    print("  2. cafe_oat_social_low.yaml (3 runs)")
    print("  3. cafe_oat_social_high.yaml (3 runs)")
    print("  4. cafe_oat_goal_low.yaml (3 runs)")
    print("  5. cafe_oat_goal_high.yaml (3 runs)")
    print("  6. cafe_oat_speed_slow.yaml (3 runs)")
    print("  7. cafe_oat_speed_fast.yaml (3 runs)")
    print("\nTotal: 21 runs (~2-2.5 hours)")
    print("=" * 70 + "\n")


if __name__ == '__main__':
    main()
