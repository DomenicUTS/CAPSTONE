#!/usr/bin/env python3
"""
Generate pure single-parameter YAML files from baseline.
Only the target parameter varies; all else matches baseline.
"""
import yaml
import copy

# Read baseline
with open('src/hunav_gazebo_wrapper/scenarios/domenic/cafe/cafe_oat_balanced_baseline.yaml', 'r') as f:
    baseline = yaml.safe_load(f)

# Scaling ratios
SCALE_SF_LOW = 5.3 / 9.15  # 0.579
SCALE_SF_HIGH = 21.7 / 9.15  # 2.371
SCALE_GF_LOW = 1.28 / 2.73  # 0.469
SCALE_GF_HIGH = 5.55 / 2.73  # 2.033
SCALE_SPEED_SLOW = 0.545 / 1.0601  # 0.514
SCALE_SPEED_FAST = 1.41 / 1.0601  # 1.330

def scale_agent_social_force(agent_data, scale):
    """Scale social_force_factor by ratio, keep all else from baseline"""
    agent_data['behavior']['social_force_factor'] = round(
        agent_data['behavior']['social_force_factor'] * scale, 2
    )

def scale_agent_goal_force(agent_data, scale):
    """Scale goal_force_factor by ratio, keep all else from baseline"""
    agent_data['behavior']['goal_force_factor'] = round(
        agent_data['behavior']['goal_force_factor'] * scale, 2
    )

def scale_agent_speed(agent_data, scale):
    """Scale max_vel by ratio, keep all else from baseline"""
    agent_data['max_vel'] = round(agent_data['max_vel'] * scale, 4)

# Generate social_low.yaml
social_low = copy.deepcopy(baseline)
for i in range(1, 13):
    agent_key = f'agent{i}'
    scale_agent_social_force(social_low['hunav_loader']['ros__parameters'][agent_key], SCALE_SF_LOW)
with open('src/hunav_gazebo_wrapper/scenarios/domenic/cafe/cafe_oat_social_low.yaml', 'w') as f:
    yaml.dump(social_low, f, default_flow_style=False, sort_keys=False)
print("✓ Created cafe_oat_social_low.yaml (SF × 0.579)")

# Generate social_high.yaml
social_high = copy.deepcopy(baseline)
for i in range(1, 13):
    agent_key = f'agent{i}'
    scale_agent_social_force(social_high['hunav_loader']['ros__parameters'][agent_key], SCALE_SF_HIGH)
with open('src/hunav_gazebo_wrapper/scenarios/domenic/cafe/cafe_oat_social_high.yaml', 'w') as f:
    yaml.dump(social_high, f, default_flow_style=False, sort_keys=False)
print("✓ Created cafe_oat_social_high.yaml (SF × 2.371)")

# Generate goal_low.yaml
goal_low = copy.deepcopy(baseline)
for i in range(1, 13):
    agent_key = f'agent{i}'
    scale_agent_goal_force(goal_low['hunav_loader']['ros__parameters'][agent_key], SCALE_GF_LOW)
with open('src/hunav_gazebo_wrapper/scenarios/domenic/cafe/cafe_oat_goal_low.yaml', 'w') as f:
    yaml.dump(goal_low, f, default_flow_style=False, sort_keys=False)
print("✓ Created cafe_oat_goal_low.yaml (GF × 0.469)")

# Generate goal_high.yaml
goal_high = copy.deepcopy(baseline)
for i in range(1, 13):
    agent_key = f'agent{i}'
    scale_agent_goal_force(goal_high['hunav_loader']['ros__parameters'][agent_key], SCALE_GF_HIGH)
with open('src/hunav_gazebo_wrapper/scenarios/domenic/cafe/cafe_oat_goal_high.yaml', 'w') as f:
    yaml.dump(goal_high, f, default_flow_style=False, sort_keys=False)
print("✓ Created cafe_oat_goal_high.yaml (GF × 2.033)")

# Generate speed_slow.yaml
speed_slow = copy.deepcopy(baseline)
for i in range(1, 13):
    agent_key = f'agent{i}'
    scale_agent_speed(speed_slow['hunav_loader']['ros__parameters'][agent_key], SCALE_SPEED_SLOW)
with open('src/hunav_gazebo_wrapper/scenarios/domenic/cafe/cafe_oat_speed_slow.yaml', 'w') as f:
    yaml.dump(speed_slow, f, default_flow_style=False, sort_keys=False)
print("✓ Created cafe_oat_speed_slow.yaml (Speed × 0.514)")

# Generate speed_fast.yaml
speed_fast = copy.deepcopy(baseline)
for i in range(1, 13):
    agent_key = f'agent{i}'
    scale_agent_speed(speed_fast['hunav_loader']['ros__parameters'][agent_key], SCALE_SPEED_FAST)
with open('src/hunav_gazebo_wrapper/scenarios/domenic/cafe/cafe_oat_speed_fast.yaml', 'w') as f:
    yaml.dump(speed_fast, f, default_flow_style=False, sort_keys=False)
print("✓ Created cafe_oat_speed_fast.yaml (Speed × 1.330)")

print("\n✅ ALL PURE SINGLE-PARAMETER YAML FILES REGENERATED")
print("\nScaling applied (all agents scaled proportionally):")
print(f"  Social Force:  baseline → 5.3 (×{SCALE_SF_LOW:.3f}) or 21.7 (×{SCALE_SF_HIGH:.3f})")
print(f"  Goal Force:    baseline → 1.28 (×{SCALE_GF_LOW:.3f}) or 5.55 (×{SCALE_GF_HIGH:.3f})")
print(f"  Max Velocity:  baseline → 0.545 (×{SCALE_SPEED_SLOW:.3f}) or 1.41 (×{SCALE_SPEED_FAST:.3f})")
