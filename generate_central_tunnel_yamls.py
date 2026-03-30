#!/usr/bin/env python3
"""
Generate Central Tunnel OAT Sensitivity Analysis YAMLs + Behavior Trees.

Creates 40 agents for bidirectional corridor flow:
  - 20 agents spawn at WEST end → goals at EAST end
  - 20 agents spawn at EAST end → goals at WEST end
  - Mix of individuals, dyads, and triads
  - Minimum goal distance ~30m (full tunnel traversal)

Uses configuration: 1 (BEH_CONF_CUSTOM) so YAML force factors are
respected exactly — not overridden by random generation.

NOTE: The cafe tests used configuration: 2 (BEH_CONF_RANDOM_NORMAL),
which causes the loader to OVERWRITE YAML force factors with random
normal values at each launch. This means the cafe OAT scaling for
social/goal forces was overridden at runtime. Only max_vel (Phase 3)
was unaffected. For central tunnel we fix this by using configuration: 1.

World geometry (from central_tunnel.world 'Untitled' model):
  Walls center: (4.908, -0.202)
  South wall (Wall_2): Y = -8.877, length 38.5m
  North wall (Wall_4): Y =  8.473, length 38.5m
  West wall  (Wall_3): X = -14.267, height 17.5m
  East wall  (Wall_5): X =  24.083, height 17.5m
  Usable area: X ∈ [-13.5, 23.5], Y ∈ [-8.0, 7.5]
"""

import yaml
import copy
import os
import math
import random

# ──────────────────────────────────────────────────────────────────────
# PATHS
# ──────────────────────────────────────────────────────────────────────
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
SCENARIO_DIR = os.path.join(BASE_DIR, 'src', 'hunav_gazebo_wrapper', 'scenarios', 'domenic', 'central_tunnel')
BT_SRC_DIR = os.path.join(BASE_DIR, 'src', 'hunav_gazebo_wrapper', 'behavior_trees', 'central_tunnel')
BT_FLAT_DIR = os.path.join(BASE_DIR, 'src', 'hunav_gazebo_wrapper', 'behavior_trees')
BT_INSTALL_DIR = os.path.join(BASE_DIR, 'install', 'hunav_gazebo_wrapper', 'share',
                               'hunav_gazebo_wrapper', 'behavior_trees')

YAML_BASE_NAME = 'agents_central_tunnel'
NUM_AGENTS = 40

# ──────────────────────────────────────────────────────────────────────
# TUNNEL GEOMETRY  (with safety margin from walls)
# ──────────────────────────────────────────────────────────────────────
WEST_SPAWN_X_MIN, WEST_SPAWN_X_MAX = -12.0, -9.0
EAST_SPAWN_X_MIN, EAST_SPAWN_X_MAX = 19.0, 22.0
Y_MIN, Y_MAX = -6.5, 6.5        # Y range for spawns/goals (margin from walls)
HEADING_EAST = 0.0               # radians, facing +X
HEADING_WEST = math.pi           # radians, facing -X

# Goal positions: opposite end of tunnel from spawn
WEST_GOAL_X = -11.0             # goals for agents walking west
EAST_GOAL_X = 21.0              # goals for agents walking east


# ──────────────────────────────────────────────────────────────────────
# GROUP DEFINITIONS
# ──────────────────────────────────────────────────────────────────────
# Each group: (group_label, [agent_indices], side)
# agent_indices are 0-based into their spawn side
# Dyads (groups of 2): 5 dyads
# Triads (groups of 3): 3 triads
# Remaining: individuals

# WEST side agents: indices 0-19 → agent IDs 1-20
# EAST side agents: indices 0-19 → agent IDs 21-40

WEST_GROUPS = [
    # Dyads
    (100, [0, 1]),     # agents 1,2
    (101, [2, 3]),     # agents 3,4
    (102, [4, 5]),     # agents 5,6
    # Triad
    (103, [6, 7, 8]),  # agents 7,8,9
    # Individuals: 9-19 → agents 10-20
]

EAST_GROUPS = [
    # Dyads
    (200, [0, 1]),     # agents 21,22
    (201, [2, 3]),     # agents 23,24
    # Triads
    (202, [4, 5, 6]),  # agents 25,26,27
    (203, [7, 8, 9]),  # agents 28,29,30
    # Individuals: 10-19 → agents 31-40
]


def get_group_id_for_agent(agent_idx_in_side, side):
    """Return group_id for a given agent index on a side. -1 for individuals."""
    groups = WEST_GROUPS if side == 'west' else EAST_GROUPS
    for gid, members in groups:
        if agent_idx_in_side in members:
            return gid
    return -1


def get_group_members(agent_idx_in_side, side):
    """Return list of all member indices in the same group, or empty for individuals."""
    groups = WEST_GROUPS if side == 'west' else EAST_GROUPS
    for _, members in groups:
        if agent_idx_in_side in members:
            return members
    return []


# ──────────────────────────────────────────────────────────────────────
# PARAMETER DISTRIBUTIONS  (realistic pedestrian values)
# ──────────────────────────────────────────────────────────────────────
# Using configuration: 1 (CUSTOM) — these values are used as-is
# Ranges must stay within loader clamping:
#   social_force_factor:   [5.0, 20.0]
#   goal_force_factor:     [2.0, 5.0]
#   obstacle_force_factor: [2.0, 50.0]
#   max_vel:               [0.0, 1.8]

random.seed(42)  # Reproducible

def generate_agent_params():
    """Generate realistic baseline parameters for one agent."""
    return {
        'social_force_factor': round(random.uniform(7.0, 15.0), 2),
        'goal_force_factor':   round(random.uniform(2.5, 4.2), 2),
        'obstacle_force_factor': round(random.uniform(10.0, 25.0), 2),
        'other_force_factor':  round(random.uniform(12.0, 20.0), 2),
        'max_vel':             round(random.uniform(1.0, 1.5), 4),
    }


def generate_group_params(n_members):
    """Generate params for a group — similar speeds, slight individual variation."""
    base = generate_agent_params()
    base_vel = base['max_vel']
    members = []
    for _ in range(n_members):
        p = copy.deepcopy(base)
        # Group members: same base speed with small jitter (±0.05 m/s)
        p['max_vel'] = round(base_vel + random.uniform(-0.05, 0.05), 4)
        # Slight personality variation in forces
        p['social_force_factor'] = round(base['social_force_factor'] + random.uniform(-1.0, 1.0), 2)
        p['goal_force_factor'] = round(base['goal_force_factor'] + random.uniform(-0.2, 0.2), 2)
        # Clamp to valid ranges
        p['social_force_factor'] = max(5.0, min(20.0, p['social_force_factor']))
        p['goal_force_factor'] = max(2.0, min(5.0, p['goal_force_factor']))
        p['max_vel'] = max(0.5, min(1.8, p['max_vel']))
        members.append(p)
    return members


# ──────────────────────────────────────────────────────────────────────
# SPAWN POSITION GENERATION
# ──────────────────────────────────────────────────────────────────────

def generate_spawn_positions(n_agents, side):
    """Generate spawn positions for agents at one end of the tunnel.
    
    Agents are spread in a grid with some jitter to avoid exact overlaps.
    Group members spawn close together (~1m apart).
    """
    positions = []
    
    if side == 'west':
        x_min, x_max = WEST_SPAWN_X_MIN, WEST_SPAWN_X_MAX
        heading = HEADING_EAST
    else:
        x_min, x_max = EAST_SPAWN_X_MIN, EAST_SPAWN_X_MAX
        heading = HEADING_WEST
    
    # Grid-based placement: 4 columns × 5 rows
    cols = 4
    rows = 5
    x_step = (x_max - x_min) / max(cols - 1, 1)
    y_step = (Y_MAX - Y_MIN) / max(rows - 1, 1)
    
    idx = 0
    for row in range(rows):
        for col in range(cols):
            if idx >= n_agents:
                break
            x = x_min + col * x_step + random.uniform(-0.2, 0.2)
            y = Y_MIN + row * y_step + random.uniform(-0.2, 0.2)
            # Keep within bounds
            x = max(x_min, min(x_max, x))
            y = max(Y_MIN, min(Y_MAX, y))
            positions.append({'x': round(x, 3), 'y': round(y, 3), 'z': 1.001, 'h': round(heading, 3)})
            idx += 1
    
    # Adjust group members to be closer together
    groups = WEST_GROUPS if side == 'west' else EAST_GROUPS
    for _, members in groups:
        if len(members) < 2:
            continue
        # Use first member's position as anchor
        anchor = positions[members[0]]
        for m in members[1:]:
            positions[m]['x'] = round(anchor['x'] + random.uniform(-0.5, 0.5), 3)
            positions[m]['y'] = round(anchor['y'] + random.uniform(0.4, 0.8), 3)
            positions[m]['y'] = max(Y_MIN, min(Y_MAX, positions[m]['y']))
    
    return positions


# ──────────────────────────────────────────────────────────────────────
# GOAL GENERATION
# ──────────────────────────────────────────────────────────────────────

def generate_global_goals():
    """Create global goals at both ends of the tunnel.
    
    Goals 1-10:  EAST end (for west-spawning agents to walk toward)
    Goals 11-20: WEST end (for east-spawning agents to walk toward)
    
    Each set of 10 goals is spread across the Y dimension.
    """
    goals = {}
    y_positions = [Y_MIN + i * (Y_MAX - Y_MIN) / 9 for i in range(10)]
    
    # East goals (1-10)
    for i, y in enumerate(y_positions):
        goals[i + 1] = {'x': round(EAST_GOAL_X + random.uniform(-0.5, 0.5), 2), 
                         'y': round(y + random.uniform(-0.3, 0.3), 2)}
    
    # West goals (11-20)
    for i, y in enumerate(y_positions):
        goals[i + 11] = {'x': round(WEST_GOAL_X + random.uniform(-0.5, 0.5), 2),
                          'y': round(y + random.uniform(-0.3, 0.3), 2)}
    
    return goals


def assign_goals(agent_idx_in_side, side, n_goals=20):
    """Assign 2 cyclic goals: first at opposite end, second back at own end.
    
    This creates back-and-forth traversal across the tunnel length (~30m each way).
    Group members share goals (handled in group param generation).
    """
    if side == 'west':
        # Walk east first (goals 1-10), then back west (goals 11-20)
        goal_a = (agent_idx_in_side % 10) + 1     # east goal
        goal_b = (agent_idx_in_side % 10) + 11    # west goal
    else:
        # Walk west first (goals 11-20), then back east (goals 1-10)
        goal_a = (agent_idx_in_side % 10) + 11    # west goal
        goal_b = (agent_idx_in_side % 10) + 1     # east goal
    
    return [goal_a, goal_b]


# ──────────────────────────────────────────────────────────────────────
# YAML GENERATION
# ──────────────────────────────────────────────────────────────────────

def build_baseline_yaml():
    """Build the complete baseline YAML for central tunnel."""
    random.seed(42)  # Reset seed for reproducibility
    
    global_goals = generate_global_goals()
    
    west_positions = generate_spawn_positions(20, 'west')
    east_positions = generate_spawn_positions(20, 'east')
    
    agents_list = []
    agents_data = {}
    
    # Pre-generate group params so group members share base values
    group_params_cache = {}
    
    # West side groups
    for gid, members in WEST_GROUPS:
        params = generate_group_params(len(members))
        for i, m in enumerate(members):
            group_params_cache[('west', m)] = params[i]
    
    # East side groups
    for gid, members in EAST_GROUPS:
        params = generate_group_params(len(members))
        for i, m in enumerate(members):
            group_params_cache[('east', m)] = params[i]
    
    # Generate all 40 agents
    for side_idx, (side, positions) in enumerate([('west', west_positions), ('east', east_positions)]):
        for local_idx in range(20):
            agent_id = side_idx * 20 + local_idx + 1
            agent_key = f'agent{agent_id}'
            agents_list.append(agent_key)
            
            # Get params (from group cache or generate individual)
            if (side, local_idx) in group_params_cache:
                params = group_params_cache[(side, local_idx)]
            else:
                params = generate_agent_params()
            
            group_id = get_group_id_for_agent(local_idx, side)
            goals = assign_goals(local_idx, side)
            
            # Group members share goals with leader
            group_members = get_group_members(local_idx, side)
            if group_members and local_idx != group_members[0]:
                leader_goals = assign_goals(group_members[0], side)
                goals = leader_goals
            
            agents_data[agent_key] = {
                'id': agent_id,
                'group_id': group_id,
                'skin': local_idx % 6,  # 6 skin options, cycle through
                'max_vel': params['max_vel'],
                'radius': 0.4,
                'goal_radius': 0.3,
                'cyclic_goals': True,
                'init_pose': positions[local_idx],
                'behavior': {
                    'type': 'Regular',
                    'configuration': 1,  # BEH_CONF_CUSTOM — YAML values used as-is!
                    'goal_force_factor': params['goal_force_factor'],
                    'obstacle_force_factor': params['obstacle_force_factor'],
                    'social_force_factor': params['social_force_factor'],
                    'other_force_factor': params['other_force_factor'],
                },
                'goals': goals,
            }
    
    yaml_data = {
        'hunav_loader': {
            'ros__parameters': {
                'yaml_base_name': YAML_BASE_NAME,
                'simulator': 'Gazebo Classic',
                'map': 'central_tunnel',
                'publish_people': True,
                'global_goals': global_goals,
                'agents': agents_list,
                **agents_data,
            }
        }
    }
    
    return yaml_data


# ──────────────────────────────────────────────────────────────────────
# PHASE VARIANT GENERATION
# ──────────────────────────────────────────────────────────────────────

# Phase scaling factors (chosen to stay within clamped ranges)
PHASES = {
    'phase1_social': {
        'param': 'social_force_factor',
        'low_scale': 0.55,   # reduce social force → less avoidance
        'high_scale': 1.7,   # increase social force → more avoidance
        'low_label': 'SF × 0.55 (weak avoidance)',
        'high_label': 'SF × 1.70 (strong avoidance)',
    },
    'phase2_goal': {
        'param': 'goal_force_factor',
        'low_scale': 0.6,    # reduce goal force → more wandering
        'high_scale': 1.6,   # increase goal force → more direct paths
        'low_label': 'GF × 0.60 (weak goal pull)',
        'high_label': 'GF × 1.60 (strong goal pull)',
    },
    'phase3_speed': {
        'param': 'max_vel',
        'low_scale': 0.75,   # slow walkers
        'high_scale': 1.25,  # fast walkers
        'low_label': 'Speed × 0.75 (cautious pace)',
        'high_label': 'Speed × 1.25 (hurried pace)',
    },
    'phase4_obstacle': {
        'param': 'obstacle_force_factor',
        'low_scale': 0.5,    # less wall avoidance
        'high_scale': 2.0,   # more wall avoidance
        'low_label': 'OF × 0.50 (low wall avoidance)',
        'high_label': 'OF × 2.00 (high wall avoidance)',
    },
}

# Clamping ranges per parameter
CLAMP_RANGES = {
    'social_force_factor': (5.0, 20.0),
    'goal_force_factor': (2.0, 5.0),
    'obstacle_force_factor': (2.0, 50.0),
    'other_force_factor': (0.0, 25.0),
    'max_vel': (0.5, 1.8),
}


def clamp(value, param_name):
    """Clamp a value to its valid range."""
    lo, hi = CLAMP_RANGES[param_name]
    return max(lo, min(hi, value))


def scale_yaml(baseline, param_name, scale_factor):
    """Scale a single parameter across all agents, clamping to valid range."""
    variant = copy.deepcopy(baseline)
    params = variant['hunav_loader']['ros__parameters']
    
    for i in range(1, NUM_AGENTS + 1):
        agent_key = f'agent{i}'
        agent = params[agent_key]
        
        if param_name == 'max_vel':
            old_val = agent['max_vel']
            new_val = clamp(round(old_val * scale_factor, 4), param_name)
            agent['max_vel'] = new_val
        else:
            old_val = agent['behavior'][param_name]
            new_val = clamp(round(old_val * scale_factor, 2), param_name)
            agent['behavior'][param_name] = new_val
    
    return variant


# ──────────────────────────────────────────────────────────────────────
# BEHAVIOR TREE XML GENERATION
# ──────────────────────────────────────────────────────────────────────

BT_REGULAR_TEMPLATE = """<?xml version="1.0" encoding="UTF-8"?>
<root main_tree_to_execute="DefaultTree" BTCPP_format="4">
  <TreeNodesModel>
    <Condition ID="IsGoalReached">
      <input_port name="agent_id" type="int">{agent_id}</input_port>
    </Condition>
    <Action ID="UpdateGoal">
      <input_port name="agent_id" type="int">{agent_id}</input_port>
    </Action>
    <Action ID="RegularNav">
      <input_port name="agent_id" type="int">{agent_id}</input_port>
      <input_port name="time_step" type="double" default="0.1"/>
    </Action>
    <Action ID="SetGoal">
      <input_port name="agent_id" type="int">{agent_id}</input_port>
      <input_port name="goal_id" type="int"/>
    </Action>
  </TreeNodesModel>

<include path="BTRegularNav.xml" />

<BehaviorTree ID="DefaultTree">
  <Fallback name="MainFallback">
    <Sequence name="SetGoals">
        <RunOnce>
          <SetGoal agent_id="{{id}}" goal_id="{goal1}"/>
        </RunOnce>
        <Inverter>
          <RunOnce>
            <SetGoal agent_id="{{id}}" goal_id="{goal2}"/>
          </RunOnce>
        </Inverter>
    </Sequence>
    <Sequence name="RegularNavigation">
      <Inverter>
        <IsGoalReached agent_id="{{id}}"/>
      </Inverter>
      <RegularNav agent_id="{{id}}" time_step="{{dt}}"/>
    </Sequence>
    <UpdateGoal agent_id="{{id}}"/>
  </Fallback>
</BehaviorTree>

</root>
"""

BT_GROUP_LEADER_TEMPLATE = """<?xml version="1.0" encoding="UTF-8"?>
<root main_tree_to_execute="DefaultTree" BTCPP_format="4">
  <TreeNodesModel>
    <Condition ID="IsGoalReached">
      <input_port name="agent_id" type="int">{agent_id}</input_port>
    </Condition>
    <Action ID="UpdateGoal">
      <input_port name="agent_id" type="int">{agent_id}</input_port>
    </Action>
    <Action ID="RegularNav">
      <input_port name="agent_id" type="int">{agent_id}</input_port>
      <input_port name="time_step" type="double" default="0.1"/>
    </Action>
    <Action ID="SetGoal">
      <input_port name="agent_id" type="int">{agent_id}</input_port>
      <input_port name="goal_id" type="int"/>
    </Action>
    <Action ID="SetGroupWalk">
      <input_port name="main_agent_id" type="int">{agent_id}</input_port>
      <input_port name="time_step" type="double"/>
      <input_port name="non_main_agent_ids" type="std::string"/>
      <input_port name="duration" type="double" default="0.0"/>
    </Action>
  </TreeNodesModel>

<include path="BTRegularNav.xml" />

<BehaviorTree ID="DefaultTree">
  <Fallback name="MainFallback">
    <Sequence name="SetGoals">
        <RunOnce>
          <SetGoal agent_id="{{id}}" goal_id="{goal1}"/>
        </RunOnce>
        <Inverter>
          <RunOnce>
            <SetGoal agent_id="{{id}}" goal_id="{goal2}"/>
          </RunOnce>
        </Inverter>
    </Sequence>
    <SetGroupWalk main_agent_id="{{id}}" time_step="{{dt}}" 
                  non_main_agent_ids="{follower_ids}" duration="0.0"/>
  </Fallback>
</BehaviorTree>

</root>
"""

BT_GROUP_FOLLOWER_TEMPLATE = """<?xml version="1.0" encoding="UTF-8"?>
<root main_tree_to_execute="DefaultTree" BTCPP_format="4">
  <TreeNodesModel>
    <Condition ID="IsGoalReached">
      <input_port name="agent_id" type="int">{agent_id}</input_port>
    </Condition>
    <Action ID="UpdateGoal">
      <input_port name="agent_id" type="int">{agent_id}</input_port>
    </Action>
    <Action ID="RegularNav">
      <input_port name="agent_id" type="int">{agent_id}</input_port>
      <input_port name="time_step" type="double" default="0.1"/>
    </Action>
    <Action ID="SetGoal">
      <input_port name="agent_id" type="int">{agent_id}</input_port>
      <input_port name="goal_id" type="int"/>
    </Action>
  </TreeNodesModel>

<include path="BTRegularNav.xml" />

<BehaviorTree ID="DefaultTree">
  <Fallback name="MainFallback">
    <Sequence name="SetGoals">
        <RunOnce>
          <SetGoal agent_id="{{id}}" goal_id="{goal1}"/>
        </RunOnce>
        <Inverter>
          <RunOnce>
            <SetGoal agent_id="{{id}}" goal_id="{goal2}"/>
          </RunOnce>
        </Inverter>
    </Sequence>
    <Sequence name="RegularNavigation">
      <Inverter>
        <IsGoalReached agent_id="{{id}}"/>
      </Inverter>
      <RegularNav agent_id="{{id}}" time_step="{{dt}}"/>
    </Sequence>
    <UpdateGoal agent_id="{{id}}"/>
  </Fallback>
</BehaviorTree>

</root>
"""


def generate_bt_files(baseline_yaml):
    """Generate BT XML files for all 40 agents."""
    params = baseline_yaml['hunav_loader']['ros__parameters']
    
    # Build group membership map: agent_id → (is_leader, group_members)
    all_groups = {}
    for side_idx, (side, groups) in enumerate([('west', WEST_GROUPS), ('east', EAST_GROUPS)]):
        for gid, members in groups:
            agent_ids = [side_idx * 20 + m + 1 for m in members]
            leader_id = agent_ids[0]
            for aid in agent_ids:
                all_groups[aid] = {
                    'is_leader': aid == leader_id,
                    'leader_id': leader_id,
                    'all_ids': agent_ids,
                    'follower_ids': [x for x in agent_ids if x != leader_id],
                }
    
    bt_files = {}
    for agent_id in range(1, NUM_AGENTS + 1):
        agent_key = f'agent{agent_id}'
        goals = params[agent_key]['goals']
        goal1 = goals[0]
        goal2 = goals[1] if len(goals) > 1 else goals[0]
        
        if agent_id in all_groups:
            group_info = all_groups[agent_id]
            if group_info['is_leader']:
                # Leader uses SetGroupWalk
                follower_ids_str = ','.join(str(x) for x in group_info['follower_ids'])
                content = BT_GROUP_LEADER_TEMPLATE.format(
                    agent_id=agent_id,
                    goal1=goal1,
                    goal2=goal2,
                    follower_ids=follower_ids_str,
                )
            else:
                # Follower uses regular nav (group walk overrides their goals)
                content = BT_GROUP_FOLLOWER_TEMPLATE.format(
                    agent_id=agent_id,
                    goal1=goal1,
                    goal2=goal2,
                )
        else:
            # Individual agent
            content = BT_REGULAR_TEMPLATE.format(
                agent_id=agent_id,
                goal1=goal1,
                goal2=goal2,
            )
        
        filename = f'{YAML_BASE_NAME}__agent_{agent_id}_bt.xml'
        bt_files[filename] = content
    
    return bt_files


# ──────────────────────────────────────────────────────────────────────
# MAIN
# ──────────────────────────────────────────────────────────────────────

def save_yaml(data, filepath):
    os.makedirs(os.path.dirname(filepath), exist_ok=True)
    with open(filepath, 'w') as f:
        yaml.dump(data, f, default_flow_style=False, sort_keys=False)


def main():
    print("=" * 70)
    print("CENTRAL TUNNEL OAT YAML + BT GENERATOR")
    print("=" * 70)
    
    # 1) Generate baseline
    print("\n[1/3] Generating baseline YAML...")
    baseline = build_baseline_yaml()
    baseline_path = os.path.join(SCENARIO_DIR, 'ct_baseline.yaml')
    save_yaml(baseline, baseline_path)
    print(f"  ✓ {baseline_path}")
    
    # Print agent summary
    params = baseline['hunav_loader']['ros__parameters']
    sf_vals = [params[f'agent{i}']['behavior']['social_force_factor'] for i in range(1, NUM_AGENTS + 1)]
    gf_vals = [params[f'agent{i}']['behavior']['goal_force_factor'] for i in range(1, NUM_AGENTS + 1)]
    vel_vals = [params[f'agent{i}']['max_vel'] for i in range(1, NUM_AGENTS + 1)]
    of_vals = [params[f'agent{i}']['behavior']['obstacle_force_factor'] for i in range(1, NUM_AGENTS + 1)]
    
    print(f"\n  Agent Summary ({NUM_AGENTS} agents):")
    print(f"    Social Force:   mean={sum(sf_vals)/len(sf_vals):.2f}  range=[{min(sf_vals):.2f}, {max(sf_vals):.2f}]")
    print(f"    Goal Force:     mean={sum(gf_vals)/len(gf_vals):.2f}  range=[{min(gf_vals):.2f}, {max(gf_vals):.2f}]")
    print(f"    Obstacle Force: mean={sum(of_vals)/len(of_vals):.2f}  range=[{min(of_vals):.2f}, {max(of_vals):.2f}]")
    print(f"    Max Velocity:   mean={sum(vel_vals)/len(vel_vals):.2f}  range=[{min(vel_vals):.2f}, {max(vel_vals):.2f}]")
    
    # Count groups
    n_groups = len(WEST_GROUPS) + len(EAST_GROUPS)
    n_in_groups = sum(len(m) for _, m in WEST_GROUPS) + sum(len(m) for _, m in EAST_GROUPS)
    n_individual = NUM_AGENTS - n_in_groups
    n_dyads = sum(1 for _, m in WEST_GROUPS + EAST_GROUPS if len(m) == 2)
    n_triads = sum(1 for _, m in WEST_GROUPS + EAST_GROUPS if len(m) == 3)
    print(f"\n    Groups: {n_groups} total ({n_dyads} dyads, {n_triads} triads)")
    print(f"    Individuals: {n_individual}  |  In groups: {n_in_groups}")
    print(f"    Spawn: 20 west-end → 20 east-end (bidirectional flow)")
    print(f"    Goal distance: ~30m (full tunnel traversal)")
    
    # 2) Generate phase variants
    print("\n[2/3] Generating phase variant YAMLs...")
    for phase_name, phase_cfg in PHASES.items():
        param = phase_cfg['param']
        
        # Low variant
        low = scale_yaml(baseline, param, phase_cfg['low_scale'])
        low_path = os.path.join(SCENARIO_DIR, f'ct_{phase_name}_low.yaml')
        save_yaml(low, low_path)
        print(f"  ✓ {os.path.basename(low_path):40s} {phase_cfg['low_label']}")
        
        # High variant
        high = scale_yaml(baseline, param, phase_cfg['high_scale'])
        high_path = os.path.join(SCENARIO_DIR, f'ct_{phase_name}_high.yaml')
        save_yaml(high, high_path)
        print(f"  ✓ {os.path.basename(high_path):40s} {phase_cfg['high_label']}")
    
    # 3) Generate BT files
    print("\n[3/3] Generating Behavior Tree XML files...")
    bt_files = generate_bt_files(baseline)
    
    # Save to source subdirectory
    os.makedirs(BT_SRC_DIR, exist_ok=True)
    for fname, content in bt_files.items():
        with open(os.path.join(BT_SRC_DIR, fname), 'w') as f:
            f.write(content)
    
    # Also save to flat source directory (for install)
    for fname, content in bt_files.items():
        with open(os.path.join(BT_FLAT_DIR, fname), 'w') as f:
            f.write(content)
    
    # Also save to installed directory (for immediate use before rebuild)
    if os.path.isdir(BT_INSTALL_DIR):
        for fname, content in bt_files.items():
            with open(os.path.join(BT_INSTALL_DIR, fname), 'w') as f:
                f.write(content)
        print(f"  ✓ {len(bt_files)} BT files → source + installed directories")
    else:
        print(f"  ✓ {len(bt_files)} BT files → source directory (run colcon build to install)")
    
    # Summary
    print("\n" + "=" * 70)
    print("GENERATED FILES:")
    print("=" * 70)
    print(f"\nScenario YAMLs ({1 + len(PHASES) * 2} files):")
    print(f"  {SCENARIO_DIR}/")
    print(f"    ct_baseline.yaml")
    for phase_name in PHASES:
        print(f"    ct_{phase_name}_low.yaml")
        print(f"    ct_{phase_name}_high.yaml")
    
    print(f"\nBehavior Trees ({len(bt_files)} files):")
    print(f"  {BT_SRC_DIR}/")
    print(f"  {BT_FLAT_DIR}/ (flat, for install)")
    
    print("\n" + "=" * 70)
    print("OAT TEST PLAN")
    print("=" * 70)
    print("""
Phase      | Runs  | Config File                | Parameter Varied
-----------|-------|----------------------------|-----------------
Baseline   | 1-3   | ct_baseline.yaml           | None (reference)
Phase 1    | 4-5   | ct_phase1_social_low.yaml  | Social Force × 0.55
Phase 1    | 6-7   | ct_phase1_social_high.yaml | Social Force × 1.70
Phase 2    | 8-9   | ct_phase2_goal_low.yaml    | Goal Force × 0.60
Phase 2    | 10-11 | ct_phase2_goal_high.yaml   | Goal Force × 1.60
Phase 3    | 12-13 | ct_phase3_speed_low.yaml   | Max Velocity × 0.75
Phase 3    | 14-15 | ct_phase3_speed_high.yaml  | Max Velocity × 1.25
Phase 4    | 16-17 | ct_phase4_obstacle_low.yaml| Obstacle Force × 0.50
Phase 4    | 18-19 | ct_phase4_obstacle_high.yaml| Obstacle Force × 2.00

Total: 19 runs (~4-5 hours at 120s/run + setup time)
""")
    
    print("NEXT STEPS:")
    print("  1. colcon build --packages-select hunav_gazebo_wrapper")
    print("     source install/setup.bash")
    print("  2. ros2 launch hunav_gazebo_wrapper central_tunnel_no_robot.launch.py \\")
    print("       configuration_file:=domenic/central_tunnel/ct_baseline.yaml")
    print("  3. python3 run_recording.py <run_id>")


if __name__ == '__main__':
    main()
