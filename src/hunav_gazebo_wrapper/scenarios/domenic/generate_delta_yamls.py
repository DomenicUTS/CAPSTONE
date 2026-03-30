#!/usr/bin/env python3
"""
Generate Delta (Triangle) OAT Sensitivity Analysis YAMLs + Behavior Trees.

Creates 80 agents in a triangular arena with 3-way crossing flow:
  - 27 agents spawn at TOP corner    → goals at bottom-left & bottom-right
  - 27 agents spawn at BOTTOM-LEFT   → goals at top & bottom-right
  - 26 agents spawn at BOTTOM-RIGHT  → goals at top & bottom-left
  - Mix of individuals, dyads, and triads
  - Goal distances ~25-35m (cross-triangle traversal)

Uses configuration: 1 (BEH_CONF_CUSTOM) so YAML force factors are
respected exactly — not overridden by random generation.

World geometry (from delta.world 'Untitled3453' model):
  Model origin: (-6.51347, 3.93864)
  Wall_11: 42.25m, center (-16.88, 3.94),  rotation 60°
  Wall_13: 42.5m,  center (-6.53, -14.25), rotation 180°
  Wall_3:  42.0m,  center (4.19, 3.88),    rotation 120°

  Triangle vertices (world coordinates):
    Top:          (-6.3,  22.15)
    Bottom-left:  (-27.6, -14.3)
    Bottom-right: (14.7,  -14.3)
  Centroid:       (-6.4,  -2.14)

  Obstacles (small cabinets, ~0.45m):
    (-11.62, -9.31), (-14.60, -1.79), (-5.21, -3.35),
    (-7.58, 4.16), (3.49, -8.64)
"""

import yaml
import copy
import os
import math
import random

# ──────────────────────────────────────────────────────────────────────
# PATHS
# ──────────────────────────────────────────────────────────────────────
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
# Navigate from scenarios/domenic/ up to hunav_gazebo_wrapper/
WRAPPER_DIR = os.path.dirname(os.path.dirname(SCRIPT_DIR))
# Workspace root (up from src/hunav_gazebo_wrapper/)
WS_ROOT = os.path.dirname(os.path.dirname(WRAPPER_DIR))

SCENARIO_DIR = os.path.join(SCRIPT_DIR, 'delta')
BT_SRC_DIR = os.path.join(WRAPPER_DIR, 'behavior_trees', 'delta')
BT_INSTALL_DIR = os.path.join(WS_ROOT, 'install', 'hunav_gazebo_wrapper', 'share',
                               'hunav_gazebo_wrapper', 'behavior_trees')

YAML_BASE_NAME = 'agents_delta'
NUM_AGENTS = 80

# ──────────────────────────────────────────────────────────────────────
# TRIANGLE GEOMETRY  (with safety margin from walls)
# ──────────────────────────────────────────────────────────────────────
# Triangle vertices (world coords, from wall endpoint calculations)
VERTEX_TOP   = (-6.3,  22.15)
VERTEX_BL    = (-27.6, -14.3)
VERTEX_BR    = (14.7,  -14.3)
CENTROID     = (-6.4,  -2.14)

# Triangle edge equations (for containment checks):
#   Left edge  (BL→Top): y = 1.709x + 32.87   (inside = below line)
#   Right edge (BR→Top): y = -1.733x + 11.18   (inside = below line)
#   Bottom edge:          y = -14.3              (inside = above line)
LEFT_SLOPE  =  1.7089
LEFT_ICEPT  = 32.8662
RIGHT_SLOPE = -1.7333
RIGHT_ICEPT = 11.1800
BOTTOM_Y    = -14.3
WALL_MARGIN = 1.5  # metres clearance from walls


def point_in_triangle(x, y, margin=WALL_MARGIN):
    """Check if (x,y) is inside the triangle with given wall margin."""
    return (y < LEFT_SLOPE * x + LEFT_ICEPT - margin and
            y < RIGHT_SLOPE * x + RIGHT_ICEPT - margin and
            y > BOTTOM_Y + margin)


def clamp_to_triangle(x, y, margin=WALL_MARGIN):
    """Push (x,y) inside the triangle if it's outside."""
    y = min(y, LEFT_SLOPE * x + LEFT_ICEPT - margin)
    y = min(y, RIGHT_SLOPE * x + RIGHT_ICEPT - margin)
    y = max(y, BOTTOM_Y + margin)
    # If still outside after Y clamp (X too extreme), pull X toward centroid
    for _ in range(5):
        if point_in_triangle(x, y, margin):
            break
        x = x * 0.85 + CENTROID[0] * 0.15
        y = min(y, LEFT_SLOPE * x + LEFT_ICEPT - margin)
        y = min(y, RIGHT_SLOPE * x + RIGHT_ICEPT - margin)
        y = max(y, BOTTOM_Y + margin)
    return round(x, 3), round(y, 3)


# Spawn zones: positioned safely inside the triangle
# (verified against edge equations with wall margin)

# Top corner spawn zone  (widened for 27 agents; triangle ~8-12m wide here)
TOP_SPAWN_X_MIN, TOP_SPAWN_X_MAX = -10.0, -3.0
TOP_SPAWN_Y_MIN, TOP_SPAWN_Y_MAX = 6.0, 14.0
TOP_HEADING = -math.pi / 2  # facing downward toward centroid

# Bottom-left corner spawn zone  (widened for 27 agents)
BL_SPAWN_X_MIN, BL_SPAWN_X_MAX = -23.0, -15.0
BL_SPAWN_Y_MIN, BL_SPAWN_Y_MAX = -12.5, -6.0
BL_HEADING = math.atan2(CENTROID[1] - VERTEX_BL[1], CENTROID[0] - VERTEX_BL[0])

# Bottom-right corner spawn zone  (widened for 26 agents)
BR_SPAWN_X_MIN, BR_SPAWN_X_MAX = 3.0, 11.0
BR_SPAWN_Y_MIN, BR_SPAWN_Y_MAX = -12.5, -6.0
BR_HEADING = math.atan2(CENTROID[1] - VERTEX_BR[1], CENTROID[0] - VERTEX_BR[0])


# ──────────────────────────────────────────────────────────────────────
# CORNER CONFIGURATION
# ──────────────────────────────────────────────────────────────────────
CORNERS = {
    'top': {
        'n_agents': 27,
        'spawn_x': (TOP_SPAWN_X_MIN, TOP_SPAWN_X_MAX),
        'spawn_y': (TOP_SPAWN_Y_MIN, TOP_SPAWN_Y_MAX),
        'heading': TOP_HEADING,
        'goal_corners': ['bl', 'br'],  # walk toward bottom-left and bottom-right
    },
    'bl': {
        'n_agents': 27,
        'spawn_x': (BL_SPAWN_X_MIN, BL_SPAWN_X_MAX),
        'spawn_y': (BL_SPAWN_Y_MIN, BL_SPAWN_Y_MAX),
        'heading': BL_HEADING,
        'goal_corners': ['top', 'br'],
    },
    'br': {
        'n_agents': 26,
        'spawn_x': (BR_SPAWN_X_MIN, BR_SPAWN_X_MAX),
        'spawn_y': (BR_SPAWN_Y_MIN, BR_SPAWN_Y_MAX),
        'heading': BR_HEADING,
        'goal_corners': ['top', 'bl'],
    },
}

# Goal zones: 5 goals per corner region, positioned ~4-8m from vertex along walls
GOAL_ZONES = {
    'top': {
        'x_range': (-9.0, -4.0),
        'y_range': (11.0, 15.0),
    },
    'bl': {
        'x_range': (-22.0, -17.0),
        'y_range': (-12.0, -8.0),
    },
    'br': {
        'x_range': (5.0, 10.0),
        'y_range': (-12.0, -8.0),
    },
}


# ──────────────────────────────────────────────────────────────────────
# GROUP DEFINITIONS
# ──────────────────────────────────────────────────────────────────────
# Groups per corner. Each: (group_label, [local_indices])
# Local indices are 0-based within each corner's agent list.
#
# Top corner (27 agents, local 0-26):
#   Dyads: (0,1), (2,3), (4,5)    Triads: (6,7,8), (9,10,11)    Individuals: 12-26
# Bottom-left (27 agents, local 0-26):
#   Dyads: (0,1), (2,3), (4,5)    Triads: (6,7,8), (9,10,11)    Individuals: 12-26
# Bottom-right (26 agents, local 0-25):
#   Dyads: (0,1), (2,3), (4,5)    Triads: (6,7,8), (9,10,11)    Individuals: 12-25

CORNER_GROUPS = {
    'top': [
        (100, [0, 1]),       # dyad
        (101, [2, 3]),       # dyad
        (102, [4, 5]),       # dyad
        (103, [6, 7, 8]),    # triad
        (104, [9, 10, 11]),  # triad
    ],
    'bl': [
        (200, [0, 1]),
        (201, [2, 3]),
        (202, [4, 5]),
        (203, [6, 7, 8]),
        (204, [9, 10, 11]),
    ],
    'br': [
        (300, [0, 1]),
        (301, [2, 3]),
        (302, [4, 5]),
        (303, [6, 7, 8]),
        (304, [9, 10, 11]),
    ],
}


def get_group_id_for_agent(local_idx, corner):
    """Return group_id for a given local agent index in a corner. -1 for individuals."""
    for gid, members in CORNER_GROUPS[corner]:
        if local_idx in members:
            return gid
    return -1


def get_group_members(local_idx, corner):
    """Return list of all member local indices in the same group, or empty for individuals."""
    for _, members in CORNER_GROUPS[corner]:
        if local_idx in members:
            return members
    return []


# ──────────────────────────────────────────────────────────────────────
# PARAMETER DISTRIBUTIONS  (same baseline as central tunnel, seed=42)
# ──────────────────────────────────────────────────────────────────────
random.seed(42)

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
        p['max_vel'] = round(base_vel + random.uniform(-0.05, 0.05), 4)
        p['social_force_factor'] = round(base['social_force_factor'] + random.uniform(-1.0, 1.0), 2)
        p['goal_force_factor'] = round(base['goal_force_factor'] + random.uniform(-0.2, 0.2), 2)
        p['social_force_factor'] = max(5.0, min(20.0, p['social_force_factor']))
        p['goal_force_factor'] = max(2.0, min(5.0, p['goal_force_factor']))
        p['max_vel'] = max(0.5, min(1.8, p['max_vel']))
        members.append(p)
    return members


# ──────────────────────────────────────────────────────────────────────
# SPAWN POSITION GENERATION
# ──────────────────────────────────────────────────────────────────────

def generate_spawn_positions(n_agents, corner):
    """Generate spawn positions for agents in a triangle corner.

    Agents are placed in a grid within the corner's spawn zone with
    jitter to avoid exact overlaps. Group members spawn close together.
    """
    cfg = CORNERS[corner]
    x_min, x_max = cfg['spawn_x']
    y_min, y_max = cfg['spawn_y']
    heading = cfg['heading']

    positions = []

    # Grid layout: spread agents across the spawn zone
    # Determine grid dimensions based on agent count
    cols = min(n_agents, 5)
    rows = math.ceil(n_agents / cols)
    x_step = (x_max - x_min) / max(cols - 1, 1)
    y_step = (y_max - y_min) / max(rows - 1, 1)

    idx = 0
    for row in range(rows):
        for col in range(cols):
            if idx >= n_agents:
                break
            x = x_min + col * x_step + random.uniform(-0.2, 0.2)
            y = y_min + row * y_step + random.uniform(-0.2, 0.2)
            x = max(x_min, min(x_max, x))
            y = max(y_min, min(y_max, y))
            # Clamp to triangle
            x, y = clamp_to_triangle(x, y)
            positions.append({'x': x, 'y': y, 'z': 1.001, 'h': round(heading, 3)})
            idx += 1

    # Adjust group members to be closer together
    for _, members in CORNER_GROUPS[corner]:
        if len(members) < 2:
            continue
        anchor = positions[members[0]]
        for m in members[1:]:
            gx = anchor['x'] + random.uniform(-0.5, 0.5)
            gy = anchor['y'] + random.uniform(0.4, 0.8)
            gx, gy = clamp_to_triangle(gx, gy)
            positions[m]['x'] = gx
            positions[m]['y'] = gy

    return positions


# ──────────────────────────────────────────────────────────────────────
# GOAL GENERATION
# ──────────────────────────────────────────────────────────────────────

def generate_global_goals():
    """Create global goals spread across 3 corner regions.

    Goals 1-5:   TOP corner region    (for BL→TOP and BR→TOP flows)
    Goals 6-10:  BOTTOM-LEFT region   (for TOP→BL and BR→BL flows)
    Goals 11-15: BOTTOM-RIGHT region  (for TOP→BR and BL→BR flows)
    """
    goals = {}
    goal_id = 1

    for corner_name in ['top', 'bl', 'br']:
        zone = GOAL_ZONES[corner_name]
        x_min, x_max = zone['x_range']
        y_min, y_max = zone['y_range']

        for i in range(5):
            # Spread goals across the zone
            frac = i / 4.0  # 0.0 to 1.0
            x = x_min + frac * (x_max - x_min) + random.uniform(-0.3, 0.3)
            y = (y_min + y_max) / 2 + random.uniform(-1.0, 1.0)
            x = max(x_min, min(x_max, x))
            y = max(y_min, min(y_max, y))
            # Clamp to triangle
            x, y = clamp_to_triangle(x, y)
            goals[goal_id] = {'x': x, 'y': y}
            goal_id += 1

    return goals


# Goal index ranges per corner region
GOAL_RANGES = {
    'top': (1, 5),    # goals 1-5
    'bl':  (6, 10),   # goals 6-10
    'br':  (11, 15),  # goals 11-15
}


def assign_goals(local_idx, corner):
    """Assign 2 cyclic goals: first at one opposite corner, second at the other.

    This creates cross-triangle traversal (~25-35m per leg).
    Agents alternate between the two opposite corners.
    """
    cfg = CORNERS[corner]
    dest_a, dest_b = cfg['goal_corners']

    # Pick goals from opposite corner regions
    a_start, a_end = GOAL_RANGES[dest_a]
    b_start, b_end = GOAL_RANGES[dest_b]

    goal_a = a_start + (local_idx % 5)  # cycle through 5 goals in dest_a
    goal_b = b_start + (local_idx % 5)  # cycle through 5 goals in dest_b

    return [goal_a, goal_b]


# ──────────────────────────────────────────────────────────────────────
# YAML GENERATION
# ──────────────────────────────────────────────────────────────────────

def build_baseline_yaml():
    """Build the complete baseline YAML for delta triangle."""
    random.seed(42)

    global_goals = generate_global_goals()

    # Generate spawn positions for each corner
    corner_positions = {}
    for corner_name, cfg in CORNERS.items():
        corner_positions[corner_name] = generate_spawn_positions(cfg['n_agents'], corner_name)

    agents_list = []
    agents_data = {}

    # Pre-generate group params so group members share base values
    group_params_cache = {}
    for corner_name, groups in CORNER_GROUPS.items():
        for gid, members in groups:
            params = generate_group_params(len(members))
            for i, m in enumerate(members):
                group_params_cache[(corner_name, m)] = params[i]

    # Build all 50 agents
    agent_id = 1
    corner_order = ['top', 'bl', 'br']
    corner_agent_id_offset = {}  # corner → first agent_id in that corner

    for corner_name in corner_order:
        cfg = CORNERS[corner_name]
        n = cfg['n_agents']
        corner_agent_id_offset[corner_name] = agent_id
        positions = corner_positions[corner_name]

        for local_idx in range(n):
            agent_key = f'agent{agent_id}'
            agents_list.append(agent_key)

            # Get params
            if (corner_name, local_idx) in group_params_cache:
                params = group_params_cache[(corner_name, local_idx)]
            else:
                params = generate_agent_params()

            group_id = get_group_id_for_agent(local_idx, corner_name)
            goals = assign_goals(local_idx, corner_name)

            # Group members share goals with leader
            group_members = get_group_members(local_idx, corner_name)
            if group_members and local_idx != group_members[0]:
                leader_goals = assign_goals(group_members[0], corner_name)
                goals = leader_goals

            agents_data[agent_key] = {
                'id': agent_id,
                'group_id': group_id,
                'skin': local_idx % 6,
                'max_vel': params['max_vel'],
                'radius': 0.4,
                'goal_radius': 0.3,
                'cyclic_goals': True,
                'init_pose': positions[local_idx],
                'behavior': {
                    'type': 'Regular',
                    'configuration': 1,  # BEH_CONF_CUSTOM
                    'goal_force_factor': params['goal_force_factor'],
                    'obstacle_force_factor': params['obstacle_force_factor'],
                    'social_force_factor': params['social_force_factor'],
                    'other_force_factor': params['other_force_factor'],
                },
                'goals': goals,
            }
            agent_id += 1

    yaml_data = {
        'hunav_loader': {
            'ros__parameters': {
                'yaml_base_name': YAML_BASE_NAME,
                'simulator': 'Gazebo Classic',
                'map': 'delta',
                'publish_people': True,
                'global_goals': global_goals,
                'agents': agents_list,
                **agents_data,
            }
        }
    }

    return yaml_data, corner_agent_id_offset


# ──────────────────────────────────────────────────────────────────────
# PHASE VARIANT GENERATION  (same scales as central tunnel)
# ──────────────────────────────────────────────────────────────────────

PHASES = {
    'phase1_social': {
        'param': 'social_force_factor',
        'low_scale': 0.55,
        'high_scale': 1.7,
        'low_label': 'SF × 0.55 (weak avoidance)',
        'high_label': 'SF × 1.70 (strong avoidance)',
    },
    'phase2_goal': {
        'param': 'goal_force_factor',
        'low_scale': 0.6,
        'high_scale': 1.6,
        'low_label': 'GF × 0.60 (weak goal pull)',
        'high_label': 'GF × 1.60 (strong goal pull)',
    },
    'phase3_speed': {
        'param': 'max_vel',
        'low_scale': 0.75,
        'high_scale': 1.25,
        'low_label': 'Speed × 0.75 (cautious pace)',
        'high_label': 'Speed × 1.25 (hurried pace)',
    },
    'phase4_obstacle': {
        'param': 'obstacle_force_factor',
        'low_scale': 0.5,
        'high_scale': 2.0,
        'low_label': 'OF × 0.50 (low wall avoidance)',
        'high_label': 'OF × 2.00 (high wall avoidance)',
    },
}

CLAMP_RANGES = {
    'social_force_factor': (5.0, 20.0),
    'goal_force_factor': (2.0, 5.0),
    'obstacle_force_factor': (2.0, 50.0),
    'other_force_factor': (0.0, 25.0),
    'max_vel': (0.5, 1.8),
}


def clamp(value, param_name):
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


def generate_bt_files(baseline_yaml, corner_agent_id_offset):
    """Generate BT XML files for all 50 agents."""
    params = baseline_yaml['hunav_loader']['ros__parameters']

    # Build group membership map: agent_id → group info
    all_groups = {}
    corner_order = ['top', 'bl', 'br']

    for corner_name in corner_order:
        offset = corner_agent_id_offset[corner_name]
        for gid, members in CORNER_GROUPS[corner_name]:
            agent_ids = [offset + m for m in members]
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
                follower_ids_str = ','.join(str(x) for x in group_info['follower_ids'])
                content = BT_GROUP_LEADER_TEMPLATE.format(
                    agent_id=agent_id,
                    goal1=goal1,
                    goal2=goal2,
                    follower_ids=follower_ids_str,
                )
            else:
                content = BT_GROUP_FOLLOWER_TEMPLATE.format(
                    agent_id=agent_id,
                    goal1=goal1,
                    goal2=goal2,
                )
        else:
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
    print("DELTA (TRIANGLE) OAT YAML + BT GENERATOR")
    print("=" * 70)

    # 1) Generate baseline
    print("\n[1/3] Generating baseline YAML...")
    baseline, corner_offsets = build_baseline_yaml()
    baseline_path = os.path.join(SCENARIO_DIR, 'delta_baseline.yaml')
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

    # Corner and group summary
    print(f"\n    Corner assignment:")
    for corner_name in ['top', 'bl', 'br']:
        offset = corner_offsets[corner_name]
        n = CORNERS[corner_name]['n_agents']
        print(f"      {corner_name.upper():12s}: agents {offset}-{offset + n - 1} ({n} agents)")

    n_groups = sum(len(g) for g in CORNER_GROUPS.values())
    n_in_groups = sum(len(m) for groups in CORNER_GROUPS.values() for _, m in groups)
    n_individual = NUM_AGENTS - n_in_groups
    n_dyads = sum(1 for groups in CORNER_GROUPS.values() for _, m in groups if len(m) == 2)
    n_triads = sum(1 for groups in CORNER_GROUPS.values() for _, m in groups if len(m) == 3)
    print(f"\n    Groups: {n_groups} total ({n_dyads} dyads, {n_triads} triads)")
    print(f"    Individuals: {n_individual}  |  In groups: {n_in_groups}")
    print(f"    Flow: 3-way crossing (each corner → opposite two corners)")
    print(f"    Goal distance: ~25-35m (cross-triangle traversal)")

    # 2) Generate phase variants
    print("\n[2/3] Generating phase variant YAMLs...")
    for phase_name, phase_cfg in PHASES.items():
        param = phase_cfg['param']

        low = scale_yaml(baseline, param, phase_cfg['low_scale'])
        low_path = os.path.join(SCENARIO_DIR, f'delta_{phase_name}_low.yaml')
        save_yaml(low, low_path)
        print(f"  ✓ {os.path.basename(low_path):44s} {phase_cfg['low_label']}")

        high = scale_yaml(baseline, param, phase_cfg['high_scale'])
        high_path = os.path.join(SCENARIO_DIR, f'delta_{phase_name}_high.yaml')
        save_yaml(high, high_path)
        print(f"  ✓ {os.path.basename(high_path):44s} {phase_cfg['high_label']}")

    # 3) Generate BT files
    print("\n[3/3] Generating Behavior Tree XML files...")
    bt_files = generate_bt_files(baseline, corner_offsets)

    os.makedirs(BT_SRC_DIR, exist_ok=True)
    for fname, content in bt_files.items():
        with open(os.path.join(BT_SRC_DIR, fname), 'w') as f:
            f.write(content)

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
    print(f"    delta_baseline.yaml")
    for phase_name in PHASES:
        print(f"    delta_{phase_name}_low.yaml")
        print(f"    delta_{phase_name}_high.yaml")

    print(f"\nBehavior Trees ({len(bt_files)} files):")
    print(f"  {BT_SRC_DIR}/")

    print("\n" + "=" * 70)
    print("OAT TEST PLAN")
    print("=" * 70)
    print("""
Phase      | Runs  | Config File                    | Parameter Varied
-----------|-------|--------------------------------|-----------------
Baseline   | 1-3   | delta_baseline.yaml            | None (reference)
Phase 1    | 4-5   | delta_phase1_social_low.yaml   | Social Force × 0.55
Phase 1    | 6-7   | delta_phase1_social_high.yaml  | Social Force × 1.70
Phase 2    | 8-9   | delta_phase2_goal_low.yaml     | Goal Force × 0.60
Phase 2    | 10-11 | delta_phase2_goal_high.yaml    | Goal Force × 1.60
Phase 3    | 12-13 | delta_phase3_speed_low.yaml    | Max Velocity × 0.75
Phase 3    | 14-15 | delta_phase3_speed_high.yaml   | Max Velocity × 1.25
Phase 4    | 16-17 | delta_phase4_obstacle_low.yaml | Obstacle Force × 0.50
Phase 4    | 18-19 | delta_phase4_obstacle_high.yaml| Obstacle Force × 2.00

Total: 19 runs (~4-5 hours at 120s/run + setup time)
""")

    print("NEXT STEPS:")
    print("  1. colcon build --packages-select hunav_gazebo_wrapper")
    print("     source install/setup.bash")
    print("  2. ros2 launch hunav_gazebo_wrapper delta_no_robot.launch.py \\")
    print("       configuration_file:=domenic/delta/delta_baseline.yaml")
    print("  3. python3 run_recording.py <run_id>")


if __name__ == '__main__':
    main()
