from os import path
from os import environ
from os import pathsep
from scripts import GazeboRosPaths
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (SetEnvironmentVariable,
                            DeclareLaunchArgument, ExecuteProcess, Shutdown,
                            RegisterEventHandler, TimerAction, LogInfo)
from launch.conditions import UnlessCondition
from launch.substitutions import (PathJoinSubstitution,
                            LaunchConfiguration, PythonExpression, EnvironmentVariable)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessStart


def generate_launch_description():

    # to activate the use of nvidia gpu
    use_nvidia_gpu = [
        '__NV_PRIME_RENDER_OFFLOAD=1 ',
        '__GLX_VENDOR_LIBRARY_NAME=nvidia ',
    ]

    # World generation parameters
    world_file_name = LaunchConfiguration('base_world')
    gz_obs = LaunchConfiguration('use_gazebo_obs')
    rate = LaunchConfiguration('update_rate')
    robot_name = LaunchConfiguration('robot_name')
    global_frame = LaunchConfiguration('global_frame_to_publish')
    use_navgoal = LaunchConfiguration('use_navgoal_to_start')
    navgoal_topic = LaunchConfiguration('navgoal_topic')
    ignore_models = LaunchConfiguration('ignore_models')
    navigation = LaunchConfiguration('navigation')

    # agent configuration file
    agent_conf_file = PathJoinSubstitution([
        FindPackageShare('hunav_gazebo_wrapper'),
        'scenarios',
        LaunchConfiguration('configuration_file')
    ])

    # Read the yaml file and load the parameters
    hunav_loader_node = Node(
        package='hunav_agent_manager',
        executable='hunav_loader',
        output='screen',
        parameters=[agent_conf_file]
    )

    # hunav agent manager
    hunav_manager_node = Node(
        package='hunav_agent_manager',
        executable='hunav_agent_manager',
        output='screen'
    )

    # hunav evaluator
    metrics_conf_file = PathJoinSubstitution([
        FindPackageShare('hunav_evaluator'),
        'config',
        LaunchConfiguration('metrics_file')
    ])
    hunav_evaluator_node = Node(
        package='hunav_evaluator',
        executable='hunav_evaluator_node',
        output='screen',
        parameters=[metrics_conf_file]
    )

    # Static TF
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='log',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'map', 'odom']
    )

    # GAZEBO launch
    model_path, plugin_path, resource_path = GazeboRosPaths.get_paths()

    # Environment variables
    set_env_gazebo_model = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=model_path
    )
    set_env_gazebo_resource = SetEnvironmentVariable(
        name='GAZEBO_RESOURCE_PATH',
        value=resource_path
    )
    set_env_gazebo_plugin = SetEnvironmentVariable(
        name='GAZEBO_PLUGIN_PATH',
        value=plugin_path
    )

    world_gen_pkg_path = get_package_share_directory('hunav_gazebo_wrapper')
    
    # world base file - full path
    world_file = PathJoinSubstitution([
        FindPackageShare('hunav_gazebo_wrapper'),
        'worlds',
        world_file_name
    ])
    
    world_gen_node = Node(
        package='hunav_gazebo_wrapper',
        executable='hunav_gazebo_world_generator',
        output='screen',
        parameters=[
            {'base_world': world_file},
            {'use_gazebo_obs': gz_obs},
            {'update_rate': rate},
            {'robot_name': robot_name},
            {'global_frame_to_publish': global_frame},
            {'use_navgoal_to_start': use_navgoal},
            {'navgoal_topic': navgoal_topic},
            {'ignore_models': ignore_models}
        ]
    )

    ordered_launch_event = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=hunav_loader_node,
            on_start=[
                LogInfo(msg='HunNavLoader started, launching HuNav_Gazebo_world_generator after 2 seconds...'),
                TimerAction(
                    period=2.0,
                    actions=[world_gen_node]
                )
            ]
        )
    )

    # After world generation, launch Gazebo
    gazebo_launch_name = path.join(
        get_package_share_directory('gazebo_ros'),
        'launch',
        'gzserver.launch.py'
    )
    gazebo_launch_arguments = {
        'world': PathJoinSubstitution([
            FindPackageShare('hunav_gazebo_wrapper'),
            'worlds',
            'generatedWorld.world'
        ]),
        'pause': 'true',
        'verbose': 'true',
        'extra_gazebo_args': '--ros-args --params-file ' + path.join(world_gen_pkg_path, 'launch', 'params.yaml')
    }

    gzserver_launch = ExecuteProcess(
        cmd=use_nvidia_gpu + ['gzserver ', ' --pause ', PathJoinSubstitution([
            FindPackageShare('hunav_gazebo_wrapper'),
            'worlds',
            'generatedWorld.world'
        ]), ' --verbose', ' -s ', ' libgazebo_ros_init.so', ' -s ', ' libgazebo_ros_factory.so',
             ' --ros-args --params-file ', path.join(world_gen_pkg_path, 'launch', 'params.yaml')],
        output='screen',
        shell=True,
        on_exit=Shutdown()
    )

    gzclient_launch = ExecuteProcess(
        cmd=use_nvidia_gpu + ['gzclient', ' --verbose'],
        output='screen',
        shell=True,
        on_exit=Shutdown()
    )

    gz_launch_event = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=world_gen_node,
            on_start=[
                LogInfo(msg='GenerateWorld started, launching Gazebo after 2 seconds...'),
                TimerAction(
                    period=2.0,
                    actions=[gzserver_launch, gzclient_launch]
                )
            ]
        )
    )

    # launch arguments
    declare_agents_conf_file = DeclareLaunchArgument(
        'configuration_file', default_value='agents_cafe.yaml',
        description='Agent configuration yaml file. Must be located in the scenarios folder'
    )
    declare_metrics_conf_file = DeclareLaunchArgument(
        'metrics_file', default_value='metrics.yaml',
        description='Metrics configuration yaml file. Must be located in the config folder of hunav_evaluator package'
    )
    declare_arg_world = DeclareLaunchArgument(
        'base_world', default_value='cafe.world',
        description='Base world file name (with .world extension) in hunav_gazebo_wrapper/worlds'
    )
    declare_gz_obs = DeclareLaunchArgument(
        'use_gazebo_obs', default_value='True',
        description='Whether to use Gazebo models as obstacles or not'
    )
    declare_update_rate = DeclareLaunchArgument(
        'update_rate', default_value='100.0',
        description='Update rate of the plugin'
    )
    declare_robot_name = DeclareLaunchArgument(
        'robot_name', default_value='dummy_robot',
        description='Name of an existing Gazebo model used as a stand-in robot (no robot spawned)'
    )
    declare_frame_to_publish = DeclareLaunchArgument(
        'global_frame_to_publish', default_value='map',
        description='Name of the global frame in which the position of the agents are provided'
    )
    declare_use_navgoal = DeclareLaunchArgument(
        'use_navgoal_to_start', default_value='False',
        description='Whether to start the agents movements when a navigation goal is received or not'
    )
    declare_navgoal_topic = DeclareLaunchArgument(
        'navgoal_topic', default_value='goal_pose',
        description='Name of the topic in which navigation goal for the robot will be published'
    )
    declare_navigation = DeclareLaunchArgument(
        'navigation', default_value='False',
        description='If launch the pmb2 navigation system'
    )
    declare_ignore_models = DeclareLaunchArgument(
        'ignore_models', default_value='ground_plane dummy_robot',
        description='list of Gazebo models that the agents should ignore as obstacles as the ground_plane. Indicate the models with a blank space between them'
    )
    declare_arg_verbose = DeclareLaunchArgument(
        'verbose', default_value='true',
        description='Set "true" to increase messages written to terminal.'
    )

    ld = LaunchDescription()

    # set environment variables
    ld.add_action(set_env_gazebo_model)
    ld.add_action(set_env_gazebo_resource)
    ld.add_action(set_env_gazebo_plugin)

    # Declare the launch arguments
    ld.add_action(declare_agents_conf_file)
    ld.add_action(declare_metrics_conf_file)
    ld.add_action(declare_arg_world)
    ld.add_action(declare_gz_obs)
    ld.add_action(declare_update_rate)
    ld.add_action(declare_robot_name)
    ld.add_action(declare_frame_to_publish)
    ld.add_action(declare_use_navgoal)
    ld.add_action(declare_navgoal_topic)
    ld.add_action(declare_navigation)
    ld.add_action(declare_ignore_models)
    ld.add_action(declare_arg_verbose)

    # Generate the world with the agents
    # launch hunav_loader and the WorldGenerator
    # 2 seconds later
    ld.add_action(hunav_loader_node)
    ld.add_action(ordered_launch_event)

    # hunav behavior manager node
    ld.add_action(hunav_manager_node)
    # hunav evaluator
    ld.add_action(hunav_evaluator_node)

    # launch Gazebo after worldGenerator
    ld.add_action(gz_launch_event)
    ld.add_action(static_tf_node)

    return ld


# Add boolean commands if true
def _boolean_command(arg):
    cmd = ['"--', arg, '" if "true" == "', LaunchConfiguration(arg), '" else ""']
    py_cmd = PythonExpression(cmd)
    return py_cmd
