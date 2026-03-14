"""Complete custom launch for the UR5e + Robotiq 2F-85 peg-in-hole scene.

Start order (event-driven, matching the upstream UR approach):
  1. Gazebo (with our world file)
  2. robot_state_publisher
  3. spawn_entity  →  robot appears in Gazebo, controller_manager comes online
  4. joint_state_broadcaster  (after spawn_entity exits)
  5. joint_trajectory_controller + gripper_controller  (after jsb exits)
  6. move_group + RViz  (immediately; they wait for joint states internally)
  7. spawn_task_objects  (Timer: 10 s after launch)
  8. grasp_and_insert    (Timer: 20 s after launch)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ur_moveit_config.launch_common import load_yaml


def launch_setup(context, *args, **kwargs):

    # ── Parameters ────────────────────────────────────────────────────────────
    ur_type        = LaunchConfiguration('ur_type').perform(context)
    safety_limits  = LaunchConfiguration('safety_limits').perform(context)
    safety_margin  = LaunchConfiguration('safety_pos_margin').perform(context)
    safety_k       = LaunchConfiguration('safety_k_position').perform(context)
    launch_rviz    = LaunchConfiguration('launch_rviz').perform(context)
    gazebo_gui     = LaunchConfiguration('gazebo_gui').perform(context)
    grasp_offset   = LaunchConfiguration('grasp_offset').perform(context)
    approach_angle = LaunchConfiguration('approach_angle').perform(context)

    desc_pkg   = 'peg_in_hole_description'
    task_pkg   = 'peg_in_hole_task'
    moveit_pkg = 'ur_moveit_config'

    controllers_yaml_path = os.path.join(
        get_package_share_directory(desc_pkg), 'config', 'controllers.yaml')

    # ── Robot description (URDF) ──────────────────────────────────────────────
    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ',
        PathJoinSubstitution([FindPackageShare(desc_pkg), 'urdf',
                              'ur5e_robotiq85.urdf.xacro']),
        ' ur_type:=',             ur_type,
        ' safety_limits:=',       safety_limits,
        ' safety_pos_margin:=',   safety_margin,
        ' safety_k_position:=',   safety_k,
        ' sim_gazebo:=true',
        ' simulation_controllers:=', controllers_yaml_path,
    ])
    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)
    }

    # ── Robot description semantic (SRDF) ────────────────────────────────────
    srdf_content = Command([
        FindExecutable(name='xacro'), ' ',
        PathJoinSubstitution([FindPackageShare(desc_pkg), 'srdf',
                              'ur5e_robotiq85.srdf.xacro']),
        ' name:=ur prefix:=',
    ])
    robot_description_semantic = {
        'robot_description_semantic': srdf_content
    }

    # ── MoveIt config (reuse upstream kinematics / OMPL / joint limits) ───────
    kinematics_yaml   = load_yaml(moveit_pkg, 'config/kinematics.yaml')
    joint_limits_yaml = load_yaml(moveit_pkg, 'config/joint_limits.yaml')
    ompl_yaml         = load_yaml(moveit_pkg, 'config/ompl_planning.yaml')
    ompl_config = {
        'move_group': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': (
                'default_planner_request_adapters/AddTimeOptimalParameterization '
                'default_planner_request_adapters/FixWorkspaceBounds '
                'default_planner_request_adapters/FixStartStateBounds '
                'default_planner_request_adapters/FixStartStateCollision '
                'default_planner_request_adapters/FixStartStatePathConstraints'
            ),
            'start_state_max_bounds_error': 0.1,
        }
    }
    ompl_config['move_group'].update(ompl_yaml)

    moveit_controllers = load_yaml(desc_pkg, 'config/moveit_controllers.yaml')

    trajectory_execution = {
        'moveit_manage_controllers': False,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
        'trajectory_execution.execution_duration_monitoring': False,
    }

    planning_scene_monitor = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    # ── Nodes ─────────────────────────────────────────────────────────────────

    # 1. Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'])
        ),
        launch_arguments={
            'world': os.path.join(
                get_package_share_directory(task_pkg), 'worlds', 'peg_in_hole.world'),
            'gui':     gazebo_gui,
            'verbose': 'false',
        }.items(),
    )

    # 2. robot_state_publisher
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}],
    )

    # 3. Spawn robot into Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'ur', '-topic', '/robot_description'],
        output='screen',
    )

    # 4. joint_state_broadcaster – starts after spawn_robot exits
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster',
                   '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # 5a. Arm controller – starts after joint_state_broadcaster exits
    arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller',
                   '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # 5b. Gripper controller – starts alongside arm controller
    gripper_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller',
                   '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # 6. MoveIt move_group
    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            {'robot_description_kinematics': kinematics_yaml},
            {'robot_description_planning': joint_limits_yaml},
            ompl_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor,
            {'use_sim_time': True},
            {'publish_robot_description_semantic': True},
        ],
    )

    # 7. RViz2
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        condition=IfCondition(launch_rviz),
        arguments=['-d', PathJoinSubstitution(
            [FindPackageShare(moveit_pkg), 'rviz', 'view_robot.rviz'])],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_config,
            {'robot_description_kinematics': kinematics_yaml},
            {'use_sim_time': True},
        ],
        output='log',
    )

    # 8. Spawn task objects (timer – give Gazebo + controllers time to be ready)
    spawn_task = TimerAction(
        period=10.0,
        actions=[Node(
            package='peg_in_hole_task',
            executable='spawn_task_objects',
            output='screen',
            parameters=[{'use_sim_time': True}],
        )],
    )

    # 9. Grasp and insert (timer – starts after spawner has had time to finish)
    grasp_and_insert = TimerAction(
        period=20.0,
        actions=[Node(
            package='peg_in_hole_task',
            executable='grasp_and_insert',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'grasp_offset':   float(grasp_offset),
                'approach_angle': float(approach_angle),
                'auto_execute':   True,
            }],
        )],
    )

    # ── Event-driven sequencing ───────────────────────────────────────────────
    # jsb starts only after spawn_entity exits (controller_manager is online)
    jsb_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[joint_state_broadcaster],
        )
    )
    # arm + gripper controllers start only after jsb exits
    controllers_after_jsb = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[arm_controller, gripper_controller],
        )
    )

    return [
        gazebo,
        rsp,
        spawn_robot,
        jsb_after_spawn,
        controllers_after_jsb,
        move_group,
        rviz,
        spawn_task,
        grasp_and_insert,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('ur_type',           default_value='ur5e'),
        DeclareLaunchArgument('safety_limits',     default_value='true'),
        DeclareLaunchArgument('safety_pos_margin', default_value='0.15'),
        DeclareLaunchArgument('safety_k_position', default_value='20'),
        DeclareLaunchArgument('launch_rviz',       default_value='true'),
        DeclareLaunchArgument('gazebo_gui',        default_value='true'),
        DeclareLaunchArgument(
            'grasp_offset',
            default_value='0.0',
            description='Offset along peg long axis from centre (m, ±0.025).',
        ),
        DeclareLaunchArgument(
            'approach_angle',
            default_value='0.0',
            description='Gripper rotation around approach axis (deg, e.g. 0/45/90/135).',
        ),
        OpaqueFunction(function=launch_setup),
    ])
