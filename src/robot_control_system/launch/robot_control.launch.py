from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    # Launch arguments
    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='true',
        description='Use simulated CAN data'
    )
    
    trajectory_file_arg = DeclareLaunchArgument(
        'trajectory_file',
        default_value='walk_cycle.csv',
        description='Trajectory CSV file'
    )
    
    enable_safety_arg = DeclareLaunchArgument(
        'enable_safety',
        default_value='true',
        description='Enable safety monitor node'
    )

    # Get configuration
    use_sim = LaunchConfiguration('use_sim')
    trajectory_file = LaunchConfiguration('trajectory_file')
    enable_safety = LaunchConfiguration('enable_safety')

    # Joint configuration - matching your system
    joints = [
        {"name": "right_hip", "can_id": 0x201},
        {"name": "right_knee", "can_id": 0x301},
        {"name": "right_ankle", "can_id": 0x401},
        {"name": "left_hip", "can_id": 0x501},
        {"name": "left_knee", "can_id": 0x601},
        {"name": "left_ankle", "can_id": 0x701},
    ]

    # Create nodes for each joint
    reception_nodes = []
    pd_controller_nodes = []
    command_sender_nodes = []

    for joint in joints:
        # Reception node
        reception_node = Node(
            package='robot_control_system',
            executable='reception_node',
            name=f"reception_{joint['name']}",
            parameters=[{
                "joint_name": joint['name'],
                "can_id": joint['can_id'],
                "use_simulated_data": use_sim,
                "position_limits": [-180.0, 180.0],
                "current_limits": [-7.0, 7.0],
                "velocity_limits": [-3800.0, 3800.0],
                "temperature_limits": [-40.0, 120.0],
                "timeout_seconds": 0.1,
                "can_interface": "can0"
            }],
            output='screen'
        )
        reception_nodes.append(reception_node)

        # PD controller node
        pd_node = Node(
            package='robot_control_system',
            executable='pd_controller_node',
            name=f"pd_controller_{joint['name']}",
            parameters=[{
                "joint_name": joint['name'],
                "kp": 250.0,
                "kd": 1.5,
                "max_velocity": 3800.0,
                "velocity_filter_alpha": 0.8,
                "derivative_window_size": 5,
                "deadband": 0.1,
                "use_feedforward": False,
                "feedforward_gain": 0.0
            }],
            output='screen'
        )
        pd_controller_nodes.append(pd_node)

        # Command sender node
        command_node = Node(
            package='robot_control_system',
            executable='command_sender_node',
            name=f"command_sender_{joint['name']}",
            parameters=[{
                "joint_name": joint['name'],
                "can_id": joint['can_id'],
                "velocity_limits": [-3800.0, 3800.0],
                "can_interface": "can0"
            }],
            output='screen'
        )
        command_sender_nodes.append(command_node)

    # Trajectory publisher node
    trajectory_node = Node(
        package='robot_control_system',
        executable='trajectory_publisher',
        name='trajectory_publisher',
        parameters=[{
            "trajectory_file": trajectory_file,
            "playback_speed": 1.0,
            "publish_rate": 500.0,
            "loop_trajectory": True,
            "interpolate": True,
            "time_column_scale": 0.001
        }],
        output='screen'
    )

    # Safety monitor node
    safety_node = Node(
        package='robot_control_system',
        executable='safety_monitor',
        name='safety_monitor',
        parameters=[{
            "max_position_error": 10.0,
            "max_current": 6.5,
            "max_temperature": 80.0,
            "watchdog_timeout": 0.5,
            "joints": [j['name'] for j in joints]
        }],
        output='screen',
        condition=IfCondition(enable_safety)
    )

    # Group nodes by function for organized startup
    reception_group = GroupAction([
        TimerAction(
            period=0.0,
            actions=reception_nodes
        )
    ])

    control_group = GroupAction([
        TimerAction(
            period=1.0,  # Wait 1 second for reception nodes
            actions=pd_controller_nodes + command_sender_nodes
        )
    ])

    trajectory_group = GroupAction([
        TimerAction(
            period=2.0,  # Wait 2 seconds for control nodes
            actions=[trajectory_node]
        )
    ])

    return LaunchDescription([
        # Arguments
        use_sim_arg,
        trajectory_file_arg,
        enable_safety_arg,
        
        # Nodes in startup order
        reception_group,
        control_group,
        trajectory_group,
        safety_node
    ])