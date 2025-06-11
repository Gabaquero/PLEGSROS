from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Launch arguments
    joint_name_arg = DeclareLaunchArgument(
        'joint',
        default_value='right_hip',
        description='Joint name to test'
    )
    
    can_id_arg = DeclareLaunchArgument(
        'can_id',
        default_value='0x201',
        description='CAN ID for the joint'
    )
    
    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='true',
        description='Use simulated CAN data'
    )

    joint_name = LaunchConfiguration('joint')
    can_id = LaunchConfiguration('can_id')
    use_sim = LaunchConfiguration('use_sim')

    # Reception node
    reception_node = Node(
        package='robot_control_system',
        executable='reception_node',
        name='reception_test',
        parameters=[{
            "joint_name": joint_name,
            "can_id": can_id,
            "use_simulated_data": use_sim,
            "position_limits": [-180.0, 180.0],
            "current_limits": [-7.0, 7.0],
            "velocity_limits": [-3800.0, 3800.0],
            "temperature_limits": [-40.0, 120.0],
        }],
        output='screen'
    )

    # PD controller node
    pd_node = Node(
        package='robot_control_system',
        executable='pd_controller_node',
        name='pd_controller_test',
        parameters=[{
            "joint_name": joint_name,
            "kp": 250.0,
            "kd": 1.5,
            "max_velocity": 3800.0,
        }],
        output='screen'
    )

    # Command sender node
    command_node = Node(
        package='robot_control_system',
        executable='command_sender_node',
        name='command_sender_test',
        parameters=[{
            "joint_name": joint_name,
            "can_id": can_id,
            "velocity_limits": [-3800.0, 3800.0],
            "can_interface": "can0"
        }],
        output='screen'
    )

    return LaunchDescription([
        joint_name_arg,
        can_id_arg,
        use_sim_arg,
        reception_node,
        pd_node,
        command_node
    ])