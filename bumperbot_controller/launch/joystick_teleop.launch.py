import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription 
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Get package directories
    bumperbot_controller_pkg = get_package_share_directory('bumperbot_controller')
    twist_mux_pkg = get_package_share_directory('twist_mux')

    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="True",
        description="Use simulated time"
    )

    # Joy teleop node
    joy_teleop = Node(
        package="joy_teleop",
        executable="joy_teleop",
        parameters=[
            os.path.join(bumperbot_controller_pkg, "config", "joy_teleop.yaml"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")}
        ],
    )

    # Joystick node
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joystick",
        parameters=[
            os.path.join(bumperbot_controller_pkg, "config", "joy_config.yaml"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")}
        ]
    )

    # Twist mux launch
    twist_mux_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(twist_mux_pkg, "launch", "twist_mux_launch.py")
        ),
        launch_arguments={
            "cmd_vel_out": "bumperbot_controller/cmd_vel_unstamped",
            "config_locks": os.path.join(bumperbot_controller_pkg, "config", "twist_mux_locks.yaml"),
            "config_topics": os.path.join(bumperbot_controller_pkg, "config", "twist_mux_topics.yaml"),
            "config_joy": os.path.join(bumperbot_controller_pkg, "config", "twist_mux_joy.yaml"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),
    )

    # Add custom teleop keyboard node with parameters
    custom_teleop_keyboard = Node(
        package="bumperbot_controller",
        executable="custom_teleop_keyboard.py",
        name="custom_teleop_keyboard",
        output="screen",
        parameters=[
            os.path.join(
                bumperbot_controller_pkg,
                'config',
                'teleop_keyboard_config.yaml'
            ),
            {"use_sim_time": LaunchConfiguration("use_sim_time")}
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        joy_teleop,
        joy_node,
        twist_mux_launch,
        custom_teleop_keyboard,
    ])
