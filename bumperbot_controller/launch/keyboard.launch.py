import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get package directory
    bumperbot_controller_pkg = get_package_share_directory('bumperbot_controller')

    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="True",
        description="Use simulated time"
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
        custom_teleop_keyboard,
    ])
