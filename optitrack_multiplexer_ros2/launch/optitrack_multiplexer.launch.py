from launch import LaunchDescription
from launch_ros.actions import Node
import os
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    ld = LaunchDescription()
    rigid_body_names_arg = DeclareLaunchArgument(
        'rigid_body_names',
        default_value='',  # Default value as an empty list
        description='List of rigid body names'
    )

    rigid_body_names = LaunchConfiguration('rigid_body_names')

    # Create the NatNet client node
    config = os.path.join(
        get_package_share_directory('optitrack_multiplexer_ros2'),
        'config',
        'optitrack_multiplexer_config.yaml'
    )
    optitrack_multiplexer = Node(
        package='optitrack_multiplexer_ros2',
        executable='multiplexer_node',
        name='optitrack_multiplexer_node',
        parameters=[config, {'rigid_body_names': rigid_body_names}],
        # prefix=['xterm -e gdb -ex run --args'],
        output='screen',
        emulate_tty=True
    )

    ld.add_action(optitrack_multiplexer)
    return ld
