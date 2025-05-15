from launch import LaunchDescription
from launch_ros.actions import Node
import os
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    ld = LaunchDescription()

    server_address_arg = DeclareLaunchArgument(
        'server_address',
        default_value=''
    )
    server_address = LaunchConfiguration('server_address')

    # Create the NatNet client node
    config = os.path.join(
        get_package_share_directory('optitrack_wrapper_ros2'),
        'config',
        'optitrack_wrapper_config.yaml'
    )
    optitrack_wrapper = Node(
        package='optitrack_wrapper_ros2',
        executable='wrapper_node',
        name='optitrack_wrapper_node',
        parameters=[config, {'server_address': server_address}],
        # prefix=['xterm -e gdb -ex run --args'],
        output='screen',
        emulate_tty=True
    )

    ld.add_action(optitrack_wrapper)
    return ld
