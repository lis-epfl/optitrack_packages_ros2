from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory



def get_nodes(server_address, rigid_body_names): 

    config_wrapper = os.path.join(
        get_package_share_directory('optitrack_wrapper_ros2'),
        'config',
        'optitrack_wrapper_config.yaml'
    )
    optitrack_wrapper = Node(
        package='optitrack_wrapper_ros2',
        executable='wrapper_node',
        name='optitrack_wrapper_node',
        parameters=[config_wrapper, {'server_address': server_address}],
        # prefix=['xterm -e gdb -ex run --args'],
        output='screen',
        emulate_tty=True
    )

    config_multiplexer = os.path.join(
        get_package_share_directory('optitrack_multiplexer_ros2'),
        'config',
        'optitrack_multiplexer_config.yaml'
    )
    optitrack_multiplexer = Node(
        package='optitrack_multiplexer_ros2',
        executable='multiplexer_node',
        name='optitrack_multiplexer_node',
        parameters=[config_multiplexer, {'rigid_body_names': rigid_body_names}],
        output='screen',
        emulate_tty=True
    )

    return optitrack_wrapper, optitrack_multiplexer

def generate_launch_description():

    # Declare the rigid_body_names argument
    rigid_body_names_arg = DeclareLaunchArgument(
        'rigid_body_names',
        default_value='',  # Default value as an empty list
        description='List of rigid body names'
    )

    rigid_body_names = LaunchConfiguration('rigid_body_names')

    server_address_arg = DeclareLaunchArgument(
        'server_address',
        default_value=''
    )
    server_address = LaunchConfiguration('server_address')

    ld = LaunchDescription()

    # Create the NatNet client node
    optitrack_wrapper, optitrack_multiplexer = get_nodes(server_address, rigid_body_names)
   

    # Add the argument and nodes to the launch description
    ld.add_action(rigid_body_names_arg)
    ld.add_action(optitrack_wrapper)
    ld.add_action(optitrack_multiplexer)

    return ld
