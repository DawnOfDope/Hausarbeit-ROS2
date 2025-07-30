# Save this file as "secure_all_in_one.launch.py"
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """
    This launch file correctly starts the entire secure environment.
    It explicitly defines each node and assigns its security enclave,
    avoiding the issues with including other launch files.
    """
    
    # --- File Paths ---
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')

    # Get the URDF file for the robot state publisher
    urdf_file_name = 'turtlebot3_burger.urdf'
    urdf_path = os.path.join(
        pkg_turtlebot3_gazebo,
        'urdf',
        urdf_file_name)
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    # --- Gazebo Server and Client ---
    # We must explicitly pass the enclave path to gzserver.
    gzserver_cmd = ExecuteProcess(
        cmd=['gzserver',
             '--verbose',
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so',
             os.path.join(pkg_turtlebot3_gazebo, 'worlds', 'turtlebot3_world.world'),
             '--ros-args', '--enclave', '/gazebo' # Explicitly set enclave
             ],
        output='screen'
    )

    gzclient_cmd = ExecuteProcess(
        cmd=['gzclient'],
        output='screen'
    )

    # --- Robot State Publisher ---
    # Define the node directly so we can pass its enclave path.
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}],
        ros_arguments=['--enclave', '/robot_state_publisher'] # Explicitly set enclave
    )

    # --- Spawn Entity Node ---
    # Define the spawner node directly to pass its enclave path.
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        arguments=['-entity', 'turtlebot3_burger',
                   '-topic', 'robot_description',
                   '-x', '0.0',
                   '-y', '0.0',
                   '-z', '0.01'],
        output='screen',
        ros_arguments=['--enclave', '/spawn_entity'] # Explicitly set enclave
    )

    # --- RViz ---
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        output='screen',
        ros_arguments=['--enclave', '/rviz']
    )

    return LaunchDescription([
        LogInfo(msg=["Starting final secure launch..."]),
        gzserver_cmd,
        gzclient_cmd,
        robot_state_publisher_node,
        spawn_entity_node,
        rviz_node,
    ])
