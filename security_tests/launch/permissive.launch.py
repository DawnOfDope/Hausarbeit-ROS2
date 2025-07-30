# this needs to be used with policy simple
# 

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    """
    This launch file starts the TurtleBot3 Gazebo simulation, RViz, and RQT,
    and applies the necessary SROS2 security enclaves.
    
    IMPORTANT: Before running this launch file, you MUST set the following
    environment variables in your terminal:
    
    export ROS_SECURITY_KEYSTORE=/path/to/your/security/keystore
    export ROS_SECURITY_ENABLE=true
    export ROS_SECURITY_STRATEGY=Enforce
    """
    
    # Get the package directory for turtlebot3_gazebo to find its launch files
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    
    # 1. Include the existing TurtleBot3 world launch file.
    # The nodes started by this included file (like gzserver, robot_state_publisher,
    # and the Gazebo plugins) will automatically find their security enclaves.
    # SROS2 does this by matching their node names (e.g., /gazebo, /robot_state_publisher)
    # to the folder names inside your keystore/enclaves directory.
    turtlebot3_world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turtlebot3_gazebo, 'launch', 'turtlebot3_world.launch.py')
        )
    )

    # 2. Launch RViz2 with its specific enclave.
    # We explicitly tell RViz to use the '/rviz' enclave.
    # The name='rviz' argument is set to match your enclave folder name.
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',  # Set node name to match the enclave folder name
        output='screen',
        ros_arguments=['--enclave', '/rviz']
    )
    
    # 3. Launch RQT with its specific enclave.
    # When RQT starts, it creates a node with a name like 'rqt_gui_py_node_...'.
    # We pass the full enclave path for that specific node name.
    rqt_node = Node(
        package='rqt_gui',
        executable='rqt_gui',
        output='screen',
        ros_arguments=['--enclave', '/rqt_gui_py_node_142017']
    )

    # Assemble the launch description
    return LaunchDescription([
        LogInfo(msg=[
            "Starting secure launch! Make sure ROS_SECURITY_KEYSTORE is set."
        ]),
        
        turtlebot3_world_launch,
        rviz_node,
        rqt_node
    ])
