import os
import xacro
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Replace this with your package name
    pkg_name = 'my_detector_pkg'
    pkg_share = get_package_share_directory(pkg_name)

    # Path to your xacro file
    xacro_file = os.path.join(pkg_share, 'urdf', 'DOUG_DIA.xacro')
    
    # Check if file exists
    if not os.path.exists(xacro_file):
        raise FileNotFoundError(f"Xacro file not found: {xacro_file}")

    # Convert xacro to URDF XML string
    urdf_doc = xacro.process_file(xacro_file)
    urdf_xml = urdf_doc.toxml()

    # Optional RViz config file
    rviz_config_file = os.path.join(pkg_share, 'configs', 'rviz_config.rviz')
    if not os.path.exists(rviz_config_file):
        print("UH OH")
        rviz_config_file = None  # Use default RViz layout

    return LaunchDescription([
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': urdf_xml}]
        ),

        # Joint State Publisher GUI (for movable joints)
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d',rviz_config_file] if rviz_config_file else []
        ),
    ])