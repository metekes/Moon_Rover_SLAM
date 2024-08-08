"""Launch Gazebo with the specified world file (empty world by default)."""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os
from launch_ros.actions import Node


def generate_launch_description():

    pkg_gazebo_ros = get_package_share_directory("gazebo_ros")
    pkg_ezrassor_sim_gazebo = get_package_share_directory("ezrassor_sim_gazebo")
    pkg_rover_control = get_package_share_directory("rover_control")

    world_argument = DeclareLaunchArgument(
        "world",
        default_value=[
            os.path.join(pkg_ezrassor_sim_gazebo, "worlds", "base.world"),
            "",
        ],
        description="Gazebo world file (full path)",
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, "launch", "gazebo.launch.py")
        ),
        launch_arguments={"world": LaunchConfiguration("world")}.items(),
    )
    
    slam_node = Node(
            package='rover_slam',
            namespace='rover_slam',
            executable='rover_slam',
            name='rover_SLAM'
    )

    '''
    c1 = Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=['/home/mete/rover.xml','/home/mete/ros_ws/src/rover_control/config/rover_controllers.yaml'],
            output='screen'
    )

    c2 = Node(
            package='controller_manager',
            executable='spawner',
            arguments=['diff_drive_controller'],
            output='screen'
    )
    
    # Your controller node
    command_publisher = Node(
        package='rover_control',
        executable='command_publisher',
        name='command_publisher',
        output='screen'
    )
    '''

    return LaunchDescription([world_argument, gazebo, slam_node])
