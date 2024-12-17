import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch.actions import TimerAction


def generate_launch_description():
    # Setup project paths
    pkg_project_bringup = get_package_share_directory('robot_bringup')
    pkg_project_description = get_package_share_directory('robot_description')
    pkg_detection = get_package_share_directory('detection')
    pkg_autorace_camera = get_package_share_directory('autorace_camera')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    signs_detection = Node(
            package='detection',
            executable='signs_detection',
            name='main',
    )
    
    generated_signs_detection = Node(
            package='detection',
            executable='generated_signs_detection',
            name='main',
    )

    traffic_light = Node(
            package='detection',
            executable='traffic_light',
            name='main',
    )

    depth_filter = Node(
            package='robot_app',
            executable='depth_filter',
            name='main',
    )
    
    # Extrinsic camera
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_autorace_camera, 'launch', 'extrinsic_camera_calibration.launch.py')),
    )

    lane_following = Node(
            package='robot_app',
            executable='lane_following',
            name='main'
    )

    head = Node(
            package='robot_app',
            executable='head',
            name='main'
    )
    
    parking = Node(
            package='robot_app',
            executable='parking',
            name='main'
    )


    return LaunchDescription([
        signs_detection,
        generated_signs_detection,
        traffic_light,
        depth_filter,
        camera,
        lane_following,
        parking,
        head
    ])
