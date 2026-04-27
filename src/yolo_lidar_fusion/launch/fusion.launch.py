import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('yolo_lidar_fusion')
    default_params_file = os.path.join(pkg_share, 'params', 'fusion.params.yaml')

    params_file = LaunchConfiguration('params_file')
    image_topic = LaunchConfiguration('image_topic')
    scan_topic = LaunchConfiguration('scan_topic')
    model_file = LaunchConfiguration('model_file')
    fov_deg = LaunchConfiguration('fov_deg')
    conf_threshold = LaunchConfiguration('conf_threshold')
    min_valid_distance = LaunchConfiguration('min_valid_distance')
    show_debug_window = LaunchConfiguration('show_debug_window')
    log_interval_sec = LaunchConfiguration('log_interval_sec')

    declare_args = [
        DeclareLaunchArgument('params_file', default_value=default_params_file),
        DeclareLaunchArgument('image_topic', default_value='/image_raw'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('model_file', default_value=''),
        DeclareLaunchArgument('fov_deg', default_value='60.0'),
        DeclareLaunchArgument('conf_threshold', default_value='0.5'),
        DeclareLaunchArgument('min_valid_distance', default_value='0.02'),
        DeclareLaunchArgument('show_debug_window', default_value='true'),
        DeclareLaunchArgument('log_interval_sec', default_value='1.0'),
    ]

    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='v4l2_camera_node',
        output='screen',
        parameters=[{
            'video_device': '/dev/my_camera0',
        }],
    )

    lidar_launch_path = os.path.join(
        get_package_share_directory('ldlidar_stl_ros2'),
        'launch',
        'ld19.launch.py',
    )

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(lidar_launch_path)
    )

    fusion_node = Node(
        package='yolo_lidar_fusion',
        executable='fusion_node',
        name='fusion_node',
        output='screen',
        parameters=[
            params_file,
            {
                'image_topic': image_topic,
                'scan_topic': scan_topic,
                'model_file': model_file,
                'fov_deg': fov_deg,
                'conf_threshold': conf_threshold,
                'min_valid_distance': min_valid_distance,
                'show_debug_window': show_debug_window,
                'log_interval_sec': log_interval_sec,
            },
        ],
    )

    return LaunchDescription(declare_args + [
        camera_node,
        lidar_launch,
        fusion_node,
    ])