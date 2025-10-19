import os
import launch
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    Generate launch description for integrated LiDAR mapping system.

    System includes:
    1. Livox LiDAR driver (publishes point cloud)
    2. FAST-LIO SLAM (processes point cloud, publishes cloud_registered)
    3. OctoMap Server (creates 3D occupancy map from point cloud)

    Execution model: All nodes start simultaneously
    ROS2 automatically manages topic connections and message flow

    Data flow: Livox (/livox/lidar) -> FAST-LIO (/cloud_registered) -> OctoMap (/octomap_binary)
    """

    # Get package share directories
    fast_lio_package_dir = get_package_share_directory('fast_lio')

    # Define launch argument values
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Livox arguments
    livox_config_path = LaunchConfiguration('livox_config_path')
    livox_frame_id = LaunchConfiguration('livox_frame_id')
    livox_bd_code = LaunchConfiguration('livox_bd_code')

    # FAST-LIO arguments
    fast_lio_config_path = LaunchConfiguration('fast_lio_config_path')
    fast_lio_config_file = LaunchConfiguration('fast_lio_config_file')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config_path = LaunchConfiguration('rviz_config_path')

    # OctoMap arguments
    octomap_resolution = LaunchConfiguration('octomap_resolution')
    octomap_frame_id = LaunchConfiguration('octomap_frame_id')
    octomap_max_range = LaunchConfiguration('octomap_max_range')

    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    # Livox driver arguments
    declare_livox_config_path_cmd = DeclareLaunchArgument(
        'livox_config_path',
        default_value=os.path.join(
            get_package_share_directory('livox_ros_driver2'), 'config'
        ),
        description='Path to Livox configuration directory'
    )

    declare_livox_frame_id_cmd = DeclareLaunchArgument(
        'livox_frame_id',
        default_value='livox_frame',
        description='Frame ID for Livox LiDAR'
    )

    declare_livox_bd_code_cmd = DeclareLaunchArgument(
        'livox_bd_code',
        default_value='livox0000000001',
        description='Board ID code for Livox device'
    )

    # FAST-LIO arguments
    declare_fast_lio_config_path_cmd = DeclareLaunchArgument(
        'fast_lio_config_path',
        default_value=os.path.join(fast_lio_package_dir, 'config'),
        description='Path to FAST-LIO configuration directory'
    )

    declare_fast_lio_config_file_cmd = DeclareLaunchArgument(
        'fast_lio_config_file',
        default_value='mid360.yaml',
        description='FAST-LIO configuration file name'
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Launch RViz for visualization'
    )

    declare_rviz_config_path_cmd = DeclareLaunchArgument(
        'rviz_config_path',
        default_value=os.path.join(
            get_package_share_directory('lidar_mapping_bringup'), 'rviz', 'rviz.rviz'
        ),
        description='Path to RViz configuration file'
    )

    # OctoMap arguments
    declare_octomap_resolution_cmd = DeclareLaunchArgument(
        'octomap_resolution',
        default_value='0.05',
        description='Resolution of OctoMap voxels in meters'
    )

    declare_octomap_frame_id_cmd = DeclareLaunchArgument(
        'octomap_frame_id',
        default_value='camera_init',
        description='Fixed map frame for OctoMap'
    )

    declare_octomap_max_range_cmd = DeclareLaunchArgument(
        'octomap_max_range',
        default_value='40.0',
        description='Maximum range for point cloud integration in OctoMap'
    )

    # Create Livox driver node with configurable parameters
    livox_driver_node = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            # User-configurable parameters
            'xfer_format': 1,  # 0=PointCloud2(PointXYZRTL), 1=customized format
            'multi_topic': 0,  # 0=shared topic, 1=one topic per LiDAR
            'data_src': 0,     # 0=lidar
            'publish_freq': 10.0,
            'output_data_type': 0,
            'frame_id': livox_frame_id,
            'user_config_path': PathJoinSubstitution([
                livox_config_path,
                'MID360_config.json'
            ]),
            'cmdline_input_bd_code': livox_bd_code,
        }],
    )

    # Include FAST-LIO launch file
    fast_lio_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                fast_lio_package_dir, 'launch', 'mapping.launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'config_path': fast_lio_config_path,
            'config_file': fast_lio_config_file,
            'rviz': use_rviz,
            'rviz_cfg': rviz_config_path,
        }.items(),
    )

    # Create OctoMap Server node with configurable parameters
    octomap_node = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            # User-configurable parameters
            'resolution': octomap_resolution,
            'frame_id': octomap_frame_id,
            'sensor_model.max_range': octomap_max_range,

            # Fixed parameters (from original XML launch file)
            'base_frame_id': 'body',
            'incremental_2D_projection': False,
            'occupancy_min_z': 0.1,
            'occupancy_max_z': 1.0,
            'filter_ground_plane': True,
            'ground_filter.distance': 0.04,
            'ground_filter.angle': 0.15,
            'ground_filter.plane_distance': 1.00,
            'pointcloud_min_z': -3.0,
            'pointcloud_max_z': 1.5,
        }],
        remappings=[
            ('cloud_in', '/cloud_registered'),
        ],
    )

    # Create and return launch description
    # All nodes start simultaneously; ROS2 manages topic connections
    ld = LaunchDescription([
        # Declare all launch arguments
        declare_use_sim_time_cmd,
        declare_livox_config_path_cmd,
        declare_livox_frame_id_cmd,
        declare_livox_bd_code_cmd,
        declare_fast_lio_config_path_cmd,
        declare_fast_lio_config_file_cmd,
        declare_use_rviz_cmd,
        declare_rviz_config_path_cmd,
        declare_octomap_resolution_cmd,
        declare_octomap_frame_id_cmd,
        declare_octomap_max_range_cmd,

        # Add all launch descriptions and nodes (start simultaneously)
        livox_driver_node,
        fast_lio_launch_description,
        octomap_node,
    ])

    return ld
