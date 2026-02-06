import os
import launch
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
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
    sc_pgo_package_dir = get_package_share_directory('sc_pgo_ros2')

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

    # SC-PGO arguments
    use_sc_pgo = LaunchConfiguration('use_sc_pgo')
    sc_pgo_rviz = LaunchConfiguration('sc_pgo_rviz')
    sc_pgo_namespace = LaunchConfiguration('sc_pgo_namespace')

    # Map->Odom TF arguments
    use_map_odom_tf = LaunchConfiguration('use_map_odom_tf')
    map_frame_id = LaunchConfiguration('map_frame_id')
    odom_frame_id = LaunchConfiguration('odom_frame_id')

    # Cloud transform arguments (odom -> map)
    cloud_in_topic = LaunchConfiguration('cloud_in_topic')
    cloud_map_topic = LaunchConfiguration('cloud_map_topic')

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

    # SC-PGO arguments
    declare_use_sc_pgo_cmd = DeclareLaunchArgument(
        'use_sc_pgo',
        default_value='true',
        description='Launch SC-PGO (loop closure + pose graph optimization)'
    )

    declare_sc_pgo_rviz_cmd = DeclareLaunchArgument(
        'sc_pgo_rviz',
        default_value='false',
        description='Launch RViz for SC-PGO'
    )

    declare_sc_pgo_namespace_cmd = DeclareLaunchArgument(
        'sc_pgo_namespace',
        default_value='',
        description='Namespace for SC-PGO nodes'
    )

    # Map->Odom TF arguments
    declare_use_map_odom_tf_cmd = DeclareLaunchArgument(
        'use_map_odom_tf',
        default_value='true',
        description='Launch map->odom TF broadcaster'
    )

    declare_map_frame_id_cmd = DeclareLaunchArgument(
        'map_frame_id',
        default_value='map',
        description='Map frame for map->odom TF'
    )

    declare_odom_frame_id_cmd = DeclareLaunchArgument(
        'odom_frame_id',
        default_value='odom',
        description='Odom frame for map->odom TF'
    )

    # Cloud transform arguments
    declare_cloud_in_topic_cmd = DeclareLaunchArgument(
        'cloud_in_topic',
        default_value='/cloud_registered',
        description='Input point cloud topic (odom frame)'
    )

    declare_cloud_map_topic_cmd = DeclareLaunchArgument(
        'cloud_map_topic',
        default_value='/cloud_registered_map',
        description='Output point cloud topic transformed to map frame'
    )

    # OctoMap arguments
    declare_octomap_resolution_cmd = DeclareLaunchArgument(
        'octomap_resolution',
        default_value='0.05',
        description='Resolution of OctoMap voxels in meters'
    )

    declare_octomap_frame_id_cmd = DeclareLaunchArgument(
        'octomap_frame_id',
        default_value='map',
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

    # Include SC-PGO launch file
    sc_pgo_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                sc_pgo_package_dir, 'launch', 'sc_pgo.launch.py'
            )
        ),
        launch_arguments={
            'rvizscpgo': sc_pgo_rviz,
            'namespace': sc_pgo_namespace,
            'publish_tf': 'false',
        }.items(),
        condition=IfCondition(use_sc_pgo),
    )

    # Map->Odom TF broadcaster node
    map_odom_tf_node = Node(
        package='map_odom_broadcaster',
        executable='map_odom_broadcaster',
        name='map_odom_broadcaster',
        output='screen',
        parameters=[{
            'pgo_odom_topic': '/aft_pgo_odom',
            'fastlio_odom_topic': '/Odometry',
            'map_frame': map_frame_id,
            'odom_frame': odom_frame_id,
            'pgo_child_frame': 'aft_pgo',
            'fastlio_child_frame': 'base_link',
            'use_pgo_stamp': False,
            'max_pair_dt_sec': 0.2,
            'max_age_sec': 1.0,
            'publish_rate_hz': 20.0,
        }],
        condition=IfCondition(use_map_odom_tf),
    )

    # Transform point cloud from odom -> map for OctoMap input
    cloud_transform_node = Node(
        package='map_odom_broadcaster',
        executable='cloud_frame_transformer',
        name='cloud_frame_transformer',
        output='screen',
        parameters=[{
            'input_topic': cloud_in_topic,
            'output_topic': cloud_map_topic,
            'target_frame': octomap_frame_id,
            'queue_size': 10,
            'transform_timeout_sec': 0.1,
            'use_latest_transform': True,
        }],
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
            'base_frame_id': 'base_link',
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
            ('cloud_in', cloud_map_topic),
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
        declare_use_sc_pgo_cmd,
        declare_sc_pgo_rviz_cmd,
        declare_sc_pgo_namespace_cmd,
        declare_use_map_odom_tf_cmd,
        declare_map_frame_id_cmd,
        declare_odom_frame_id_cmd,
        declare_cloud_in_topic_cmd,
        declare_cloud_map_topic_cmd,
        declare_octomap_resolution_cmd,
        declare_octomap_frame_id_cmd,
        declare_octomap_max_range_cmd,

        # Add all launch descriptions and nodes (start simultaneously)
        livox_driver_node,
        fast_lio_launch_description,
        sc_pgo_launch_description,
        map_odom_tf_node,
        cloud_transform_node,
        octomap_node,
    ])

    return ld
