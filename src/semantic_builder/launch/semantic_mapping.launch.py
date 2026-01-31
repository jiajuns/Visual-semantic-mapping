from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import yaml
import os

def generate_launch_description():
    
    # 加载 YAML 配置文件
    pkg_share = FindPackageShare('semantic_builder').find('semantic_builder')
    config_file = os.path.join(pkg_share, 'config', 'semantic_mapping.yaml')
    
    with open(config_file, 'r') as f:
        config = yaml.safe_load(f)
    
    # 提取配置
    rs_cfg = config['realsense']
    rtab_cfg = config['rtabmap']
    yolo_cfg = config['yolo_node']
    cloud_cfg = config['cloud_saver']
    sys_cfg = config['system']
    
    # 1. 启动相机
    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py'
            ])
        ]),
        launch_arguments={
            'align_depth.enable': str(rs_cfg['align_depth_enable']).lower(),
            'enable_sync': str(rs_cfg['enable_sync']).lower(),
            'enable_gyro': str(rs_cfg['enable_gyro']).lower(),
            'enable_accel': str(rs_cfg['enable_accel']).lower(),
            'unite_imu_method': str(rs_cfg['unite_imu_method']),
            'rgb_camera.profile': rs_cfg['rgb_camera_profile'],
            'depth_module.profile': rs_cfg['depth_module_profile'],
            'gyro_fps': str(rs_cfg['gyro_fps']),
            'accel_fps': str(rs_cfg['accel_fps']),
        }.items()
    )

    # 2. 启动 YOLO 节点 (带参数)
    yolo = Node(
        package='semantic_builder',
        executable='yolo_node',
        output='screen',
        parameters=[{
            'model_name': yolo_cfg['model_name'],
            'confidence_threshold': yolo_cfg['confidence_threshold'],
            'frame_skip': yolo_cfg['frame_skip'],
            'qos_depth': yolo_cfg['qos_depth'],
            'mask_erosion_kernel_size': yolo_cfg['mask_erosion_kernel_size'],
            'topic_rgb_input': yolo_cfg['topics']['rgb_input'],
            'topic_depth_input': yolo_cfg['topics']['depth_input'],
            'topic_camera_info': yolo_cfg['topics']['camera_info'],
            'topic_overlay_output': yolo_cfg['topics']['overlay_output'],
            'topic_mask_output': yolo_cfg['topics']['mask_output'],
            'topic_cloud_output': yolo_cfg['topics']['cloud_output'],
        }]
    )

    # 3. 启动 RTAB-Map
    rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('rtabmap_launch'), 'launch', 'rtabmap.launch.py'
            ])
        ]),
        launch_arguments={
            'rtabmap_args': '--delete_db_on_start' if rtab_cfg['delete_db_on_start'] else '',
            'database_path': rtab_cfg['database_path'],
            'rgb_topic': rtab_cfg['topics']['rgb'],
            'depth_topic': rtab_cfg['topics']['depth'],
            'camera_info_topic': rtab_cfg['topics']['camera_info'],
            'frame_id': rtab_cfg['frames']['frame_id'],
            'approx_sync': str(rtab_cfg['sync']['approx_sync']).lower(),
            'queue_size': str(rtab_cfg['sync']['queue_size']),
            'topic_queue_size': str(rtab_cfg['sync']['topic_queue_size']),
            'sync_queue_size': str(rtab_cfg['sync']['sync_queue_size']),
            'odom_frame_id': rtab_cfg['frames']['odom_frame_id'],
            'map_frame_id': rtab_cfg['frames']['map_frame_id'],
            'qos': str(rtab_cfg['sync']['qos']),
            # 点云质量优化参数
            'Grid/CellSize': str(rtab_cfg['grid']['cell_size']),
            'Grid/RangeMax': str(rtab_cfg['grid']['range_max']),
            'Grid/RangeMin': str(rtab_cfg['grid']['range_min']),
            'Grid/MaxGroundHeight': str(rtab_cfg['grid']['max_ground_height']),
            'Grid/MaxObstacleHeight': str(rtab_cfg['grid']['max_obstacle_height']),
            'Grid/NoiseFilteringRadius': str(rtab_cfg['grid']['noise_filtering_radius']),
            'Grid/NoiseFilteringMinNeighbors': str(rtab_cfg['grid']['noise_filtering_min_neighbors']),
            'Reg/Strategy': str(rtab_cfg['registration']['strategy']),
            'Vis/MaxDepth': str(rtab_cfg['visual']['max_depth']),
            'Vis/MinDepth': str(rtab_cfg['visual']['min_depth']),
            'RGBD/ProximityMaxGraphDepth': str(rtab_cfg['rgbd']['proximity_max_graph_depth']),
            'RGBD/AngularUpdate': str(rtab_cfg['rgbd']['angular_update']),
            'RGBD/LinearUpdate': str(rtab_cfg['rgbd']['linear_update']),
            'Mem/STMSize': str(rtab_cfg['memory']['stm_size']),
        }.items()
    )
    
    # 4. 语义点云保存节点 (带参数)
    cloud_saver = Node(
        package='semantic_builder',
        executable='cloud_saver',
        output='screen',
        parameters=[{
            'save_dir': cloud_cfg['save_dir'],
            'world_frame': cloud_cfg['frames']['world_frame'],
            'camera_frame': cloud_cfg['frames']['camera_frame'],
            'output_frame': cloud_cfg['frames']['output_frame'],
            'voxel_length': cloud_cfg['tsdf']['voxel_length'],
            'sdf_trunc': cloud_cfg['tsdf']['sdf_trunc'],
            'depth_trunc': cloud_cfg['tsdf']['depth_trunc'],
            'depth_scale': cloud_cfg['tsdf']['depth_scale'],
            'frame_skip': cloud_cfg['fusion']['frame_skip'],
            'sync_slop': cloud_cfg['fusion']['sync_slop'],
            'timer_period': cloud_cfg['publish']['timer_period'],
            'background_scale': cloud_cfg['visual']['background_scale'],
            'tf_wait_timeout': cloud_cfg['tf']['wait_timeout'],
            'tf_use_latest_fallback': cloud_cfg['tf']['use_latest_fallback'],
            'intrinsics_auto_scale': cloud_cfg['intrinsics']['auto_scale'],
            'topic_rgb_input': cloud_cfg['topics']['rgb_input'],
            'topic_depth_input': cloud_cfg['topics']['depth_input'],
            'topic_mask_input': cloud_cfg['topics']['mask_input'],
            'topic_camera_info': cloud_cfg['topics']['camera_info'],
            'topic_mesh_cloud_output': cloud_cfg['topics']['mesh_cloud_output'],
        }]
    )
    
    # 5. 启动 RViz2 可视化语义点云
    rviz_config = PathJoinSubstitution([
        FindPackageShare('semantic_builder'), 'rviz', 'semantic_mapping.rviz'
    ])
    
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([
        # 先启动相机
        realsense,
        # 等待指定时间后再启动其他节点，让相机充分加载
        TimerAction(
            period=sys_cfg['timer_delay'],
            actions=[
                yolo,
                rtabmap,
                cloud_saver,
                rviz
            ]
        )
    ])
