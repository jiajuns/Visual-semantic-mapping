from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetParameter
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os
import yaml

def generate_launch_description():
    
    # 全局设置为仿真时间 (关键：让节点使用 Bag 的时间戳，而不是当前系统时间)
    sim_time = SetParameter(name='use_sim_time', value=True)

    # 读取统一配置
    pkg_share = FindPackageShare('semantic_builder').find('semantic_builder')
    config_file = os.path.join(pkg_share, 'config', 'semantic_mapping.yaml')
    with open(config_file, 'r') as f:
        config = yaml.safe_load(f)

    rtab_cfg = config['rtabmap']
    yolo_cfg = config['yolo_node']
    cloud_cfg = config['cloud_saver']

    # 1. YOLO 节点
    yolo = Node(
        package='semantic_builder',
        executable='yolo_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
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

    # 2. RTAB-Map (离线模式配置)
    map_dir = cloud_cfg['save_dir']
    offline_db = os.path.join(map_dir, 'rtabmap_offline.db')
    
    rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('rtabmap_launch'), 'launch', 'rtabmap.launch.py'
            ])
        ]),
        launch_arguments={
            'rtabmap_args': '--delete_db_on_start' if rtab_cfg['delete_db_on_start'] else '',
            'database_path': offline_db,  # 区分数据库名称
            'namespace': '',  # 防止 RTAB-Map 添加 /camera 前缀
            'rgb_topic': rtab_cfg['topics']['rgb'],
            'depth_topic': rtab_cfg['topics']['depth'],
            'camera_info_topic': rtab_cfg['topics']['camera_info'],
            'frame_id': rtab_cfg['frames']['frame_id'],
            'approx_sync': str(rtab_cfg['sync']['approx_sync']).lower(),
            'use_sim_time': 'true',  # 关键
            # 离线时加大队列，防止丢帧
            'queue_size': str(rtab_cfg['sync']['queue_size']),
            'topic_queue_size': str(rtab_cfg['sync']['topic_queue_size']),
            'sync_queue_size': str(rtab_cfg['sync']['sync_queue_size']),
            'odom_frame_id': rtab_cfg['frames']['odom_frame_id'],
            'map_frame_id': rtab_cfg['frames']['map_frame_id'],
            # 离线回放通常需要 SYSTEM_DEFAULT 兼容 QoS
            'qos': '2',
            # === 建图参数 (保持一致) ===
            'Grid/CellSize': str(rtab_cfg['grid']['cell_size']),
            'Grid/RangeMax': str(rtab_cfg['grid']['range_max']),
            'Grid/RangeMin': str(rtab_cfg['grid']['range_min']),
            'Reg/Strategy': str(rtab_cfg['registration']['strategy']),
            'Vis/MaxDepth': str(rtab_cfg['visual']['max_depth']),
            'Vis/MinDepth': str(rtab_cfg['visual']['min_depth']),
            'RGBD/AngularUpdate': str(rtab_cfg['rgbd']['angular_update']),
            'RGBD/LinearUpdate': str(rtab_cfg['rgbd']['linear_update']),
            'Mem/STMSize': str(rtab_cfg['memory']['stm_size']),
        }.items()
    )
    
    # 3. TSDF 融合节点
    cloud_saver = Node(
        package='semantic_builder',
        executable='cloud_saver',
        output='screen',
        parameters=[{
            'use_sim_time': True,
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
    
    # 4. RViz
    rviz_config = PathJoinSubstitution([
        FindPackageShare('semantic_builder'), 'rviz', 'semantic_mapping.rviz'
    ])
    
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        sim_time,
        yolo,
        rtabmap,
        cloud_saver,
        rviz
    ])
