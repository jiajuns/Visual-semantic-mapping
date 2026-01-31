from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
import datetime

def generate_launch_description():
    # 生成带时间戳的默认包名
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    default_bag_name = f"semantic_session_{timestamp}"

    # 1. 启动相机 (配置与实时建图完全一致)
    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py'
            ])
        ]),
        launch_arguments={
            'align_depth.enable': 'true',
            'enable_sync': 'true',
            'enable_gyro': 'true',
            'enable_accel': 'true',
            'unite_imu_method': '2',
            'rgb_camera.profile': '640x480x15',
            'depth_module.profile': '640x480x15',
            'gyro_fps': '200',
            'accel_fps': '100',
        }.items()
    )

    # 2. 录制 Bag
    # 录制所有必要的话题：RGB, 深度, 相机参数, IMU (如果需要), TF
    record_cmd = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '-o', default_bag_name,
            '/camera/color/image_raw',
            '/camera/aligned_depth_to_color/image_raw',
            '/camera/color/camera_info',
            '/camera/imu',  # 录制 IMU 有助于离线优化
            '/tf',
            '/tf_static'
        ],
        output='screen'
    )

    return LaunchDescription([
        realsense,
        record_cmd
    ])
