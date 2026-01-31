#!/usr/bin/env python3
"""
TSDF 语义融合建图节点 - 使用 Open3D 的 TSDF 体素融合生成光滑的语义网格

核心特性：
1. 使用 ScalableTSDFVolume 进行体素融合
2. RGB-D 图像结合位姿反向投影到体素网格
3. 使用 TF2 查询图像时间戳对应的精确变换
4. 向量化点云发布
5. 提取光滑、去噪的 Mesh 模型

发布话题：
  /tsdf/mesh_cloud - 从 TSDF 提取的融合点云
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
from datetime import datetime
from scipy.spatial.transform import Rotation as R
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.duration import Duration
import message_filters
import open3d as o3d
import tf2_ros
import colorsys
from rclpy.time import Time


# COCO 80 类别名称
COCO_CLASSES = [
    'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck', 'boat',
    'traffic light', 'fire hydrant', 'stop sign', 'parking meter', 'bench', 'bird', 'cat',
    'dog', 'horse', 'sheep', 'cow', 'elephant', 'bear', 'zebra', 'giraffe', 'backpack',
    'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee', 'skis', 'snowboard', 'sports ball',
    'kite', 'baseball bat', 'baseball glove', 'skateboard', 'surfboard', 'tennis racket',
    'bottle', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple',
    'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair',
    'couch', 'potted plant', 'bed', 'dining table', 'toilet', 'tv', 'laptop', 'mouse', 'remote',
    'keyboard', 'cell phone', 'microwave', 'oven', 'toaster', 'sink', 'refrigerator', 'book',
    'clock', 'vase', 'scissors', 'teddy bear', 'hair drier', 'toothbrush'
]

# 使用 HSV 生成均匀分布的颜色 (高饱和度、适中亮度)
COCO_COLORS = {}
for i, _ in enumerate(COCO_CLASSES):
    hue = i / len(COCO_CLASSES)
    rgb = colorsys.hsv_to_rgb(hue, 0.85, 0.9)
    COCO_COLORS[i] = tuple(int(c * 255) for c in rgb)


class TSDFFusionNode(Node):
    def __init__(self):
        super().__init__('tsdf_fusion_node')
        
        # ========== 从 ROS 参数获取配置 (由 launch 传入) ==========
        # 保存路径
        self.declare_parameter('save_dir', '/home/jj/project_total/camera_ws/src/semantic_builder/map')
        
        # 坐标系配置
        self.declare_parameter('world_frame', 'odom')
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        self.declare_parameter('output_frame', 'map')
        
        # TSDF 参数
        self.declare_parameter('voxel_length', 0.015)
        self.declare_parameter('sdf_trunc', 0.06)
        self.declare_parameter('depth_trunc', 3.0)
        self.declare_parameter('depth_scale', 1000.0)
        
        # 融合性能参数
        self.declare_parameter('frame_skip', 5)
        self.declare_parameter('sync_slop', 0.3)
        
        # 发布参数
        self.declare_parameter('timer_period', 5.0)
        # 可视化参数
        self.declare_parameter('background_scale', 0.25)
        # TF/内参健壮性参数
        self.declare_parameter('tf_wait_timeout', 0.5)
        self.declare_parameter('tf_use_latest_fallback', False)
        self.declare_parameter('intrinsics_auto_scale', True)
        
        # 话题配置
        self.declare_parameter('topic_rgb_input', '/camera/color/image_raw')
        self.declare_parameter('topic_depth_input', '/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('topic_mask_input', '/yolo/semantic_mask')
        self.declare_parameter('topic_camera_info', '/camera/color/camera_info')
        self.declare_parameter('topic_mesh_cloud_output', '/tsdf/mesh_cloud')
        
        # 读取参数
        self.save_dir = self.get_parameter('save_dir').value
        self.world_frame = self.get_parameter('world_frame').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.output_frame = self.get_parameter('output_frame').value
        self.voxel_length = self.get_parameter('voxel_length').value
        self.sdf_trunc = self.get_parameter('sdf_trunc').value
        self.depth_trunc = self.get_parameter('depth_trunc').value
        self.depth_scale = self.get_parameter('depth_scale').value
        self.frame_skip = self.get_parameter('frame_skip').value
        sync_slop = self.get_parameter('sync_slop').value
        timer_period = self.get_parameter('timer_period').value
        self.background_scale = float(self.get_parameter('background_scale').value)
        self.tf_wait_timeout = float(self.get_parameter('tf_wait_timeout').value)
        self.tf_use_latest_fallback = bool(self.get_parameter('tf_use_latest_fallback').value)
        self.intrinsics_auto_scale = bool(self.get_parameter('intrinsics_auto_scale').value)
        
        topic_rgb_input = self.get_parameter('topic_rgb_input').value
        topic_depth_input = self.get_parameter('topic_depth_input').value
        topic_mask_input = self.get_parameter('topic_mask_input').value
        topic_camera_info = self.get_parameter('topic_camera_info').value
        topic_mesh_cloud_output = self.get_parameter('topic_mesh_cloud_output').value
        # ========================================================
        
        os.makedirs(self.save_dir, exist_ok=True)
        
        # 相机内参
        self.camera_info = None
        self.intrinsic = None
        self.intrinsic_base = None
        self.intrinsic_base_size = None
        self._scaled_intrinsic = None
        self._scaled_size = None
        
        # 帧计数
        self.frame_count = 0
        
        # TF2 buffer 用于时间戳精确的变换查询
        # 离线回放时 TF 往往滞后，适当加大缓存
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # TSDF Volume 配置
        self.volume = o3d.pipelines.integration.ScalableTSDFVolume(
            voxel_length=self.voxel_length,
            sdf_trunc=self.sdf_trunc,
            color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8
        )
        
        self.bridge = CvBridge()
        
        # QoS 配置
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # 订阅相机内参
        self.create_subscription(
            CameraInfo,
            topic_camera_info,
            self.camera_info_callback,
            qos
        )
        
        # 同步订阅 RGB + Depth + 语义 Mask
        self.rgb_sub = message_filters.Subscriber(
            self, Image, topic_rgb_input
        )
        self.depth_sub = message_filters.Subscriber(
            self, Image, topic_depth_input
        )
        self.mask_sub = message_filters.Subscriber(
            self, Image, topic_mask_input
        )
        
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub, self.mask_sub],
            queue_size=10,
            slop=sync_slop
        )
        self.ts.registerCallback(self.synced_callback)
        
        # 发布融合后的点云
        self.mesh_cloud_pub = self.create_publisher(
            PointCloud2, topic_mesh_cloud_output, 10
        )
        
        # 定时发布融合点云
        self.create_timer(timer_period, self.publish_fused_cloud)
        
        self.get_logger().info("=" * 50)
        self.get_logger().info("TSDF 语义融合建图节点已启动 (TF2 + 向量化)")
        self.get_logger().info(f"  体素大小: {self.voxel_length*100:.1f} cm, 截断: {self.sdf_trunc*100:.1f} cm")
        self.get_logger().info(f"  深度截断: {self.depth_trunc} m, 深度缩放: {self.depth_scale}")
        self.get_logger().info(f"  帧跳过: {self.frame_skip}, 同步容差: {sync_slop} s")
        self.get_logger().info(f"  世界帧: {self.world_frame}, 相机帧: {self.camera_frame}")
        self.get_logger().info(f"  保存目录: {self.save_dir}")
        self.get_logger().info("按 Ctrl+C 停止并保存网格")
        self.get_logger().info("=" * 50)
    
    def camera_info_callback(self, msg):
        """接收相机内参"""
        if self.camera_info is None:
            self.camera_info = msg
            # 创建 Open3D 相机内参
            self.intrinsic = o3d.camera.PinholeCameraIntrinsic(
                width=msg.width,
                height=msg.height,
                fx=msg.k[0],
                fy=msg.k[4],
                cx=msg.k[2],
                cy=msg.k[5]
            )
            self.intrinsic_base = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])
            self.intrinsic_base_size = (msg.width, msg.height)
            self.get_logger().info(
                f"相机内参已接收: {msg.width}x{msg.height}, "
                f"fx={msg.k[0]:.1f}, fy={msg.k[4]:.1f}"
            )
    
    def synced_callback(self, rgb_msg, depth_msg, mask_msg):
        """同步处理 RGB + Depth + 语义 Mask，使用 TF2 查询精确位姿"""
        # 调试日志：确认是否收到同步消息
        if self.frame_count % 30 == 0:
            self.get_logger().info(f"收到同步消息: Frame {self.frame_count}")
        if self.intrinsic is None:
            return
        
        self.frame_count += 1
        
        # 每 N 帧融合一次，降低计算量
        if self.frame_count % self.frame_skip != 0:
            return
        
        # 使用图像时间戳查询精确的 TF 变换
        try:
            stamp = Time.from_msg(rgb_msg.header.stamp)
            # 先尝试严格时间同步；若 TF 滞后则退回最新 TF，避免大量丢帧
            if self.tf_buffer.can_transform(
                self.world_frame,
                self.camera_frame,
                stamp,
                timeout=Duration(seconds=self.tf_wait_timeout)
            ):
                transform = self.tf_buffer.lookup_transform(
                    self.world_frame,  # target frame (world/map)
                    self.camera_frame,  # source frame (camera optical)
                    stamp
                )
            else:
                if not self.tf_use_latest_fallback:
                    if self.frame_count % 30 == 0:
                        self.get_logger().warn("TF 滞后，跳过当前帧以保证几何一致性")
                    return
                if self.frame_count % 30 == 0:
                    self.get_logger().warn(
                        "TF 滞后，使用最新可用变换进行融合（可能出现几何拖影）"
                    )
                transform = self.tf_buffer.lookup_transform(
                    self.world_frame,
                    self.camera_frame,
                    Time(),  # time=0 => latest
                    timeout=Duration(seconds=0.2)
                )
        except (tf2_ros.LookupException, 
                tf2_ros.ConnectivityException, 
                tf2_ros.ExtrapolationException) as e:
            if self.frame_count % 30 == 0:
                self.get_logger().warn(f"TF lookup failed: {e}")
            return
        
        try:
            # 构建 4x4 变换矩阵: camera_optical -> odom
            t = transform.transform.translation
            r = transform.transform.rotation
            rotation = R.from_quat([r.x, r.y, r.z, r.w])
            
            camera_to_world = np.eye(4)
            camera_to_world[:3, :3] = rotation.as_matrix()
            camera_to_world[:3, 3] = [t.x, t.y, t.z]
            
            # 转换图像
            rgb = self.bridge.imgmsg_to_cv2(rgb_msg, 'rgb8')
            depth = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough')
            mask = self.bridge.imgmsg_to_cv2(mask_msg, 'mono8')

            # 内参尺寸与图像尺寸不一致时自动缩放
            intrinsic = self.intrinsic
            if self.intrinsics_auto_scale and self.intrinsic_base_size is not None:
                h, w = rgb.shape[:2]
                if (self.intrinsic_base_size[0] != w) or (self.intrinsic_base_size[1] != h):
                    if self._scaled_size != (w, h):
                        fx, fy, cx, cy = self.intrinsic_base
                        base_w, base_h = self.intrinsic_base_size
                        sx = w / float(base_w)
                        sy = h / float(base_h)
                        self._scaled_intrinsic = o3d.camera.PinholeCameraIntrinsic(
                            width=w,
                            height=h,
                            fx=fx * sx,
                            fy=fy * sy,
                            cx=cx * sx,
                            cy=cy * sy
                        )
                        self._scaled_size = (w, h)
                        self.get_logger().warn(
                            f"相机内参与图像尺寸不一致，已自动缩放到 {w}x{h}"
                        )
                    intrinsic = self._scaled_intrinsic
            
            # 将语义颜色混合到 RGB 图像
            semantic_rgb = self.apply_semantic_colors(rgb, mask)
            
            # 转换为 Open3D 格式
            rgb_o3d = o3d.geometry.Image(semantic_rgb.astype(np.uint8))
            # 兼容 16UC1(mm) 和 32FC1(m) 深度编码
            depth_scale = self.depth_scale
            if depth_msg.encoding in ('32FC1', '32FC'):
                depth_o3d = o3d.geometry.Image(depth.astype(np.float32))
                depth_scale = 1.0
                if self.frame_count % 30 == 0:
                    self.get_logger().info("深度编码为 32FC1，按米处理")
            else:
                depth_o3d = o3d.geometry.Image(depth.astype(np.uint16))
            
            rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
                rgb_o3d, depth_o3d,
                depth_scale=depth_scale,
                depth_trunc=self.depth_trunc,
                convert_rgb_to_intensity=False
            )
            
            # TSDF 融合需要相机外参的逆矩阵
            # extrinsic = world -> camera, 所以取 camera_to_world 的逆
            extrinsic = np.linalg.inv(camera_to_world)
            
            # 融合到 TSDF Volume
            self.volume.integrate(
                rgbd,
                intrinsic,
                extrinsic
            )
            
            if self.frame_count % 30 == 0:
                self.get_logger().info(f"已融合 {self.frame_count // 2} 帧")
                
        except Exception as e:
            self.get_logger().error(f"融合错误: {e}")
            import traceback
            traceback.print_exc()
    
    def apply_semantic_colors(self, rgb, mask):
        """
        将语义颜色应用到 RGB 图像
        改进策略 (Log-Odds 模拟):
        1. 前景: 使用 100% 纯语义颜色。保证 TSDF 积分时 '投票' 鲜明，抗噪性更强。
        2. 背景: 适当压暗原始 RGB，突出语义颜色。
        """
        # 初始化结果为原图(背景)并压暗，突出语义颜色
        scale = max(0.0, min(1.0, float(self.background_scale)))
        result = (rgb.astype(np.float32) * scale)
        
        # 叠加前景
        for class_id, color in COCO_COLORS.items():
            # mask 值 = class_id + 1 (0 是背景)
            mask_region = (mask == class_id + 1)
            if np.any(mask_region):
                # 100% 语义颜色 (Winner-Takes-All 策略)
                semantic_color = np.array(color)
                result[mask_region] = semantic_color
        
        return result.astype(np.uint8)
    
    def publish_fused_cloud(self):
        """发布从 TSDF 提取的融合点云"""
        try:
            # 从 TSDF 提取点云
            pcd = self.volume.extract_point_cloud()
            
            if len(pcd.points) == 0:
                return
            
            # 转换为 NumPy 数组
            points = np.asarray(pcd.points, dtype=np.float32)
            colors = (np.asarray(pcd.colors) * 255).astype(np.uint8)
            
            n_points = len(points)
            self.get_logger().info(f"发布点云: {n_points} 个点")
            
            # 向量化 RGB 打包: 将 (R, G, B) 打包成 uint32，再视为 float32
            rgb_uint32 = (colors[:, 0].astype(np.uint32) << 16 |
                          colors[:, 1].astype(np.uint32) << 8 |
                          colors[:, 2].astype(np.uint32))
            rgb_float = rgb_uint32.view(np.float32)
            
            # 创建结构化数组
            cloud_array = np.zeros(n_points, dtype=[
                ('x', np.float32),
                ('y', np.float32),
                ('z', np.float32),
                ('rgb', np.float32),
            ])
            cloud_array['x'] = points[:, 0]
            cloud_array['y'] = points[:, 1]
            cloud_array['z'] = points[:, 2]
            cloud_array['rgb'] = rgb_float
            
            # 构建 PointCloud2 消息
            msg = PointCloud2()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.output_frame
            msg.height = 1
            msg.width = n_points
            msg.is_dense = True
            msg.is_bigendian = False
            
            msg.fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
            ]
            msg.point_step = 16
            msg.row_step = msg.point_step * n_points
            msg.data = cloud_array.tobytes()
            
            self.mesh_cloud_pub.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f"提取点云错误: {e}")
    
    def save_mesh(self):
        """保存融合后的网格和点云"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        try:
            # 提取网格
            self.get_logger().info("正在提取网格...")
            mesh = self.volume.extract_triangle_mesh()
            mesh.compute_vertex_normals()
            
            mesh_file = os.path.join(self.save_dir, f"semantic_mesh_{timestamp}.ply")
            o3d.io.write_triangle_mesh(mesh_file, mesh)
            self.get_logger().info(f"✅ 网格已保存: {mesh_file}")
            self.get_logger().info(f"   顶点数: {len(mesh.vertices)}")
            self.get_logger().info(f"   三角形数: {len(mesh.triangles)}")
            
            # 提取点云
            pcd = self.volume.extract_point_cloud()
            pcd_file = os.path.join(self.save_dir, f"semantic_cloud_{timestamp}.ply")
            o3d.io.write_point_cloud(pcd_file, pcd)
            self.get_logger().info(f"✅ 点云已保存: {pcd_file}")
            self.get_logger().info(f"   点数: {len(pcd.points)}")
            
        except Exception as e:
            self.get_logger().error(f"保存错误: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = TSDFFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("正在保存融合结果...")
        node.save_mesh()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
