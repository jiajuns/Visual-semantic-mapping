#!/usr/bin/env python3
"""
语义分割节点 - 使用 YOLOv8-Seg 生成语义点云

发布话题：
  /yolo/overlay_image - 带分割可视化的 RGB 图像
  /yolo/semantic_mask - 语义分割 mask（每个像素值 = 类别ID）
  /yolo/semantic_cloud - 语义点云（每个点带 RGB 颜色表示类别）
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField, CameraInfo
from cv_bridge import CvBridge
from ultralytics import YOLO
import torch
import cv2
import numpy as np
import colorsys
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import message_filters
import struct

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
for i, class_name in enumerate(COCO_CLASSES):
    hue = i / len(COCO_CLASSES)
    # HSV: H=均匀分布, S=0.85(高饱和), V=0.9(高亮度)
    rgb = colorsys.hsv_to_rgb(hue, 0.85, 0.9)
    COCO_COLORS[i] = tuple(int(c * 255) for c in rgb)

DEFAULT_COLOR = (200, 200, 200)  # 背景点使用原始 RGB 颜色


class SemanticSegmentationNode(Node):
    def __init__(self):
        super().__init__('semantic_seg_node')
        
        # ========== 从 ROS 参数获取配置 (由 launch 传入) ==========
        # 模型配置
        self.declare_parameter('model_name', 'yolov8s-seg.pt')
        self.declare_parameter('confidence_threshold', 0.25)
        self.declare_parameter('frame_skip', 3)
        self.declare_parameter('qos_depth', 10)
        self.declare_parameter('mask_erosion_kernel_size', 5)
        
        # 话题配置
        self.declare_parameter('topic_rgb_input', '/camera/color/image_raw')
        self.declare_parameter('topic_depth_input', '/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('topic_camera_info', '/camera/color/camera_info')
        self.declare_parameter('topic_overlay_output', '/yolo/overlay_image')
        self.declare_parameter('topic_mask_output', '/yolo/semantic_mask')
        self.declare_parameter('topic_cloud_output', '/yolo/semantic_cloud')
        
        # 读取参数
        self.model_name = self.get_parameter('model_name').value
        self.conf_threshold = self.get_parameter('confidence_threshold').value
        self.frame_skip = self.get_parameter('frame_skip').value
        qos_depth = self.get_parameter('qos_depth').value
        self.erosion_kernel_size = self.get_parameter('mask_erosion_kernel_size').value
        
        topic_rgb_input = self.get_parameter('topic_rgb_input').value
        topic_depth_input = self.get_parameter('topic_depth_input').value
        topic_camera_info = self.get_parameter('topic_camera_info').value
        topic_overlay_output = self.get_parameter('topic_overlay_output').value
        topic_mask_output = self.get_parameter('topic_mask_output').value
        topic_cloud_output = self.get_parameter('topic_cloud_output').value
        # ========================================================
        
        # 设备配置
        self.device = '0' if torch.cuda.is_available() else 'cpu'
        self.use_half = self.device == '0'
        
        # 加载模型
        self.get_logger().info(f"语义分割节点启动中... GPU: {self.device}")
        self.get_logger().info(f"正在加载分割模型 {self.model_name}...")
        import time
        start = time.time()
        self.model = YOLO(self.model_name)
        self.get_logger().info(f"模型加载完成，耗时: {time.time()-start:.1f}秒")
        
        self.bridge = CvBridge()
        self.frame_count = 0
        self.camera_info = None
        
        # QoS 配置
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=qos_depth
        )
        
        # 订阅 camera_info 获取相机内参
        self.create_subscription(
            CameraInfo,
            topic_camera_info,
            self.camera_info_callback,
            qos
        )
        
        # 使用 message_filters 同步 RGB 和 Depth，使用 RELIABLE QoS 与相机匹配
        self.rgb_sub = message_filters.Subscriber(
            self, Image, topic_rgb_input, qos_profile=qos
        )
        self.depth_sub = message_filters.Subscriber(
            self, Image, topic_depth_input, qos_profile=qos
        )
        
        # 近似时间同步
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub],
            queue_size=qos_depth,
            slop=0.1
        )
        self.ts.registerCallback(self.synced_callback)
        
        # 发布器 - 使用 RELIABLE QoS 与 RTAB-Map 兼容
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=qos_depth
        )
        self.overlay_pub = self.create_publisher(Image, topic_overlay_output, pub_qos)
        self.mask_pub = self.create_publisher(Image, topic_mask_output, pub_qos)
        self.cloud_pub = self.create_publisher(PointCloud2, topic_cloud_output, pub_qos)
        
        self.get_logger().info("=" * 50)
        self.get_logger().info("语义分割节点已就绪")
        self.get_logger().info(f"  模型: {self.model_name}, 置信度: {self.conf_threshold}")
        self.get_logger().info(f"  帧跳过: {self.frame_skip}, 腐蚀核: {self.erosion_kernel_size}")
        self.get_logger().info(f"  输入: {topic_rgb_input}")
        self.get_logger().info(f"  输出: {topic_overlay_output}, {topic_mask_output}")
        self.get_logger().info("=" * 50)
    
    def camera_info_callback(self, msg):
        if self.camera_info is None:
            self.camera_info = msg
            self.get_logger().info(f"相机内参已接收: fx={msg.k[0]:.1f}, fy={msg.k[4]:.1f}")
    
    def synced_callback(self, rgb_msg, depth_msg):
        """同步处理 RGB 和深度图像"""
        try:
            self.frame_count += 1
            
            # 转换图像 (始终需要，用于发布 mask)
            rgb_img = self.bridge.imgmsg_to_cv2(rgb_msg, 'bgr8')
            depth_img = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough')
            h, w = rgb_img.shape[:2]
            
            # 帧率限制：每 N 帧处理一次 YOLO，降低 GPU 负载
            skip_yolo = (self.frame_count % self.frame_skip != 0)
            
            if self.frame_count % 30 == 0:
                self.get_logger().info(f"已处理 {self.frame_count} 帧 (YOLO: {self.frame_count // self.frame_skip})")
            
            # 初始化空的语义 mask
            semantic_mask = np.zeros((h, w), dtype=np.uint8)
            
            # 根据帧率限制决定是否运行 YOLO
            if skip_yolo:
                # 跳帧：发布原始图像和空 mask
                overlay_msg = self.bridge.cv2_to_imgmsg(rgb_img, 'bgr8')
                overlay_msg.header = rgb_msg.header
                self.overlay_pub.publish(overlay_msg)
            else:
                # 关键帧：运行 YOLO 分割
                results = self.model.predict(
                    rgb_img,
                    device=self.device,
                    half=self.use_half,
                    conf=self.conf_threshold,
                    verbose=False
                )
                
                result = results[0]
                
                # 1. 发布带标注的图像
                annotated = result.plot()
                overlay_msg = self.bridge.cv2_to_imgmsg(annotated, 'bgr8')
                overlay_msg.header = rgb_msg.header
                self.overlay_pub.publish(overlay_msg)
                
                # 2. 生成语义 mask
                if result.masks is not None:
                    for i, (mask, box) in enumerate(zip(result.masks.data, result.boxes)):
                        class_id = int(box.cls[0])
                        # 将 mask 缩放到原图尺寸
                        mask_resized = cv2.resize(
                            mask.cpu().numpy().astype(np.uint8),
                            (w, h),
                            interpolation=cv2.INTER_NEAREST
                        )
                        
                        # === 腐蚀操作 (Shrink mask to avoid boundary bleeding) ===
                        kernel = np.ones((self.erosion_kernel_size, self.erosion_kernel_size), np.uint8)
                        mask_resized = cv2.erode(mask_resized, kernel, iterations=1)
                        # ========================================================
                        
                        # 写入语义 mask (类别 ID + 1, 0 保留给背景)
                        semantic_mask[mask_resized > 0] = class_id + 1
            
            # 发布语义 mask (始终发布，保证 TSDF 节点同步)
            mask_msg = self.bridge.cv2_to_imgmsg(semantic_mask, 'mono8')
            mask_msg.header = rgb_msg.header
            self.mask_pub.publish(mask_msg)
            
            # 3. 生成语义点云 (禁用以提高性能，建图不需要这个实时点云)
            # if not skip_yolo and self.camera_info is not None:
            #     cloud_msg = self.create_semantic_cloud(
            #         rgb_img, depth_img, semantic_mask, rgb_msg.header
            #     )
            #     if cloud_msg is not None:
            #         self.cloud_pub.publish(cloud_msg)
            
        except Exception as e:
            self.get_logger().error(f"处理错误: {e}")
            import traceback
            traceback.print_exc()
    
    def create_semantic_cloud(self, rgb, depth, semantic_mask, header):
        """从 RGB + Depth + 语义 Mask 创建语义点云 (向量化版本，50-100x 性能提升)"""
        if self.camera_info is None:
            return None
        
        # 相机内参
        fx = self.camera_info.k[0]
        fy = self.camera_info.k[4]
        cx = self.camera_info.k[2]
        cy = self.camera_info.k[5]
        
        h, w = depth.shape
        
        # 降采样以提高性能 (每隔4个像素取一个点)
        step = 4
        
        # 向量化：创建像素坐标网格
        v_coords, u_coords = np.meshgrid(
            np.arange(0, h, step), 
            np.arange(0, w, step), 
            indexing='ij'
        )
        v_flat = v_coords.flatten()
        u_flat = u_coords.flatten()
        
        # 批量获取深度值
        z = depth[v_flat, u_flat].astype(np.float32) / 1000.0  # mm -> m
        
        # 过滤无效深度
        valid = (z > 0.1) & (z < 10.0)
        if not np.any(valid):
            return None
        
        z = z[valid]
        u_valid = u_flat[valid]
        v_valid = v_flat[valid]
        
        # 批量反投影到 3D
        x = (u_valid - cx) * z / fx
        y = (v_valid - cy) * z / fy
        
        # 批量获取语义类别
        class_ids = semantic_mask[v_valid, u_valid]
        
        # 批量获取颜色
        n_points = len(z)
        colors = np.zeros((n_points, 3), dtype=np.uint8)
        
        # 有语义标签的点使用类别颜色
        for class_id, color in COCO_COLORS.items():
            mask = (class_ids == class_id + 1)
            if np.any(mask):
                colors[mask] = color
        
        # 背景点使用原始 RGB 颜色
        bg_mask = (class_ids == 0)
        if np.any(bg_mask):
            # rgb 是 BGR 格式
            colors[bg_mask, 0] = rgb[v_valid[bg_mask], u_valid[bg_mask], 2]  # R
            colors[bg_mask, 1] = rgb[v_valid[bg_mask], u_valid[bg_mask], 1]  # G
            colors[bg_mask, 2] = rgb[v_valid[bg_mask], u_valid[bg_mask], 0]  # B
        
        # 未匹配的语义点使用默认颜色
        unmatched = (class_ids > 0) & (colors.sum(axis=1) == 0)
        if np.any(unmatched):
            colors[unmatched] = DEFAULT_COLOR
        
        # 向量化 RGB 打包
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
        cloud_array['x'] = x.astype(np.float32)
        cloud_array['y'] = y.astype(np.float32)
        cloud_array['z'] = z.astype(np.float32)
        cloud_array['rgb'] = rgb_float
        
        # 构建 PointCloud2 消息
        msg = PointCloud2()
        msg.header = header
        msg.header.frame_id = 'camera_color_optical_frame'
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
        
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = SemanticSegmentationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("节点关闭中...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()