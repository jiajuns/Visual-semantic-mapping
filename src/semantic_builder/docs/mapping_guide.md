# 语义建图方案指南

本文档提供三种建图方案，根据你的硬件性能和使用场景选择最合适的方案。

---

## 方案概览

| 方案 | 适用场景 | 硬件要求 | 优点 | 缺点 |
|------|----------|----------|------|------|
| **方案1: 在线建图** | 实时查看建图效果 | GPU + 较强 CPU | 实时反馈，简单直接 | 运行卡顿可能影响地图质量 |
| **方案2: 纯离线建图** | GPU 弱 / 运行卡顿 | 普通 CPU | 建图质量稳定，可重复处理 | 需要两步操作，无实时反馈 |
| **方案3: 在线+离线混合** | 兼顾实时效果和质量 | 中等配置 | 先快速预览，后精细处理 | 需要更多存储空间 |

---

## 方案1: 在线建图（实时）

### 适用场景
- 有 NVIDIA GPU，且系统运行流畅
- 需要实时查看建图效果
- 快速验证环境

### 操作步骤

```bash
# 1. 启动建图系统
source ~/camera_ws/install/setup.bash
ros2 launch semantic_builder semantic_mapping.launch.py

# 2. 缓慢移动相机，扫描环境

# 3. 按 Ctrl+C 停止，地图自动保存
```

### 优化建议（如果卡顿）

| 症状 | 解决方案 |
|------|----------|
| YOLO 推理慢 | 降低分辨率：修改 launch 中 `rgb_camera.profile` 为 `424x240x15` |
| 点云显示卡 | 关闭 RViz，或降低点云密度：`Grid/CellSize: 0.05` |
| 内存占用高 | 减少 STM：`Mem/STMSize: 15` |
| 整体卡顿 | 跳帧处理：修改 yolo_node.py 中 `skip_yolo = (self.frame_count % 3 != 0)` 为 `% 5` |

### 输出文件
```
src/semantic_builder/map/
├── rtabmap.db        # 地图数据库
└── rtabmap_cloud.ply # 点云文件
```

---

## 方案2: 纯离线建图（推荐用于弱机）

### 适用场景
- 无 GPU 或 GPU 性能较弱
- 系统运行卡顿，影响建图质量
- 需要多次尝试不同参数

### 操作步骤

#### 第一步：录制数据

```bash
# 1. 只启动相机
ros2 launch realsense2_camera rs_launch.py \
    align_depth.enable:=true \
    enable_sync:=true \
    rgb_camera.profile:=640x480x15 \
    depth_module.profile:=640x480x15

# 新终端：录制数据
mkdir -p src/semantic_builder/map/bags
ros2 bag record -o src/semantic_builder/map/bags/scan_$(date +%Y%m%d_%H%M%S) \
    /camera/camera/color/image_raw \
    /camera/camera/aligned_depth_to_color/image_raw \
    /camera/camera/color/camera_info
```

#### 第二步：离线建图

```bash
# 1. 回放 bag 并启动建图（先启动 YOLO 和 RTAB-Map）
source ~/camera_ws/install/setup.bash

# 2. 在一个终端启动 YOLO
ros2 run semantic_builder yolo_node

# 3. 在另一个终端启动 RTAB-Map
ros2 launch rtabmap_launch rtabmap.launch.py \
    rgb_topic:=/camera/camera/color/image_raw \
    depth_topic:=/camera/camera/aligned_depth_to_color/image_raw \
    camera_info_topic:=/camera/camera/color/camera_info \
    database_path:=src/semantic_builder/map/rtabmap.db \
    qos:=0

# 4. 在第三个终端回放 bag（可加 --rate 0.5 降速回放）
ros2 bag play src/semantic_builder/map/bags/scan_xxx.db3 --rate 1.0
```

### 输出文件
同方案1

---

## 方案3: 在线+离线混合（推荐）

### 适用场景
- 想实时预览效果，但系统有点卡
- 需要保证最终地图质量
- 有足够存储空间

### 操作步骤

#### 第一步：在线录制 + 实时预览

```bash
# 1. 启动系统并录制
source ~/camera_ws/install/setup.bash

# 2. 启动相机 + 录制
ros2 launch realsense2_camera rs-launch.py \
    align_depth.enable:=true \
    rgb_camera.profile:=640x480x15

# 3. 新终端：开始录制
ros2 bag record -o src/semantic_builder/map/bags/scan_$(date +%Y%m%d_%H%M%S) \
    /camera/camera/color/image_raw \
    /camera/camera/aligned_depth_to_color/image_raw \
    /camera/camera/color/camera_info

# 4. 新终端：启动 YOLO（降频模式，减少卡顿）
# 先修改 yolo_node.py: skip_yolo = (self.frame_count % 5 != 0)
ros2 run semantic_builder yolo_node

# 5. 新终端：启动 RTAB-Map（较低配置）
ros2 launch rtabmap_launch rtabmap.launch.py \
    rgb_topic:=/yolo/overlay_image \
    depth_topic:=/camera/camera/aligned_depth_to_color/image_raw \
    database_path:=src/semantic_builder/map/rtabmap.db \
    Grid/CellSize:=0.05 \
    Mem/STMSize:=15

# 6. 扫描环境（此时可以实时看到效果）
```

#### 第二步：离线精细处理

```bash
# 扫描完成后，停止所有节点，使用录制的 bag 重新建图
# 使用更高质量的参数重新处理

ros2 launch rtabmap_launch rtabmap.launch.py \
    rgb_topic:=/camera/camera/color/image_raw \
    depth_topic:=/camera/camera/aligned_depth_to_color/image_raw \
    database_path:=src/semantic_builder/map/rtabmap_final.db \
    rtabmap_args:="--delete_db_on_start" \
    Grid/CellSize:=0.02 \
    Grid/RangeMax:=4.0 \
    Reg/Strategy:=1

# 另一个终端：回放 bag（慢速，保证质量）
ros2 bag play src/semantic_builder/map/bags/scan_xxx.db3 --rate 0.5
```

### 输出文件
```
src/semantic_builder/map/
├── bags/                    # 录制的原始数据
│   └── scan_xxx.db3
├── rtabmap_preview.db       # 在线预览地图（低质量）
└── rtabmap_final.db         # 最终地图（高质量）
```

---

## 性能优化建议

### GPU 内存不足

```yaml
# yolo_node.py 修改
self.use_half = False        # 禁用半精度
conf=0.25 → conf=0.35        # 提高置信度阈值，减少检测
```

### CPU 负载过高

```yaml
# 降低帧率
rgb_camera.profile: 424x240x15  # 或更低

# 增加跳帧
skip_yolo = (self.frame_count % 5 != 0)  # 每5帧处理1次
```

### 磁盘空间不足

```bash
# 录制完成后只保留 bag，删除中间文件
rm src/semantic_builder/map/rtabmap_preview.db
```

---

## 故障排查

| 问题 | 可能原因 | 解决方案 |
|------|----------|----------|
| 相机连接失败 | USB 带宽不足 | 使用 USB 3.0 接口，独占使用 |
| YOLO 启动很慢 | 首次下载模型 | 等待下载完成，或手动下载 yolov8s-seg.pt |
| RTAB-Map 丢帧 | 处理速度 < 录制速度 | 使用 `--rate 0.5` 降速回放 |
| 地图拼接错位 | 移动过快或纹理单一 | 缓慢移动，确保场景有足够纹理 |
| 点云有噪点 | 深度图质量差 | 增加 `Grid/NoiseFilteringRadius` |

---

## 导出 glTF

建图完成后，导出为 glTF 格式：

```bash
# 方法1: 使用项目脚本（推荐）
python3 src/semantic_builder/semantic_builder/ply2gltf.py \
    src/semantic_builder/map/rtabmap_cloud.ply

# 方法2: 先导出点云，再转换
rtabmap-databaseViewer src/semantic_builder/map/rtabmap.db
# File → Export 3D Cloud → PLY

# 然后转换
python3 src/semantic_builder/semantic_builder/ply2gltf.py output.ply
```

---

## 方案选择决策树

```
是否有 GPU？
├── 否 → 方案2（纯离线）
└── 是
    ├── 运行流畅吗？
    │   ├── 是 → 方案1（在线）
    │   └── 否
    │       ├── 只需要结果 → 方案2（纯离线）
    │       └── 需要预览 → 方案3（混合）
```
