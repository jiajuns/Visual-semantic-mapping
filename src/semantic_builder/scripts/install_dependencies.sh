#!/bin/bash
# Semantic Builder 依赖安装脚本 (无 RealSense 版)
# 适用于 Ubuntu 22.04 + ROS2 Humble
# realsense要额外从官方库下载
set -e  # 遇到错误立即退出

# 颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}======================================${NC}"
echo -e "${GREEN}  Semantic Builder 依赖安装脚本 (无 RS)${NC}"
echo -e "${GREEN}======================================${NC}"
echo ""

# 检测 Ubuntu 版本
if [ ! -f /etc/os-release ]; then
    echo -e "${RED}错误: 无法检测操作系统版本${NC}"
    exit 1
fi

. /etc/os-release
if [ "$UBUNTU_CODENAME" != "jammy" ]; then
    echo -e "${YELLOW}警告: 此脚本适用于 Ubuntu 22.04 (jammy)${NC}"
    echo -e "${YELLOW}当前系统: $PRETTY_NAME${NC}"
    read -p "是否继续? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# 检查是否已安装 ROS2
echo -e "${GREEN}[1/6] 检查 ROS2 安装...${NC}"
if [ ! -f /opt/ros/humble/setup.bash ]; then
    echo -e "${RED}错误: 未检测到 ROS2 Humble${NC}"
    echo -e "${YELLOW}请先安装 ROS2 Humble:${NC}"
    echo "  sudo apt update"
    echo "  sudo apt install ros-humble-desktop python3-rosdep -y"
    exit 1
else
    echo -e "${GREEN}✓ ROS2 Humble 已安装${NC}"
fi

# Source ROS2 环境
source /opt/ros/humble/setup.bash

# 安装 ROS2 依赖包
echo -e "${GREEN}[2/6] 安装 ROS2 依赖包...${NC}"
# 注意：已移除 realsense2 相关包
sudo apt update
sudo apt install -y \
    ros-humble-rtabmap-ros \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-image-transport-plugins \
    ros-humble-tf2-ros \
    ros-humble-tf2-sensor-msgs \
    ros-humble-tf2-geometry-msgs \
    ros-humble-rviz2 \
    ros-humble-rviz-common \
    ros-humble-rviz-default-plugins \
    python3-opencv \
    python3-pip

echo -e "${GREEN}✓ ROS2 依赖包安装完成${NC}"

# 步骤 3 原为安装 RealSense 驱动，已跳过

# 安装 Python 依赖
echo -e "${GREEN}[3/6] 安装 Python 依赖...${NC}"
pip3 install --upgrade pip

# 核心 Python 库
pip3 install --user \
    ultralytics \
    opencv-python \
    numpy \
    scipy \
    open3d \
    torch \
    torchvision

echo -e "${GREEN}✓ Python 依赖安装完成${NC}"

# 验证安装
echo -e "${GREEN}[4/6] 验证安装...${NC}"

# 检查 Python 库
echo "检查 Python 库..."
python3 -c "import torch; print('  PyTorch:', torch.__version__)" || echo -e "${RED}  ✗ PyTorch 安装失败${NC}"
python3 -c "import cv2; print('  OpenCV:', cv2.__version__)" || echo -e "${RED}  ✗ OpenCV 安装失败${NC}"
python3 -c "import open3d; print('  Open3D:', open3d.__version__)" || echo -e "${RED}  ✗ Open3D 安装失败${NC}"
python3 -c "from ultralytics import YOLO; print('  YOLO: OK')" || echo -e "${RED}  ✗ YOLO 安装失败${NC}"

# 检查 CUDA (可选)
echo ""
python3 -c "import torch; print('  CUDA 可用:', torch.cuda.is_available())" || true

# 编译工作空间
echo -e "${GREEN}[5/6] 编译工作空间...${NC}"
# 获取当前脚本所在目录的上三级目录 (假设脚本在 src/semantic_builder/scripts 下)
# 如果脚本位置不同，请确保这里能正确找到 workspace 根目录
WORKSPACE_DIR="$(dirname "$(dirname "$(dirname "$0")")")"

# 如果上面路径不对，这里做一个简单的检查机制：
if [ ! -f "$WORKSPACE_DIR/src" ]; then
    # 尝试回退到当前目录
    WORKSPACE_DIR=$(pwd)
fi

echo "工作空间: $WORKSPACE_DIR"

cd "$WORKSPACE_DIR"
# 使用 colcon 编译
colcon build --packages-select semantic_builder --symlink-install

echo -e "${GREEN}✓ 编译完成${NC}"

# 完成
echo -e "${GREEN}[6/6] 安装完成！${NC}"
echo ""
echo -e "${GREEN}======================================${NC}"
echo -e "${GREEN}  安装成功 (无 RealSense)${NC}"
echo -e "${GREEN}======================================${NC}"
echo ""
echo -e "请运行以下命令启动建图:"
echo -e "  ${YELLOW}cd $WORKSPACE_DIR${NC}"
echo -e "  ${YELLOW}source install/setup.bash${NC}"
echo -e "  ${YELLOW}ros2 launch semantic_builder semantic_mapping.launch.py${NC}"
echo ""