# ros2-stairs-detection

ROS2 楼梯检测与相机+LiDAR融合测距项目。

## 1. 项目说明

本项目基于 ROS2 Humble，使用 USB 工业相机与 2D LiDAR，结合 YOLO 模型实现楼梯相关目标检测与距离估计。

当前核心包为 [src/yolo_lidar_fusion](src/yolo_lidar_fusion)。

## 2. 环境要求

- Ubuntu 22.04
- ROS2 Humble
- Python 3.10
- 可用摄像头设备（示例为 `/dev/my_camera0`）
- LiDAR 驱动包：`ldlidar_stl_ros2`

## 3. 工作区结构

- [src](src)：源码目录
- [src/yolo_lidar_fusion](src/yolo_lidar_fusion)：融合功能包
- `build/ install/ log/`：colcon 构建产物（已建议忽略）

## 4. 构建

在工作区根目录执行：

```bash
cd /home/xpy/stair_chapter2
colcon build --packages-select yolo_lidar_fusion
source install/setup.bash
```

## 5. 启动

```bash
cd /home/xpy/stair_chapter2
source install/setup.bash
ros2 launch yolo_lidar_fusion fusion.launch.py
```

## 6. 常用启动参数

可在 launch 时覆盖参数，例如：

```bash
ros2 launch yolo_lidar_fusion fusion.launch.py \
	model_file:=best_seg.pt \
	conf_threshold:=0.6 \
	show_debug_window:=false
```

主要参数文件在 [src/yolo_lidar_fusion/params/fusion.params.yaml](src/yolo_lidar_fusion/params/fusion.params.yaml)。

## 7. 常见排查

1. 看不到图像
- 检查摄像头设备路径是否正确。
- 检查 launch 中 `v4l2_camera` 配置。

2. 没有雷达数据
- 检查 LiDAR 连接与权限。
- 用 `ros2 topic echo /scan` 确认是否有数据。

3. 模型加载失败
- 检查模型文件是否存在于 [src/yolo_lidar_fusion/model](src/yolo_lidar_fusion/model)。
- 检查参数 `model_file` 是否与实际文件名一致。

## 8. 常用命令

```bash
# 查看当前话题
ros2 topic list

# 查看融合节点参数
ros2 param list /fusion_node

# 查看扫描频率
ros2 topic hz /scan
```

## 9. Git 使用

Git 常用指令见 [git_about.md](git_about.md)。
