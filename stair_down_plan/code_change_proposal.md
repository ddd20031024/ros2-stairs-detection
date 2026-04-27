# 代码改动建议清单（对应当前工程）

## A. [src/yolo_lidar_fusion/yolo_lidar_fusion/fusion_node.py](src/yolo_lidar_fusion/yolo_lidar_fusion/fusion_node.py)

### A1. 参数扩展

建议在初始化参数中新增：

- `angle_window_deg` (float, default 8.0)
- `range_statistic` (string, default "median")
- `stability_frames` (int, default 3)
- `enable_risk_output` (bool, default true)

### A2. 关键逻辑替换

把现有“单索引测距”替换为：

1. 计算中心角 `center_angle`
2. 在 `[center_angle - half_window, center_angle + half_window]` 采样多条激光
3. 有效值做 `median/p30` 得到 `robust_dist`
4. 连续 `stability_frames` 帧一致后确认有效

### A3. 新增发布器

可新增发布器：

- `/stair/edge_distance` (`std_msgs/Float32`)
- `/stair/risk` (`std_msgs/Float32`)

风险分数示例：

- `dist > 1.2m` -> `risk=0.1`
- `0.6m < dist <= 1.2m` -> `risk=0.5`
- `dist <= 0.6m` -> `risk=0.9`

## B. [src/yolo_lidar_fusion/launch/fusion.launch.py](src/yolo_lidar_fusion/launch/fusion.launch.py)

### B1. 参数新增

新增声明：

- `scan_down_topic`，默认 `/scan_down`
- `enable_stair_safety`，默认 `true`

### B2. 节点拓展

在 launch 中新增节点：

- `stair_safety_node`（新建Python节点）

该节点参数至少包含：

- `scan_topic`
- `scan_down_topic`
- `risk_stop_threshold`
- `risk_slow_threshold`

## C. [src/yolo_lidar_fusion/params/fusion.params.yaml](src/yolo_lidar_fusion/params/fusion.params.yaml)

建议补充：

```yaml
fusion_node:
  ros__parameters:
    angle_window_deg: 8.0
    range_statistic: median
    stability_frames: 3
    enable_risk_output: true
```

## D. 新增文件建议

1. `src/yolo_lidar_fusion/yolo_lidar_fusion/stair_safety_node.py`

- 职责：融合视觉结果 + 双雷达趋势，输出风险分级。

2. `src/yolo_lidar_fusion/params/stair_safety.params.yaml`

- 职责：固化阈值，便于现场调参。

3. `src/yolo_lidar_fusion/README.md` 增补章节

- 说明双雷达接线、安装角度、调参方法、验收命令。

## E. 推荐实施顺序

1. 先改 `fusion_node` 的多射线鲁棒测距（最小收益最大）。
2. 再加 `stair_safety_node` 风险状态机。
3. 最后接入下倾雷达并标定，做实车回归。

## F. 快速验证命令（改造后）

```bash
ros2 topic echo /stair/risk
ros2 topic echo /stair/edge_distance
ros2 topic hz /scan
ros2 topic hz /scan_down
```
