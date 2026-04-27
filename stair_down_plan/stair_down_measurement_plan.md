
## 1. 现状诊断（基于当前代码）

当前工程中的融合逻辑主要在 [src/yolo_lidar_fusion/yolo_lidar_fusion/fusion_node.py](src/yolo_lidar_fusion/yolo_lidar_fusion/fusion_node.py) ：

- YOLO检测后只取最高置信度框中心点 `x_center`。
- 用简化的线性FOV映射把 `x_center` 转成一个角度，再去 `LaserScan` 里取单个索引距离。
- 输出的是“目标中心方向的一条射线距离”。

该方案在上楼时通常可用，但在下楼时存在天然短板：

- 下楼关键风险不在“前方障碍物距离”，而在“前方地面高度突变（台阶坠落边）”。
- 单线测距对空洞/边缘不稳定，容易读到远处背景或无效值（inf）。
- 相机与雷达外参未显式标定，FOV线性映射在楼梯边缘场景误差较大。
- 目前没有“上楼/下楼状态机”和“连续帧置信判定”，缺少动作约束与保护策略。

## 2. 总体目标

在不引入3D传感器的前提下，完成“可下楼、可测距、可触发减速/停车”的稳定能力。

输出指标建议：

- 下楼触发检测延迟 < 200 ms。
- 一级台阶边缘距离误差 <= 8~12 cm（取决于标定质量）。
- 连续10次下楼场景触发成功率 >= 95%。
- 误触发（平地误判为下楼） <= 3%。

## 3. 硬件改造建议（仅2D雷达）

### 方案A（推荐，成本与效果平衡）

新增一台前向下倾斜安装的2D雷达（与现有LD雷达互补）：

- 现有LD雷达：保留水平安装，用于障碍物与环境轮廓。
- 新增2D雷达：安装在机器人前部，俯角约 20°~35°，主要扫地面与台阶边缘。

建议参数：

- 扫描频率 >= 10 Hz（最好 15 Hz+）。
- 角分辨率 <= 0.5°。
- 近距能力好（0.05~0.1 m 起测更佳）。
- 抗室内光照干扰能力稳定。

可选设备方向（按预算分层，不限定品牌唯一）：

- 入门：YDLIDAR X4/X4 Pro 类（先验证算法可行性）。
- 中档：RPLIDAR A2/A3 类（精度和稳定性更均衡）。
- 工业：Hokuyo UST 系列（预算充足、可靠性优先）。

### 方案B（仅软件增强，不新增雷达）

继续使用单雷达 + 相机，仅通过算法增强。

- 优点：成本最低。
- 缺点：下楼边缘可靠性上限低，尤其对反光地砖/黑色台阶/纹理弱场景。

结论：如果项目目标是可交付、可复现，建议走方案A。

## 4. 软件架构改造（建议）

## 4.1 新增节点与话题

建议保留当前 `fusion_node`，再增加一个面向下楼的风险评估节点：

- 新节点：`stair_safety_node`
- 输入：
  - 相机图像（可复用YOLO结果或独立订阅检测结果）
  - 水平雷达 `/scan`
  - 下倾雷达 `/scan_down`（新增）
- 输出：
  - `/stair/risk`（`std_msgs/Float32`，0~1）
  - `/stair/edge_distance`（`std_msgs/Float32`，单位m）
  - `/stair/safe_velocity_scale`（`std_msgs/Float32`，给底盘速度限幅）

## 4.2 关键算法链路

1. 视觉ROI约束

- YOLO识别楼梯区域（建议区分 `stairs_up` 与 `stairs_down` 两类）。
- 将检测框底边附近作为重点ROI（下楼边缘通常在图像中下部）。

2. 雷达多射线鲁棒测距（替换单点测距）

- 不再只取1个角度，而是在目标中心左右扩展一个角窗口（如 ±4°）。
- 在窗口内采集多条距离，去除 `nan/inf` 后取分位值（如P30/P50）。
- 增加连续帧平滑（中值滤波或一阶低通）。

3. 坠落边判定（针对下倾雷达）

- 比较“近端地面距离趋势”和“远端突增/丢失”特征。
- 满足连续N帧（建议N=3~5）才触发 `down_stair_detected`。

4. 控制策略

- 风险分级：`safe` / `slow_down` / `stop`。
- 将风险转化为速度缩放系数（如 1.0 / 0.4 / 0.0）。

## 5. 对现有代码的具体修改建议

### 5.1 修改 [src/yolo_lidar_fusion/yolo_lidar_fusion/fusion_node.py](src/yolo_lidar_fusion/yolo_lidar_fusion/fusion_node.py)

目标：把“单点测距”升级为“角窗口鲁棒测距 + 连续帧滤波 + 可发布结构化结果”。

建议新增参数：

- `angle_window_deg`：默认 `8.0`
- `range_statistic`：`median` / `p30`
- `stability_frames`：默认 `3`
- `publish_topics`：是否发布风险信息

建议新增方法（示意）：

```python
# 1) 根据检测框中心角度，获取角窗口内有效距离

def sample_ranges(scan, center_angle_rad, window_deg):
    half = math.radians(window_deg / 2.0)
    samples = []
    angle = center_angle_rad - half
    while angle <= center_angle_rad + half:
        idx = int((angle - scan.angle_min) / scan.angle_increment)
        if 0 <= idx < len(scan.ranges):
            d = scan.ranges[idx]
            if not (math.isinf(d) or math.isnan(d)):
                samples.append(d)
        angle += scan.angle_increment
    return samples

# 2) 采用分位值/中位值抑制离群点

def robust_distance(samples, mode="median"):
    if not samples:
        return None
    arr = sorted(samples)
    if mode == "p30":
        return arr[max(0, int(len(arr) * 0.3) - 1)]
    return arr[len(arr) // 2]
```

同时建议：

- 发布 `edge_distance` 和 `detection_confidence` 到新topic，供控制层直接使用。
- 日志保留节流，但增加“触发状态变化日志”（从safe->risk才打印）。

### 5.2 修改 [src/yolo_lidar_fusion/launch/fusion.launch.py](src/yolo_lidar_fusion/launch/fusion.launch.py)

目标：支持双雷达与下楼安全节点。

建议新增 LaunchArgument：

- `scan_down_topic`（默认 `/scan_down`）
- `enable_stair_safety`（默认 `true`）

新增 `stair_safety_node` 启动段，并将上述参数注入。

### 5.3 修改 [src/yolo_lidar_fusion/params/fusion.params.yaml](src/yolo_lidar_fusion/params/fusion.params.yaml)

新增建议参数：

```yaml
fusion_node:
  ros__parameters:
    angle_window_deg: 8.0
    range_statistic: median
    stability_frames: 3
    # 下倾雷达话题（若融合在同节点中）
    scan_down_topic: /scan_down
```

建议为 `stair_safety_node` 独立增加一个参数文件（例如 `params/stair_safety.params.yaml`）。

## 6. 里程碑计划（两周版）

第1阶段（D1-D3）：数据与标定

- 采集上楼/下楼/平地数据包（白天+夜晚）。
- 完成相机-雷达外参粗标定与FOV修正验证。

第2阶段（D4-D7）：算法与节点改造

- 实现角窗口鲁棒测距。
- 实现下楼风险判定状态机（连续帧确认）。
- 打通 `/stair/safe_velocity_scale` 输出。

第3阶段（D8-D10）：双雷达接入（若采购）

- 新雷达驱动与话题接入。
- 完成俯角安装调试，确定最佳安装高度与角度。

第4阶段（D11-D14）：实测验收

- 10组上下楼重复测试。
- 统计误报、漏报、触发延迟、停车距离。
- 固化参数并输出交付说明。

## 7. 验证与验收用例

- 用例1：平地直行，不应触发下楼风险。
- 用例2：接近下行楼梯边缘，应先降速再停。
- 用例3：玻璃/反光地面，风险值不应频繁抖动。
- 用例4：有人遮挡楼梯口，系统应保持保守策略（慢行/停）。

## 8. 风险与规避

- YOLO误检：增加“视觉+雷达”双条件触发，降低单模态误报。
- 雷达安装抖动：机械固定+上线前重复标定。
- 环境光影响：优先使用雷达结果做安全兜底，视觉只做先验。

## 9. 你可以直接着手的最小改动集（MVP）

1. 在 `fusion_node` 中把单点测距改为角窗口中位值测距。
2. 增加连续3帧一致性判断后再对外发布风险。
3. 在 launch 中加入 `scan_down_topic` 参数占位。
4. 新增 `stair_safety_node`（可先简单版：只做风险打分与速度缩放输出）。

---

如果你愿意，我下一步可以直接按这份计划给你落第一版代码补丁（先做MVP版：不改模型、先改测距和风险状态机），并给出可直接运行的 `ros2 launch` 与 `ros2 topic echo` 验证命令。
