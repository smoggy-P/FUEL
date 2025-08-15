# Trajectory Bridge Package

这个包提供了一个桥接节点，用于将 FUEL 无人机系统的轨迹消息转换为 agilicious 系统使用的参考轨迹格式。

## 功能描述

`trajectory_bridge` 节点订阅 FUEL 系统的轨迹消息，并将其转换为 agilicious 系统可以理解的 `Reference` 消息格式。主要功能包括：

- 订阅 `planning/pos_cmd` 话题（FUEL 位置命令）
- 订阅 `planning/trajectory` 话题（FUEL 轨迹路径）
- 发布 `agilicious/reference` 话题（agilicious 参考轨迹）

## 消息转换

### 输入消息类型
- **FUEL PositionCommand**: 包含位置、速度、加速度、偏航角等信息
- **FUEL Path**: 包含一系列位姿点的轨迹

### 输出消息类型
- **agilicious Reference**: 包含多个 Setpoint 的参考轨迹
- **agilicious Setpoint**: 每个点包含 QuadState 和 Command 信息

## 参数配置

| 参数名 | 类型 | 默认值 | 描述 |
|--------|------|--------|------|
| `trajectory_duration` | double | 10.0 | 轨迹持续时间（秒） |
| `sampling_rate` | double | 100.0 | 采样频率（Hz） |
| `output_frame_id` | string | "world" | 输出坐标系ID |
| `enable_jerk_snap` | bool | false | 是否启用jerk和snap计算 |

## 使用方法

### 启动节点
```bash
roslaunch trajectory_bridge trajectory_bridge.launch
```

### 参数调整
```bash
rosrun trajectory_bridge trajectory_bridge_node _trajectory_duration:=15.0 _sampling_rate:=50.0
```

### 话题重映射
```bash
rosrun trajectory_bridge trajectory_bridge_node _planning/pos_cmd:=/fuel/pos_cmd _agilicious/reference:=/uav/reference
```

## 依赖项

- `roscpp`: ROS C++ 客户端库
- `std_msgs`: 标准消息类型
- `geometry_msgs`: 几何消息类型
- `nav_msgs`: 导航消息类型
- `tf`: 坐标变换库
- `quadrotor_msgs`: FUEL 四旋翼消息类型
- `agiros_msgs`: agilicious 消息类型

## 构建说明

确保你的工作空间已经包含了 FUEL 和 agilicious 的依赖包，然后执行：

```bash
cd ~/workspace/fuel_ws
catkin_make
```

## 注意事项

1. 确保 agilicious 消息包已正确安装
2. 检查坐标系设置是否正确
3. 轨迹插值基于时间戳，确保输入消息的时间戳是有效的
4. 默认假设无人机质量为 1kg，可根据实际情况调整推力计算

## 故障排除

### 常见问题
- **消息类型错误**: 检查 agiros_msgs 是否正确安装
- **话题未发布**: 确认输入话题有数据发布
- **坐标系错误**: 检查 frame_id 设置

### 调试信息
节点启动时会输出详细的配置信息，包括订阅和发布的话题名称。 