# UUV路径跟踪问题诊断

## 观察到的问题

从日志分析，系统表现出以下症状：

1. **横向误差持续增长**：从0.3m增长到8+米
2. **偏航误差很大**：达到-69度甚至更高
3. **速度控制正常**：能够维持在目标速度附近
4. **推力输出合理**：在0-40N范围内

## 根本原因分析

### 问题1：控制策略缺陷

当前的"前瞻点追踪"策略存在致命缺陷：

```
UUV偏离路径 → 前瞻点在远处 → UUV转向前瞻点 → 
但转向时继续前进 → 横向误差继续增大 → 前瞻点继续前移 → 恶性循环
```

**解决方案**：实现两阶段控制
- 阶段1（横向误差>3m）：直接指向最近的路径点，回到路径上
- 阶段2（横向误差<3m）：正常跟踪前瞻点

### 问题2：PID参数不匹配UUV动力学

ECA A9 UUV的特性：
- 转向响应慢（大惯性）
- 舵面效果有延迟
- 在低速时舵面效率低

当前参数问题：
- `kp_yaw = 0.25` 太小，无法产生足够的转向力
- `lookahead_distance = 10m` 对于慢速转向的UUV太短
- `max_fin_angle = 1.396` 太大，可能导致过度控制

### 问题3：路径本身的问题

从路径规划器输出：
```
Waypoint spacing - Min: 2.58 m, Max: 22.86 m, Avg: 10.42 m
32 waypoint pairs have spacing outside recommended range (10-40m)
```

路径点间距不均匀，导致：
- 曲率计算不准确
- 速度目标跳变
- 前瞻点跳跃

## 推荐的修复方案

### 方案A：修改控制器代码（需要重新编译）

在 `src/path_following_controller.cpp` 的偏航控制部分添加两阶段逻辑：

```cpp
// 判断是否需要先回到路径上
double cross_track_threshold = 3.0;

if (std::abs(cross_track_error) > cross_track_threshold) {
    // 阶段1：直接指向最近的路径点
    double dx_closest = path_[closest_idx].x - current_pose_.position.x;
    double dy_closest = path_[closest_idx].y - current_pose_.position.y;
    desired_yaw = std::atan2(dy_closest, dx_closest);
} else {
    // 阶段2：正常跟踪前瞻点
    double dx_lookahead = lookahead_point.x - current_pose_.position.x;
    double dy_lookahead = lookahead_point.y - current_pose_.position.y;
    desired_yaw = std::atan2(dy_lookahead, dx_lookahead);
}
```

### 方案B：使用参数文件（无需重新编译）

创建 `config/path_following_stable.yaml`（已创建）并修改启动方式：

```bash
rosrun uuv_project_core path_following_node _lookahead_distance:=20.0 _max_speed:=0.4 _kp_yaw:=1.2 _max_fin_angle:=0.5
```

### 方案C：使用官方控制器

UUV Simulator自带的控制器可能更适合ECA A9：

```bash
roslaunch eca_a9_control start_geometric_tracking_control.launch
```

然后发布路径到 `/a9/path_following/path` 话题。

## 下一步行动

1. **立即尝试**：使用方案C（官方控制器）
2. **短期**：实现方案A（修改代码并重新编译）
3. **长期**：优化路径规划器，生成更均匀的路径点

## 参数调优指南

如果继续使用自定义控制器，按以下顺序调整参数：

1. **增加前瞻距离**：`lookahead_distance: 15-20m`
2. **降低速度**：`max_speed: 0.4-0.5 m/s`
3. **增大偏航增益**：`kp_yaw: 0.8-1.2`
4. **限制舵面角度**：`max_fin_angle: 0.5-0.7`
5. **移除D项**：`kd_yaw: 0.0`（避免噪声放大）

## 测试建议

创建一个简单的直线路径进行测试：

```python
# 简单直线路径
path = [
    (-50, -50, -30),
    (-40, -50, -30),
    (-30, -50, -30),
    # ... 每10米一个点
    (50, -50, -30)
]
```

如果直线都跟不好，说明基础控制有问题。
如果直线能跟好，说明是转弯处理有问题。
