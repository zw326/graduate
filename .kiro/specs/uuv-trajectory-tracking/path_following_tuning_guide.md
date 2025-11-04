# 路径跟踪控制器参数调优指南

## 控制器架构

```
路径跟踪控制器
├── 速度控制 (PID推力控制)
├── 偏航控制 (PID + 横向误差修正)
├── 俯仰控制 (PID)
├── 横滚控制 (P控制,保持水平)
└── 四舵面分配 (官方映射矩阵)
```

## 参数说明

### 1. 路径跟踪参数

| 参数 | 默认值 | 说明 | 调整建议 |
|------|--------|------|----------|
| `lookahead_distance` | 8.0m | 前瞻距离 | 增大→更平滑但反应慢; 减小→反应快但可能振荡 |
| `kp_cross_track` | 0.3 | 横向误差增益 | 增大→更快回到路径; 减小→更平滑 |
| `k_curvature` | 2.0 | 曲率影响系数 | 增大→转弯时减速更多 |

### 2. 速度控制参数

| 参数 | 默认值 | 说明 | 调整建议 |
|------|--------|------|----------|
| `max_speed` | 0.8 m/s | 最大速度 | 根据任务需求调整 |
| `min_speed` | 0.3 m/s | 最小速度 | 转弯时的最低速度 |
| `kp_thrust` | 50.0 | 推力比例增益 | 增大→加速快; 减小→更平滑 |
| `kd_thrust` | 5.0 | 推力微分增益 | 增大→阻尼大,减少超调 |

### 3. 偏航控制参数

| 参数 | 默认值 | 说明 | 调整建议 |
|------|--------|------|----------|
| `kp_yaw` | 0.4 | 偏航比例增益 | **关键参数!** 太大→螺旋; 太小→转弯慢 |
| `kd_yaw` | 0.8 | 偏航微分增益 | 增大→抑制振荡 |

### 4. 俯仰控制参数

| 参数 | 默认值 | 说明 | 调整建议 |
|------|--------|------|----------|
| `kp_pitch` | 0.6 | 俯仰比例增益 | 控制深度变化的响应速度 |
| `kd_pitch` | 0.4 | 俯仰微分增益 | 抑制俯仰振荡 |

### 5. 横滚控制参数

| 参数 | 默认值 | 说明 | 调整建议 |
|------|--------|------|----------|
| `kp_roll` | 0.2 | 横滚比例增益 | 保持UUV水平,通常不需要调整 |

## 调优流程

### 阶段1: 直线跟踪测试

**目标**: 确保UUV能平稳地沿直线前进

1. 设置简单的直线路径
2. 调整推力PID参数
   - 先调 `kp_thrust`,让速度能跟上
   - 再调 `kd_thrust`,减少速度振荡
3. 调整偏航PID参数
   - 先调 `kp_yaw`,让UUV能转向
   - 再调 `kd_yaw`,减少偏航振荡

**期望结果**: UUV平稳前进,速度稳定,航向稳定

### 阶段2: 转弯性能测试

**目标**: 确保UUV能平滑转弯,不螺旋

1. 测试90度转弯
2. 如果出现螺旋:
   - 减小 `kp_yaw` (0.4 → 0.3 → 0.2)
   - 增大 `kd_yaw` (0.8 → 1.0 → 1.2)
   - 增大 `lookahead_distance` (8 → 10 → 12)
3. 如果转弯太慢:
   - 增大 `kp_yaw`
   - 增大 `kp_cross_track`

**期望结果**: 转弯平滑,无螺旋,横向误差小

### 阶段3: 完整路径测试

**目标**: 测试弓字形覆盖路径

1. 运行完整的覆盖路径
2. 观察转弯处的表现
3. 根据需要微调参数

**期望结果**: 完整覆盖,路径平滑,无遗漏

## 常见问题和解决方案

### 问题1: 螺旋运动

**现象**: UUV围绕路径做螺旋运动

**原因**: 偏航控制增益太大

**解决**:
```xml
<param name="kp_yaw" value="0.3"/>  <!-- 减小 -->
<param name="kd_yaw" value="1.0"/>  <!-- 增大 -->
<param name="lookahead_distance" value="10.0"/>  <!-- 增大 -->
```

### 问题2: 转弯切角

**现象**: UUV在转弯处切内角,偏离路径

**原因**: 前瞻距离太小或横向误差增益太小

**解决**:
```xml
<param name="lookahead_distance" value="10.0"/>  <!-- 增大 -->
<param name="kp_cross_track" value="0.5"/>  <!-- 增大 -->
```

### 问题3: 速度不稳定

**现象**: UUV速度忽快忽慢

**原因**: 推力控制增益不合适

**解决**:
```xml
<param name="kp_thrust" value="40.0"/>  <!-- 减小比例增益 -->
<param name="kd_thrust" value="8.0"/>   <!-- 增大微分增益 -->
```

### 问题4: 深度控制不稳定

**现象**: UUV上下振荡

**原因**: 俯仰控制增益太大

**解决**:
```xml
<param name="kp_pitch" value="0.4"/>  <!-- 减小 -->
<param name="kd_pitch" value="0.6"/>  <!-- 增大阻尼 -->
```

## 推荐的参数组合

### 保守配置 (平滑优先)

```xml
<param name="lookahead_distance" value="10.0"/>
<param name="max_speed" value="0.6"/>
<param name="kp_yaw" value="0.3"/>
<param name="kd_yaw" value="1.0"/>
<param name="kp_cross_track" value="0.2"/>
```

**特点**: 非常平滑,几乎不振荡,但转弯较慢

### 平衡配置 (推荐)

```xml
<param name="lookahead_distance" value="8.0"/>
<param name="max_speed" value="0.8"/>
<param name="kp_yaw" value="0.4"/>
<param name="kd_yaw" value="0.8"/>
<param name="kp_cross_track" value="0.3"/>
```

**特点**: 平滑和响应速度平衡,适合大多数情况

### 激进配置 (速度优先)

```xml
<param name="lookahead_distance" value="6.0"/>
<param name="max_speed" value="1.0"/>
<param name="kp_yaw" value="0.5"/>
<param name="kd_yaw" value="0.6"/>
<param name="kp_cross_track" value="0.4"/>
```

**特点**: 响应快,速度高,但可能有轻微振荡

## 调试技巧

### 1. 使用ROS日志

控制器每2秒输出一次状态:
```
Progress: 45/87 | Speed: 0.75/0.80 m/s | Yaw error: 5.2° | Cross-track: 0.3 m | Thrust: 45.0 N
```

关注:
- `Yaw error`: 偏航误差,应该小于10度
- `Cross-track`: 横向误差,应该小于2米
- `Speed`: 实际速度应该接近期望速度

### 2. 使用RViz可视化

- 绿色路径: 规划的覆盖路径
- 红色轨迹: UUV实际轨迹
- 橙色球: 当前前瞻目标点

观察红色轨迹是否平滑,是否接近绿色路径

### 3. 使用rqt_plot

实时绘制控制变量:
```bash
rqt_plot /a9/pose_gt/pose/pose/position/x:y /a9/thrusters/0/input
```

### 4. 录制bag文件

```bash
rosbag record /a9/pose_gt /a9/thrusters/0/input /a9/fins/*/input
```

事后分析控制性能

## 总结

**关键参数优先级**:
1. `kp_yaw` - 最关键,决定转弯性能
2. `kd_yaw` - 抑制振荡
3. `lookahead_distance` - 影响平滑度
4. `kp_cross_track` - 影响路径跟踪精度

**调优原则**:
- 从保守参数开始
- 一次只调一个参数
- 小步调整,观察效果
- 记录每次调整的结果
