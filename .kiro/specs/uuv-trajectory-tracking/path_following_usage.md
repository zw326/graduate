# 路径跟踪控制器使用说明

## 系统架构

```
你的新系统:
┌────────────────────────────────────────┐
│ path_following_node                    │
│ ├── Coverage Planner (路径规划)       │
│ ├── Path Following Controller          │
│ │   ├── PID推力控制                   │
│ │   ├── PID偏航控制                   │
│ │   ├── PID俯仰控制                   │
│ │   ├── P横滚控制                     │
│ │   └── 四舵面分配                    │
│ └── 直接发布到推进器和舵面            │
└────────────────────────────────────────┘
            ↓ 话题发布
    /a9/thrusters/0/input
    /a9/fins/*/input
            ↓
┌────────────────────────────────────────┐
│ Gazebo 插件 (uuv_simulator)            │
│ ├── 推进器物理模型                     │
│ ├── 舵面物理模型                       │
│ └── 水动力模型 (Fossen)                │
└────────────────────────────────────────┘
```

**关键特点**:
- ✅ 完全绕过 uuv_simulator 的控制器
- ✅ 自己实现PID控制
- ✅ 路径跟踪算法(不是点跟踪)
- ✅ 自适应速度(转弯时减速)
- ✅ 横向误差修正
- ✅ 官方四舵面分配矩阵

## 编译

```bash
cd ~/master_ws
catkin_make --pkg uuv_project_core
source devel/setup.bash
```

## 运行

### 方法1: 使用launch文件(推荐)

```bash
roslaunch uuv_project_core start_path_following.launch
```

这会启动:
1. Gazebo仿真环境
2. ECA A9 UUV模型
3. 建图节点
4. 路径跟踪控制器
5. RViz可视化

### 方法2: 分步启动

```bash
# 终端1: 启动Gazebo和UUV
roslaunch uuv_project_core start_my_world.launch

# 终端2: 启动路径跟踪控制器
rosrun uuv_project_core path_following_node \
  _map_origin_x:=-50.0 \
  _map_origin_y:=-50.0 \
  _map_origin_z:=-30.0 \
  _map_width:=100.0 \
  _map_height:=100.0 \
  _map_depth:=20.0 \
  _sonar_range:=35.0 \
  _overlap_ratio:=0.25 \
  _max_speed:=0.8 \
  _kp_yaw:=0.4 \
  _kd_yaw:=0.8
```

## 观察和监控

### 1. 查看控制器输出

控制器会每2秒输出一次状态:
```
Progress: 45/87 | Speed: 0.75/0.80 m/s | Yaw error: 5.2° | Cross-track: 0.3 m | Thrust: 45.0 N
```

### 2. RViz可视化

- **绿色路径**: 规划的覆盖路径
- **红色轨迹**: UUV实际运动轨迹
- **橙色球**: 当前前瞻目标点
- **UUV模型**: 实时位姿

### 3. 查看话题

```bash
# 查看推力命令
rostopic echo /a9/thrusters/0/input

# 查看舵面命令
rostopic echo /a9/fins/0/input
rostopic echo /a9/fins/1/input
rostopic echo /a9/fins/2/input
rostopic echo /a9/fins/3/input

# 查看UUV位姿
rostopic echo /a9/pose_gt
```

### 4. 实时绘图

```bash
# 绘制速度和推力
rqt_plot /a9/pose_gt/twist/twist/linear/x /a9/thrusters/0/input

# 绘制位置
rqt_plot /a9/pose_gt/pose/pose/position/x:y:z
```

## 参数调整

### 在launch文件中调整

编辑 `launch/start_path_following.launch`:

```xml
<!-- 如果出现螺旋,减小偏航增益 -->
<param name="kp_yaw" value="0.3"/>  <!-- 0.4 → 0.3 -->
<param name="kd_yaw" value="1.0"/>  <!-- 0.8 → 1.0 -->

<!-- 如果转弯太慢,增大前瞻距离 -->
<param name="lookahead_distance" value="10.0"/>  <!-- 8.0 → 10.0 -->

<!-- 如果速度不稳定,调整推力增益 -->
<param name="kp_thrust" value="40.0"/>  <!-- 50.0 → 40.0 -->
<param name="kd_thrust" value="8.0"/>   <!-- 5.0 → 8.0 -->
```

### 运行时调整(使用rosparam)

```bash
# 调整偏航增益
rosparam set /path_following_node/kp_yaw 0.3

# 调整前瞻距离
rosparam set /path_following_node/lookahead_distance 10.0

# 注意: 需要重启节点才能生效
```

## 与原方案对比

### 原方案 (waypoint_navigation_node + uuv_simulator控制器)

```
你的代码 → 服务调用 → uuv_simulator控制器 → 推进器/舵面
```

**优点**:
- 成熟的控制器
- 不需要自己实现控制算法

**缺点**:
- 黑盒,不知道内部逻辑
- 参数调整受限
- 点跟踪,转弯处可能螺旋

### 新方案 (path_following_node)

```
你的代码(路径跟踪+PID) → 直接控制 → 推进器/舵面
```

**优点**:
- ✅ 完全透明,可以调试
- ✅ 路径跟踪,更平滑
- ✅ 自适应速度
- ✅ 横向误差修正
- ✅ 可以实现任何控制算法

**缺点**:
- 需要自己调PID参数
- 需要理解控制原理

## 故障排查

### 问题1: 节点启动失败

**检查**:
```bash
# 检查可执行文件是否存在
ls ~/master_ws/devel/lib/uuv_project_core/path_following_node

# 如果不存在,重新编译
cd ~/master_ws
catkin_make --pkg uuv_project_core
```

### 问题2: UUV不动

**检查**:
```bash
# 1. 检查节点是否在运行
rosnode list | grep path_following

# 2. 检查是否收到位姿
rostopic echo /a9/pose_gt -n 1

# 3. 查看节点日志
rosnode info /path_following_node
```

### 问题3: 螺旋运动

**解决**: 参考 `path_following_tuning_guide.md`

减小 `kp_yaw`,增大 `kd_yaw` 和 `lookahead_distance`

### 问题4: 编译错误

**常见错误**:
```
error: 'clamp' is not a member of 'std'
```

**解决**: 确保使用C++17
```cmake
set(CMAKE_CXX_STANDARD 17)
```

## 下一步

### 1. 参数调优

参考 `path_following_tuning_guide.md` 进行系统的参数调优

### 2. 性能评估

记录以下指标:
- 路径跟踪误差 (RMS)
- 完成时间
- 能量消耗
- 覆盖完整性

### 3. 算法改进

可以尝试:
- 模型预测控制 (MPC)
- 滑模控制
- 自适应控制
- 强化学习

### 4. 实际测试

在不同条件下测试:
- 不同速度
- 不同路径复杂度
- 有水流扰动
- 不同UUV模型

## 总结

你现在有了一个**完全自主的路径跟踪控制器**:
- ✅ 使用PID控制推进器和四舵面
- ✅ 路径跟踪算法(不是点跟踪)
- ✅ 完全透明,可调试
- ✅ 适合你的覆盖扫描任务

开始测试吧!
