# UUV 轨迹跟踪系统故障排查

## 当前问题

**症状:**
1. ❌ 没有红色实际轨迹显示
2. ❌ UUV 停在原地不动
3. ❌ `/a9/actual_trajectory` 话题没有消息
4. ❌ `waypoint_navigation_node` 节点不在运行列表中

## 根本原因

`waypoint_navigation_node` 节点没有启动或启动后立即崩溃。

## 排查步骤

### 1. 检查节点是否编译成功

```bash
# 检查可执行文件是否存在
ls -lh ~/master_ws/devel/lib/uuv_project_core/waypoint_navigation_node

# 如果不存在,重新编译
cd ~/master_ws
catkin_make --pkg uuv_project_core
```

### 2. 检查节点日志

```bash
# 查看最新的日志
cat ~/.ros/log/latest/waypoint_navigation_node*.log

# 或者实时查看
tail -f ~/.ros/log/latest/waypoint_navigation_node*.log
```

### 3. 手动启动节点测试

```bash
# 先启动 roscore 和 Gazebo
roslaunch uuv_project_core start_my_world.launch

# 在另一个终端手动启动节点
rosrun uuv_project_core waypoint_navigation_node
```

### 4. 检查依赖是否满足

节点需要以下话题和服务:
- **订阅:** `/a9/pose_gt` (nav_msgs/Odometry)
- **发布:** `/a9/coverage_path` (nav_msgs/Path)
- **发布:** `/a9/actual_trajectory` (nav_msgs/Path)
- **服务:** `/a9/start_waypoint_list` (uuv_control_msgs/InitWaypointSet)

```bash
# 检查话题
rostopic list | grep -E "(pose_gt|coverage_path|actual_trajectory)"

# 检查服务
rosservice list | grep waypoint
```

## 可能的问题和解决方案

### 问题 1: 编译失败

**原因:** 缺少依赖或代码错误

**解决:**
```bash
cd ~/master_ws
catkin_make clean
catkin_make --pkg uuv_project_core
```

### 问题 2: 服务不可用导致节点退出

**原因:** `/a9/start_waypoint_list` 服务在10秒内不可用

**解决:** 确保 `geometric_tracking_controller` 先启动
- 检查 launch 文件中的启动顺序
- 增加服务等待时间

### 问题 3: 位姿信息不可用

**原因:** `/a9/pose_gt` 话题没有发布

**解决:**
```bash
# 检查话题是否存在
rostopic echo /a9/pose_gt -n 1

# 检查 ground_truth_to_tf 节点是否运行
rosnode list | grep ground_truth
```

### 问题 4: 参数配置错误

**原因:** ROS 参数命名空间问题

**解决:** 检查 launch 文件中的参数是否正确传递

## 修复后的验证步骤

### 1. 确认节点运行
```bash
rosnode list | grep waypoint
# 应该看到: /waypoint_navigation_node
```

### 2. 确认路径发布
```bash
rostopic echo /a9/coverage_path -n 1
# 应该看到绿色路径的航路点
```

### 3. 确认轨迹发布
```bash
rostopic echo /a9/actual_trajectory -n 1
# 应该看到 UUV 的实际位置轨迹
```

### 4. 确认服务调用成功
查看节点日志,应该看到:
```
✅ Service call completed
✅ Service returned SUCCESS!
UUV should now be following waypoints
```

### 5. 在 RViz 中验证
- 绿色线: 规划的覆盖路径 (`/a9/coverage_path`)
- 红色线: UUV 实际轨迹 (`/a9/actual_trajectory`)
- UUV 模型应该沿着绿色路径移动

## 快速修复命令

```bash
# 1. 停止所有节点
rosnode kill -a
killall -9 gzserver gzclient

# 2. 重新编译
cd ~/master_ws
source devel/setup.bash
catkin_make --pkg uuv_project_core

# 3. 重新启动
roslaunch uuv_project_core start_my_world.launch

# 4. 在另一个终端检查
rosnode list | grep waypoint
rostopic list | grep trajectory
```

## 调试技巧

### 增加日志输出

在 `waypoint_navigation_node_v2.cpp` 中添加更多 ROS_INFO:

```cpp
ROS_INFO("Node starting...");
ROS_INFO("Waiting for pose...");
ROS_INFO("Got pose, sending waypoints...");
```

### 使用 roswtf 诊断

```bash
roswtf
```

### 检查 TF 树

```bash
rosrun tf view_frames
evince frames.pdf
```
