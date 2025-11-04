# Design Document

## Overview

本设计文档描述了水下无人航行器(UUV)轨迹跟踪控制系统的架构和实现方案。系统采用分层控制架构,包括路径规划层、轨迹生成层和底层控制层,通过ROS框架实现模块间通信。

### System Context

系统运行在Gazebo仿真环境中,使用ECA A9 AUV模型。主要组件包括:
- **Coverage Planner**: 生成覆盖整个目标区域的弓字形扫描路径
- **Waypoint Navigation Node**: 管理航路点序列并与控制器通信
- **Geometric Tracking Controller**: UUV Simulator提供的轨迹跟踪控制器
- **UUV Dynamics**: Gazebo中的UUV物理模型,包括推进器和舵面

### Design Goals

1. **精确性**: 轨迹跟踪误差 < 5米
2. **平滑性**: 避免急转弯,使用Dubins路径插值
3. **鲁棒性**: 处理位姿信息丢失、服务调用失败等异常
4. **可配置性**: 通过launch文件配置所有关键参数
5. **可观测性**: 提供完整的可视化和日志输出

## Architecture

### System Architecture Diagram

```
┌─────────────────────────────────────────────────────────────┐
│                    Gazebo Simulation                        │
│  ┌──────────────────────────────────────────────────────┐  │
│  │              ECA A9 UUV Model                        │  │
│  │  ┌────────────┐  ┌────────────┐  ┌──────────────┐  │  │
│  │  │ Thrusters  │  │   Fins     │  │   Sensors    │  │  │
│  │  └────────────┘  └────────────┘  └──────────────┘  │  │
│  └──────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────┘
                           ↑ ↓
                    /a9/pose_gt (Odometry)
                    /a9/thrusters/*/input
                    /a9/fins/*/input
                           ↑ ↓
┌─────────────────────────────────────────────────────────────┐
│           Geometric Tracking Controller                     │
│  ┌──────────────────────────────────────────────────────┐  │
│  │  Trajectory Interpolator (Dubins/Linear)            │  │
│  │  ┌────────────┐  ┌────────────┐  ┌──────────────┐  │  │
│  │  │ Thrust PID │  │  Yaw PID   │  │  Pitch PID   │  │  │
│  │  └────────────┘  └────────────┘  └──────────────┘  │  │
│  └──────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────┘
                           ↑
                    /a9/start_waypoint_list (Service)
                           ↑
┌─────────────────────────────────────────────────────────────┐
│          Waypoint Navigation Node                           │
│  ┌──────────────────────────────────────────────────────┐  │
│  │  Coverage Planner                                    │  │
│  │  ┌────────────────────────────────────────────────┐ │  │
│  │  │  Path Generation (Boustrophedon)              │ │  │
│  │  │  Path Smoothing (Bezier Curves)               │ │  │
│  │  │  Waypoint Filtering                            │ │  │
│  │  └────────────────────────────────────────────────┘ │  │
│  └──────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────┘
                           ↓
                    /a9/coverage_path (Path)
                    /a9/actual_trajectory (Path)
                           ↓
┌─────────────────────────────────────────────────────────────┐
│                      RViz Visualization                     │
└─────────────────────────────────────────────────────────────┘
```

### Control Flow

1. **Initialization Phase**:
   - Launch Gazebo with custom world
   - Spawn ECA A9 UUV at starting position
   - Start Geometric Tracking Controller
   - Start Waypoint Navigation Node

2. **Path Planning Phase**:
   - Coverage Planner generates boustrophedon path
   - Path is smoothed using Bezier curves
   - Waypoints are filtered to match controller constraints
   - Path is published for visualization

3. **Trajectory Execution Phase**:
   - Navigation Node waits for UUV pose
   - Waypoint list is sent via service call
   - Controller interpolates smooth trajectory
   - PID controllers compute control commands
   - UUV follows trajectory

4. **Monitoring Phase**:
   - Actual trajectory is recorded and published
   - Distance to current waypoint is monitored
   - Waypoint switching occurs when within acceptance radius

## Components and Interfaces

### 1. Coverage Planner

**Responsibility**: Generate 3D coverage path for sonar scanning

**Key Methods**:
- `generatePath()`: Creates boustrophedon pattern
- `smoothPath()`: Applies Bezier curve smoothing
- `interpolateLinear()`: Inserts intermediate points
- `smoothCorner()`: Generates smooth turns

**Parameters**:
- Map dimensions: 100m × 100m × 20m
- Sonar FOV: 90° horizontal, 60° vertical
- Sonar range: 35m
- Overlap ratio: 25%
- Point spacing: 12m on straight segments
- Corner radius: 10m

**Output**: Vector of Waypoint structs (x, y, z)

### 2. Waypoint Navigation Node

**Responsibility**: Manage waypoint sequence and communicate with controller

**ROS Interfaces**:
- **Subscribers**:
  - `/a9/pose_gt` (nav_msgs/Odometry): UUV ground truth pose
- **Publishers**:
  - `/a9/coverage_path` (nav_msgs/Path): Planned coverage path
  - `/a9/actual_trajectory` (nav_msgs/Path): Actual UUV trajectory
  - `/a9/current_target` (visualization_msgs/Marker): Current target waypoint
- **Service Clients**:
  - `/a9/start_waypoint_list` (uuv_control_msgs/InitWaypointSet): Send waypoints

**Key Logic**:

```cpp
// Waypoint filtering algorithm
std::vector<Waypoint> filtered;
filtered.push_back(waypoints[0]);  // Always include first

for (size_t i = 1; i < waypoints.size(); ++i) {
    double dist = distance(filtered.back(), waypoints[i]);
    if (dist >= min_distance) {  // min_distance = 25m
        filtered.push_back(waypoints[i]);
    }
}

filtered.push_back(waypoints.back());  // Always include last
```

**Waypoint Message Configuration**:
- `max_forward_speed`: 0.4 m/s
- `radius_of_acceptance`: 8-10m (< dubins_radius/2)
- `use_fixed_heading`: false (let controller compute heading)
- `interpolator`: "dubins" for smooth turns

### 3. Geometric Tracking Controller

**Responsibility**: Generate smooth trajectory and compute control commands

**Configuration** (from launch file):
```xml
<arg name="max_forward_speed" value="0.4"/>
<arg name="dubins_radius" value="20"/>
<arg name="idle_radius" value="300"/>
<arg name="look_ahead_delay" value="6.0"/>

<arg name="thrust_p_gain" value="8.0"/>
<arg name="thrust_d_gain" value="0.8"/>
<arg name="p_yaw" value="1.0"/>
<arg name="d_yaw" value="0.6"/>
<arg name="p_pitch" value="1.2"/>
<arg name="d_pitch" value="0.5"/>
```

**Control Algorithm**:
1. **Trajectory Interpolation**: Dubins path between waypoints
2. **Thrust Control**: PID on forward velocity error
3. **Attitude Control**: PD on yaw/pitch/roll errors
4. **Fin Allocation**: Map attitude commands to 4 fins

**State Machine**:
- **IDLE**: No waypoints or distance > idle_radius
- **TRACKING**: Following trajectory
- **STATION_KEEPING**: Reached final waypoint

### 4. UUV Dynamics Model

**Physical Parameters** (from eca_a9_base.xacro):
- Mass: 69.7 kg
- Length: 1.98 m
- Diameter: 0.23 m
- Volume: 0.068 m³
- Center of Buoyancy: (0, 0, 0.06)

**Hydrodynamic Model** (Fossen):
- Added mass matrix (6×6)
- Linear damping (forward speed dependent)
- Quadratic damping
- Fin lift/drag coefficients

**Actuators**:
- 1 thruster: max 150N, time constant 0.1s
- 4 fins: ±80° range, area 0.04155 m²

## Data Models

### Waypoint Structure

```cpp
struct Waypoint {
    double x;  // meters
    double y;  // meters
    double z;  // meters (negative = depth)
    
    Waypoint(double x_, double y_, double z_)
        : x(x_), y(y_), z(z_) {}
};
```

### ROS Message Types

**uuv_control_msgs/Waypoint**:
```
Header header
geometry_msgs/Point point
float64 max_forward_speed
float64 heading_offset
bool use_fixed_heading
float64 radius_of_acceptance
```

**uuv_control_msgs/InitWaypointSet**:
```
Request:
  Time start_time
  bool start_now
  Waypoint[] waypoints
  float64 max_forward_speed
  float64 heading_offset
  string interpolator

Response:
  bool success
```

## Error Handling

### 1. Pose Information Loss

**Scenario**: `/a9/pose_gt` topic stops publishing

**Handling**:
```cpp
if (!has_pose_) {
    ROS_WARN_THROTTLE(5.0, "Waiting for pose...");
    ros::spinOnce();
    rate.sleep();
    continue;
}
```

**Recovery**: Wait indefinitely for pose to become available

### 2. Service Call Failure

**Scenario**: `/a9/start_waypoint_list` service unavailable

**Handling**:
```cpp
if (!ros::service::waitForService(service_name, ros::Duration(10.0))) {
    ROS_ERROR("Service %s not available", service_name.c_str());
    return false;
}
```

**Recovery**: Log error and exit (requires manual restart)

### 3. Large Initial Distance

**Scenario**: UUV spawns far from first waypoint (> 20m)

**Handling**:
```cpp
if (dist > 20.0) {
    ROS_INFO("Adding intermediate waypoint");
    Waypoint intermediate(
        current_x + dx * 0.5,
        current_y + dy * 0.5,
        current_z + dz * 0.5
    );
    waypoints.insert(waypoints.begin(), intermediate);
}
```

**Recovery**: Insert intermediate waypoint to avoid idle mode

### 4. Controller Idle Mode

**Scenario**: Distance to all waypoints > idle_radius (300m)

**Prevention**:
- Set idle_radius large enough (300m)
- Ensure waypoint spacing < idle_radius
- Add intermediate waypoints if needed

**Detection**: Monitor controller state topic

## Testing Strategy

### Unit Tests

1. **Coverage Planner Tests**:
   - Test path generation with various map sizes
   - Verify waypoint spacing constraints
   - Test Bezier curve smoothing
   - Validate corner radius calculations

2. **Waypoint Filtering Tests**:
   - Test minimum distance filtering
   - Verify first/last waypoint preservation
   - Test edge cases (empty list, single waypoint)

### Integration Tests

1. **Service Communication Test**:
   - Verify service call succeeds
   - Check waypoint message format
   - Validate response handling

2. **Trajectory Tracking Test**:
   - Spawn UUV at known position
   - Send simple waypoint sequence
   - Measure tracking error
   - Verify waypoint switching

### Simulation Tests

1. **Full Mission Test**:
   - Run complete coverage mission
   - Monitor tracking error throughout
   - Verify all waypoints reached
   - Check trajectory smoothness

2. **Parameter Sensitivity Test**:
   - Vary control gains
   - Test different speeds
   - Adjust dubins_radius
   - Measure performance metrics

### Performance Metrics

- **Tracking Error**: RMS distance from desired trajectory
- **Waypoint Arrival Time**: Time to reach each waypoint
- **Path Smoothness**: Maximum acceleration/jerk
- **Coverage Completeness**: Percentage of area scanned

## Configuration Management

### Launch File Parameters

All parameters are centralized in `start_my_world.launch`:

```xml
<!-- Map Configuration -->
<param name="map_origin_x" value="-50.0"/>
<param name="map_origin_y" value="-50.0"/>
<param name="map_origin_z" value="-30.0"/>
<param name="map_width" value="100.0"/>
<param name="map_height" value="100.0"/>
<param name="map_depth" value="20.0"/>

<!-- Sonar Configuration -->
<param name="sonar_range" value="35.0"/>
<param name="overlap_ratio" value="0.25"/>

<!-- Control Configuration -->
<arg name="max_forward_speed" value="0.4"/>
<arg name="dubins_radius" value="20"/>
<arg name="arrival_threshold" value="8.0"/>
```

### Parameter Tuning Guidelines

**Speed vs. Accuracy Trade-off**:
- Lower speed → better tracking, longer mission time
- Higher speed → faster mission, larger tracking error
- Recommended: 0.3-0.5 m/s

**Dubins Radius Selection**:
- Must match UUV turning capability
- Formula: R_min ≈ v² / (g * tan(max_roll))
- For v=0.4 m/s: R_min ≈ 15-20m
- Set dubins_radius = 1.2 * R_min for safety margin

**Waypoint Spacing**:
- Must be > dubins_radius to avoid path conflicts
- Recommended: 1.5-2.0 * dubins_radius
- Current: 25-35m with dubins_radius=20m

**Acceptance Radius**:
- Must be < dubins_radius / 2
- Too small → oscillation around waypoint
- Too large → path cutting
- Recommended: 0.3-0.5 * dubins_radius

## Visualization Design

### RViz Configuration

Display elements in `exploration.rviz`:
1. **Grid**: World reference frame
2. **Robot Model**: ECA A9 with fins and thruster
3. **TF Tree**: All coordinate frames
4. **Coverage Path**: Green line showing planned path
5. **Actual Trajectory**: Blue line showing UUV track
6. **Current Target**: Red sphere at active waypoint
7. **Sonar Visualization**: Point cloud from lidar sensor

### Color Scheme

- Planned path: Green (RGB: 0, 255, 0)
- Actual trajectory: Blue (RGB: 0, 0, 255)
- Current target: Red (RGB: 255, 0, 0)
- Waypoints: Yellow spheres (RGB: 255, 255, 0)

## Future Enhancements

1. **Adaptive Speed Control**: Slow down in turns, speed up on straights
2. **Obstacle Avoidance**: Integrate SAVFH algorithm for dynamic obstacles
3. **Re-planning**: Update path based on sonar feedback
4. **Multi-Layer Scanning**: Optimize vertical transitions
5. **Battery Management**: Consider energy consumption in planning
