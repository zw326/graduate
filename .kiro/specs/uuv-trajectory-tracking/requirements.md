# Requirements Document

## Introduction

本文档定义了水下无人航行器(UUV)轨迹跟踪控制系统的需求。该系统需要使UUV能够精确跟踪预设的三维覆盖路径,以完成水下环境的声纳扫描和三维重建任务。系统基于ROS框架,集成了ECA A9 AUV模型、几何跟踪控制器和覆盖路径规划器。

## Glossary

- **UUV (Unmanned Underwater Vehicle)**: 水下无人航行器,本项目中特指ECA A9模型
- **Trajectory Tracking System**: 轨迹跟踪系统,负责控制UUV沿预设路径运动
- **Coverage Path**: 覆盖路径,用于声纳扫描的弓字形三维路径
- **Geometric Tracking Controller**: 几何跟踪控制器,UUV Simulator提供的轨迹跟踪控制器
- **Waypoint Navigation Node**: 航路点导航节点,负责生成和发送航路点
- **ROS (Robot Operating System)**: 机器人操作系统框架
- **Gazebo**: 物理仿真环境
- **Dubins Path**: 杜宾斯路径,考虑最小转弯半径的平滑路径

## Requirements

### Requirement 1: 路径生成与发布

**User Story:** 作为系统操作员,我希望系统能够自动生成覆盖整个目标区域的三维扫描路径,以便UUV能够完整地采集环境数据

#### Acceptance Criteria

1. WHEN THE Trajectory Tracking System 启动时, THE Waypoint Navigation Node SHALL 根据地图参数和声纳参数生成弓字形覆盖路径
2. THE Coverage Path SHALL 包含足够的航路点以覆盖100m×100m×20m的目标区域
3. THE Waypoint Navigation Node SHALL 在路径生成后立即发布路径消息到ROS话题以供可视化
4. THE Coverage Path 中相邻航路点之间的距离 SHALL 大于20米且小于40米,以匹配控制器的转弯能力
5. THE Waypoint Navigation Node SHALL 在发送航路点前等待UUV位姿信息可用

### Requirement 2: 轨迹跟踪控制

**User Story:** 作为系统操作员,我希望UUV能够平滑且精确地跟踪预设路径,以确保声纳数据的质量和完整性

#### Acceptance Criteria

1. WHEN THE Geometric Tracking Controller 接收到航路点列表时, THE Geometric Tracking Controller SHALL 生成平滑的轨迹并控制UUV跟踪
2. THE Geometric Tracking Controller SHALL 使用Dubins路径插值器生成考虑最小转弯半径的平滑轨迹
3. THE UUV 的最大前进速度 SHALL 被限制在0.4 m/s以内,以确保有足够时间完成转向
4. THE Geometric Tracking Controller SHALL 使用PID控制算法调节推力和舵面角度
5. WHILE THE UUV 跟踪轨迹时, THE Geometric Tracking Controller SHALL 保持跟踪误差小于5米

### Requirement 3: 航路点切换逻辑

**User Story:** 作为系统操作员,我希望UUV能够在合适的时机切换到下一个航路点,避免过早或过晚切换导致的路径偏差

#### Acceptance Criteria

1. THE Waypoint Navigation Node SHALL 设置每个航路点的接受半径为8-10米
2. WHEN THE UUV 与当前目标航路点的距离小于接受半径时, THE Geometric Tracking Controller SHALL 切换到下一个航路点
3. THE 接受半径 SHALL 小于Dubins路径半径的一半,以避免路径跳跃
4. IF THE UUV 距离第一个航路点超过20米, THEN THE Waypoint Navigation Node SHALL 在路径开始处插入中间航路点
5. THE Geometric Tracking Controller SHALL 在距离所有航路点超过300米时进入空闲模式

### Requirement 4: 控制参数调优

**User Story:** 作为系统开发者,我希望控制器参数能够匹配UUV的动力学特性,以实现稳定且响应迅速的控制

#### Acceptance Criteria

1. THE Geometric Tracking Controller 的推力比例增益 SHALL 设置为8.0,以平衡响应速度和稳定性
2. THE Geometric Tracking Controller 的推力微分增益 SHALL 设置为0.8,以提供足够的阻尼
3. THE Geometric Tracking Controller 的偏航角比例增益 SHALL 设置为1.0,以匹配UUV的偏航惯性(67 kg·m²)
4. THE Geometric Tracking Controller 的偏航角微分增益 SHALL 设置为0.6,以抑制振荡
5. THE Geometric Tracking Controller 的前瞻延迟 SHALL 设置为6.0秒,以增大前瞻距离到2.4米

### Requirement 5: 系统集成与通信

**User Story:** 作为系统架构师,我希望各个模块能够通过标准的ROS接口正确通信,确保系统的可靠运行

#### Acceptance Criteria

1. THE Waypoint Navigation Node SHALL 订阅 `/a9/pose_gt` 话题以获取UUV的真实位姿
2. THE Waypoint Navigation Node SHALL 通过调用 `/a9/start_waypoint_list` 服务将航路点列表发送给控制器
3. THE Geometric Tracking Controller SHALL 发布控制命令到 `/a9/thrusters/0/input` 和 `/a9/fins/*/input` 话题
4. THE Waypoint Navigation Node SHALL 发布可视化消息到 `/a9/coverage_path` 和 `/a9/actual_trajectory` 话题
5. WHEN 服务调用失败时, THE Waypoint Navigation Node SHALL 记录错误信息并等待服务可用

### Requirement 6: 路径平滑与优化

**User Story:** 作为系统开发者,我希望生成的路径能够考虑UUV的运动约束,避免急转弯和不可达的轨迹

#### Acceptance Criteria

1. THE Coverage Planner SHALL 在关键航路点之间使用二次贝塞尔曲线进行平滑
2. THE Coverage Planner SHALL 设置转角半径为10米,以生成可跟踪的平滑转角
3. THE Coverage Planner SHALL 在直线段上每12米插入一个航路点
4. THE Waypoint Navigation Node SHALL 过滤掉距离小于25米的相邻航路点
5. THE Coverage Path SHALL 确保第一个和最后一个航路点始终被包含

### Requirement 7: 可视化与监控

**User Story:** 作为系统操作员,我希望能够在RViz中实时查看UUV的位置、目标路径和实际轨迹,以便监控任务执行情况

#### Acceptance Criteria

1. THE Waypoint Navigation Node SHALL 发布规划路径为 `nav_msgs/Path` 消息
2. THE Waypoint Navigation Node SHALL 发布当前目标航路点为可视化标记
3. THE Waypoint Navigation Node SHALL 实时记录并发布UUV的实际运动轨迹
4. THE RViz 配置文件 SHALL 包含路径、轨迹和UUV模型的显示设置
5. THE Waypoint Navigation Node SHALL 每2秒输出一次当前航路点和距离信息

### Requirement 8: 错误处理与鲁棒性

**User Story:** 作为系统操作员,我希望系统能够处理异常情况,如位姿信息丢失或控制器无响应,确保任务的可靠性

#### Acceptance Criteria

1. WHEN THE Waypoint Navigation Node 未接收到位姿信息时, THE Waypoint Navigation Node SHALL 每5秒输出一次警告并等待
2. IF THE `/a9/start_waypoint_list` 服务在10秒内不可用, THEN THE Waypoint Navigation Node SHALL 记录错误并退出
3. WHEN 服务调用返回失败时, THE Waypoint Navigation Node SHALL 记录详细的错误信息
4. THE Waypoint Navigation Node SHALL 限制记录的轨迹点数量为1000个,以避免内存溢出
5. THE Geometric Tracking Controller SHALL 在接收到无效航路点时忽略该点并继续执行

### Requirement 9: 启动配置与参数管理

**User Story:** 作为系统部署人员,我希望能够通过launch文件方便地配置系统参数,无需修改代码

#### Acceptance Criteria

1. THE `start_my_world.launch` 文件 SHALL 包含所有关键参数的配置,包括速度、转弯半径和控制增益
2. THE launch文件 SHALL 按正确顺序启动Gazebo、UUV模型、控制器和导航节点
3. THE launch文件 SHALL 设置 `GAZEBO_MODEL_PATH` 环境变量以加载自定义模型
4. THE launch文件 SHALL 提供参数注释说明每个参数的作用和推荐值
5. THE launch文件 SHALL 配置RViz自动加载预设的可视化配置
