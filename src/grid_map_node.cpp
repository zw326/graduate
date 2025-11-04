#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/MarkerArray.h>

// OctoMap 相关头文件
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h> // 用于消息转换

// PCL 相关头文件 (仅用于接收点云消息)
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <memory> // For std::unique_ptr
#include <cmath>

// ================== 地图参数定义 ==================
// 从之前的代码继承，确保一致性
const double MAP_RESOLUTION = 0.5;                    // Octree 分辨率
const double MAP_WIDTH_METERS = 100.0;                // 仅用于可视化边界
const double MAP_HEIGHT_METERS = 100.0;               // 仅用于可视化边界
const double MAP_DEPTH_METERS = 40.0;                 // 仅用于可视化边界
const double MAP_ORIGIN_X = -MAP_WIDTH_METERS / 2.0;  // 仅用于可视化边界
const double MAP_ORIGIN_Y = -MAP_HEIGHT_METERS / 2.0; // 仅用于可视化边界
const double MAP_ORIGIN_Z = -MAP_DEPTH_METERS;        // 仅用于可视化边界

// ================== 贝叶斯更新参数 (Log-Odds形式) ==================
// 调整参数以提高精确率，减少误报
const float LOG_ODDS_OCC = 1.2f;    // 更保守的占据更新值，减少误报
const float LOG_ODDS_FREE = -0.4f;  // 增加空闲更新值，更快清除不确定区域
const float LOG_ODDS_MAX = 10.0f;   // 增加上限
const float LOG_ODDS_MIN = -10.0f;  // 增加下限

// ================== 声呐参数 ==================
const double MAX_SONAR_RANGE = 80.0; // 声呐最大有效探测距离，用于 insertPointCloud
const double MIN_SONAR_RANGE = 1.0;  // 声呐最小有效探测距离 (可选，用于过滤近处噪点)

// ================== 全局变量 ==================
// --- 核心修改：使用 Octree 替代 vector ---
std::unique_ptr<octomap::OcTree> octree; // 使用 OctoMap 作为核心地图
// ------------------------------------------
ros::Publisher octomap_pub;                   // 发布 OctoMap 消息
ros::Publisher map_viz_pub;                   // 发布 RViz 可视化标记 (可选)
pcl::PointXYZ auv_position = {NAN, NAN, NAN}; // AUV 当前位置，初始化为无效值

// ================== 函数声明 ==================
void publishVisualizationCallback(const ros::TimerEvent &event);
void publishOctomapCallback(const ros::TimerEvent &event);
void odom_callback(const nav_msgs::Odometry::ConstPtr &msg);
void pointcloud_callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud);

// ================== 回调函数 ==================
void odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    auv_position.x = msg->pose.pose.position.x;
    auv_position.y = msg->pose.pose.position.y;
    auv_position.z = msg->pose.pose.position.z;
}

// 【核心修改】使用 OctoMap API 更新地图
void pointcloud_callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
{
    if (!octree) // 确保八叉树已初始化
    {
        ROS_WARN_THROTTLE(5.0, "Octree not initialized yet.");
        return;
    }
    // 检查是否有有效的 AUV 位置 (使用 isnan 检查初始化值)
    if (std::isnan(auv_position.x) || std::isnan(auv_position.y) || std::isnan(auv_position.z))
    {
        ROS_WARN_THROTTLE(5.0, "AUV position not yet available or invalid.");
        return;
    }

    // --- 开始修改 ---
    // 1. 将 PCL 点云转换为 OctoMap 点云
    octomap::Pointcloud octo_cloud;
    octo_cloud.reserve(cloud->points.size()); // 预分配内存提高效率

    for (const auto &pt : cloud->points)
    {
        // 可选: 在这里添加基于距离或其他条件的点过滤
        double range_sq = pow(pt.x - auv_position.x, 2) +
                          pow(pt.y - auv_position.y, 2) +
                          pow(pt.z - auv_position.z, 2);
        if (range_sq < MIN_SONAR_RANGE * MIN_SONAR_RANGE || range_sq > MAX_SONAR_RANGE * MAX_SONAR_RANGE)
        {
            continue; // 忽略过近或过远的点
        }
        octo_cloud.push_back(pt.x, pt.y, pt.z);
    }

    // 2. 获取传感器原点 (AUV 位置)
    octomap::point3d sensor_origin(auv_position.x, auv_position.y, auv_position.z);

    // 3. 将点云数据插入到八叉树中
    //    参数: 点云, 传感器原点, 最大探测距离 (用于射线投射), 是否懒惰评估(lazy evaluation, 通常true)
    //    该函数会自动处理射线路径上的空闲更新和击中点的占据更新
    octree->insertPointCloud(octo_cloud, sensor_origin, MAX_SONAR_RANGE, true);

    // 注意： prune() 操作比较耗时，不建议在每次点云回调中都调用。
    // 可以考虑在发布地图的回调中定期调用。
    // octree->prune();
    // --- 结束修改 ---
    
    // 统计占据节点数量用于调试
    size_t occupied_count = 0;
    for (octomap::OcTree::leaf_iterator it = octree->begin_leafs(), end = octree->end_leafs(); it != end; ++it)
    {
        if (octree->isNodeOccupied(*it))
        {
            occupied_count++;
        }
    }
    
    ROS_INFO_THROTTLE(2.0, "PointCloud processed: %zu points inserted, Octree size: %zu nodes, Occupied: %zu", 
                      octo_cloud.size(), octree->size(), occupied_count);
}

// 【核心修改】从 Octree 生成可视化标记
// 强烈建议：使用 RViz 的 OctoMap 插件直接可视化 /a9/octomap 话题，效果更好。
// 此函数仅作为备选或调试使用。
void publishVisualizationCallback(const ros::TimerEvent &event)
{
    if (!octree || octree->size() <= 1)
        return; // 检查树是否有效且非空

    visualization_msgs::MarkerArray marker_array;

    // 可视化被占据的叶子节点 (CUBE_LIST 效率较高)
    visualization_msgs::Marker occupied_marker;
    occupied_marker.header.frame_id = "world"; // 确保坐标系正确
    occupied_marker.header.stamp = ros::Time::now();
    occupied_marker.ns = "octomap_occupied_viz"; // 改个名字区分
    occupied_marker.id = 0;
    occupied_marker.type = visualization_msgs::Marker::CUBE_LIST;
    occupied_marker.action = visualization_msgs::Marker::ADD;
    occupied_marker.pose.orientation.w = 1.0;
    // CUBE_LIST 的 scale 定义了每个 cube 的边长，但 OctoMap 叶子大小不一
    // 这里简单使用基础分辨率，或者可以为不同层级的节点发布不同的 Marker
    occupied_marker.scale.x = octree->getResolution();
    occupied_marker.scale.y = octree->getResolution();
    occupied_marker.scale.z = octree->getResolution();

    // 遍历所有叶子节点
    size_t occupied_count = 0;
    double min_z = std::numeric_limits<double>::max();
    double max_z = std::numeric_limits<double>::lowest();
    
    for (octomap::OcTree::leaf_iterator it = octree->begin_leafs(), end = octree->end_leafs(); it != end; ++it)
    {
        if (octree->isNodeOccupied(*it)) // 检查节点是否被占据
        {
            geometry_msgs::Point p;
            p.x = it.getX(); // 获取节点中心坐标
            p.y = it.getY();
            p.z = it.getZ();
            occupied_marker.points.push_back(p);
            
            // 统计 Z 轴范围
            min_z = std::min(min_z, p.z);
            max_z = std::max(max_z, p.z);
            occupied_count++;

            // 可以根据占据概率或其他信息设置颜色
            std_msgs::ColorRGBA color;
            float prob = it->getOccupancy(); // 获取占据概率 (0 到 1)
            color.r = prob;                  // 越红越可能被占据
            color.g = 1.0 - prob;
            color.b = 0.0;
            color.a = 0.8; // 设置透明度
            occupied_marker.colors.push_back(color);

            // 如果需要可视化不同大小的叶子，需要创建多个 Marker 或使用 Marker::CUBE
            // 并设置 marker.scale.x = it.getSize(); 等
        }
        // else if (octree->isNodeKnown(*it)) { // 可选：可视化空闲节点 (会非常多)
        //    ... (类似逻辑添加空闲节点到另一个 marker)
        // }
    }
    
    // 输出调试信息
    if (occupied_count > 0)
    {
        ROS_INFO_THROTTLE(5.0, "Visualization: %zu occupied voxels, Z range: [%.2f, %.2f]", 
                          occupied_count, min_z, max_z);
    }

    if (!occupied_marker.points.empty())
    {
        marker_array.markers.push_back(occupied_marker);
    }
    else
    {
        // 如果没有被占据的单元格，发送删除消息清理之前的显示
        occupied_marker.action = visualization_msgs::Marker::DELETEALL;
        marker_array.markers.push_back(occupied_marker);
    }

    // 可视化地图边界 (保持不变)
    visualization_msgs::Marker boundary;
    boundary.header.frame_id = "world";
    boundary.header.stamp = ros::Time::now();
    boundary.ns = "map_boundary";
    boundary.id = 1;
    boundary.type = visualization_msgs::Marker::CUBE;
    boundary.action = visualization_msgs::Marker::ADD;
    boundary.pose.position.x = MAP_ORIGIN_X + MAP_WIDTH_METERS / 2.0;
    boundary.pose.position.y = MAP_ORIGIN_Y + MAP_HEIGHT_METERS / 2.0;
    boundary.pose.position.z = MAP_ORIGIN_Z + MAP_DEPTH_METERS / 2.0;
    boundary.pose.orientation.w = 1.0;
    boundary.scale.x = MAP_WIDTH_METERS;
    boundary.scale.y = MAP_HEIGHT_METERS;
    boundary.scale.z = MAP_DEPTH_METERS;
    boundary.color.r = 1.0;
    boundary.color.g = 1.0;
    boundary.color.b = 1.0;
    boundary.color.a = 0.1; // 更透明
    marker_array.markers.push_back(boundary);

    map_viz_pub.publish(marker_array);
}

// 【核心修改】直接从核心 Octree 对象发布消息
void publishOctomapCallback(const ros::TimerEvent &event)
{
    // --- 在函数入口打印 ---
    ROS_INFO("publishOctomapCallback called at time %.2f", event.current_real.toSec());

    if (!octree)
    {
        ROS_WARN("Octree pointer is null. Cannot publish map.");
        return;
    }

    // --- 打印当前 Octree 的节点数 ---
    size_t tree_size = octree->size();
    ROS_INFO("Current Octree size: %zu nodes.", tree_size);

    if (tree_size <= 1)
    { // 只有根节点，认为是空的
        ROS_WARN("Octree is empty or only contains root node. Not publishing map.");
        return; // 不发布空地图
    }

    // 可选: 压缩
    // octree->prune();
    // ROS_INFO("Octree pruned. Size after prune: %zu", octree->size());

    octomap_msgs::Octomap map_msg;
    map_msg.header.frame_id = "world";
    map_msg.header.stamp = ros::Time::now();

    // --- 检查序列化是否成功 ---
    bool success = octomap_msgs::fullMapToMsg(*octree, map_msg);

    if (success)
    {
        // --- 确认即将发布的消息不是空的 ---
        ROS_INFO("Successfully serialized OctoMap with %zu nodes. Publishing message (data size: %zu bytes)...",
                 octree->size(), map_msg.data.size());
        octomap_pub.publish(map_msg);
    }
    else
    {
        // --- 如果序列化失败，打印错误 ---
        ROS_ERROR("Failed to serialize OctoMap to message!");
    }
}

// ================== 主函数 ==================
int main(int argc, char **argv)
{
    ros::init(argc, argv, "grid_map_node_octomap"); // 建议改个名字区分
    ros::NodeHandle nh;

    // --- 核心修改：初始化 Octree 对象 ---
    octree = std::make_unique<octomap::OcTree>(MAP_RESOLUTION);

    // 设置 OctoMap 的概率模型参数，使其与之前的 Log-Odds 参数行为一致
    // P(occ) = 1 - 1 / (1 + exp(log_odds))
    octree->setProbHit(1.0f - 1.0f / (1.0f + exp(LOG_ODDS_OCC)));   // 对应 LOG_ODDS_OCC
    octree->setProbMiss(1.0f - 1.0f / (1.0f + exp(LOG_ODDS_FREE))); // 对应 LOG_ODDS_FREE
    // --- 开始修正 ---
    // 计算对应的概率阈值
    double probability_min = 1.0 - 1.0 / (1.0 + exp(LOG_ODDS_MIN));
    double probability_max = 1.0 - 1.0 / (1.0 + exp(LOG_ODDS_MAX));

    // 使用概率值设置阈值
    octree->setClampingThresMin(probability_min); // 设置概率下限
    octree->setClampingThresMax(probability_max); // 设置概率上限
    
    // 设置占据阈值（提高阈值以减少误报，提高精确率）
    octree->setOccupancyThres(0.6);  // 从 0.5 提高到 0.6，减少误报
    
    // --- 结束修正 ---
    ROS_INFO("OctoMap Grid Map Node initialized with resolution %.2f", octree->getResolution());
    ROS_INFO("  Prob Hit: %.3f (LogOdds: %.2f)", octree->getProbHit(), octree->getProbHitLog());
    ROS_INFO("  Prob Miss: %.3f (LogOdds: %.2f)", octree->getProbMiss(), octree->getProbMissLog());
    ROS_INFO("  Clamping Min: %.3f (LogOdds: %.2f)", octree->getClampingThresMin(), octree->getClampingThresMinLog());
    ROS_INFO("  Clamping Max: %.3f (LogOdds: %.2f)", octree->getClampingThresMax(), octree->getClampingThresMaxLog());
    ROS_INFO("  Occupancy Threshold: %.3f", octree->getOccupancyThres());
    // --- 结束修改 ---

    // 订阅 AUV 位姿和声呐点云
    ros::Subscriber odom_sub = nh.subscribe("/a9/pose_gt", 10, odom_callback);
    ros::Subscriber cloud_sub = nh.subscribe("/a9/my_sonar_points", 1, pointcloud_callback); // 确保与 sonar_sensor.cpp 的发布话题一致

    // ...
    octomap_pub = nh.advertise<octomap_msgs::Octomap>("/a9/octomap", 1);
    map_viz_pub = nh.advertise<visualization_msgs::MarkerArray>("/a9/octomap_markers", 1);

    // --- 在创建定时器后立刻打印 ---
    ROS_INFO("Creating Octomap publishing timer (1 Hz)...");
    ros::Timer octomap_timer = nh.createTimer(ros::Duration(1.0), publishOctomapCallback); // 1 Hz
    ROS_INFO("Octomap publishing timer created!");

    // (可视化定时器暂时可以保留或注释掉，先聚焦Octomap发布)
    ros::Timer viz_timer = nh.createTimer(ros::Duration(0.5), publishVisualizationCallback); // 2 Hz

    ROS_INFO("OctoMap Grid Map Node (Real Octree Core) started...");
    ros::spin(); // 进入 ROS 事件循环
    return 0;
    // ...
}
// #include <ros/ros.h>
// #include <sensor_msgs/PointCloud2.h>
// #include <pcl_ros/point_cloud.h>
// #include <pcl/point_types.h>
// #include <visualization_msgs/MarkerArray.h>
// #include <nav_msgs/Odometry.h>
// #include <vector>
// #include <cmath>

// // 引入OctoMap相关的头文件
// #include <octomap/octomap.h>
// #include <octomap_msgs/Octomap.h>
// #include <octomap_msgs/conversions.h>

// // ================== 地图参数定义 ==================
// const double MAP_RESOLUTION = 0.5;
// const double MAP_WIDTH_METERS = 100.0;
// const double MAP_HEIGHT_METERS = 100.0;
// const double MAP_DEPTH_METERS = 40.0;
// const int MAP_SIZE_X = static_cast<int>(ceil(MAP_WIDTH_METERS / MAP_RESOLUTION));
// const int MAP_SIZE_Y = static_cast<int>(ceil(MAP_HEIGHT_METERS / MAP_RESOLUTION));
// const int MAP_SIZE_Z = static_cast<int>(ceil(MAP_DEPTH_METERS / MAP_RESOLUTION));
// const double MAP_ORIGIN_X = -MAP_WIDTH_METERS / 2.0;
// const double MAP_ORIGIN_Y = -MAP_HEIGHT_METERS / 2.0;
// const double MAP_ORIGIN_Z = -MAP_DEPTH_METERS;

// // ================== 贝叶斯更新参数 (Log-Odds形式) ==================
// const float LOG_ODDS_OCC = 0.9f;
// const float LOG_ODDS_FREE = -0.4f;
// const float LOG_ODDS_MAX = 5.0f;
// const float LOG_ODDS_MIN = -5.0f;

// // ================== 全局变量 ==================
// std::vector<float> occupancy_grid;
// ros::Publisher map_viz_pub;
// ros::Publisher octomap_pub;
// pcl::PointXYZ auv_position;

// // ================== 函数声明 (前向声明) ==================
// void publishVisualizationCallback(const ros::TimerEvent &event);
// void publishOctomapCallback(const ros::TimerEvent &event);

// // ================== 工具函数 ==================
// inline bool inBoundsIdx(int x, int y, int z)
// {
//     return (x >= 0 && x < MAP_SIZE_X && y >= 0 && y < MAP_SIZE_Y && z >= 0 && z < MAP_SIZE_Z);
// }

// inline void addLogOddsSafe(int x, int y, int z, float delta)
// {
//     if (!inBoundsIdx(x, y, z))
//         return;
//     size_t idx = static_cast<size_t>(x) + static_cast<size_t>(MAP_SIZE_X) * (static_cast<size_t>(y) + static_cast<size_t>(MAP_SIZE_Y) * static_cast<size_t>(z));
//     if (idx >= occupancy_grid.size())
//         return;
//     occupancy_grid[idx] += delta;
//     if (occupancy_grid[idx] > LOG_ODDS_MAX)
//         occupancy_grid[idx] = LOG_ODDS_MAX;
//     if (occupancy_grid[idx] < LOG_ODDS_MIN)
//         occupancy_grid[idx] = LOG_ODDS_MIN;
// }

// // 【健壮版】Bresenham's 3D line algorithm
// void raycast(int x0, int y0, int z0, int x1, int y1, int z1)
// {
//     int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
//     int dy = abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
//     int dz = abs(z1 - z0), sz = z0 < z1 ? 1 : -1;

//     int i = 0, x = x0, y = y0, z = z0;
//     int i_max = dx + dy + dz;

//     if (dx >= dy && dx >= dz)
//     { // X-axis dominant
//         int p1 = 2 * dy - dx, p2 = 2 * dz - dx;
//         while (i <= i_max)
//         {
//             if (i > 0)
//                 addLogOddsSafe(x, y, z, LOG_ODDS_FREE);
//             if (x == x1)
//                 break;
//             x += sx;
//             if (p1 >= 0)
//             {
//                 y += sy;
//                 p1 -= 2 * dx;
//             }
//             if (p2 >= 0)
//             {
//                 z += sz;
//                 p2 -= 2 * dx;
//             }
//             p1 += 2 * dy;
//             p2 += 2 * dz;
//             i++;
//         }
//     }
//     else if (dy >= dx && dy >= dz)
//     { // Y-axis dominant
//         int p1 = 2 * dx - dy, p2 = 2 * dz - dy;
//         while (i <= i_max)
//         {
//             if (i > 0)
//                 addLogOddsSafe(x, y, z, LOG_ODDS_FREE);
//             if (y == y1)
//                 break;
//             y += sy;
//             if (p1 >= 0)
//             {
//                 x += sx;
//                 p1 -= 2 * dy;
//             }
//             if (p2 >= 0)
//             {
//                 z += sz;
//                 p2 -= 2 * dy;
//             }
//             p1 += 2 * dx;
//             p2 += 2 * dz;
//             i++;
//         }
//     }
//     else
//     { // Z-axis dominant
//         int p1 = 2 * dy - dz, p2 = 2 * dx - dz;
//         while (i <= i_max)
//         {
//             if (i > 0)
//                 addLogOddsSafe(x, y, z, LOG_ODDS_FREE);
//             if (z == z1)
//                 break;
//             z += sz;
//             if (p1 >= 0)
//             {
//                 y += sy;
//                 p1 -= 2 * dz;
//             }
//             if (p2 >= 0)
//             {
//                 x += sx;
//                 p2 -= 2 * dz;
//             }
//             p1 += 2 * dy;
//             p2 += 2 * dx;
//             i++;
//         }
//     }
// }

// bool world_to_grid(double world_x, double world_y, double world_z, int &grid_ix, int &grid_iy, int &grid_iz)
// {
//     grid_ix = static_cast<int>((world_x - MAP_ORIGIN_X) / MAP_RESOLUTION);
//     grid_iy = static_cast<int>((world_y - MAP_ORIGIN_Y) / MAP_RESOLUTION);
//     grid_iz = static_cast<int>((world_z - MAP_ORIGIN_Z) / MAP_RESOLUTION);
//     return inBoundsIdx(grid_ix, grid_iy, grid_iz);
// }

// // ================== 回调函数 ==================
// void odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
// {
//     auv_position.x = msg->pose.pose.position.x;
//     auv_position.y = msg->pose.pose.position.y;
//     auv_position.z = msg->pose.pose.position.z;
// }

// // 【高频】只负责更新地图数据，不负责可视化
// void pointcloud_callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
// {
//     int start_ix, start_iy, start_iz;
//     if (!world_to_grid(auv_position.x, auv_position.y, auv_position.z, start_ix, start_iy, start_iz))
//         return;

//     for (const auto &point : cloud->points)
//     {
//         int end_ix, end_iy, end_iz;
//         if (world_to_grid(point.x, point.y, point.z, end_ix, end_iy, end_iz))
//         {
//             addLogOddsSafe(end_ix, end_iy, end_iz, LOG_ODDS_OCC);
//             raycast(start_ix, start_iy, start_iz, end_ix, end_iy, end_iz);
//         }
//     }
// }

// // 【低频】负责发布RViz可视化消息
// void publishVisualizationCallback(const ros::TimerEvent &event)
// {
//     visualization_msgs::MarkerArray marker_array;
//     visualization_msgs::Marker marker;
//     marker.header.frame_id = "world";
//     marker.header.stamp = ros::Time::now();
//     marker.ns = "occupancy_grid";
//     marker.id = 0;
//     marker.type = visualization_msgs::Marker::CUBE_LIST;
//     marker.action = visualization_msgs::Marker::ADD;
//     marker.pose.orientation.w = 1.0;
//     marker.scale.x = MAP_RESOLUTION;
//     marker.scale.y = MAP_RESOLUTION;
//     marker.scale.z = MAP_RESOLUTION;

//     for (int i = 0; i < MAP_SIZE_X; ++i)
//     {
//         for (int j = 0; j < MAP_SIZE_Y; ++j)
//         {
//             for (int k = 0; k < MAP_SIZE_Z; ++k)
//             {
//                 size_t index = i + MAP_SIZE_X * (j + MAP_SIZE_Y * k);
//                 if (occupancy_grid[index] > 0.0)
//                 {
//                     geometry_msgs::Point p;
//                     p.x = MAP_ORIGIN_X + (i + 0.5) * MAP_RESOLUTION;
//                     p.y = MAP_ORIGIN_Y + (j + 0.5) * MAP_RESOLUTION;
//                     p.z = MAP_ORIGIN_Z + (k + 0.5) * MAP_RESOLUTION;
//                     marker.points.push_back(p);

//                     std_msgs::ColorRGBA color;
//                     float prob = 1.0 - 1.0 / (1.0 + exp(occupancy_grid[index]));
//                     color.r = prob;
//                     color.g = 1.0 - prob;
//                     color.b = 0.0;
//                     color.a = 0.6;
//                     marker.colors.push_back(color);
//                 }
//             }
//         }
//     }

//     if (marker.points.empty())
//     {
//         marker.action = visualization_msgs::Marker::DELETEALL;
//     }

//     marker_array.markers.push_back(marker);

//     // ========= 新增: 边界可视化 CUBE =========
//     visualization_msgs::Marker boundary;
//     boundary.header.frame_id = "world";
//     boundary.header.stamp = ros::Time::now();
//     boundary.ns = "map_boundary";
//     boundary.id = 1;
//     boundary.type = visualization_msgs::Marker::CUBE;
//     boundary.action = visualization_msgs::Marker::ADD;

//     // 地图中心
//     boundary.pose.position.x = MAP_ORIGIN_X + MAP_WIDTH_METERS / 2.0;
//     boundary.pose.position.y = MAP_ORIGIN_Y + MAP_HEIGHT_METERS / 2.0;
//     boundary.pose.position.z = MAP_ORIGIN_Z + MAP_DEPTH_METERS / 2.0;
//     boundary.pose.orientation.w = 1.0;

//     // 尺寸
//     boundary.scale.x = MAP_WIDTH_METERS;
//     boundary.scale.y = MAP_HEIGHT_METERS;
//     boundary.scale.z = MAP_DEPTH_METERS;

//     // 半透明外框（白色）
//     boundary.color.r = 1.0;
//     boundary.color.g = 1.0;
//     boundary.color.b = 1.0;
//     boundary.color.a = 0.15;

//     marker_array.markers.push_back(boundary);

//     map_viz_pub.publish(marker_array);
// }

// // 【低频】负责发布程序可读的OctoMap消息
// void publishOctomapCallback(const ros::TimerEvent &event)
// {
//     octomap::OcTree tree(MAP_RESOLUTION);

//     for (int i = 0; i < MAP_SIZE_X; ++i)
//     {
//         for (int j = 0; j < MAP_SIZE_Y; ++j)
//         {
//             for (int k = 0; k < MAP_SIZE_Z; ++k)
//             {
//                 size_t index = i + MAP_SIZE_X * (j + MAP_SIZE_Y * k);
//                 if (occupancy_grid[index] != 0.0)
//                 {
//                     double world_x = MAP_ORIGIN_X + (i + 0.5) * MAP_RESOLUTION;
//                     double world_y = MAP_ORIGIN_Y + (j + 0.5) * MAP_RESOLUTION;
//                     double world_z = MAP_ORIGIN_Z + (k + 0.5) * MAP_RESOLUTION;
//                     tree.updateNode(octomap::point3d(world_x, world_y, world_z), occupancy_grid[index]);
//                 }
//             }
//         }
//     }

//     octomap_msgs::Octomap map_msg;
//     map_msg.header.frame_id = "world";
//     map_msg.header.stamp = ros::Time::now();
//     if (octomap_msgs::fullMapToMsg(tree, map_msg))
//     {
//         octomap_pub.publish(map_msg);
//     }
//     else
//     {
//         ROS_ERROR("Error serializing OctoMap");
//     }
// }

// // ================== 主函数 ==================
// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "grid_map_node");
//     ros::NodeHandle nh;

//     occupancy_grid.assign(MAP_SIZE_X * MAP_SIZE_Y * MAP_SIZE_Z, 0.0f);
//     ROS_INFO("全局栅格地图已创建，总栅格数: %zu", occupancy_grid.size());

//     ros::Subscriber odom_sub = nh.subscribe("/a9/pose_gt", 10, odom_callback);
//     ros::Subscriber cloud_sub = nh.subscribe("/a9/my_sonar_points", 1, pointcloud_callback);

//     map_viz_pub = nh.advertise<visualization_msgs::MarkerArray>("/a9/occupancy_map_viz", 1);
//     octomap_pub = nh.advertise<octomap_msgs::Octomap>("/a9/octomap", 1);

//     ros::Timer viz_timer = nh.createTimer(ros::Duration(1.0), publishVisualizationCallback);
//     ros::Timer octomap_timer = nh.createTimer(ros::Duration(2.0), publishOctomapCallback);

//     ROS_INFO("三维栅格地图节点（OctoMap版）已启动...");
//     ros::spin();
//     return 0;
// }
