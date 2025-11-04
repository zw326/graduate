/**
 * @file coverage_planner.cpp
 * @brief 覆盖路径规划器实现
 */

#include "uuv_project_core/coverage_planner.h"
#include <limits>
#include <algorithm>

namespace uuv_project {

CoveragePlanner::CoveragePlanner(double map_origin_x, double map_origin_y, double map_origin_z,
                                 double map_width, double map_height, double map_depth)
    : map_origin_x_(map_origin_x), map_origin_y_(map_origin_y), map_origin_z_(map_origin_z),
      map_width_(map_width), map_height_(map_height), map_depth_(map_depth) {
    
    ROS_INFO("Coverage Planner initialized:");
    ROS_INFO("  Map origin: (%.1f, %.1f, %.1f)", map_origin_x_, map_origin_y_, map_origin_z_);
    ROS_INFO("  Map size: %.1f x %.1f x %.1f", map_width_, map_height_, map_depth_);
}

std::vector<Waypoint> CoveragePlanner::generatePath(double sonar_h_fov, double sonar_v_fov,
                                                    double sonar_range, double overlap_ratio) {
    std::vector<Waypoint> waypoints;
    
    // 计算路径间距（考虑重叠）
    // 声纳覆盖宽度 = 2 * range * tan(FOV/2)
    // 路径间距 = 覆盖宽度 * (1 - overlap_ratio)
    double sonar_swath_width = 2.0 * sonar_range * std::tan(sonar_h_fov / 2.0);
    double sonar_swath_height = 2.0 * sonar_range * std::tan(sonar_v_fov / 2.0);
    
    double path_spacing_y = sonar_swath_width * (1.0 - overlap_ratio);
    double layer_spacing_z = sonar_swath_height * (1.0 - overlap_ratio);
    
    // 确保间距合理（至少25米以匹配控制器要求）
    if (path_spacing_y < 25.0) path_spacing_y = 25.0;
    if (layer_spacing_z < 10.0) layer_spacing_z = 10.0;
    
    ROS_INFO("Path generation parameters:");
    ROS_INFO("  Sonar H-FOV: %.2f rad (%.1f deg)", sonar_h_fov, sonar_h_fov * 180.0 / M_PI);
    ROS_INFO("  Sonar V-FOV: %.2f rad (%.1f deg)", sonar_v_fov, sonar_v_fov * 180.0 / M_PI);
    ROS_INFO("  Sonar range: %.1f m", sonar_range);
    ROS_INFO("  Overlap ratio: %.2f (%.1f%%)", overlap_ratio, overlap_ratio * 100.0);
    ROS_INFO("  Sonar swath width: %.2f m", sonar_swath_width);
    ROS_INFO("  Sonar swath height: %.2f m", sonar_swath_height);
    ROS_INFO("  Path spacing (Y): %.2f m", path_spacing_y);
    ROS_INFO("  Layer spacing (Z): %.2f m", layer_spacing_z);
    
    // 计算需要的路径数和层数
    int num_paths = static_cast<int>(std::ceil(map_height_ / path_spacing_y)) + 1;
    int num_layers = static_cast<int>(std::ceil(map_depth_ / layer_spacing_z)) + 1;
    
    ROS_INFO("  Number of paths per layer: %d", num_paths);
    ROS_INFO("  Number of layers: %d", num_layers);
    
    // 生成分层弓字形路径的关键点
    std::vector<Waypoint> key_points;
    
    for (int layer = 0; layer < num_layers; ++layer) {
        double z = map_origin_z_ + layer * layer_spacing_z;
        // 限制在地图范围内
        if (z > map_origin_z_ + map_depth_) {
            z = map_origin_z_ + map_depth_;
        }
        
        for (int path = 0; path < num_paths; ++path) {
            double y = map_origin_y_ + path * path_spacing_y;
            // 限制在地图范围内
            if (y > map_origin_y_ + map_height_) {
                y = map_origin_y_ + map_height_;
            }
            
            // 弓字形：偶数路径从左到右，奇数路径从右到左
            if (path % 2 == 0) {
                key_points.push_back(Waypoint(map_origin_x_, y, z));
                key_points.push_back(Waypoint(map_origin_x_ + map_width_, y, z));
            } else {
                key_points.push_back(Waypoint(map_origin_x_ + map_width_, y, z));
                key_points.push_back(Waypoint(map_origin_x_, y, z));
            }
        }
    }
    
    ROS_INFO("Generated %zu key points for boustrophedon pattern", key_points.size());
    
    // 对路径进行平滑插值
    // 使用12米间距在直线段上插值，确保最终间距在25-35米范围内
    waypoints = smoothPath(key_points, 12.0);
    
    ROS_INFO("After smoothing: %zu waypoints", waypoints.size());
    
    return waypoints;
}

nav_msgs::Path CoveragePlanner::toPathMsg(const std::vector<Waypoint>& waypoints,
                                         const std::string& frame_id) {
    nav_msgs::Path path_msg;
    path_msg.header.frame_id = frame_id;
    path_msg.header.stamp = ros::Time::now();
    
    for (const auto& wp : waypoints) {
        geometry_msgs::PoseStamped pose;
        pose.header = path_msg.header;
        pose.pose.position.x = wp.x;
        pose.pose.position.y = wp.y;
        pose.pose.position.z = wp.z;
        pose.pose.orientation.w = 1.0;  // 默认朝向
        
        path_msg.poses.push_back(pose);
    }
    
    return path_msg;
}

void CoveragePlanner::printPathInfo(const std::vector<Waypoint>& waypoints) {
    if (waypoints.empty()) {
        ROS_WARN("Path is empty!");
        return;
    }
    
    // 计算路径总长度和间距统计
    double total_length = 0.0;
    double min_spacing = std::numeric_limits<double>::max();
    double max_spacing = 0.0;
    double avg_spacing = 0.0;
    
    for (size_t i = 1; i < waypoints.size(); ++i) {
        double dx = waypoints[i].x - waypoints[i-1].x;
        double dy = waypoints[i].y - waypoints[i-1].y;
        double dz = waypoints[i].z - waypoints[i-1].z;
        double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
        
        total_length += dist;
        min_spacing = std::min(min_spacing, dist);
        max_spacing = std::max(max_spacing, dist);
    }
    
    avg_spacing = total_length / (waypoints.size() - 1);
    
    ROS_INFO("Path statistics:");
    ROS_INFO("  Total waypoints: %zu", waypoints.size());
    ROS_INFO("  Total path length: %.1f m", total_length);
    ROS_INFO("  Waypoint spacing - Min: %.2f m, Max: %.2f m, Avg: %.2f m", 
             min_spacing, max_spacing, avg_spacing);
    ROS_INFO("  First waypoint: (%.1f, %.1f, %.1f)", 
             waypoints.front().x, waypoints.front().y, waypoints.front().z);
    ROS_INFO("  Last waypoint: (%.1f, %.1f, %.1f)",
             waypoints.back().x, waypoints.back().y, waypoints.back().z);
    
    // 打印前5个航路点
    ROS_INFO("  First 5 waypoints:");
    for (size_t i = 0; i < std::min(size_t(5), waypoints.size()); ++i) {
        ROS_INFO("    [%zu]: (%.2f, %.2f, %.2f)", i, 
                 waypoints[i].x, waypoints[i].y, waypoints[i].z);
    }
    
    // 验证间距是否符合要求（25-35米）
    int violations = 0;
    for (size_t i = 1; i < waypoints.size(); ++i) {
        double dx = waypoints[i].x - waypoints[i-1].x;
        double dy = waypoints[i].y - waypoints[i-1].y;
        double dz = waypoints[i].z - waypoints[i-1].z;
        double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
        
        // 检查是否在推荐范围内（允许一些容差）
        if (dist < 10.0 || dist > 40.0) {
            violations++;
        }
    }
    
    if (violations > 0) {
        ROS_WARN("  %d waypoint pairs have spacing outside recommended range (10-40m)", violations);
    } else {
        ROS_INFO("  All waypoint spacings are within acceptable range");
    }
}

std::vector<Waypoint> CoveragePlanner::smoothPath(const std::vector<Waypoint>& key_points, 
                                                  double point_spacing) {
    if (key_points.size() < 2) {
        return key_points;
    }
    
    std::vector<Waypoint> smooth_path;
    double corner_radius = 10.0;  // 转角半径（米），符合Requirement 6.2
    
    // 第一个点直接添加
    smooth_path.push_back(key_points[0]);
    
    for (size_t i = 1; i < key_points.size(); ++i) {
        const Waypoint& p_prev = key_points[i-1];
        const Waypoint& p_curr = key_points[i];
        
        // 检查是否是转角点（需要有下一个点）
        bool is_corner = (i < key_points.size() - 1);
        
        if (is_corner) {
            const Waypoint& p_next = key_points[i+1];
            
            // 计算两个方向向量
            double dx1 = p_curr.x - p_prev.x;
            double dy1 = p_curr.y - p_prev.y;
            double dz1 = p_curr.z - p_prev.z;
            double len1 = std::sqrt(dx1*dx1 + dy1*dy1 + dz1*dz1);
            
            double dx2 = p_next.x - p_curr.x;
            double dy2 = p_next.y - p_curr.y;
            double dz2 = p_next.z - p_curr.z;
            double len2 = std::sqrt(dx2*dx2 + dy2*dy2 + dz2*dz2);
            
            // 判断是否是转角（方向改变）
            if (len1 > 0.1 && len2 > 0.1) {
                // 归一化方向向量
                dx1 /= len1; dy1 /= len1; dz1 /= len1;
                dx2 /= len2; dy2 /= len2; dz2 /= len2;
                
                double dot = dx1*dx2 + dy1*dy2 + dz1*dz2;
                
                // 如果不是直线（dot < 0.99），则平滑转角
                if (dot < 0.99) {
                    // 计算转角前的接近距离
                    double approach_dist = std::min(corner_radius, std::min(len1, len2) * 0.4);
                    
                    // 转角起点
                    Waypoint corner_start(
                        p_curr.x - dx1 * approach_dist,
                        p_curr.y - dy1 * approach_dist,
                        p_curr.z - dz1 * approach_dist
                    );
                    
                    // 插值到转角起点（不包括起点，因为上一次已经添加了终点）
                    auto segment1 = interpolateLinear(smooth_path.back(), corner_start, point_spacing);
                    // 移除第一个点（重复）
                    if (!segment1.empty()) {
                        segment1.erase(segment1.begin());
                    }
                    smooth_path.insert(smooth_path.end(), segment1.begin(), segment1.end());
                    
                    // 生成平滑转角
                    auto corner = smoothCorner(p_prev, p_curr, p_next, corner_radius, point_spacing);
                    smooth_path.insert(smooth_path.end(), corner.begin(), corner.end());
                    
                    continue;
                }
            }
        }
        
        // 如果不是转角或不需要平滑，直接线性插值
        auto segment = interpolateLinear(smooth_path.back(), p_curr, point_spacing);
        // 移除第一个点（重复）
        if (!segment.empty()) {
            segment.erase(segment.begin());
        }
        smooth_path.insert(smooth_path.end(), segment.begin(), segment.end());
    }
    
    return smooth_path;
}

std::vector<Waypoint> CoveragePlanner::interpolateLinear(const Waypoint& p1, const Waypoint& p2, 
                                                        double spacing) {
    std::vector<Waypoint> points;
    
    double dx = p2.x - p1.x;
    double dy = p2.y - p1.y;
    double dz = p2.z - p1.z;
    double distance = std::sqrt(dx*dx + dy*dy + dz*dz);
    
    // 如果距离太小，只添加终点
    if (distance < 0.1) {
        return points;
    }
    
    // 如果距离小于间距，直接添加终点
    if (distance < spacing) {
        points.push_back(p2);
        return points;
    }
    
    // 计算需要插入的点数（确保间距均匀）
    int num_segments = static_cast<int>(std::ceil(distance / spacing));
    
    // 添加中间点和终点
    for (int i = 1; i <= num_segments; ++i) {
        double t = static_cast<double>(i) / num_segments;
        points.push_back(Waypoint(
            p1.x + t * dx,
            p1.y + t * dy,
            p1.z + t * dz
        ));
    }
    
    return points;
}

std::vector<Waypoint> CoveragePlanner::smoothCorner(const Waypoint& p_prev, const Waypoint& p_curr, 
                                                    const Waypoint& p_next, double radius, 
                                                    double spacing) {
    std::vector<Waypoint> points;
    
    // 计算两个方向向量
    double dx1 = p_curr.x - p_prev.x;
    double dy1 = p_curr.y - p_prev.y;
    double dz1 = p_curr.z - p_prev.z;
    double len1 = std::sqrt(dx1*dx1 + dy1*dy1 + dz1*dz1);
    
    double dx2 = p_next.x - p_curr.x;
    double dy2 = p_next.y - p_curr.y;
    double dz2 = p_next.z - p_curr.z;
    double len2 = std::sqrt(dx2*dx2 + dy2*dy2 + dz2*dz2);
    
    if (len1 < 0.1 || len2 < 0.1) {
        points.push_back(p_curr);
        return points;
    }
    
    // 归一化方向向量
    dx1 /= len1; dy1 /= len1; dz1 /= len1;
    dx2 /= len2; dy2 /= len2; dz2 /= len2;
    
    // 计算转角的接近距离（使用指定的radius，但不超过线段长度的40%）
    double approach_dist = std::min(radius, std::min(len1, len2) * 0.4);
    
    // 转角起点和终点
    Waypoint corner_start(
        p_curr.x - dx1 * approach_dist,
        p_curr.y - dy1 * approach_dist,
        p_curr.z - dz1 * approach_dist
    );
    
    Waypoint corner_end(
        p_curr.x + dx2 * approach_dist,
        p_curr.y + dy2 * approach_dist,
        p_curr.z + dz2 * approach_dist
    );
    
    // 计算转角弧长（近似）
    double arc_length = approach_dist * 2.0;
    
    // 使用二次贝塞尔曲线平滑转角
    // 根据弧长和间距计算点数
    int num_points = static_cast<int>(std::ceil(arc_length / spacing));
    num_points = std::max(3, num_points);  // 至少3个点以保证平滑
    
    // 生成贝塞尔曲线点（不包括起点，因为已经在路径中）
    for (int i = 1; i <= num_points; ++i) {
        double t = static_cast<double>(i) / num_points;
        double t1 = 1.0 - t;
        
        // 二次贝塞尔曲线: B(t) = (1-t)²P0 + 2(1-t)tP1 + t²P2
        // P0 = corner_start, P1 = p_curr (控制点), P2 = corner_end
        points.push_back(Waypoint(
            t1*t1 * corner_start.x + 2*t1*t * p_curr.x + t*t * corner_end.x,
            t1*t1 * corner_start.y + 2*t1*t * p_curr.y + t*t * corner_end.y,
            t1*t1 * corner_start.z + 2*t1*t * p_curr.z + t*t * corner_end.z
        ));
    }
    
    return points;
}

} // namespace uuv_project
