/**
 * @file coverage_planner.h
 * @brief 覆盖路径规划器 - 生成弓字形扫描路径
 */

#ifndef COVERAGE_PLANNER_H
#define COVERAGE_PLANNER_H

#include <vector>
#include <cmath>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>

namespace uuv_project {

/**
 * @brief 航路点结构
 */
struct Waypoint {
    double x;
    double y;
    double z;
    
    Waypoint(double x_ = 0.0, double y_ = 0.0, double z_ = 0.0)
        : x(x_), y(y_), z(z_) {}
};

/**
 * @brief 覆盖路径规划器类
 */
class CoveragePlanner {
public:
    /**
     * @brief 构造函数
     * @param map_origin_x 地图原点 X 坐标
     * @param map_origin_y 地图原点 Y 坐标
     * @param map_origin_z 地图原点 Z 坐标
     * @param map_width 地图宽度（X 方向）
     * @param map_height 地图高度（Y 方向）
     * @param map_depth 地图深度（Z 方向）
     */
    CoveragePlanner(double map_origin_x, double map_origin_y, double map_origin_z,
                   double map_width, double map_height, double map_depth);
    
    /**
     * @brief 生成覆盖路径
     * @param sonar_h_fov 声呐水平视场角（弧度）
     * @param sonar_v_fov 声呐垂直视场角（弧度）
     * @param sonar_range 声呐有效探测距离（米）
     * @param overlap_ratio 重叠率（0.0-1.0）
     * @return 航路点列表
     */
    std::vector<Waypoint> generatePath(double sonar_h_fov, double sonar_v_fov,
                                       double sonar_range, double overlap_ratio);
    
    /**
     * @brief 将路径转换为 ROS Path 消息（用于可视化）
     */
    nav_msgs::Path toPathMsg(const std::vector<Waypoint>& waypoints,
                            const std::string& frame_id = "world");
    
    /**
     * @brief 获取路径统计信息
     */
    void printPathInfo(const std::vector<Waypoint>& waypoints);

private:
    /**
     * @brief 对路径进行平滑插值
     * @param key_points 关键点
     * @param point_spacing 插值点间距（米）
     * @return 平滑后的航路点列表
     */
    std::vector<Waypoint> smoothPath(const std::vector<Waypoint>& key_points, 
                                     double point_spacing);
    
    /**
     * @brief 在两点之间进行线性插值
     */
    std::vector<Waypoint> interpolateLinear(const Waypoint& p1, const Waypoint& p2, 
                                           double spacing);
    
    /**
     * @brief 在转角处生成平滑曲线（贝塞尔曲线）
     */
    std::vector<Waypoint> smoothCorner(const Waypoint& p_prev, const Waypoint& p_curr, 
                                      const Waypoint& p_next, double radius, double spacing);

    // 地图参数
    double map_origin_x_;
    double map_origin_y_;
    double map_origin_z_;
    double map_width_;
    double map_height_;
    double map_depth_;
};

} // namespace uuv_project

#endif // COVERAGE_PLANNER_H
