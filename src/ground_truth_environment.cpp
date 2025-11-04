/**
 * @file ground_truth_environment.cpp
 * @brief Ground Truth 环境实现
 */

#include "uuv_project_core/ground_truth_environment.h"
#include <cmath>
#include <ros/ros.h>

namespace uuv_project {

// ==================== 构造函数 ====================
GroundTruthEnvironment::GroundTruthEnvironment() {
    initializeGaussianFeatures();
    initializeObstacles();
    
    ROS_INFO("Ground Truth Environment initialized:");
    ROS_INFO("  - %zu Gaussian features", gaussian_features_.size());
    ROS_INFO("  - %zu Box obstacles", box_obstacles_.size());
    ROS_INFO("  - %zu Cylinder obstacles", cylinder_obstacles_.size());
}

// ==================== 初始化函数 ====================

void GroundTruthEnvironment::initializeGaussianFeatures() {
    // 这些参数必须与 generate_seafloor.py 中的 GAUSSIAN_FEATURES 完全一致
    gaussian_features_.clear();
    
    // 海山 1: 较缓的凸起
    gaussian_features_.emplace_back(20.0, 20.0, 8.0, 12.0);
    
    // 海山 2: 较陡的凸起
    gaussian_features_.emplace_back(-25.0, -20.0, 6.0, 8.0);
    
    // 海沟 1: 凹陷
    gaussian_features_.emplace_back(-15.0, 25.0, -5.0, 10.0);
}

void GroundTruthEnvironment::initializeObstacles() {
    box_obstacles_.clear();
    cylinder_obstacles_.clear();
    
    // ========== 长方体障碍物 ==========
    
    // 障碍物 1: large_rotated_wall
    // 位置: (20, 5, -15), 尺寸: (15, 1.5, 20), 旋转: 绕Z轴 0.785 弧度 (45度)
    {
        octomap::point3d center(20.0, 5.0, -15.0);
        octomap::point3d size(15.0, 1.5, 20.0);
        // 创建绕 Z 轴旋转 45 度的四元数（使用 TF）
        tf::Quaternion rotation;
        rotation.setRPY(0.0, 0.0, 0.785);  // roll, pitch, yaw
        box_obstacles_.emplace_back(center, size, rotation, "large_rotated_wall");
    }
    
    // 障碍物 3: small_tilted_box
    // 位置: (25, -15, -10), 尺寸: (8, 4, 3), 旋转: roll=0, pitch=0.5, yaw=0.3
    {
        octomap::point3d center(25.0, -15.0, -10.0);
        octomap::point3d size(8.0, 4.0, 3.0);
        // 创建具有俯仰和偏航的四元数（使用 TF）
        tf::Quaternion rotation;
        rotation.setRPY(0.0, 0.5, 0.3);  // roll, pitch, yaw
        box_obstacles_.emplace_back(center, size, rotation, "small_tilted_box");
    }
    
    // ========== 圆柱体障碍物 ==========
    
    // 障碍物 2: vertical_pillar
    // 位置: (10, 25, -20), 半径: 2m, 高度: 30m
    // 注意: Gazebo 中圆柱体的 pose.z 是中心点，但我们存储底面中心更方便计算
    {
        double center_z = -20.0;  // Gazebo 中的中心高度
        double height = 30.0;
        double bottom_z = center_z - height / 2.0;  // 底面 Z 坐标 = -35.0
        
        octomap::point3d center_bottom(10.0, 25.0, bottom_z);
        cylinder_obstacles_.emplace_back(center_bottom, 2.0, height, "vertical_pillar");
    }
}

// ==================== 海底高度计算 ====================

double GroundTruthEnvironment::gaussian2D(double x, double y, const GaussianFeature& feature) const {
    double dx = x - feature.center_x;
    double dy = y - feature.center_y;
    double distance_sq = dx * dx + dy * dy;
    double sigma_sq = feature.sigma * feature.sigma;
    
    return feature.height * std::exp(-distance_sq / (2.0 * sigma_sq));
}

double GroundTruthEnvironment::getSeafloorZ(double x, double y) const {
    double z = SEAFLOOR_BASE_Z;
    
    // 叠加所有高斯特征
    for (const auto& feature : gaussian_features_) {
        z += gaussian2D(x, y, feature);
    }
    
    return z;
}

// ==================== 碰撞检测函数 ====================

bool GroundTruthEnvironment::isInsideBox(const octomap::point3d& world_point, 
                                          const BoxObstacle& box) const {
    // 将世界坐标点转换到长方体的局部坐标系
    // 步骤：
    // 1. 平移：将点移到以长方体中心为原点的坐标系
    // 2. 旋转：应用旋转的逆变换（四元数的共轭）
    // 3. 检查：在局部坐标系中检查点是否在长方体范围内
    
    // 1. 平移到局部坐标系
    octomap::point3d local_point = world_point - box.center;
    
    // 2. 应用旋转的逆变换（使用 TF）
    // 将 octomap::point3d 转换为 tf::Vector3
    tf::Vector3 local_vec(local_point.x(), local_point.y(), local_point.z());
    
    // 获取逆四元数并应用旋转
    tf::Quaternion inv_rotation = box.rotation.inverse();
    tf::Vector3 rotated_vec = tf::quatRotate(inv_rotation, local_vec);
    
    // 转换回 octomap::point3d
    local_point.x() = rotated_vec.x();
    local_point.y() = rotated_vec.y();
    local_point.z() = rotated_vec.z();
    
    // 3. 在局部坐标系中检查边界
    // 长方体在局部坐标系中是轴对齐的，中心在原点
    double half_x = box.size.x() / 2.0;
    double half_y = box.size.y() / 2.0;
    double half_z = box.size.z() / 2.0;
    
    bool inside = (std::abs(local_point.x()) <= half_x) &&
                  (std::abs(local_point.y()) <= half_y) &&
                  (std::abs(local_point.z()) <= half_z);
    
    return inside;
}

bool GroundTruthEnvironment::isInsideAnyBox(const octomap::point3d& world_point) const {
    for (const auto& box : box_obstacles_) {
        if (isInsideBox(world_point, box)) {
            return true;
        }
    }
    return false;
}

bool GroundTruthEnvironment::isInsideCylinder(const octomap::point3d& world_point,
                                               const CylinderObstacle& cylinder) const {
    // 圆柱体碰撞检测：
    // 1. 检查 Z 坐标是否在圆柱体高度范围内
    // 2. 检查水平距离是否小于等于半径
    
    double z = world_point.z();
    double z_min = cylinder.center_bottom.z();
    double z_max = z_min + cylinder.height;
    
    // 检查高度范围
    if (z < z_min || z > z_max) {
        return false;
    }
    
    // 检查水平距离
    double dx = world_point.x() - cylinder.center_bottom.x();
    double dy = world_point.y() - cylinder.center_bottom.y();
    double horizontal_distance_sq = dx * dx + dy * dy;
    double radius_sq = cylinder.radius * cylinder.radius;
    
    return horizontal_distance_sq <= radius_sq;
}

bool GroundTruthEnvironment::isInsideAnyCylinder(const octomap::point3d& world_point) const {
    for (const auto& cylinder : cylinder_obstacles_) {
        if (isInsideCylinder(world_point, cylinder)) {
            return true;
        }
    }
    return false;
}

// ==================== 主查询接口 ====================

bool GroundTruthEnvironment::isOccupied(const octomap::point3d& world_point) const {
    // 1. 检查是否在任意障碍物内部
    if (isInsideAnyBox(world_point) || isInsideAnyCylinder(world_point)) {
        return true;
    }
    
    // 2. 检查是否在海底表面附近（而不是整个海底以下）
    // 只标记海底表面薄层为占据，模拟声呐只能探测到表面的情况
    double seafloor_z = getSeafloorZ(world_point.x(), world_point.y());
    double surface_thickness = 1.0;  // 海底表面厚度（米）
    
    // 如果点在海底表面薄层内，认为被占据
    if (world_point.z() <= seafloor_z && world_point.z() >= (seafloor_z - surface_thickness)) {
        return true;  // 点在海底表面薄层内
    }
    
    // 3. 否则是空闲空间（包括海底深处，因为声呐探测不到）
    return false;
}

} // namespace uuv_project
