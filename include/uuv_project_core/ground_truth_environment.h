/**
 * @file ground_truth_environment.h
 * @brief Ground Truth 环境定义 - 用于地图准确率评估
 * 
 * 该文件定义了仿真环境的精确数学模型，包括：
 * 1. 高斯函数生成的海底地形
 * 2. 长方体障碍物（支持旋转）
 * 3. 圆柱体障碍物
 * 
 * 这些定义必须与 Gazebo world 文件中的模型保持一致
 */

#ifndef GROUND_TRUTH_ENVIRONMENT_H
#define GROUND_TRUTH_ENVIRONMENT_H

#include <vector>
#include <string>
#include <octomap/octomap.h>
#include <tf/tf.h>  // 使用 TF 的四元数

namespace uuv_project {

// ==================== 环境参数常量 ====================
// 这些参数必须与 grid_map_node.cpp 和 generate_seafloor.py 保持一致
const double MAP_WIDTH = 100.0;
const double MAP_HEIGHT = 100.0;
const double MAP_ORIGIN_X = -MAP_WIDTH / 2.0;
const double MAP_ORIGIN_Y = -MAP_HEIGHT / 2.0;
const double SEAFLOOR_BASE_Z = -40.0;

// ==================== 高斯地形参数 ====================
struct GaussianFeature {
    double center_x;
    double center_y;
    double height;      // 正值=海山，负值=海沟
    double sigma;       // 标准差，控制坡度
    
    GaussianFeature(double cx, double cy, double h, double s)
        : center_x(cx), center_y(cy), height(h), sigma(s) {}
};

// ==================== 障碍物结构体 ====================

/**
 * @brief 长方体障碍物定义
 */
struct BoxObstacle {
    octomap::point3d center;        // 中心点位置
    octomap::point3d size;          // 尺寸 (长, 宽, 高)
    tf::Quaternion rotation;        // 旋转四元数（使用 TF）
    std::string name;               // 障碍物名称（用于调试）
    
    BoxObstacle(const octomap::point3d& c, const octomap::point3d& s,
                const tf::Quaternion& r, const std::string& n)
        : center(c), size(s), rotation(r), name(n) {}
};

/**
 * @brief 圆柱体障碍物定义
 */
struct CylinderObstacle {
    octomap::point3d center_bottom;  // 底面中心点
    double radius;                   // 半径
    double height;                   // 高度
    std::string name;                // 障碍物名称
    
    CylinderObstacle(const octomap::point3d& cb, double r, double h, const std::string& n)
        : center_bottom(cb), radius(r), height(h), name(n) {}
};

// ==================== Ground Truth 环境类 ====================

/**
 * @brief Ground Truth 环境类
 * 
 * 提供精确的环境查询接口，用于：
 * 1. 计算任意点的海底高度
 * 2. 判断任意点是否被障碍物占据
 * 3. 与生成的 OctoMap 进行对比评估
 */
class GroundTruthEnvironment {
public:
    /**
     * @brief 构造函数 - 初始化环境参数
     */
    GroundTruthEnvironment();
    
    /**
     * @brief 析构函数
     */
    ~GroundTruthEnvironment() = default;
    
    /**
     * @brief 计算给定 (x, y) 位置的海底高度
     * 
     * @param x 世界坐标系 X 坐标
     * @param y 世界坐标系 Y 坐标
     * @return 该位置的海底 Z 坐标
     */
    double getSeafloorZ(double x, double y) const;
    
    /**
     * @brief 判断给定点是否被占据（障碍物或海底）
     * 
     * @param world_point 世界坐标系中的查询点
     * @return true 如果点被占据，false 如果点是空闲空间
     */
    bool isOccupied(const octomap::point3d& world_point) const;
    
    /**
     * @brief 判断点是否在任意长方体障碍物内部
     * 
     * @param world_point 世界坐标系中的查询点
     * @return true 如果点在任意长方体内部
     */
    bool isInsideAnyBox(const octomap::point3d& world_point) const;
    
    /**
     * @brief 判断点是否在任意圆柱体障碍物内部
     * 
     * @param world_point 世界坐标系中的查询点
     * @return true 如果点在任意圆柱体内部
     */
    bool isInsideAnyCylinder(const octomap::point3d& world_point) const;
    
    /**
     * @brief 获取所有长方体障碍物列表（用于可视化或调试）
     */
    const std::vector<BoxObstacle>& getBoxObstacles() const { return box_obstacles_; }
    
    /**
     * @brief 获取所有圆柱体障碍物列表
     */
    const std::vector<CylinderObstacle>& getCylinderObstacles() const { return cylinder_obstacles_; }
    
    /**
     * @brief 获取高斯地形特征列表
     */
    const std::vector<GaussianFeature>& getGaussianFeatures() const { return gaussian_features_; }

private:
    /**
     * @brief 初始化高斯地形参数
     * 必须与 generate_seafloor.py 中的参数保持一致
     */
    void initializeGaussianFeatures();
    
    /**
     * @brief 初始化障碍物列表
     * 必须与 my_obstacles_world.world 中的模型定义保持一致
     */
    void initializeObstacles();
    
    /**
     * @brief 判断点是否在单个长方体内部（考虑旋转）
     * 
     * @param world_point 世界坐标系中的查询点
     * @param box 长方体障碍物
     * @return true 如果点在长方体内部
     */
    bool isInsideBox(const octomap::point3d& world_point, const BoxObstacle& box) const;
    
    /**
     * @brief 判断点是否在单个圆柱体内部
     * 
     * @param world_point 世界坐标系中的查询点
     * @param cylinder 圆柱体障碍物
     * @return true 如果点在圆柱体内部
     */
    bool isInsideCylinder(const octomap::point3d& world_point, const CylinderObstacle& cylinder) const;
    
    /**
     * @brief 计算单个高斯函数的贡献值
     */
    double gaussian2D(double x, double y, const GaussianFeature& feature) const;

    // 成员变量
    std::vector<GaussianFeature> gaussian_features_;     // 高斯地形特征列表
    std::vector<BoxObstacle> box_obstacles_;             // 长方体障碍物列表
    std::vector<CylinderObstacle> cylinder_obstacles_;   // 圆柱体障碍物列表
};

} // namespace uuv_project

#endif // GROUND_TRUTH_ENVIRONMENT_H
