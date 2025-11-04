/**
 * @file map_evaluation_node.cpp
 * @brief 地图准确率评估节点示例
 * 
 * 该节点订阅生成的 OctoMap，并使用 Ground Truth 环境进行准确率评估
 */

#include <ros/ros.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>
#include "uuv_project_core/ground_truth_environment.h"

class MapEvaluationNode {
public:
    MapEvaluationNode() : nh_("~") {
        // 初始化 Ground Truth 环境
        gt_env_ = std::make_unique<uuv_project::GroundTruthEnvironment>();
        
        // 订阅 OctoMap 话题
        octomap_sub_ = nh_.subscribe("/a9/octomap", 1, 
                                     &MapEvaluationNode::octomapCallback, this);
        
        // 获取参数
        nh_.param("evaluation_interval", evaluation_interval_, 10.0);  // 默认每10秒评估一次
        nh_.param("min_z", min_z_, -40.0);  // 评估区域的最小 Z 坐标
        nh_.param("max_z", max_z_, 0.0);    // 评估区域的最大 Z 坐标
        
        ROS_INFO("Map Evaluation Node started");
        ROS_INFO("  Evaluation interval: %.1f seconds", evaluation_interval_);
        ROS_INFO("  Evaluation Z range: [%.1f, %.1f]", min_z_, max_z_);
        
        // 创建定时器
        evaluation_timer_ = nh_.createTimer(ros::Duration(evaluation_interval_),
                                           &MapEvaluationNode::evaluationTimerCallback, this);
    }

private:
    void octomapCallback(const octomap_msgs::Octomap::ConstPtr& msg) {
        // 将消息转换为 OctoMap
        octomap::AbstractOcTree* abstract_tree = octomap_msgs::msgToMap(*msg);
        if (!abstract_tree) {
            ROS_ERROR("Failed to convert OctoMap message");
            return;
        }
        
        // 转换为 OcTree
        octomap::OcTree* octree = dynamic_cast<octomap::OcTree*>(abstract_tree);
        if (!octree) {
            ROS_ERROR("OctoMap is not an OcTree");
            delete abstract_tree;
            return;
        }
        
        // 保存最新的地图
        latest_map_.reset(octree);
        
        ROS_INFO_THROTTLE(5.0, "Received OctoMap with %zu nodes", latest_map_->size());
    }
    
    void evaluationTimerCallback(const ros::TimerEvent& event) {
        if (!latest_map_) {
            ROS_WARN("No map received yet, skipping evaluation");
            return;
        }
        
        ROS_INFO("Starting map accuracy evaluation...");
        
        // 统计变量
        int total_voxels = 0;
        int correct_occupied = 0;
        int correct_free = 0;
        int false_positives = 0;  // 地图认为占据，但实际空闲
        int false_negatives = 0;  // 地图认为空闲，但实际占据
        
        // 遍历地图中的所有叶子节点
        for (auto it = latest_map_->begin_leafs(); it != latest_map_->end_leafs(); ++it) {
            // 只评估指定 Z 范围内的体素
            double z = it.getZ();
            if (z < min_z_ || z > max_z_) {
                continue;
            }
            
            octomap::point3d point(it.getX(), it.getY(), z);
            
            // 获取地图中的占据状态
            bool map_occupied = latest_map_->isNodeOccupied(*it);
            
            // 获取 Ground Truth 占据状态
            bool gt_occupied = gt_env_->isOccupied(point);
            
            total_voxels++;
            
            // 统计
            if (map_occupied && gt_occupied) {
                correct_occupied++;
            } else if (!map_occupied && !gt_occupied) {
                correct_free++;
            } else if (map_occupied && !gt_occupied) {
                false_positives++;
            } else {  // !map_occupied && gt_occupied
                false_negatives++;
            }
        }
        
        // 计算准确率指标
        int correct_total = correct_occupied + correct_free;
        double accuracy = (total_voxels > 0) ? 
                         (double)correct_total / total_voxels * 100.0 : 0.0;
        
        double precision = (correct_occupied + false_positives > 0) ?
                          (double)correct_occupied / (correct_occupied + false_positives) * 100.0 : 0.0;
        
        double recall = (correct_occupied + false_negatives > 0) ?
                       (double)correct_occupied / (correct_occupied + false_negatives) * 100.0 : 0.0;
        
        double f1_score = (precision + recall > 0) ?
                         2.0 * precision * recall / (precision + recall) : 0.0;
        
        // 输出评估结果
        ROS_INFO("========================================");
        ROS_INFO("Map Accuracy Evaluation Results:");
        ROS_INFO("========================================");
        ROS_INFO("Total evaluated voxels: %d", total_voxels);
        ROS_INFO("  Correct occupied: %d", correct_occupied);
        ROS_INFO("  Correct free: %d", correct_free);
        ROS_INFO("  False positives: %d", false_positives);
        ROS_INFO("  False negatives: %d", false_negatives);
        ROS_INFO("----------------------------------------");
        ROS_INFO("Metrics:");
        ROS_INFO("  Overall Accuracy: %.2f%%", accuracy);
        ROS_INFO("  Precision: %.2f%%", precision);
        ROS_INFO("  Recall: %.2f%%", recall);
        ROS_INFO("  F1 Score: %.2f%%", f1_score);
        ROS_INFO("========================================");
    }

    ros::NodeHandle nh_;
    ros::Subscriber octomap_sub_;
    ros::Timer evaluation_timer_;
    
    std::unique_ptr<uuv_project::GroundTruthEnvironment> gt_env_;
    std::unique_ptr<octomap::OcTree> latest_map_;
    
    double evaluation_interval_;
    double min_z_;
    double max_z_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "map_evaluation_node");
    
    MapEvaluationNode node;
    
    ros::spin();
    
    return 0;
}
