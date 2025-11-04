#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
检查点云数据是否包含三维信息的调试脚本
"""

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import numpy as np

class PointCloudChecker:
    def __init__(self):
        rospy.init_node('pointcloud_checker', anonymous=True)
        
        self.z_values = []
        self.point_count = 0
        
        # 订阅点云话题
        rospy.Subscriber('/a9/my_sonar_points', PointCloud2, self.pointcloud_callback)
        
        rospy.loginfo("点云检查器已启动，正在监听 /a9/my_sonar_points...")
        
    def pointcloud_callback(self, msg):
        """点云回调函数"""
        points = []
        
        # 解析点云数据
        for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            points.append(point)
        
        if len(points) == 0:
            rospy.logwarn("收到空点云！")
            return
        
        # 转换为 numpy 数组便于统计
        points_array = np.array(points)
        
        x_values = points_array[:, 0]
        y_values = points_array[:, 1]
        z_values = points_array[:, 2]
        
        self.point_count += len(points)
        
        # 统计信息
        rospy.loginfo("=" * 60)
        rospy.loginfo("点云统计信息:")
        rospy.loginfo("  点数: %d", len(points))
        rospy.loginfo("  X 范围: [%.2f, %.2f]", np.min(x_values), np.max(x_values))
        rospy.loginfo("  Y 范围: [%.2f, %.2f]", np.min(y_values), np.max(y_values))
        rospy.loginfo("  Z 范围: [%.2f, %.2f]", np.min(z_values), np.max(z_values))
        rospy.loginfo("  Z 标准差: %.2f", np.std(z_values))
        
        # 检查是否是二维数据（所有 Z 值相同或接近）
        z_std = np.std(z_values)
        if z_std < 0.1:
            rospy.logwarn("警告：Z 轴变化很小 (std=%.4f)，可能是二维数据！", z_std)
        else:
            rospy.loginfo("✓ Z 轴有明显变化，这是三维数据")
        
        # 显示一些示例点
        rospy.loginfo("  示例点 (前5个):")
        for i, point in enumerate(points[:5]):
            rospy.loginfo("    点 %d: (%.2f, %.2f, %.2f)", i, point[0], point[1], point[2])
        
        rospy.loginfo("  累计处理点数: %d", self.point_count)
        rospy.loginfo("=" * 60)

if __name__ == '__main__':
    try:
        checker = PointCloudChecker()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
