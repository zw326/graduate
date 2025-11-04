#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
# 核心修正1：导入新的消息类型和库
from sensor_msgs.msg import LaserScan, PointCloud2
from laser_geometry import LaserProjection

# 全局变量
pub = None
laser_proj = None

def laserscan_callback(data):
    """
    这个回调函数现在接收LaserScan消息。
    """
    global pub, laser_proj
    try:
        # rospy.loginfo("成功接收到LaserScan数据！")
        
        # ===============================================================
        # ===             核心算法：2D LaserScan -> 3D PointCloud2      ===
        # ===============================================================
        # 使用我们创建的laser_proj对象，调用projectLaser方法
        # 这个方法会自动将传入的LaserScan消息，根据其坐标系信息，转换为PointCloud2消息
        cloud_out = laser_proj.projectLaser(data)
        
        # ===============================================================
        # === 在这里，我们将对转换后的cloud_out添加噪声等处理 ... ===
        # ===============================================================
        processed_pointcloud = cloud_out

        # 发布转换和处理后的3D点云
        if pub is not None:
            pub.publish(processed_pointcloud)

    except Exception as e:
        rospy.logerr("处理LaserScan时发生错误: %s", e)

def main():
    """
    主函数
    """
    global pub, laser_proj

    rospy.init_node('scan_to_cloud_converter_node', anonymous=True)
    
    # 核心修正2：初始化LaserProjection对象
    # 这个对象是完成转换工作的核心工具
    laser_proj = LaserProjection()
    
    # 1. 创建一个订阅者
    # 它订阅的是LaserScan类型的 /a9/points 话题
    rospy.Subscriber("/a9/points", LaserScan, laserscan_callback)
    
    # 2. 创建一个发布者
    # 核心修正3：它现在发布的是PointCloud2类型的 /a9/simulated_sonar_points 话题
    # 我们的“数据管道”的出口，现在流出的是真正的3D点云了！
    pub = rospy.Publisher("/a9/simulated_sonar_points", PointCloud2, queue_size=10)
    
    rospy.loginfo("2D激光转3D点云节点已启动...")
    
    rospy.spin()

if __name__ == '__main__':
    main()
