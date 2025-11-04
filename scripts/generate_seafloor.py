#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
海底地形生成脚本 - 使用高斯函数生成起伏的海底地形
生成的 STL 文件可被 Gazebo 加载
"""

import numpy as np
import sys

try:
    from stl import mesh
except ImportError:
    print("错误: 需要安装 numpy-stl 库")
    print("请运行: pip install numpy-stl")
    sys.exit(1)

# ==================== 环境参数配置 ====================
# 这些参数应与 grid_map_node.cpp 中的定义保持一致
MAP_WIDTH = 100.0   # X 方向范围: [-50, 50]
MAP_HEIGHT = 100.0  # Y 方向范围: [-50, 50]
MAP_ORIGIN_X = -MAP_WIDTH / 2.0
MAP_ORIGIN_Y = -MAP_HEIGHT / 2.0

# 海底基准深度（相对于世界坐标系 z=0 的水面）
SEAFLOOR_BASE_Z = -40.0

# 网格分辨率（米）- 影响 STL 文件大小和精度
MESH_RESOLUTION = 1.0  # 1米一个网格点，可以调整为 0.5 或 2.0

# ==================== 高斯地形参数 ====================
# 每个高斯特征定义为: (center_x, center_y, height, sigma)
# height > 0: 海山（凸起）
# height < 0: 海沟（凹陷）
# sigma: 控制坡度，值越大越平缓
GAUSSIAN_FEATURES = [
    # (中心X, 中心Y, 高度, 坡度sigma)
    (20.0,  20.0,  8.0, 12.0),   # 海山 1: 较缓的凸起
    (-25.0, -20.0, 6.0, 8.0),    # 海山 2: 较陡的凸起
    (-15.0, 25.0, -5.0, 10.0),   # 海沟 1: 凹陷
]


def gaussian_2d(x, y, center_x, center_y, height, sigma):
    """
    计算二维高斯函数值
    
    Args:
        x, y: 查询点坐标
        center_x, center_y: 高斯中心
        height: 高斯峰值高度
        sigma: 标准差（控制宽度）
    
    Returns:
        该点的高度贡献值
    """
    dx = x - center_x
    dy = y - center_y
    distance_sq = dx * dx + dy * dy
    return height * np.exp(-distance_sq / (2.0 * sigma * sigma))


def compute_seafloor_height(x, y):
    """
    计算给定 (x, y) 位置的海底高度
    这个函数的逻辑必须与 ground_truth_environment.cpp 中的实现完全一致
    
    Args:
        x, y: 世界坐标系中的水平位置
    
    Returns:
        该位置的海底 Z 坐标
    """
    z = SEAFLOOR_BASE_Z
    
    # 叠加所有高斯特征
    for center_x, center_y, height, sigma in GAUSSIAN_FEATURES:
        z += gaussian_2d(x, y, center_x, center_y, height, sigma)
    
    return z


def generate_seafloor_mesh():
    """
    生成海底地形的三角网格
    
    Returns:
        numpy-stl mesh 对象
    """
    print("开始生成海底地形网格...")
    
    # 生成网格点
    x_points = int(MAP_WIDTH / MESH_RESOLUTION) + 1
    y_points = int(MAP_HEIGHT / MESH_RESOLUTION) + 1
    
    print(f"  网格分辨率: {MESH_RESOLUTION}m")
    print(f"  网格尺寸: {x_points} x {y_points} = {x_points * y_points} 个顶点")
    
    # 创建网格坐标
    x = np.linspace(MAP_ORIGIN_X, MAP_ORIGIN_X + MAP_WIDTH, x_points)
    y = np.linspace(MAP_ORIGIN_Y, MAP_ORIGIN_Y + MAP_HEIGHT, y_points)
    X, Y = np.meshgrid(x, y)
    
    # 计算每个网格点的高度
    print("  计算高度值...")
    Z = np.zeros_like(X)
    for i in range(x_points):
        for j in range(y_points):
            Z[j, i] = compute_seafloor_height(X[j, i], Y[j, i])
    
    print(f"  高度范围: [{np.min(Z):.2f}, {np.max(Z):.2f}] 米")
    
    # 生成三角形面片
    # 每个网格单元生成 2 个三角形
    print("  生成三角形面片...")
    num_triangles = 2 * (x_points - 1) * (y_points - 1)
    seafloor_mesh = mesh.Mesh(np.zeros(num_triangles, dtype=mesh.Mesh.dtype))
    
    triangle_index = 0
    for i in range(x_points - 1):
        for j in range(y_points - 1):
            # 当前网格单元的四个顶点
            # v0 --- v1
            # |  \   |
            # |   \  |
            # v2 --- v3
            v0 = [X[j, i],     Y[j, i],     Z[j, i]]
            v1 = [X[j, i+1],   Y[j, i+1],   Z[j, i+1]]
            v2 = [X[j+1, i],   Y[j+1, i],   Z[j+1, i]]
            v3 = [X[j+1, i+1], Y[j+1, i+1], Z[j+1, i+1]]
            
            # 第一个三角形: v0-v1-v2
            seafloor_mesh.vectors[triangle_index] = [v0, v1, v2]
            triangle_index += 1
            
            # 第二个三角形: v1-v3-v2
            seafloor_mesh.vectors[triangle_index] = [v1, v3, v2]
            triangle_index += 1
    
    print(f"  生成了 {num_triangles} 个三角形")
    
    return seafloor_mesh


def save_stl(mesh_obj, output_path):
    """
    保存网格为 STL 文件
    
    Args:
        mesh_obj: numpy-stl mesh 对象
        output_path: 输出文件路径
    """
    print(f"\n保存 STL 文件到: {output_path}")
    mesh_obj.save(output_path)
    
    # 计算文件大小
    import os
    file_size = os.path.getsize(output_path)
    print(f"  文件大小: {file_size / 1024:.2f} KB")
    print("✓ STL 文件生成成功！")


def print_summary():
    """打印配置摘要"""
    print("=" * 60)
    print("海底地形生成配置摘要")
    print("=" * 60)
    print(f"地图范围: X=[{MAP_ORIGIN_X}, {MAP_ORIGIN_X + MAP_WIDTH}], "
          f"Y=[{MAP_ORIGIN_Y}, {MAP_ORIGIN_Y + MAP_HEIGHT}]")
    print(f"基准深度: {SEAFLOOR_BASE_Z} 米")
    print(f"网格分辨率: {MESH_RESOLUTION} 米")
    print(f"\n高斯地形特征 ({len(GAUSSIAN_FEATURES)} 个):")
    for i, (cx, cy, h, sigma) in enumerate(GAUSSIAN_FEATURES, 1):
        feature_type = "海山" if h > 0 else "海沟"
        print(f"  {i}. {feature_type}: 中心({cx}, {cy}), "
              f"高度{h:+.1f}m, 坡度σ={sigma}")
    print("=" * 60 + "\n")


if __name__ == "__main__":
    import os
    
    # 打印配置信息
    print_summary()
    
    # 确定输出路径
    # 假设脚本在 uuv_project_core/scripts/ 目录下运行
    script_dir = os.path.dirname(os.path.abspath(__file__))
    package_root = os.path.dirname(script_dir)  # 上一级目录
    output_dir = os.path.join(package_root, "models", "seafloor")
    
    # 创建输出目录
    os.makedirs(output_dir, exist_ok=True)
    
    output_file = os.path.join(output_dir, "custom_seafloor.stl")
    
    # 生成网格
    seafloor_mesh = generate_seafloor_mesh()
    
    # 保存文件
    save_stl(seafloor_mesh, output_file)
    
    print("\n" + "=" * 60)
    print("下一步操作:")
    print("=" * 60)
    print("1. 在 worlds/my_obstacles_world.world 中添加海底模型")
    print("2. 确保 ground_truth_environment.cpp 使用相同的高斯参数")
    print("3. 运行 Gazebo 仿真验证海底地形")
    print("=" * 60)
