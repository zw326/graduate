cmake_minimum_required(VERSION 3.0.2)
project(uuv_project_core)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 忽略库路径冲突警告
if(POLICY CMP0060)
  cmake_policy(SET CMP0060 NEW)
endif()

# 设置 RPATH 策略
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

set(gazebo_DIR "/usr/lib/x86_64-linux-gnu/cmake/gazebo")
find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  gazebo_ros
  gazebo
  gazebo_plugins
  nav_msgs
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  tf
  visualization_msgs
  octomap_ros
  octomap_msgs
  geometry_msgs
  std_msgs
  uuv_gazebo_ros_plugins_msgs
  uuv_control_msgs
)


# Gazebo依赖
find_package(gazebo REQUIRED)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  include

)

# PCL依赖
find_package(PCL REQUIRED COMPONENTS common filters)
include_directories(${PCL_INCLUDE_DIRS})

# 插件库


add_library(sonar_sensor src/sonar_sensor.cpp)
target_link_libraries(sonar_sensor
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${PCL_LIBRARIES}
)

# 编译 Ground Truth 环境库
add_library(ground_truth_environment src/ground_truth_environment.cpp)
target_link_libraries(ground_truth_environment
  ${catkin_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
)

# 编译覆盖路径规划库
add_library(coverage_planner src/coverage_planner.cpp)
target_link_libraries(coverage_planner
  ${catkin_LIBRARIES}
)

# 编译 SAVFH 避障库
add_library(savfh src/savfh.cpp)
target_link_libraries(savfh
  ${catkin_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
)

# 编译 PID 控制器库
add_library(pid_controller src/pid_controller.cpp)
target_link_libraries(pid_controller
  ${catkin_LIBRARIES}
)

# 编译我们的栅格地图节点可执行文件
add_executable(grid_map_node src/grid_map_node.cpp)

# 链接这个可执行文件所需要的所有外部库
target_link_libraries(grid_map_node
  ${catkin_LIBRARIES}
  # PCL的库也需要链接进来
  ${PCL_LIBRARIES}
  ${OCTOMAP_LIBRARIES} # 【新增】
)
set_target_properties(grid_map_node PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/lib/${PROJECT_NAME}
  OUTPUT_NAME grid_map_node
)

# 编译地图评估节点
add_executable(map_evaluation_node src/map_evaluation_node.cpp)
target_link_libraries(map_evaluation_node
  ${catkin_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
  ground_truth_environment
)
set_target_properties(map_evaluation_node PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/lib/${PROJECT_NAME}
  OUTPUT_NAME map_evaluation_node
)

# 编译覆盖扫描导航节点
#add_executable(coverage_navigation_node src/coverage_navigation_node.cpp)
#target_link_libraries(coverage_navigation_node
#  ${catkin_LIBRARIES}
#  ${OCTOMAP_LIBRARIES}
#  coverage_planner
#  savfh
#  pid_controller
#)
#set_target_properties(coverage_navigation_node PROPERTIES
#  RUNTIME_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/lib/${PROJECT_NAME}
#  OUTPUT_NAME coverage_navigation_node
#)

# 编译航路点导航节点 V2 (使用服务调用)
#add_executable(waypoint_navigation_node src/waypoint_navigation_node_v2.cpp)
#target_link_libraries(waypoint_navigation_node
#  ${catkin_LIBRARIES}
#  coverage_planner
#)
#set_target_properties(waypoint_navigation_node PROPERTIES
#  RUNTIME_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/lib/${PROJECT_NAME}
#  OUTPUT_NAME waypoint_navigation_node
#)

# 编译路径跟踪控制器库
#add_library(path_following_controller src/path_following_controller.cpp)
#target_link_libraries(path_following_controller
 # ${catkin_LIBRARIES}
#)

# 编译路径跟踪控制器库 (使用v2版本)
add_library(path_following_controller src/path_following_controller_v2.cpp)
target_link_libraries(path_following_controller
  ${catkin_LIBRARIES}
)

# 编译路径跟踪节点 (自定义PID控制)
add_executable(path_following_node src/path_following_node.cpp)
target_link_libraries(path_following_node
  ${catkin_LIBRARIES}
  coverage_planner
  path_following_controller
)
set_target_properties(path_following_node PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/lib/${PROJECT_NAME}
  OUTPUT_NAME path_following_node
)

# 编译几何控制器路径跟踪节点
add_executable(geometric_path_follower src/geometric_path_follower.cpp)
target_link_libraries(geometric_path_follower
  ${catkin_LIBRARIES}
  coverage_planner
)
set_target_properties(geometric_path_follower PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/lib/${PROJECT_NAME}
  OUTPUT_NAME geometric_path_follower
)






# add_library(left_sonar_sensor plugins/sonar_sensor.cpp)
# target_link_libraries(left_sonar_sensor
#   ${catkin_LIBRARIES}
#   ${GAZEBO_LIBRARIES}
#   ${PCL_LIBRARIES}
# )

# add_library(underwater_effects plugins/underwater_effects.cpp)
# target_link_libraries(underwater_effects
#   ${catkin_LIBRARIES}
#   ${GAZEBO_LIBRARIES}
# )

# 节点
# add_executable(sonar_processor src/sonar_processor.cpp)
# target_link_libraries(sonar_processor
#   ${catkin_LIBRARIES}
#   ${PCL_LIBRARIES}
# )





catkin_package(
  INCLUDE_DIRS include
  LIBRARIES  sonar_sensor ground_truth_environment coverage_planner savfh pid_controller
  CATKIN_DEPENDS roscpp nav_msgs sensor_msgs gazebo_ros geometry_msgs std_msgs uuv_gazebo_ros_plugins_msgs uuv_control_msgs
)

# 安装
install(TARGETS 
  #auto_frontier_selector
  #simple_auv_controller
  grid_map_node
  map_evaluation_node
  #coverage_navigation_node
  #waypoint_navigation_node
  sonar_sensor
  ground_truth_environment
  coverage_planner
  savfh
  pid_controller
  # left_sonar_sensor
  # sonar_processor
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


# 安装资源文件
install(DIRECTORY 
  launch 
  urdf 
  worlds 
  config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
