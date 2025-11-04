#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/PhysicsIface.hh> // 添加物理引擎头文件
#include <gazebo/physics/Entity.hh>       // 包含Entity头文件
#include <ignition/math/Pose3.hh>         // 添加位姿类
#include <ignition/math/Matrix3.hh>       // 添加旋转矩阵
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cmath>
#include <tf2_ros/transform_broadcaster.h> // TF转换支持

// #include <memory>

namespace gazebo
{

  class Sonar3DSensor : public SensorPlugin
  {
  public:
    Sonar3DSensor() : SensorPlugin() {}

    virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
    {

      // 获取父传感器
      sensor_ = std::dynamic_pointer_cast<sensors::RaySensor>(_sensor);
      if (!sensor_)
      {
        gzerr << "Sonar3DSensor requires a RaySensor\n";
        return;
      }

      // 验证是否3D传感器
      if (sensor_->VerticalRangeCount() <= 1)
      {
        gzerr << "Sonar3DSensor requires vertical resolution > 1 (3D sensor)\n";
        return;
      }

      // 初始化ROS节点
      std::string node_name = "sonar3d_" + sensor_->Name();
      // 替换名称中的非法字符
      std::replace(node_name.begin(), node_name.end(), ':', '_');

      // 初始化ROS节点（带唯一名称）
      if (!ros::isInitialized())
      {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, node_name, ros::init_options::NoSigintHandler);
      }
      ros_node_ = new ros::NodeHandle(node_name);
      //  【核心修正2】使用智能指针来创建和管理NodeHandle对象
      // ros_node_.reset(new ros::NodeHandle(node_name));

      // 从SDF获取参数
      std::string topic_name = "/sonar";
      sdf::ElementPtr topic_elem = _sdf->GetElement("topic");
      if (topic_elem)
      {
        topic_name = topic_elem->Get<std::string>();
      }

      // 创建发布者
      pointcloud_pub_ = ros_node_->advertise<sensor_msgs::PointCloud2>(
          topic_name, 10);

      // 获取传感器参数
      horizontal_samples_ = sensor_->RangeCount();
      vertical_samples_ = sensor_->VerticalRangeCount();

      horizontal_min_angle_ = sensor_->AngleMin().Radian();
      horizontal_max_angle_ = sensor_->AngleMax().Radian();
      horizontal_angle_step_ = (horizontal_max_angle_ - horizontal_min_angle_) /
                               (horizontal_samples_ - 1);

      vertical_min_angle_ = sensor_->VerticalAngleMin().Radian();
      vertical_max_angle_ = sensor_->VerticalAngleMax().Radian();
      vertical_angle_step_ = (vertical_max_angle_ - vertical_min_angle_) /
                             (vertical_samples_ - 1);

      // 添加坐标系名称配置
      output_frame_ = "world"; // 默认世界坐标系
      if (_sdf->HasElement("output_frame"))
      {
        sdf::ElementPtr frame_elem = _sdf->GetElement("output_frame");
        if (frame_elem)
        {
          output_frame_ = frame_elem->Get<std::string>();
        }
      }

      // 获取物理世界
      world_ = physics::get_world(sensor_->WorldName());
      if (!world_)
      {
        ROS_ERROR("Failed to get physics world");
        return;
      }

      // 通过父名称获取父实体
      parent_entity_ = world_->EntityByName(sensor_->ParentName());
      if (!parent_entity_)
      {
        ROS_ERROR("Failed to get parent entity for sensor: %s", sensor_->Name().c_str());
        ROS_INFO("Parent entity name: %s", sensor_->ParentName().c_str());
      }

      // 连接更新回调
      update_connection_ = sensor_->ConnectUpdated(
          std::bind(&Sonar3DSensor::OnUpdate, this));

      ROS_INFO("3D Sonar sensor plugin loaded: %s", sensor_->Name().c_str());
    }

  private:
    void OnUpdate()
    {

      if (!ros::ok())
        return;

      // 获取物理世界（如果需要）
      if (!world_)
      {
        world_ = physics::get_world(sensor_->WorldName());
        if (!world_)
        {
          ROS_ERROR("Still failed to get physics world instance");
          return;
        }
      }
      // 检查传感器是否已激活
      if (!sensor_->IsActive())
      {
        sensor_->SetActive(true);
        return; // 本次更新跳过处理
      }

      if (sensor_->RangeCount() <= 0)
      {
        ROS_WARN_THROTTLE(5.0, "Range data not ready yet");
        return;
      }

      // 添加全面的空指针和状态检查
      if (!sensor_ || !ros_node_ || !pointcloud_pub_ ||
          horizontal_samples_ <= 0 || vertical_samples_ <= 0)
      {
        ROS_ERROR("[Sonar] Invalid state detected, skipping update");
        return;
      }
      // 获取最新的3D扫描数据
      std::vector<double> ranges;
      try
      {
        // 使用安全的方法获取数据
        sensor_->Ranges(ranges);
      }
      catch (const std::exception &e)
      {
        ROS_ERROR("Failed to get ranges: %s", e.what());
        return;
      }

      // 检查范围向量大小是否与预期一致
      size_t expected_ranges = horizontal_samples_ * vertical_samples_;
      if (ranges.size() != expected_ranges)
      {
        ROS_WARN("[Sonar] Ranges vector size mismatch: expected %lu, got %lu",
                 expected_ranges, ranges.size());
        return;
      }

      // 位姿获取前检查
      if (!sensor_)
      {
        ROS_ERROR("[Sonar] Sensor pointer is null when getting pose!");
        return;
      }
      // ROS_INFO("r, %.2lf\n", ranges[0]);

      // 创建点云
      pcl::PointCloud<pcl::PointXYZ> cloud;
      cloud.header.frame_id = output_frame_;
      cloud.is_dense = false;
      cloud.points.reserve(horizontal_samples_ * vertical_samples_);

      // 获取传感器当前在世界坐标系中的位姿
      ignition::math::Pose3d sensor_pose = GetSensorWorldPose();

      // 直接从四元数获取旋转分量
      ignition::math::Quaterniond rotation = sensor_pose.Rot();
      ignition::math::Vector3d position = sensor_pose.Pos();

      // 预计算四元数的分量用于高效转换
      const double qw = rotation.W();
      const double qx = rotation.X();
      const double qy = rotation.Y();
      const double qz = rotation.Z();

      // 转换为3D点云
      for (int v = 0; v < vertical_samples_; ++v)
      {
        double vertical_angle = vertical_min_angle_ + v * vertical_angle_step_;
        double cos_vertical = cos(vertical_angle);
        double sin_vertical = sin(vertical_angle);

        for (int h = 0; h < horizontal_samples_; ++h)
        {
          size_t index = v * horizontal_samples_ + h;
          if (index >= ranges.size())
            break;

          double range = ranges[index];

          // 跳过无效距离
          if (range < sensor_->RangeMin() || range > sensor_->RangeMax())
          {
            continue;
          }

          double horizontal_angle = horizontal_min_angle_ + h * horizontal_angle_step_;
          double cos_horizontal = cos(horizontal_angle);
          double sin_horizontal = sin(horizontal_angle);

          // 1. 传感器坐标系中的点（局部坐标系）
          double x = range * cos_vertical * cos_horizontal;
          double y = range * cos_vertical * sin_horizontal;
          double z = range * sin_vertical;
          // ROS_INFO("%.2lf, %.2lf, %.2lf\n", position.X(), position.Y(), position.Z());
          // ROS_INFO("%.2lf, %.2lf, %.2lf\n", x, y, z);

          // 2. 使用四元数旋转向量（绕过矩阵API问题）
          // 直接应用四元数旋转公式
          double tx = (1.0 - 2 * qy * qy - 2 * qz * qz) * x + (2 * qx * qy - 2 * qz * qw) * y + (2 * qx * qz + 2 * qy * qw) * z;
          double ty = (2 * qx * qy + 2 * qz * qw) * x + (1.0 - 2 * qx * qx - 2 * qz * qz) * y + (2 * qy * qz - 2 * qx * qw) * z;
          double tz = (2 * qx * qz - 2 * qy * qw) * x + (2 * qy * qz + 2 * qx * qw) * y + (1.0 - 2 * qx * qx - 2 * qy * qy) * z;

          // 3. 应用平移（转换为世界坐标系）
          pcl::PointXYZ point;
          point.x = tx + position.X();
          point.y = ty + position.Y();
          point.z = tz + position.Z();
          // ROS_INFO("%.2lf, %.2lf, %.2lf\n\n", point.x, point.y, point.z);

          cloud.points.push_back(point);
        }
      }

      // 发布点云
      if (cloud.points.size() > 0)
      {
        cloud.width = cloud.points.size();
        cloud.height = 1;
        sensor_msgs::PointCloud2 msg;
        pcl::toROSMsg(cloud, msg);
        msg.header.frame_id = output_frame_;
        msg.header.stamp = ros::Time::now();
        pointcloud_pub_.publish(msg);
      }
    }

    // 获取传感器在世界坐标系中的位姿
    ignition::math::Pose3d GetSensorWorldPose()
    {
      if (!parent_entity_)
      {
        ROS_ERROR_THROTTLE(1.0, "No parent entity for sensor");
        return ignition::math::Pose3d();
      }

      // 获取父实体的世界位姿
      ignition::math::Pose3d parent_pose = parent_entity_->WorldPose();

      // 获取传感器相对于父实体的位姿
      ignition::math::Pose3d sensor_relative_pose = sensor_->Pose();

      // 计算完整的世界位姿
      return parent_pose * sensor_relative_pose;
    }

    // 传感器参数
    int horizontal_samples_;
    int vertical_samples_;
    double horizontal_min_angle_;
    double horizontal_max_angle_;
    double horizontal_angle_step_;
    double vertical_min_angle_;
    double vertical_max_angle_;
    double vertical_angle_step_;

    // ROS组件
    sensors::RaySensorPtr sensor_;
    ros::NodeHandle *ros_node_;
    //  【核心修正3】将原始指针 ros_node_* 替换为智能指针
    // std::unique_ptr<ros::NodeHandle> ros_node_;
    ros::Publisher pointcloud_pub_;
    event::ConnectionPtr update_connection_;

    // 父实体（传感器附加的对象）
    physics::EntityPtr parent_entity_;

    physics::WorldPtr world_;
    std::string output_frame_;
  };

  GZ_REGISTER_SENSOR_PLUGIN(Sonar3DSensor)

} // namespace gazebo