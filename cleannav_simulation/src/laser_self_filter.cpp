#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo_ros/node.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>

#include <sstream>
#include "cleannav_simulation/laser_self_filter.hpp"

namespace cleannav_simulation
{
class LaserSelfFilter : public gazebo::SensorPlugin
{
public:
  void Load(gazebo::sensors::SensorPtr sensor, sdf::ElementPtr sdf) override
  {
    sensor_ = std::dynamic_pointer_cast<gazebo::sensors::RaySensor>(sensor);
    if (!sensor_ || sensor_->VerticalRayCount() != 1 ||
      sensor_->VerticalRangeCount() != 1 || sensor_->RayCount() != sensor_->RangeCount())
    {
      gzerr << "CleanNav self filter requires a 2D ray sensor without interpolation\n";
      return;  // No scan output: readiness and Safety cannot authorize motion.
    }
    node_ = gazebo_ros::Node::Get(sdf);
    frame_ = sdf->Get<std::string>("frame_name");
    robot_ = sdf->Get<std::string>("robot_name");
    publisher_ = node_->create_publisher<sensor_msgs::msg::LaserScan>(
      "~/out", node_->get_qos().get_publisher_qos("~/out", rclcpp::SensorDataQoS().reliable()));
    audit_ = node_->create_publisher<std_msgs::msg::String>(
      "/cleannav/simulation/lidar_self_filter", 10);
    // Sensor::Update emits this event after RaySensor::UpdateImpl, on that
    // sensor's own thread. Ranges include the original noise; entity names
    // come from the same completed physics update, not a late ROS callback.
    updated_ = sensor_->ConnectUpdated(std::bind(&LaserSelfFilter::Publish, this));
  }

  ~LaserSelfFilter() override {updated_.reset();}

private:
  void Publish()
  {
    sensor_msgs::msg::LaserScan scan;
    const auto stamp = sensor_->LastMeasurementTime();
    scan.header.stamp.sec = stamp.sec;
    scan.header.stamp.nanosec = stamp.nsec;
    scan.header.frame_id = frame_;
    scan.angle_min = sensor_->AngleMin().Radian();
    scan.angle_max = sensor_->AngleMax().Radian();
    scan.angle_increment = (sensor_->AngleMax().Radian() - sensor_->AngleMin().Radian()) /
      (sensor_->RangeCount() - 1);
    scan.range_min = sensor_->RangeMin();
    scan.range_max = sensor_->RangeMax();
    std::vector<double> ranges;
    sensor_->Ranges(ranges);
    scan.ranges.assign(ranges.begin(), ranges.end());
    std::vector<std::string> collisions;
    auto shape = sensor_->LaserShape();
    for (size_t i = 0; i < ranges.size(); ++i) {
      collisions.push_back(shape->Ray(i)->CollisionName());
      scan.intensities.push_back(sensor_->Retro(i));
    }
    const auto masked = mask_self_returns(scan.ranges, collisions, robot_);
    publisher_->publish(scan);
    if (!masked.empty()) {
      std::ostringstream text;
      text << "{\"stamp\":[" << stamp.sec << "," << stamp.nsec << "],\"masked\":[";
      for (size_t j = 0; j < masked.size(); ++j) {
        if (j) {text << ',';}
        text << "{\"index\":" << masked[j] << ",\"collision\":\""
             << collisions[masked[j]] << "\",\"raw_range\":" << ranges[masked[j]] << '}';
      }
      text << "]}";
      std_msgs::msg::String event;
      event.data = text.str();
      audit_->publish(event);
    }
  }

  gazebo::sensors::RaySensorPtr sensor_;
  gazebo::event::ConnectionPtr updated_;
  gazebo_ros::Node::SharedPtr node_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr audit_;
  std::string frame_, robot_;
};
GZ_REGISTER_SENSOR_PLUGIN(LaserSelfFilter)
}  // namespace cleannav_simulation
