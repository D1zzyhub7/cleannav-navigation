// 调用系统真实 MPPI 后再次约束滤波结果；不改采样、critics 或 Safety。
#include "nav2_mppi_controller/controller.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "cleannav_navigation/ackermann_output.hpp"
namespace cleannav_navigation {
class BoundedMPPIController : public nav2_mppi_controller::MPPIController {
 public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    const std::shared_ptr<tf2_ros::Buffer> tf,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap) override {
    nav2_mppi_controller::MPPIController::configure(parent,name,tf,costmap);
    auto node=parent.lock();
    if(node->get_parameter(name+".motion_model").as_string()!="Ackermann")
      throw std::runtime_error("BoundedMPPIController requires Ackermann");
    vx_min_=node->get_parameter(name+".vx_min").as_double();
    vx_max_=node->get_parameter(name+".vx_max").as_double();
    wz_max_=node->get_parameter(name+".wz_max").as_double();
    radius_=node->get_parameter(name+".AckermannConstraints.min_turning_r").as_double();
    constrain_output(geometry_msgs::msg::Twist(),vx_min_,vx_max_,wz_max_,radius_);
  }
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & speed,
    nav2_core::GoalChecker * checker) override {
    auto command=nav2_mppi_controller::MPPIController::computeVelocityCommands(pose,speed,checker);
    command.twist=constrain_output(command.twist,vx_min_,vx_max_,wz_max_,radius_);
    return command;
  }
 private:
  double vx_min_=0.,vx_max_=0.,wz_max_=0.,radius_=1.12;
};
}
PLUGINLIB_EXPORT_CLASS(cleannav_navigation::BoundedMPPIController,nav2_core::Controller)
