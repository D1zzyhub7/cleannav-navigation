// 最终滤波输出约束；共享真实组件回归，禁止非有限控制量。
#pragma once
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include "geometry_msgs/msg/twist.hpp"
namespace cleannav_navigation {
inline geometry_msgs::msg::Twist constrain_output(
  const geometry_msgs::msg::Twist & input, double vx_min, double vx_max, double wz_max, double radius)
{
  if (!std::isfinite(vx_min) || !std::isfinite(vx_max) || !std::isfinite(wz_max) ||
      !std::isfinite(radius) || vx_min>0 || vx_max<0 || wz_max<0 || radius<=0)
    throw std::runtime_error("invalid Ackermann output bounds");
  if (!std::isfinite(input.linear.x) || !std::isfinite(input.linear.y) ||
      !std::isfinite(input.linear.z) || !std::isfinite(input.angular.x) ||
      !std::isfinite(input.angular.y) || !std::isfinite(input.angular.z))
    throw std::runtime_error("non-finite filtered MPPI output");
  auto output=geometry_msgs::msg::Twist();
  output.linear.x=std::clamp(input.linear.x,vx_min,vx_max);
  double scaled=input.linear.x!=0 ? input.angular.z*(output.linear.x/input.linear.x) : 0.;
  double angular_bound=std::min(wz_max,std::abs(output.linear.x)/radius);
  output.angular.z=std::clamp(scaled,-angular_bound,angular_bound);
  return output;
}
}
