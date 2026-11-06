#pragma once

#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace cleannav_simulation
{
// NaN means invalid observation: it must never become a free-space clearing ray.
inline std::vector<size_t> mask_self_returns(
  std::vector<float> & ranges, const std::vector<std::string> & collisions,
  const std::string & robot)
{
  if (robot.empty() || ranges.size() != collisions.size()) {
    throw std::invalid_argument("self filter requires a robot name and one entity per ray");
  }
  const auto prefix = robot + "::";
  std::vector<size_t> masked;
  for (size_t i = 0; i < ranges.size(); ++i) {
    if (std::isfinite(ranges[i]) && collisions[i].compare(0, prefix.size(), prefix) == 0) {
      ranges[i] = std::numeric_limits<float>::quiet_NaN();
      masked.push_back(i);
    }
  }
  return masked;
}
}  // namespace cleannav_simulation
