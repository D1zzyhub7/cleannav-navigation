// Internal R7 soft speed objective. No external ROS control interface.
#pragma once
#include "nav2_mppi_controller/critic_function.hpp"
#include <vector>
namespace mppi::critics {
struct CurveSpeedPoint {float x, y, s, kappa, speed;};
std::vector<CurveSpeedPoint> curvatureSpeedProfile(
  const models::Path & path, float vmax, float wmax, float alat, float decel, float span);
class CurvatureSpeedCritic : public CriticFunction {
public:
  void initialize() override;
  void score(CriticData & data) override;
private:
  float weight_, vmax_, wmax_, alat_, decel_, span_;
};
}
