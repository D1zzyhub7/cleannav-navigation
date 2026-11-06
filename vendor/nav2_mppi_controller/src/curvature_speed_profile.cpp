#include "nav2_mppi_controller/critics/curvature_speed_critic.hpp"
#include <algorithm>
#include <cmath>
namespace mppi::critics {
std::vector<CurveSpeedPoint> curvatureSpeedProfile(
  const models::Path & path, float vmax, float wmax, float alat, float decel, float span)
{
  std::vector<CurveSpeedPoint> p;
  for (size_t i = 0; i < path.x.size(); ++i) {
    const float x = path.x(i), y = path.y(i);
    const float ds = p.empty() ? 0.0f : std::hypot(x-p.back().x, y-p.back().y);
    if (!p.empty() && ds < 1e-5f) {continue;}
    p.push_back({x, y, p.empty() ? 0.0f : p.back().s+ds, 0.0f, vmax});
  }
  if (p.size() < 3) {return p;}
  for (size_t i = 0; i < p.size(); ++i) {
    size_t a = i, c = i;
    while (a > 0 && p[i].s-p[a].s < span) {--a;}
    while (c+1 < p.size() && p[c].s-p[i].s < span) {++c;}
    // One-sided circle at endpoints; no zero-curvature blind spot at entry.
    if (a == i) {while (c+1 < p.size() && p[c].s-p[a].s < 2*span) {++c;}}
    if (c == i) {while (a > 0 && p[c].s-p[a].s < 2*span) {--a;}}
    const size_t b = (a+c)/2;
    const float ux=p[b].x-p[a].x, uy=p[b].y-p[a].y;
    const float vx=p[c].x-p[b].x, vy=p[c].y-p[b].y;
    const float denominator=std::hypot(ux,uy)*std::hypot(vx,vy)*
      std::hypot(p[c].x-p[a].x,p[c].y-p[a].y);
    const float k=denominator > 1e-8f ? 2*(ux*vy-uy*vx)/denominator : 0.0f;
    p[i].kappa=k;
  }
  // Circle fitting straddles a curvature step. Its support-radius envelope
  // prevents that averaging delay from moving braking inside the bend.
  const auto geometry = p;
  for (size_t i = 0; i < p.size(); ++i) {
    for (size_t j = i; j > 0 && p[i].s-p[j-1].s <= span; --j) {
      if (std::abs(geometry[j-1].kappa) > std::abs(p[i].kappa)) {
        p[i].kappa = geometry[j-1].kappa;
      }
    }
    for (size_t j = i+1; j < p.size() && p[j].s-p[i].s <= span; ++j) {
      if (std::abs(geometry[j].kappa) > std::abs(p[i].kappa)) {
        p[i].kappa = geometry[j].kappa;
      }
    }
    const float abs_k=std::abs(p[i].kappa);
    if (abs_k > 1e-6f) {
      p[i].speed=std::min({vmax,std::sqrt(alat/abs_k),wmax/abs_k});
    }
  }
  // Independent of the distance travelled by the current rollout.
  for (size_t i = p.size()-1; i > 0; --i) {
    p[i-1].speed=std::min(p[i-1].speed,std::sqrt(
      p[i].speed*p[i].speed+2*decel*(p[i].s-p[i-1].s)));
  }
  for (size_t i = 1; i < p.size(); ++i) {
    p[i].speed=std::min(p[i].speed,std::sqrt(
      p[i-1].speed*p[i-1].speed+2*decel*(p[i].s-p[i-1].s)));
  }
  return p;
}
}
