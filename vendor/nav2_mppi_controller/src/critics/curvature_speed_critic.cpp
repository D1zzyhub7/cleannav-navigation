#include "nav2_mppi_controller/critics/curvature_speed_critic.hpp"
#include <algorithm>
#include <limits>
#include <cmath>
#include <stdexcept>
namespace mppi::critics {
void CurvatureSpeedCritic::initialize()
{
  auto get=parameters_handler_->getParamGetter(name_);
  auto parent=parameters_handler_->getParamGetter(parent_name_);
  get(weight_,"cost_weight",5.0);
  get(alat_,"max_lateral_accel",0.0);
  get(decel_,"deceleration",0.0);
  get(span_,"curvature_span",0.1);
  parent(vmax_,"vx_max",0.3);
  parent(wmax_,"wz_max",0.267857142857);
  if (!(alat_>0 && decel_>0 && span_>0 && vmax_>0 && wmax_>0 && weight_>=0)) {
    throw std::invalid_argument("CurvatureSpeedCritic requires sourced positive speed budgets");
  }
}
void CurvatureSpeedCritic::score(CriticData & data)
{
  if (!enabled_) {return;}
  const auto p=curvatureSpeedProfile(data.curvature_path ? *data.curvature_path : data.path,vmax_,wmax_,alat_,decel_,span_);
  if (p.empty()) {return;}
  for (size_t n=0; n<data.state.vx.shape(0); ++n) {
    float sum=0.0f;
    for (size_t t=0; t<data.state.vx.shape(1); ++t) {
      const float x=data.trajectories.x(n,t), y=data.trajectories.y(n,t);
      // Closest segment projection yields continuous arc-length reference.
      float best=std::numeric_limits<float>::max(), reference=p.front().speed;
      for (size_t j=1; j<p.size(); ++j) {
        const float dx=p[j].x-p[j-1].x, dy=p[j].y-p[j-1].y;
        const float q=std::clamp(((x-p[j-1].x)*dx+(y-p[j-1].y)*dy)/
          (dx*dx+dy*dy),0.0f,1.0f);
        const float ex=x-p[j-1].x-q*dx, ey=y-p[j-1].y-q*dy;
        const float d=ex*ex+ey*ey;
        if (d<best) {best=d;reference=p[j-1].speed+q*(p[j].speed-p[j-1].speed);}
      }
      const float excess=std::max(0.0f,(data.state.vx(n,t)-reference)/vmax_);
      sum+=excess*excess;
    }
    data.costs(n)+=weight_*data.model_dt*sum;
  }
}
}
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mppi::critics::CurvatureSpeedCritic,mppi::critics::CriticFunction)
