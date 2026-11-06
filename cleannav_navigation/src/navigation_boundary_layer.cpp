// Additional task geometry restriction. Does not replace Keepout/Unknown/Safety.
#include <algorithm>
#include <cmath>
#include <mutex>
#include <stdexcept>
#include <vector>
#include <nav2_costmap_2d/layer.hpp>
#include <nav2_costmap_2d/layered_costmap.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cleannav_interfaces/msg/navigation_boundary.hpp>
#include <cleannav_interfaces/msg/navigation_boundary_state.hpp>

namespace cleannav_navigation {
struct XY {double x, y;};
static double cross(XY a, XY b, XY c) {
  return (b.x-a.x)*(c.y-a.y)-(b.y-a.y)*(c.x-a.x);
}
static double distance_segment(XY p, XY a, XY b) {
  const double dx=b.x-a.x,dy=b.y-a.y, n=dx*dx+dy*dy;
  const double t=n>0?std::clamp(((p.x-a.x)*dx+(p.y-a.y)*dy)/n,0.,1.):0.;
  return std::hypot(p.x-a.x-t*dx,p.y-a.y-t*dy);
}
static bool crosses(XY a, XY b, XY c, XY d) {
  if (distance_segment(a,c,d)<1e-9 || distance_segment(b,c,d)<1e-9 ||
      distance_segment(c,a,b)<1e-9 || distance_segment(d,a,b)<1e-9) return true;
  return (cross(a,b,c)>0)!=(cross(a,b,d)>0) && (cross(c,d,a)>0)!=(cross(c,d,b)>0);
}
static bool valid_polygon(const std::vector<XY> & ps) {
  if (ps.size()<3 || ps.size()>256) return false;
  double area=0;
  for (size_t i=0;i<ps.size();++i) {
    const auto a=ps[i],b=ps[(i+1)%ps.size()];
    if (!std::isfinite(a.x)||!std::isfinite(a.y)||std::hypot(a.x-b.x,a.y-b.y)<1e-6) return false;
    area+=a.x*b.y-a.y*b.x;
    for (size_t j=i+1;j<ps.size();++j) {
      if (j==i+1 || (i==0 && j==ps.size()-1)) continue;
      if (crosses(a,b,ps[j],ps[(j+1)%ps.size()])) return false;
    }
  }
  return std::abs(area)>.0001;
}
static bool contains_disk(const std::vector<XY> & ps, XY p, double radius) {
  bool inside=false; double distance=1e20;
  for (size_t i=0,j=ps.size()-1;i<ps.size();j=i++) {
    const auto a=ps[i], b=ps[j];
    if ((a.y>p.y)!=(b.y>p.y) && p.x<(b.x-a.x)*(p.y-a.y)/(b.y-a.y)+a.x) inside=!inside;
    distance=std::min(distance,distance_segment(p,a,b));
  }
  return inside && distance>radius+1e-9;
}

class NavigationBoundaryLayer : public nav2_costmap_2d::Layer {
public:
  void onInitialize() override {
    auto node=node_.lock(); if(!node) throw std::runtime_error("boundary layer has no node");
    declareParameter("enabled",rclcpp::ParameterValue(true));
    declareParameter("minimum_clearance_m",rclcpp::ParameterValue(0.));
    node->get_parameter(name_+".enabled",enabled_);
    node->get_parameter(name_+".minimum_clearance_m",minimum_clearance_);
    if (!std::isfinite(minimum_clearance_) || minimum_clearance_<0.)
      throw std::runtime_error("invalid navigation boundary clearance");
    costmap_id_=node->get_node_base_interface()->get_fully_qualified_name();
    publisher_=node->create_publisher<cleannav_interfaces::msg::NavigationBoundaryState>(
      "/cleannav/navigation/boundary_state",rclcpp::QoS(10).reliable());
    publisher_->on_activate();
    rclcpp::SubscriptionOptions options; options.callback_group=callback_group_;
    subscription_=node->create_subscription<cleannav_interfaces::msg::NavigationBoundary>(
      "/cleannav/navigation/boundary",rclcpp::QoS(1).transient_local().reliable(),
      [this](cleannav_interfaces::msg::NavigationBoundary::SharedPtr m) {
        std::lock_guard<std::mutex> guard(mutex_);
        // A generation is immutable. Invalid replacements block the whole map.
        if (message_ && m->token==message_->token && *message_!=*m) {
          valid_=false;active_=true;reason_="constraint generation mutated";return;
        }
        message_=m;polygon_.clear();token_=m->token;active_=m->active;
        for(const auto &p:m->boundary.polygon.points) polygon_.push_back({p.x,p.y});
        valid_=!token_.empty() && (!active_ ||
          (m->boundary.header.frame_id=="map" && std::isfinite(m->clearance_m) &&
          m->clearance_m>=minimum_clearance_ && m->clearance_m<=2. && valid_polygon(polygon_)));
        if (!valid_) active_=true;
        clearance_=m->clearance_m;reason_=valid_?"":"invalid navigation boundary";
        current_=false;
      },options);
    current_=true;
  }
  void reset() override {current_=false;} // Recovery cannot clear a task boundary.
  bool isClearable() override {return false;}
  void updateBounds(double,double,double,double*minx,double*miny,double*maxx,double*maxy) override {
    auto m=layered_costmap_->getCostmap();
    *minx=std::min(*minx,m->getOriginX());*miny=std::min(*miny,m->getOriginY());
    *maxx=std::max(*maxx,m->getOriginX()+m->getSizeInMetersX());
    *maxy=std::max(*maxy,m->getOriginY()+m->getSizeInMetersY());
  }
  void updateCosts(nav2_costmap_2d::Costmap2D &master,int mini,int minj,int maxi,int maxj) override {
    std::lock_guard<std::mutex> guard(mutex_);
    auto node=node_.lock(); if(!node) return;
    bool ready=enabled_ && valid_;
    tf2::Transform map_from_global;map_from_global.setIdentity();
    std::string reason=reason_;
    if(ready && active_ && layered_costmap_->getGlobalFrameID()!="map") {
      try {
        const auto t=tf_->lookupTransform("map",layered_costmap_->getGlobalFrameID(),tf2::TimePointZero);
        const double age=(node->now()-rclcpp::Time(t.header.stamp)).seconds();
        if ((t.header.stamp.sec || t.header.stamp.nanosec) && (age<-.3 || age>.5)) throw std::runtime_error("stale map transform");
        tf2::fromMsg(t.transform,map_from_global);
      } catch(const std::exception &e) {ready=false;reason=e.what();}
    }
    // Preserve every existing cost, including unknown=255. Mark a cell lethal
    // unless its complete square + task margin lies inside the polygon. The
    // half-diagonal bound applies to the cell, not to the vehicle footprint.
    if(active_ || !ready) {
      const double radius=clearance_+master.getResolution()*std::sqrt(2.)*.5;
      for(int y=minj;y<maxj;++y)for(int x=mini;x<maxi;++x) {
        double wx,wy;master.mapToWorld(x,y,wx,wy);
        const auto p=map_from_global*tf2::Vector3(wx,wy,0.);
        if(!ready || !contains_disk(polygon_,{p.x(),p.y()},radius))
          master.setCost(x,y,std::max(master.getCost(x,y),nav2_costmap_2d::LETHAL_OBSTACLE));
      }
    }
    current_=ready;
    cleannav_interfaces::msg::NavigationBoundaryState s;
    s.header.stamp=node->now();s.header.frame_id=layered_costmap_->getGlobalFrameID();
    s.costmap_id=costmap_id_;s.token=token_;s.active=active_;s.ready=ready;s.reason=reason;
    publisher_->publish(s);
  }
private:
  std::mutex mutex_;std::vector<XY> polygon_;
  bool active_{false},valid_{true}; double clearance_{0.},minimum_clearance_{0.};
  std::string token_,costmap_id_,reason_;
  cleannav_interfaces::msg::NavigationBoundary::SharedPtr message_;
  rclcpp::Subscription<cleannav_interfaces::msg::NavigationBoundary>::SharedPtr subscription_;
  rclcpp_lifecycle::LifecyclePublisher<cleannav_interfaces::msg::NavigationBoundaryState>::SharedPtr publisher_;
};
}
// Narrow installed-library ABI for geometry parity tests, without a ROS graph.
extern "C" bool cleannav_navigation_boundary_contains_disk(const double*points,int count,double x,double y,double radius) {
  std::vector<cleannav_navigation::XY> ps;
  if(count<3 || count>256 || !std::isfinite(radius) || radius<0) return false;
  for(int i=0;i<count;++i)ps.push_back({points[2*i],points[2*i+1]});
  return cleannav_navigation::valid_polygon(ps) && cleannav_navigation::contains_disk(ps,{x,y},radius);
}
PLUGINLIB_EXPORT_CLASS(cleannav_navigation::NavigationBoundaryLayer,nav2_costmap_2d::Layer)
