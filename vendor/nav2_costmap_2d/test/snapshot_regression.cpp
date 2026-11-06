// Offline regression of the real Filter/LayeredCostmap/Publisher; no task or motion nodes.
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <thread>
#include <unistd.h>
#include <nlohmann/json.hpp>
#include "nav2_costmap_2d/costmap_filters/keepout_filter.hpp"
#include "nav2_costmap_2d/costmap_2d_publisher.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "snapshot_legacy_keepout.hpp"
#include "tf2_ros/buffer.h"
#include "pluginlib/class_loader.hpp"
using json = nlohmann::json;
using namespace nav2_costmap_2d;
static int checks = 0;
static void check(bool good, const std::string & why) {
  if (!good) {throw std::runtime_error(why);} ++checks;
}
static rclcpp::Time stamp(double s) {return rclcpp::Time(static_cast<int64_t>(std::llround(s * 1e9)), RCL_ROS_TIME);}
static std::vector<unsigned char> bytes(const std::string & path) {
  std::ifstream f(path, std::ios::binary); if (!f) {throw std::runtime_error(path);}
  return std::vector<unsigned char>(std::istreambuf_iterator<char>(f), {});
}
static void save(const std::string & path, const std::vector<unsigned char> & data) {
  std::ofstream f(path, std::ios::binary);f.write(reinterpret_cast<const char *>(data.data()), data.size());
}
static std::vector<unsigned char> data(Costmap2D & c) {
  return {c.getCharMap(), c.getCharMap()+c.getSizeInCellsX()*c.getSizeInCellsY()};
}
class RecordingBuffer : public tf2_ros::Buffer {
public:
  explicit RecordingBuffer(rclcpp::Clock::SharedPtr c): Buffer(c) {setUsingDedicatedThread(true);}
  mutable std::vector<int64_t> requested, returned;
  geometry_msgs::msg::TransformStamped lookupTransform(const std::string & t,
    const std::string & s, const tf2::TimePoint & time, tf2::Duration timeout) const override {
    requested.push_back(time.time_since_epoch().count());
    auto value=Buffer::lookupTransform(t,s,time,timeout);
    returned.push_back(rclcpp::Time(value.header.stamp).nanoseconds());return value;
  }
  void put(double t, const json & p) {
    geometry_msgs::msg::TransformStamped v;v.header.frame_id="map";v.child_frame_id="odom";
    v.header.stamp=stamp(t);v.transform.translation.x=p[0];v.transform.translation.y=p[1];
    tf2::Quaternion q;q.setRPY(0,0,p[2]);v.transform.rotation=tf2::toMsg(q);setTransform(v,"offline",false);
  }
};
namespace nav2_costmap_2d {
class SnapshotRegressionAccess {
public:
  template<class F> static void configure(F & f, RecordingBuffer & b,
    const nav_msgs::msg::OccupancyGrid & mask, rclcpp::Clock::SharedPtr clock) {
    f.mask_costmap_=std::make_unique<Costmap2D>(mask);f.mask_frame_="map";f.global_frame_="odom";
    f.tf_=&b;f.clock_=clock;f.enabled_=true;f.current_=false;
    f.transform_tolerance_=tf2::durationFromSec(.03);
  }
  static nav2_msgs::msg::Costmap raw(Costmap2DPublisher & p) {
    std::unique_lock<Costmap2D::mutex_t> lock(*p.costmap_->getMutex());
    if (!p.snapshot_valid_) {return nav2_msgs::msg::Costmap();}
    p.prepareCostmap();return *p.costmap_raw_;
  }
  static nav_msgs::msg::OccupancyGrid grid(Costmap2DPublisher & p) {
    std::unique_lock<Costmap2D::mutex_t> lock(*p.costmap_->getMutex());
    p.prepareGrid();return *p.grid_;
  }
  static nav2_msgs::msg::Costmap service(Costmap2DPublisher & p) {
    auto r=std::make_shared<nav2_msgs::srv::GetCostmap::Response>();
    p.costmap_service_callback(nullptr,nullptr,r);return r->map;
  }
};
}
class ReplayLayer : public Layer {
public:
  std::vector<unsigned char> values;
  double last_origin_x = 0;
  explicit ReplayLayer(LayeredCostmap & owner, const std::vector<unsigned char> & v): values(v) {
    layered_costmap_=&owner;enabled_=current_=true;
  }
  void onInitialize() override {}
  void reset() override {}
  bool isClearable() override {return false;}
  void updateBounds(double,double,double,double * x0,double * y0,double * x1,double * y1) override {
    auto c=layered_costmap_->getCostmap();*x0=c->getOriginX();*y0=c->getOriginY();
    *x1=*x0+c->getSizeInMetersX();*y1=*y0+c->getSizeInMetersY();
  }
  void updateCosts(Costmap2D & c,int,int,int,int) override {last_origin_x=c.getOriginX();std::copy(values.begin(),values.end(),c.getCharMap());}
};
static int badCore(Costmap2D & c, const nav_msgs::msg::OccupancyGrid & m,
  const json & tf, int * count=nullptr) {
  int bad=0, core=0;double q=tf[2],cs=std::cos(q),sn=std::sin(q);
  for (unsigned int y=0;y<c.getSizeInCellsY();++y) for(unsigned int x=0;x<c.getSizeInCellsX();++x) {
    double a,b;c.mapToWorld(x,y,a,b);double mx=double(tf[0])+cs*a-sn*b,my=double(tf[1])+sn*a+cs*b;
    int col=std::floor((mx-m.info.origin.position.x)/m.info.resolution);
    int row=std::floor((my-m.info.origin.position.y)/m.info.resolution);
    if(col>=0&&row>=0&&col<int(m.info.width)&&row<int(m.info.height)&&m.data[row*m.info.width+col]==100) {
      ++core;if(c.getCost(x,y)!=254) {++bad;}
    }
  }
  if (count) {*count=core;} return bad;
}
static void geometry(LayeredCostmap & l,const json & j) {
  l.resizeMap(j["width"],j["height"],j["resolution"],j["origin"][0],j["origin"][1]);
}
class StampFilter : public CostmapFilter {
public:
  std::vector<int64_t> seen;
  explicit StampFilter(LayeredCostmap & l) {layered_costmap_=&l;enabled_=true;}
  void initializeFilter(const std::string &) override {}
  void resetFilter() override {}
  void process(Costmap2D & c,int,int,int,int,const geometry_msgs::msg::Pose2D &) override {
    std::fill(c.getCharMap(),c.getCharMap()+c.getSizeInCellsX()*c.getSizeInCellsY(),17);
  }
  void processAtTime(Costmap2D & c,int,int,int,int,const geometry_msgs::msg::Pose2D &,
    const rclcpp::Time & s) override {
    seen.push_back(s.nanoseconds());std::fill(c.getCharMap(),c.getCharMap()+c.getSizeInCellsX()*c.getSizeInCellsY(),(s.seconds()>0?int(s.seconds()):0)%251);
  }
};
class LegacyFilter : public CostmapFilter {
public:
  LegacyFilter(){enabled_=true;}
  void initializeFilter(const std::string &) override {}
  void resetFilter() override {}
  void process(Costmap2D & c,int,int,int,int,const geometry_msgs::msg::Pose2D &) override {c.setCost(0,0,19);}
};
int main(int argc,char ** argv) {
  try {
    if(argc!=3){throw std::runtime_error("fixture directory and output directory required");}
    std::string root=argv[1],out=argv[2];json j;std::ifstream(root+"/fixtures.json")>>j;
    auto clock=std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
    nav_msgs::msg::OccupancyGrid mask;auto jm=j["mask"];mask.header.frame_id="map";
    mask.info.width=jm["width"];mask.info.height=jm["height"];mask.info.resolution=jm["resolution"];
    mask.info.origin.position.x=jm["origin"][0];mask.info.origin.position.y=jm["origin"][1];mask.info.origin.orientation.w=1;
    auto mb=bytes(root+"/mask.bin");mask.data.assign(mb.begin(),mb.end());auto input=bytes(root+"/failed.bin");
    auto spec=j["failed"];RecordingBuffer buffer(clock);buffer.put(110.1,j["old_tf"]);buffer.put(110.2,j["new_tf"]);
    Costmap2D old(spec["width"],spec["height"],spec["resolution"],spec["origin"][0],spec["origin"][1]);
    std::copy(input.begin(),input.end(),old.getCharMap());LegacyKeepoutFilter original;
    SnapshotRegressionAccess::configure(original,buffer,mask,clock);geometry_msgs::msg::Pose2D pose;
    original.process(old,0,0,100,100,pose);int core=0;
    check(badCore(old,mask,j["old_tf"],&core)==8&&core==2144,"A upstream latest vs110.1 must reproduce8");
    save(out+"/A-original.bin",data(old));check(buffer.requested.back()==0,"A must actually request TimePointZero");
    LayeredCostmap l("odom",false,true);geometry(l,spec);
    auto replay=std::make_shared<ReplayLayer>(l,input);l.addPlugin(replay);
    auto fixed=std::make_shared<KeepoutFilter>();SnapshotRegressionAccess::configure(*fixed,buffer,mask,clock);l.addFilter(fixed);
    l.updateMap(0,0,0,stamp(110.1));
    check(badCore(*l.getCostmap(),mask,j["old_tf"],&core)==0&&core==2144,"B2144core must all254");
    check(buffer.requested.back()==stamp(110.1).nanoseconds()&&buffer.returned.back()==stamp(110.1).nanoseconds(),"B exactrequested/returnedstamp");
    for(size_t i=0;i<input.size();++i) if(input[i]==254){check(l.getCostmap()->getCharMap()[i]==254,"B preserve original254");}
    save(out+"/B-fixed.bin",data(*l.getCostmap()));
    // A single native serialization fixture node; no Navigation/MM/Safety/Gateway.
    rclcpp::init(argc,argv);auto options=rclcpp::NodeOptions().enable_rosout(false).start_parameter_services(false).start_parameter_event_publisher(false);
    auto node=std::make_shared<nav2_util::LifecycleNode>("snapshot_native_"+std::to_string(getpid()),"",options);
    Costmap2DPublisher pub(node,l.getCostmap(),"odom","snapshot_native_"+std::to_string(getpid()),true);
    check(SnapshotRegressionAccess::service(pub).data.empty(),"uncommitted snapshot cannot be served");
    pub.commitSnapshot(stamp(110.1));auto raw=SnapshotRegressionAccess::raw(pub);auto grid=SnapshotRegressionAccess::grid(pub);auto rpc=SnapshotRegressionAccess::service(pub);
    check(rclcpp::Time(raw.header.stamp).nanoseconds()==buffer.returned.back(),"FilterreturnedTF/rawheader same");
    check(raw.header.stamp==rpc.header.stamp&&raw.header.stamp==grid.header.stamp,"raw/grid/service same snapshot timestamp");
    check(raw.data==rpc.data,"raw/service same data");
    l.updateMap(0,0,0,stamp(110.2));pub.commitSnapshot(stamp(110.2));
    check(badCore(*l.getCostmap(),mask,j["new_tf"],&core)==0&&core==2136,"C synchronized110.2 core");
    save(out+"/C-fixed.bin",data(*l.getCostmap()));
    auto previous=data(*l.getCostmap());auto before_origin=l.getCostmap()->getOriginX();bool threw=false;
    replay->values[0]=33;check(previous[0]!=33,"failure fixture must mutate the pending grid");
    try {l.updateMap(4,2,0,stamp(110.3));}catch(const tf2::TransformException&){threw=true;pub.invalidateSnapshot();}
    check(threw,"D unavailable exactTF must throw");check(buffer.requested.back()==stamp(110.3).nanoseconds(),"D never fallbacklatest");
    check(data(*l.getCostmap())==previous&&l.getCostmap()->getOriginX()==before_origin,"D rollback completegrid/origin");
    check(!l.isCurrent(),"D existing current protection false");check(SnapshotRegressionAccess::service(pub).data.empty(),"D no validRPC afterfailure");pub.publishCostmap();
    replay->values=input;
    // Existing transform tolerance may wait for exact TF; a direct Buffer insertion is not DDS.
    std::thread arrival([&](){std::this_thread::sleep_for(std::chrono::milliseconds(10));buffer.put(110.3,j["new_tf"]);});
    l.updateMap(0,0,0,stamp(110.3));arrival.join();pub.commitSnapshot(stamp(110.3));check(l.isCurrent(),"exactTF recovery current");
    check(buffer.returned.back()==stamp(110.3).nanoseconds(),"wait returned exact110.3");
    threw=false;try {l.updateMap(0,0,0);}catch(const std::runtime_error&){threw=true;}
    check(threw,"legacy unstamped Keepout must reject rather than reuse priorstamp");
    // Exercise a real rolling-origin mutation before a failed Filter lookup.
    LayeredCostmap rolling("odom",true,true);geometry(rolling,spec);
    auto rolling_input=std::make_shared<ReplayLayer>(rolling,input);rolling.addPlugin(rolling_input);
    auto rolling_filter=std::make_shared<KeepoutFilter>();SnapshotRegressionAccess::configure(*rolling_filter,buffer,mask,clock);rolling.addFilter(rolling_filter);
    const double cx=double(spec["origin"][0])+rolling.getCostmap()->getSizeInMetersX()/2;
    const double cy=double(spec["origin"][1])+rolling.getCostmap()->getSizeInMetersY()/2;
    rolling.updateMap(cx,cy,0,stamp(110.1));const auto stable_origin=rolling.getCostmap()->getOriginX();const auto stable_data=data(*rolling.getCostmap());
    threw=false;try {rolling.updateMap(cx+.5,cy+.25,0,stamp(111.0));}catch(const tf2::TransformException&){threw=true;}
    check(threw&&rolling_input->last_origin_x!=stable_origin,"failed cycle really moved the origin before Filter failure");
    check(rolling.getCostmap()->getOriginX()==stable_origin&&data(*rolling.getCostmap())==stable_data,"rolling grid/origin rollback");
    int r6_count=0;
    for(auto & f:j["r6"]) {
      RecordingBuffer b(clock);double t=f["stamp"];b.put(t-.1,f["tf"]);b.put(t,f["tf"]);
      auto values=bytes(root+"/"+f["data_file"].get<std::string>());LayeredCostmap map("odom",false,true);geometry(map,f);
      auto r=std::make_shared<ReplayLayer>(map,values);map.addPlugin(r);auto k=std::make_shared<KeepoutFilter>();SnapshotRegressionAccess::configure(*k,b,mask,clock);map.addFilter(k);
      map.updateMap(0,0,0,stamp(t));check(badCore(*map.getCostmap(),mask,f["tf"])==0,"E R6 core regression");
      for(size_t n=0;n<values.size();++n) if(values[n]==254){check(map.getCostmap()->getCharMap()[n]==254,"E preserve254");}
      save(out+"/"+f["data_file"].get<std::string>(),data(*map.getCostmap()));++r6_count;
    }
    auto good=data(old); // Use B result for strict Security negatives.
    good=bytes(out+"/B-fixed.bin");size_t index=35*100+92;
    for(auto value:{253,255,252,0}){std::copy(good.begin(),good.end(),old.getCharMap());old.getCharMap()[index]=value;check(badCore(old,mask,j["old_tf"])>0,"F core non254 remains unsafe");}
    // Generic old Filter behavior is retained by the timed default hook.
    LegacyFilter legacy;Costmap2D small(4,4,.05,0,0);legacy.updateCosts(small,0,0,4,4,stamp(1));check(small.getCost(0,0)==19,"other filters legacy process hook");
    LayeredCostmap concurrent("odom",false,true);concurrent.resizeMap(8,8,.05,0,0);
    auto seed=std::make_shared<ReplayLayer>(concurrent,std::vector<unsigned char>(64,0));concurrent.addPlugin(seed);auto sf=std::make_shared<StampFilter>(concurrent);concurrent.addFilter(sf);
    Costmap2DPublisher cp(node,concurrent.getCostmap(),"odom","snapshot_concurrent_"+std::to_string(getpid()),true);
    std::atomic<bool> done{false},bad{false};std::atomic<int> observed{0};
    std::thread reader([&](){while(!done){auto v=SnapshotRegressionAccess::service(cp);if(v.data.empty())continue;int expected=v.header.stamp.sec%251;for(auto x:v.data)if(x!=expected)bad=true;++observed;}});
    for(int n=1;n<=1000;++n){
      {std::unique_lock<Costmap2D::mutex_t> lock(*concurrent.getCostmap()->getMutex());concurrent.updateMap(0,0,0,stamp(n));cp.commitSnapshot(stamp(n));}
      std::this_thread::yield();
    }
    done=true;reader.join();check(!bad&&observed>0,"atomic data/timestamp under concurrent update/read");
    check(sf->seen.size()==1000,"one time parameter percycle");for(int n=1;n<=1000;++n)check(sf->seen[n-1]==stamp(n).nanoseconds(),"no priorcycle stamp");
    pluginlib::ClassLoader<Layer> loader("nav2_costmap_2d","nav2_costmap_2d::Layer");int loaded=0;std::vector<std::string> plugin_types;
    for(const auto & type:loader.getDeclaredClasses()){auto plugin=loader.createSharedInstance(type);check(bool(plugin),"plugin loads:"+type);plugin_types.push_back(type);++loaded;}
    json result={{"checks",checks},{"A_bad",8},{"B_core",2144},{"B_bad",0},{"C_core",2136},{"C_bad",0},{"R6_replayed",r6_count},{"concurrent_cycles",1000},{"concurrent_reads",observed.load()},{"stock_plugins_loaded",loaded},{"plugins_loaded",plugin_types},{"fixture_nodes",1},{"gateway_posts",0},{"cmd_vel_publishers",0},{"sizes",{{"Layer",sizeof(Layer)},{"CostmapLayer",sizeof(CostmapLayer)},{"CostmapFilter",sizeof(CostmapFilter)},{"LayeredCostmap",sizeof(LayeredCostmap)},{"Costmap2DROS",sizeof(Costmap2DROS)},{"Costmap2DPublisher",sizeof(Costmap2DPublisher)}}}};
    std::ofstream(out+"/native-results.json")<<result.dump(2)<<"\n";std::cout<<result.dump(2)<<std::endl;rclcpp::shutdown();return 0;
  }catch(const std::exception&e){std::cerr<<"SNAPSHOT_REGRESSION_FAILED "<<e.what()<<std::endl;if(rclcpp::ok())rclcpp::shutdown();return 1;}
}
