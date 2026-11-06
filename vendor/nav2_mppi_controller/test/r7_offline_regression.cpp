// Native offline fixtures: no launch, no task/Lease/cmd_vel publisher.
#include "nav2_mppi_controller/optimizer.hpp"
#include "nav2_mppi_controller/tools/path_handler.hpp"
#include "nav2_mppi_controller/critics/curvature_speed_critic.hpp"
#include "nav2_mppi_controller/critics/cost_critic.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav2_costmap_2d/inflation_layer.hpp"
#include "nav2_mppi_controller/controller.hpp"
#include <xtensor/xrandom.hpp>
#include <fstream>
#include "pluginlib/class_loader.hpp"
#include <iostream>
#include <sstream>
#include <chrono>
#include <iomanip>
#include <algorithm>
#include <stdexcept>
#include <thread>
#include <set>
#include <tuple>
#include <cstring>
using Clock=std::chrono::steady_clock;
void check(bool value,const std::string &s){if(!value)throw std::runtime_error("CHECK: "+s);}
struct Point {double x,y,h,v=0,w=0;};
struct Grid {int id;double res,ox,oy,mx,my,mh;Point anchor;std::vector<unsigned char> cells;};
std::vector<std::vector<double>> csv(const std::string &file){std::ifstream f(file);check(bool(f),file);std::vector<std::vector<double>> r;std::string l;while(std::getline(f,l)){std::replace(l.begin(),l.end(),',',' ');std::istringstream s(l);double v;std::vector<double> q;while(s>>v)q.push_back(v);if(!q.empty())r.push_back(q);}return r;}
Point toOdom(Point p,const Grid &g){double x=p.x-g.mx,y=p.y-g.my;return {cos(g.mh)*x+sin(g.mh)*y,-sin(g.mh)*x+cos(g.mh)*y,p.h-g.mh,p.v,p.w};}
geometry_msgs::msg::PoseStamped pose(Point p){geometry_msgs::msg::PoseStamped q;q.header.frame_id="odom";q.pose.position.x=p.x;q.pose.position.y=p.y;q.pose.orientation.z=sin(p.h/2);q.pose.orientation.w=cos(p.h/2);return q;}
std::vector<geometry_msgs::msg::Point> footprint(){std::vector<geometry_msgs::msg::Point> f;for(auto p:std::vector<std::pair<double,double>>{{-.56,-.35},{-.56,.35},{.56,.35},{.56,-.35}}){geometry_msgs::msg::Point q;q.x=p.first;q.y=p.second;f.push_back(q);}return f;}
class Probe : public mppi::Optimizer {
public:
 using mppi::Optimizer::prepare;using mppi::Optimizer::fallback;using mppi::Optimizer::generateNoisedTrajectories;
 mppi::CriticData &data(){return critics_data_;}mppi::models::State &state(){return state_;}
 void preview(const nav_msgs::msg::Path &path){curvature_path_=mppi::utils::toTensor(path);}
 void warm(float v,float w){control_sequence_.vx.fill(v);control_sequence_.wz.fill(w);}
 const mppi::models::ControlSequence &controls(){return control_sequence_;}
 void score(){critic_manager_.evalTrajectoriesScores(critics_data_);}
 int iterations=0,scores=0;float best=0;double millis=0;
 mppi::critics::CostCritic * scoring_probe=nullptr;
 bool collect_extra=false;
 double w0=0,ess=0,entropy=0,endpoint_unique=0,sequence_unique=0,feasible=-1,physical_feasible=-1;
 uint64_t noiseFingerprint(const std::string &file){
  noise_generator_.setNoisedControls(state_,control_sequence_);
  std::ofstream raw(file,std::ios::binary);uint64_t hash=1469598103934665603ULL;
  for(auto tensor:{&state_.cvx,&state_.cwz}){raw.write(reinterpret_cast<const char*>(tensor->data()),tensor->size()*sizeof(float));auto ptr=reinterpret_cast<const unsigned char*>(tensor->data());for(size_t i=0;i<tensor->size()*sizeof(float);++i){hash^=ptr[i];hash*=1099511628211ULL;}}
  return hash;
 }
 void inspectBatch(){
  auto weights=xt::eval(xt::exp(-(costs_-xt::amin(costs_)())/settings_.temperature));double sum=xt::sum(weights)();double sum2=0;entropy=0;
  for(float z:weights){double w=z/sum;sum2+=w*w;if(w>0)entropy-=w*std::log(w);}
  w0=weights(0)/sum;ess=1/sum2;
  std::set<std::tuple<int,int,int>> endpoints;
  for(size_t n=0;n<750;++n){endpoints.emplace(std::lround(generated_trajectories_.x(n,49)*100),std::lround(generated_trajectories_.y(n,49)*100),std::lround(generated_trajectories_.yaws(n,49)*100));}
  endpoint_unique=endpoints.size();feasible=physical_feasible=-1;
  if(collect_extra && scoring_probe){
   auto clone=critics_data_;xt::xtensor<float,1> cost=xt::zeros<float>({750});mppi::CriticData d{clone.state,clone.trajectories,clone.path,cost,clone.model_dt,false,nullptr,clone.motion_model,std::nullopt,std::nullopt};scoring_probe->score(d);size_t free=0,validfree=0;std::set<uint64_t> seqs;
   for(size_t n=0;n<750;++n){bool valid=true;uint64_t h=1469598103934665603ULL;for(size_t t=0;t<50;++t){float v=state_.cvx(n,t),w=state_.cwz(n,t);valid &= v>=-1e-6 && v<=.300001 && std::abs(w)<=.267858 && std::abs(w)<=std::abs(v)/1.12+1e-6;for(float q:{v,w}){uint32_t bits;std::memcpy(&bits,&q,4);h^=bits;h*=1099511628211ULL;}}seqs.insert(h);if(cost(n)<100){++free;if(valid)++validfree;}}
   feasible=free/750.;physical_feasible=validfree/750.;sequence_unique=seqs.size();
  }
 }

 geometry_msgs::msg::Twist cycle(Point p,const nav_msgs::msg::Path &path,bool legacy,bool projection,const nav_msgs::msg::Path *preview_path=nullptr){
  auto begin=Clock::now();double diagnostic_ms=0;geometry_msgs::msg::Twist speed;speed.linear.x=p.v;speed.angular.z=p.w;prepare(pose(p),speed,path,nullptr);if(preview_path){preview(*preview_path);}iterations=scores=0;
  try {bool again;do {if(!critics_data_.fail_flag)++scores;++iterations;optimize();auto diagnostic_begin=Clock::now();inspectBatch();diagnostic_ms+=std::chrono::duration<double,std::milli>(Clock::now()-diagnostic_begin).count();bool failed=critics_data_.fail_flag;again=fallback(failed);if(legacy&&again)critics_data_.fail_flag=failed;}while(again);
   best=xt::amin(costs_)();mppi::utils::savitskyGolayFilter(control_sequence_,control_history_,settings_);
   if(projection){applyControlSequenceConstraints();}auto cmd=getControlFromSequenceAsTwist(path.header.stamp).twist;
   // The unchanged production BoundedMPPIController final-output contract.
   double v=std::clamp(cmd.linear.x,0.,.3);double w=cmd.linear.x!=0?cmd.angular.z*v/cmd.linear.x:0.;cmd.linear.x=v;cmd.angular.z=std::clamp(w,-std::min(.267857142857,v/1.12),std::min(.267857142857,v/1.12));
   if(settings_.shift_control_sequence){shiftControlSequence();}millis=std::chrono::duration<double,std::milli>(Clock::now()-begin).count()-diagnostic_ms;return cmd;
  }catch(...){millis=std::chrono::duration<double,std::milli>(Clock::now()-begin).count()-diagnostic_ms;throw;}
 }
};
struct Fixture {
 std::shared_ptr<nav2_costmap_2d::Costmap2DROS> map;
 std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node;
 std::unique_ptr<mppi::ParametersHandler> params;
 Probe optimizer;mppi::critics::CostCritic cost;
 std::shared_ptr<tf2_ros::Buffer> tf;std::unique_ptr<mppi::PathHandler> handler;
 Fixture(bool curve,bool projected,double alat,bool guided=false,double speed_weight=5.){
  rclcpp::NodeOptions mo;mo.parameter_overrides({rclcpp::Parameter("plugins",std::vector<std::string>{"inflation_layer"}),rclcpp::Parameter("inflation_layer.plugin","nav2_costmap_2d::InflationLayer"),rclcpp::Parameter("inflation_layer.inflation_radius",.5),rclcpp::Parameter("inflation_layer.cost_scaling_factor",3.),rclcpp::Parameter("global_frame","odom"),rclcpp::Parameter("robot_base_frame","base_link"),rclcpp::Parameter("track_unknown_space",true),rclcpp::Parameter("footprint_padding",0.),rclcpp::Parameter("footprint","[[-0.56,-0.35],[-0.56,0.35],[0.56,0.35],[0.56,-0.35]]")});
  map=std::make_shared<nav2_costmap_2d::Costmap2DROS>(mo);check(map->on_configure(rclcpp_lifecycle::State{})==nav2_util::CallbackReturn::SUCCESS,"fixture costmap configure");map->setRobotFootprint(footprint());
  std::vector<std::string> critics{"ConstraintCritic","CostCritic","GoalCritic","GoalAngleCritic","PathAlignCritic","PathFollowCritic","PathAngleCritic","PreferForwardCritic"};if(curve)critics.push_back("CurvatureSpeedCritic");
  std::vector<rclcpp::Parameter> v{rclcpp::Parameter("controller_frequency",10.),rclcpp::Parameter("FollowPath.batch_size",750),rclcpp::Parameter("FollowPath.time_steps",50),rclcpp::Parameter("FollowPath.iteration_count",1),rclcpp::Parameter("FollowPath.model_dt",.1),rclcpp::Parameter("FollowPath.temperature",.3),rclcpp::Parameter("FollowPath.gamma",.015),rclcpp::Parameter("FollowPath.vx_max",.3),rclcpp::Parameter("FollowPath.vx_min",0.),rclcpp::Parameter("FollowPath.vy_max",0.),rclcpp::Parameter("FollowPath.wz_max",.267857142857),rclcpp::Parameter("FollowPath.vx_std",.2),rclcpp::Parameter("FollowPath.wz_std",.16),rclcpp::Parameter("FollowPath.vy_std",0.),rclcpp::Parameter("FollowPath.motion_model","Ackermann"),rclcpp::Parameter("FollowPath.AckermannConstraints.min_turning_r",1.12),rclcpp::Parameter("FollowPath.AckermannConstraints.project_samples",projected),rclcpp::Parameter("FollowPath.AckermannConstraints.path_guided_sample",guided),rclcpp::Parameter("FollowPath.transform_tolerance",0.),rclcpp::Parameter("FollowPath.curvature_lookahead_distance",curve?2.5:0.),rclcpp::Parameter("FollowPath.prune_distance",1.5),rclcpp::Parameter("FollowPath.enforce_path_inversion",true),rclcpp::Parameter("FollowPath.inversion_xy_tolerance",.2),rclcpp::Parameter("FollowPath.inversion_yaw_tolerance",.4),rclcpp::Parameter("FollowPath.regenerate_noises",false),rclcpp::Parameter("FollowPath.critics",critics),rclcpp::Parameter("FollowPath.CostCritic.consider_footprint",true),rclcpp::Parameter("FollowPath.PathAlignCritic.cost_weight",14.),rclcpp::Parameter("FollowPath.PathAlignCritic.use_path_orientations",true),rclcpp::Parameter("FollowPath.PathAlignCritic.threshold_to_consider",.15),rclcpp::Parameter("FollowPath.PathAlignCritic.offset_from_furthest",4),rclcpp::Parameter("FollowPath.PathFollowCritic.threshold_to_consider",.15),rclcpp::Parameter("FollowPath.GoalAngleCritic.threshold_to_consider",1.4),rclcpp::Parameter("FollowPath.PathAngleCritic.forward_preference",false),rclcpp::Parameter("FollowPath.CurvatureSpeedCritic.max_lateral_accel",alat),rclcpp::Parameter("FollowPath.CurvatureSpeedCritic.deceleration",.25),rclcpp::Parameter("FollowPath.CurvatureSpeedCritic.cost_weight",speed_weight)};
  rclcpp::NodeOptions no;no.parameter_overrides(v);node=std::make_shared<rclcpp_lifecycle::LifecycleNode>("r7_fixture_optimizer",no);params=std::make_unique<mppi::ParametersHandler>(node);optimizer.initialize(node,"FollowPath",map,params.get());cost.on_configure(node,"FollowPath","FollowPath.CostCritic",map,params.get());optimizer.scoring_probe=&cost;tf=map->getTfBuffer();handler=std::make_unique<mppi::PathHandler>();handler->initialize(node,"FollowPath",map,tf,params.get());
 }
 void setPath(const std::vector<Point> &r){nav_msgs::msg::Path path;path.header.frame_id="map";for(auto p:r){auto q=pose(p);q.header.frame_id="map";path.poses.push_back(q);}handler->setPath(path);}
 nav_msgs::msg::Path transform(Point p,const Grid &g){geometry_msgs::msg::TransformStamped t;t.header.frame_id="map";t.child_frame_id="odom";t.transform.translation.x=g.mx;t.transform.translation.y=g.my;t.transform.rotation.z=sin(g.mh/2);t.transform.rotation.w=cos(g.mh/2);tf->setTransform(t,"offline_A3",true);auto q=pose(p);q.header.frame_id="map";return handler->transformPath(q);}
 nav_msgs::msg::Path preview(Point p){auto q=pose(p);q.header.frame_id="map";return handler->curvaturePreview(q,2.5);}
 void load(const Grid &g){auto m=map->getCostmap();m->resizeMap(100,100,g.res,g.ox,g.oy);std::copy(g.cells.begin(),g.cells.end(),m->getCharMap());}
};
nav_msgs::msg::Path pathFor(const std::vector<Point> &r,Point p,const Grid &g,double length,size_t &near){double best=1e99;near=0;for(size_t j=0;j<std::min(size_t(200),r.size());++j){double d=hypot(r[j].x-p.x,r[j].y-p.y);if(d<best){best=d;near=j;}}nav_msgs::msg::Path out;out.header.frame_id="odom";double s=0;for(size_t j=near;j<r.size();++j){if(j>near)s+=hypot(r[j].x-r[j-1].x,r[j].y-r[j-1].y);if(s>length)break;auto q=toOdom(r[j],g);if(q.x<g.ox||q.y<g.oy||q.x>=g.ox+5||q.y>=g.oy+5)break;out.poses.push_back(pose(q));}return out;}
struct Stats{int collision=0,violating=0;double maxk=0,mincost=0,mean_v=0,std_v=0,mean_w=0,std_w=0,zero_fraction=0,edge_fraction=0;};
Stats stats(Fixture &f){auto &s=f.optimizer.state();Stats out;double sv=0,sv2=0,sw=0,sw2=0,zero=0,edge=0;for(size_t n=0;n<s.cvx.shape(0);++n)for(size_t t=0;t<s.cvx.shape(1);++t){float v=s.cvx(n,t),w=s.cwz(n,t);sv+=v;sv2+=v*v;sw+=w;sw2+=w*w;zero+=abs(v)<1e-6;edge+=(v>1e-6&&abs(abs(w)-v/1.12)<1e-6);if(v< -1e-6||v>.300001||abs(w)>.267858||abs(w)>abs(v)/1.12+1e-6)++out.violating;if(v>1e-4)out.maxk=std::max(out.maxk,double(abs(w)/v));}
 double count=s.cvx.size();out.mean_v=sv/count;out.mean_w=sw/count;out.std_v=sqrt(std::max(0.,sv2/count-out.mean_v*out.mean_v));out.std_w=sqrt(std::max(0.,sw2/count-out.mean_w*out.mean_w));out.zero_fraction=zero/count;out.edge_fraction=edge/count;
 auto d=f.optimizer.data();xt::xtensor<float,1> costs=xt::zeros<float>({750});mppi::CriticData copy{d.state,d.trajectories,d.path,costs,d.model_dt,false,d.goal_checker,d.motion_model,std::nullopt,std::nullopt};f.cost.score(copy);for(float v:costs)if(v>100)++out.collision;out.mincost=xt::amin(d.costs)();return out;}
class CountCritic:public mppi::critics::CriticFunction{public:int *count;bool fail;CountCritic(int *c,bool f):count(c),fail(f){}void initialize()override{}void score(mppi::CriticData &d)override{++*count;if(fail)d.fail_flag=true;}};
class CountManager:public mppi::CriticManager{public:void add(int *n,bool fail){critics_.push_back(std::make_unique<CountCritic>(n,fail));}};
void units(Fixture &f,const std::vector<Point> &r,const Grid &g,std::ofstream &o,double alat){
 size_t near;Point q=r[0];auto path=pathFor(r,q,g,2.5,near);geometry_msgs::msg::Twist speed;speed.linear.x=.12;f.optimizer.prepare(pose(toOdom(q,g)),speed,path,nullptr);int failed=0,later=0;CountManager first;first.add(&failed,true);first.add(&later,false);first.evalTrajectoriesScores(f.optimizer.data());check(failed==1&&later==0,"first all-fail short circuit");f.optimizer.generateNoisedTrajectories();float old=xt::sum(f.optimizer.state().cvx)();check(f.optimizer.fallback(true),"retry allowed");check(!f.optimizer.data().fail_flag,"fallback clears flag");check(f.optimizer.controls().vx(0)==0,"control reset");check(f.optimizer.state().pose.pose.position.x==toOdom(q,g).x,"pose preserved");check(f.optimizer.state().speed.linear.x==.12,"speed preserved");CountManager second;int a=0,b=0;second.add(&a,false);second.add(&b,false);second.evalTrajectoriesScores(f.optimizer.data());check(a==1&&b==1,"all second critics scored");check(!f.optimizer.fallback(false),"recovery completes");f.optimizer.generateNoisedTrajectories();check(std::isfinite(f.optimizer.state().cvx(0,0))&&xt::sum(f.optimizer.state().cvx)()!=old,"noise regenerated");
 first.evalTrajectoriesScores(f.optimizer.data());check(f.optimizer.fallback(true),"second failure first retry");first.evalTrajectoriesScores(f.optimizer.data());bool threw=false;try{f.optimizer.fallback(true);}catch(const std::runtime_error &){threw=true;}check(threw,"bounded second failure abort");f.optimizer.prepare(pose(toOdom(q,g)),speed,path,nullptr);check(!f.optimizer.data().fail_flag,"next period clean");f.optimizer.fallback(false);o<<"fallback_first_batch_fail_second_all_critics_recovery PASS\nfallback_second_batch_failure_bounded_abort PASS\nfallback_cross_cycle_pose_speed_controls_noise PASS\n";
 auto &st=f.optimizer.state();st.cvx.fill(-.1);st.cwz.fill(.2);f.optimizer.data().motion_model->predict(st);check(xt::amax(xt::abs(st.cvx))()==0&&xt::amax(xt::abs(st.cwz))()==0,"negative/zero vx forces zero wz");st.cvx.fill(.8);st.cwz.fill(2.);f.optimizer.data().motion_model->predict(st);auto s=stats(f);check(s.violating==0,"projected cvx/cwz constraints");o<<"sampling37500_controls_projected PASS\n";
 mppi::models::Path model;model.x=xt::zeros<float>({r.size()});model.y=model.x;model.yaws=model.x;for(size_t i=0;i<r.size();++i){model.x(i)=r[i].x;model.y(i)=r[i].y;model.yaws(i)=r[i].h;}auto profile=mppi::critics::curvatureSpeedProfile(model,.3,.26785714,alat,.25,.1);check(profile.front().speed<.25,"curve slows from start");check(profile[180].speed>.29,"straight recovers");for(size_t i=1;i<profile.size();++i){double ds=profile[i].s-profile[i-1].s;check(profile[i-1].speed*profile[i-1].speed<=profile[i].speed*profile[i].speed+.5*ds+1e-6,"braking propagation");}
 // A3 curve translated after a 1m straight: only a unit geometry, not an A3 replay.
 mppi::models::Path preview;preview.x=xt::zeros<float>({size_t(141)});preview.y=preview.x;preview.yaws=preview.x;for(size_t i=0;i<141;++i){double s=i*.025;if(s<=1){preview.x(i)=s;preview.y(i)=0;}else{double a=(s-1)/1.1536;preview.x(i)=1+1.1536*sin(a);preview.y(i)=1.1536*(1-cos(a));}}auto pr=mppi::critics::curvatureSpeedProfile(preview,.3,.26785714,alat,.25,.1);check(pr[37].speed<.29,"braking before bend independent of rollout speed");o<<"curvature_reference_straight_turn_braking_recovery PASS\n";
 auto map=f.map->getCostmap();std::vector<unsigned char> saved(map->getCharMap(),map->getCharMap()+10000);
 std::fill(map->getCharMap(),map->getCharMap()+10000,254);f.optimizer.prepare(pose(toOdom(q,g)),speed,path,nullptr);f.optimizer.generateNoisedTrajectories();f.optimizer.score();check(f.optimizer.data().fail_flag,"actual CostCritic first all-collision batch");check(f.optimizer.fallback(true),"actual CostCritic fallback retry");
 std::fill(map->getCharMap(),map->getCharMap()+10000,0);f.optimizer.generateNoisedTrajectories();f.optimizer.score();check(!f.optimizer.data().fail_flag,"actual CostCritic second batch rescored free");check(!f.optimizer.fallback(false),"actual collision recovery");std::copy(saved.begin(),saved.end(),map->getCharMap());
 o<<"real_CostCritic_colliding_then_free_batches PASS\n";

}
void samplingParity(const std::vector<Point> &r,const Grid &g,double alat,std::ofstream &unit,const std::string &output){
 std::vector<float> tail_v,tail_w;std::ofstream dist(output+"/sampling-distribution.csv");dist<<"mode,mean_v,std_v,mean_w,std_w,zero_fraction,edge_fraction,violations,collision\n";
 for(int mode=0;mode<3;++mode){Fixture f(true,mode>0,alat,mode==2);f.load(g);f.setPath(r);auto path=f.transform(g.anchor,g);xt::random::seed(1907);f.optimizer.reset();geometry_msgs::msg::Twist speed;speed.linear.x=g.anchor.v;speed.angular.z=g.anchor.w;f.optimizer.prepare(pose(toOdom(g.anchor,g)),speed,path,nullptr);f.optimizer.preview(f.preview(g.anchor));f.optimizer.generateNoisedTrajectories();f.optimizer.score();auto z=stats(f);auto &state=f.optimizer.state();
 if(mode==1){tail_v.assign(state.cvx.data()+50,state.cvx.data()+state.cvx.size());tail_w.assign(state.cwz.data()+50,state.cwz.data()+state.cwz.size());}
 if(mode==2){check(std::equal(tail_v.begin(),tail_v.end(),state.cvx.data()+50)&&std::equal(tail_w.begin(),tail_w.end(),state.cwz.data()+50),"other749 projected Gaussian rows identical");check(z.violating==0,"proposal plus Gaussian all physically constrained");}
 dist<<mode<<","<<z.mean_v<<","<<z.std_v<<","<<z.mean_w<<","<<z.std_w<<","<<z.zero_fraction<<","<<z.edge_fraction<<","<<z.violating<<","<<z.collision<<"\n";
 }unit<<"749_Gaussian_rows_unchanged_by_reference_proposal PASS\n";
}
void rcModes(const std::string&,const std::string&,const std::vector<Point>&,const std::vector<Grid>&,Point,double);
int main(int argc,char **argv){
 if(argc!=4 && argc!=5){std::cerr<<"input_dir output_dir a_lat\n";return 2;}std::string input=argv[1],output=argv[2];double alat=std::stod(argv[3]);rclcpp::init(0,nullptr);int rc=0;try{
 std::ofstream unit(output+"/native-tests.txt");std::ofstream snap(output+"/snapshots.csv");snap<<"variant,seed,map_id,candidates,collision,control_constraint_violations,max_control_curvature,best_total_cost,mean_v,std_v,mean_w,std_w,zero_fraction,edge_fraction\n";
 std::vector<Point> r;for(auto v:csv(input+"/route.csv"))r.push_back({v[0],v[1],v[2]});std::vector<Grid> maps;for(auto v:csv(input+"/maps.csv")){Grid g;g.id=v[0];g.res=v[1];g.ox=v[2];g.oy=v[3];g.mx=v[4];g.my=v[5];g.mh=v[6];g.anchor={v[7],v[8],v[9],v[10],v[11]};std::ifstream f(input+"/grid_"+std::to_string(g.id)+".txt");int c;while(f>>c)g.cells.push_back(c);check(g.cells.size()==10000,"actual raw grid size");maps.push_back(g);}auto points=csv(input+"/poses.csv");Point entry{points[0][0],points[0][1],points[0][2],points[0][3],points[0][4]};
 if(argc==5 && (std::string(argv[4])=="--paired" || std::string(argv[4])=="--prebend" || std::string(argv[4])=="--cadence")){rcModes(argv[4],output,r,maps,entry,alat);rclcpp::shutdown();return 0;}
 if(argc==5 && std::string(argv[4])=="--units"){{Fixture f(true,true,alat);f.load(maps.front());units(f,r,maps.front(),unit,alat);}samplingParity(r,maps.front(),alat,unit,output);rclcpp::shutdown();return 0;}
 if(argc==5 && std::string(argv[4])=="--binding"){
  {Fixture f(true,true,alat,true);f.load(maps.front());f.setPath(r);f.transform(entry,maps.front());pluginlib::ClassLoader<nav2_core::Controller> loader("nav2_core","nav2_core::Controller");auto controller=loader.createUniqueInstance("cleannav_navigation::BoundedMPPIController");controller->configure(f.node,"FollowPath",f.tf,f.map);nav_msgs::msg::Path full;full.header.frame_id="map";for(auto p:r){auto q=pose(p);q.header.frame_id="map";full.poses.push_back(q);}controller->setPlan(full);controller->activate();geometry_msgs::msg::Twist speed;speed.linear.x=entry.v;speed.angular.z=entry.w;auto cmd=controller->computeVelocityCommands(pose(toOdom(entry,maps.front())),speed,nullptr);auto track=f.transform(entry,maps.front());auto preview=f.preview(entry);auto length=[](const nav_msgs::msg::Path &p){double s=0;for(size_t j=1;j<p.poses.size();++j){s+=std::hypot(p.poses[j].pose.position.x-p.poses[j-1].pose.position.x,p.poses[j].pose.position.y-p.poses[j-1].pose.position.y);}return s;};check(preview.poses.size()>track.poses.size(),"independent preview longer than tracking");for(size_t j=0;j<track.poses.size();++j){check(std::hypot(track.poses[j].pose.position.x-preview.poses[j].pose.position.x,track.poses[j].pose.position.y-preview.poses[j].pose.position.y)<1e-9,"tracking prefix preserved");}unit<<"actual_PathHandler_tracking_m="<<length(track)<<" preview_m="<<length(preview)<<" tracking_poses="<<track.poses.size()<<" preview_poses="<<preview.poses.size()<<" prefix_unchanged PASS\n";check(std::isfinite(cmd.twist.linear.x)&&std::isfinite(cmd.twist.angular.z),"actual wrapper finite command");check(cmd.twist.linear.x>=0&&cmd.twist.linear.x<=.300001&&abs(cmd.twist.angular.z)<=cmd.twist.linear.x/1.12+1e-6,"actual wrapper command constrained");unit<<"private_BoundedMPPIController_configure_setPlan_compute_cleanup PASS v="<<cmd.twist.linear.x<<" w="<<cmd.twist.angular.z<<" optimizer_size="<<sizeof(mppi::Optimizer)<<" controller_size="<<sizeof(nav2_mppi_controller::MPPIController)<<"\n";controller->deactivate();controller->cleanup();}rclcpp::shutdown();return 0;
 }
 std::ofstream curve(output+"/reference-profile.csv");curve<<"s,kappa,v_ref\n";mppi::models::Path model;model.x=xt::zeros<float>({r.size()});model.y=model.x;model.yaws=model.x;for(size_t i=0;i<r.size();++i){model.x(i)=r[i].x;model.y(i)=r[i].y;model.yaws(i)=r[i].h;}for(auto p:mppi::critics::curvatureSpeedProfile(model,.3,.26785714,alat,.25,.1))curve<<p.s<<","<<p.kappa<<","<<p.speed<<"\n";
 std::ofstream trace(output+"/native-traces.csv");trace<<"variant,seed,t,x,y,yaw,v,w,route_index,cross_track,heading_error,footprint_cost,solve_ms,optimize_calls,real_score_passes,best_total_cost,status\n";
 for(char variant:std::string(argc==5 && std::string(argv[4])=="--projected-only"?"FDE":"ABCFDE"))for(int seed=0;seed<4;++seed){xt::random::seed(700+seed);bool curvature=variant!='A',projection=variant=='D'||variant=='E'||variant=='F',legacy=variant<'C';Fixture f(curvature,projection,alat,variant=='D'||variant=='E');f.load(maps.front());if(variant=='D'&&seed==0 && !(argc==5 && std::string(argv[4])=="--snapshots-only"))units(f,r,maps.front(),unit,alat);
  for(auto &g:maps){xt::random::seed(700+seed+g.id);f.optimizer.reset();f.load(g);f.setPath(r);auto path=f.transform(g.anchor,g);geometry_msgs::msg::Twist v;v.linear.x=g.anchor.v;v.angular.z=g.anchor.w;f.optimizer.prepare(pose(toOdom(g.anchor,g)),v,path,nullptr);if(curvature){f.optimizer.preview(f.preview(g.anchor));}f.optimizer.generateNoisedTrajectories();f.optimizer.score();auto s=stats(f);snap<<variant<<","<<seed<<","<<g.id<<",750,"<<s.collision<<","<<s.violating<<","<<s.maxk<<","<<s.mincost<<","<<s.mean_v<<","<<s.std_v<<","<<s.mean_w<<","<<s.std_w<<","<<s.zero_fraction<<","<<s.edge_fraction<<"\n";}
  if(argc==5 && std::string(argv[4])=="--snapshots-only"){continue;}
  xt::random::seed(700+seed);f.optimizer.reset();f.setPath(r);Point p=entry;if(variant=='E'){p=r[0];p.v=0;p.w=0;}Point anchor=p;double anchor_t=0;std::string status="TIME_LIMIT";
  for(int step=0;step<600;++step){double t=step*.1;size_t mi=0;double md=1e99;for(size_t i=0;i<maps.size();++i){double d=hypot(p.x-maps[i].anchor.x,p.y-maps[i].anchor.y);if(d<md){md=d;mi=i;}}auto &g=maps[mi];f.load(g);size_t near;pathFor(r,p,g,curvature?2.5:1.5,near);auto path=f.transform(p,g);check(path.poses.size()>1,"preview nonempty");auto local=toOdom(p,g);nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D*> checker(f.map->getCostmap());double fc=checker.footprintCostAtPose(local.x,local.y,local.h,footprint());double error=hypot(p.x-r[near].x,p.y-r[near].y),heading=atan2(sin(p.h-r[near].h),cos(p.h-r[near].h));geometry_msgs::msg::Twist cmd;status="RUNNING";
   if(fc==254)status="FOOTPRINT254";else if(near>=120)status="FIRST_TURN_EXIT";else if(t-anchor_t>15.)status="STALLED";else try{auto preview=f.preview(p);cmd=f.optimizer.cycle(local,path,legacy,projection,curvature?&preview:nullptr);}catch(const std::exception &){status="OPTIMIZER_ABORT";}
   trace<<variant<<","<<seed<<","<<t<<","<<std::setprecision(12)<<p.x<<","<<p.y<<","<<p.h<<","<<cmd.linear.x<<","<<cmd.angular.z<<","<<near<<","<<error<<","<<heading<<","<<fc<<","<<f.optimizer.millis<<","<<f.optimizer.iterations<<","<<f.optimizer.scores<<","<<f.optimizer.best<<","<<status<<"\n";
   if(status!="RUNNING"){break;}check(cmd.linear.x>=-1e-7&&cmd.linear.x<=.300001&&abs(cmd.angular.z)<=cmd.linear.x/1.12+1e-6,"final executable constraints");p.x+=cmd.linear.x*cos(p.h)*.1;p.y+=cmd.linear.x*sin(p.h)*.1;p.h+=cmd.angular.z*.1;p.v=cmd.linear.x;p.w=cmd.angular.z;if(hypot(p.x-anchor.x,p.y-anchor.y)>=.1){anchor=p;anchor_t=t;}
  }std::cout<<"REPLAY "<<variant<<" seed="<<seed<<" "<<status<<std::endl;
 }

 std::ofstream response(output+"/native-speed-objective.csv");response<<"seed,curve_enabled,path,command_v,command_w,solve_ms\n";
 for(int seed=0;seed<4;++seed){for(int mode=0;mode<3;++mode){xt::random::seed(1700+seed);Fixture f(mode!=0,true,alat);Grid free;free.id=0;free.res=.05;free.ox=-2;free.oy=-2;free.mx=free.my=free.mh=0;free.cells.assign(10000,0);f.load(free);nav_msgs::msg::Path path;path.header.frame_id="odom";
 for(int j=0;j<100;++j){double ss=j*.025;Point q=mode==2?Point{ss,0,0}:Point{1.1536*sin(ss/1.1536),1.1536*(1-cos(ss/1.1536)),ss/1.1536};path.poses.push_back(pose(q));}
 f.optimizer.warm(.3,mode==2?0:.3/1.1536);Point start{0,0,0,.3,mode==2?0:.3/1.1536};auto cmd=f.optimizer.cycle(start,path,false,true);response<<seed<<","<<(mode!=0)<<","<<(mode==2?"straight":"curve")<<","<<cmd.linear.x<<","<<cmd.angular.z<<","<<f.optimizer.millis<<"\n";check(cmd.linear.x>.03,"speed objective cannot pass by parking");}}
 unit<<"native_speed_objective_response_recorded PASS\n";
 unit<<"final_commands_all_ablations_bounded PASS\n";
 }catch(const std::exception &e){std::cerr<<e.what()<<std::endl;rc=1;}rclcpp::shutdown();return rc;
}

// RC-only measurements. Frozen A3 grids and ideal plant are an approximate
// counterfactual, not a reproduction of the unrecorded original random batch.
void rcModes(const std::string &mode,const std::string &output,
 const std::vector<Point> &original,const std::vector<Grid> &maps,Point entry,double alat)
{
 auto r=original;
 bool synthetic=mode=="--prebend",cadence=mode=="--cadence";
 if(synthetic){
  r.clear();for(int i=0;i<80;++i)r.push_back({i*.025,0,0});
  for(int i=0;i<=73;++i)r.push_back({2+original[i].x-original[0].x,original[i].y-original[0].y,original[i].h});
  auto end=r.back();for(int i=1;i<=50;++i)r.push_back({end.x+i*.025*cos(end.h),end.y+i*.025*sin(end.h),end.h});
  entry={0,0,0,.3,0};
 }
 std::vector<double> arc(r.size(),0);mppi::models::Path tensor;
 tensor.x=xt::zeros<float>({r.size()});tensor.y=tensor.x;tensor.yaws=tensor.x;
 for(size_t i=0;i<r.size();++i){tensor.x(i)=r[i].x;tensor.y(i)=r[i].y;tensor.yaws(i)=r[i].h;if(i)arc[i]=arc[i-1]+hypot(r[i].x-r[i-1].x,r[i].y-r[i-1].y);}
 auto profile=mppi::critics::curvatureSpeedProfile(tensor,.3,.26785714,alat,.25,.1);
 std::ofstream out(output+"/cycles.csv"),seeds(output+"/seeds.csv");
 out<<"variant,seed,step,t,x,y,yaw,v,w,route_index,cross_track,heading_error,footprint_cost,solve_ms,weight0,ess,entropy,unique_endpoints,unique_sequences,collision_free,physical_collision_free,kappa,preview_kappa,v_ref,distance_to_bend,arrival_s,status\n";
 seeds<<"variant,seed,status,steps,noise_fingerprint\n";
 int count=synthetic?4:(cadence?1:30);
 for(char variant:std::string(cadence?"D":(synthetic?"ADJ":"AD")))for(int seed=0;seed<count;++seed){
  bool d=variant!='A';Fixture f(d,d,alat,d,variant=='J'?0.:5.);Point p=entry,anchor=p;double anchor_t=0;
  Grid free=maps.front();if(synthetic){free.mx=free.my=free.mh=0;free.ox=free.oy=-5;free.res=.05;free.cells.assign(10000,0);}
  f.load(maps.front());f.setPath(r);xt::random::seed(700+seed);f.optimizer.reset();
  auto fingerprint=f.optimizer.noiseFingerprint(output+"/noise-"+variant+"-"+std::to_string(seed)+".bin");
  if(synthetic)f.optimizer.warm(.3,0);
  pluginlib::ClassLoader<nav2_core::Controller> loader("nav2_core","nav2_core::Controller");
  // pluginlib's unique pointer has a type-erased deleter.
  auto native=loader.createUniqueInstance("cleannav_navigation::BoundedMPPIController");
  if(cadence){f.transform(p,maps.front());native->configure(f.node,"FollowPath",f.tf,f.map);nav_msgs::msg::Path full;full.header.frame_id="map";for(auto q:r){auto msg=pose(q);msg.header.frame_id="map";full.poses.push_back(msg);}native->setPlan(full);native->activate();}
  std::string status="TIME_LIMIT";int step=0;auto start=Clock::now();
  for(;step<600;++step){
   if(cadence)std::this_thread::sleep_until(start+std::chrono::milliseconds(100*step));
   double arrival=std::chrono::duration<double>(Clock::now()-start).count(),t=step*.1;
   size_t mi=0,near=0;double md=1e99,nd=1e99;
   for(size_t i=0;i<maps.size();++i){double z=hypot(p.x-maps[i].anchor.x,p.y-maps[i].anchor.y);if(z<md){md=z;mi=i;}}
   for(size_t i=0;i<r.size();++i){double z=hypot(p.x-r[i].x,p.y-r[i].y);if(z<nd){nd=z;near=i;}}
   auto g=synthetic?free:maps[mi];f.load(g);
   if(synthetic){auto m=f.map->getCostmap();m->resizeMap(400,400,.05,-5,-5);std::fill(m->getCharMap(),m->getCharMap()+160000,0);}
   auto path=f.transform(p,g),preview=f.preview(p);auto local=toOdom(p,g);
   nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D*> checker(f.map->getCostmap());
   double fc=checker.footprintCostAtPose(local.x,local.y,local.h,footprint());
   double heading=atan2(sin(p.h-r[near].h),cos(p.h-r[near].h));
   double pk=0;for(size_t i=near;i<r.size()&&arc[i]-arc[near]<=2.5;++i)pk=std::max(pk,std::abs(double(profile[i].kappa)));
   geometry_msgs::msg::Twist cmd;status="RUNNING";
   if(fc==254)status="FOOTPRINT254";else if(near>=(synthetic?155:120))status="FIRST_TURN_EXIT";else if(t-anchor_t>15)status="STALLED";
   else try{
    if(cadence){geometry_msgs::msg::Twist speed;speed.linear.x=p.v;speed.angular.z=p.w;auto begin=Clock::now();cmd=native->computeVelocityCommands(pose(local),speed,nullptr).twist;f.optimizer.millis=std::chrono::duration<double,std::milli>(Clock::now()-begin).count();}
    else{f.optimizer.collect_extra=(step%20==0);cmd=f.optimizer.cycle(local,path,!d,d,d?&preview:nullptr);}
   }catch(const std::exception &e){status="OPTIMIZER_ABORT";std::cerr<<variant<<" seed="<<seed<<" step="<<step<<" "<<e.what()<<std::endl;}
   out<<std::setprecision(12)<<variant<<","<<seed<<","<<step<<","<<t<<","<<p.x<<","<<p.y<<","<<p.h<<","<<cmd.linear.x<<","<<cmd.angular.z<<","<<near<<","<<nd<<","<<heading<<","<<fc<<","<<f.optimizer.millis<<","<<f.optimizer.w0<<","<<f.optimizer.ess<<","<<f.optimizer.entropy<<","<<f.optimizer.endpoint_unique<<","<<f.optimizer.sequence_unique<<","<<f.optimizer.feasible<<","<<f.optimizer.physical_feasible<<","<<profile[near].kappa<<","<<pk<<","<<profile[near].speed<<","<<2-arc[near]<<","<<arrival<<","<<status<<"\n";
   if(status!="RUNNING")break;
   check(cmd.linear.x>=-1e-7&&cmd.linear.x<=.300001&&abs(cmd.angular.z)<=cmd.linear.x/1.12+1e-6,"RC executable constraints");
   p.x+=cmd.linear.x*cos(p.h)*.1;p.y+=cmd.linear.x*sin(p.h)*.1;p.h+=cmd.angular.z*.1;p.v=cmd.linear.x;p.w=cmd.angular.z;
   if(hypot(p.x-anchor.x,p.y-anchor.y)>=.1){anchor=p;anchor_t=t;}
  }
  if(cadence){native->deactivate();native->cleanup();}
  seeds<<variant<<","<<seed<<","<<status<<","<<step<<","<<fingerprint<<"\n";seeds.flush();out.flush();
  std::cout<<"RC "<<mode<<" "<<variant<<" seed="<<seed<<" "<<status<<" steps="<<step<<std::endl;
 }
}
