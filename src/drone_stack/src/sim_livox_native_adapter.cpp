// Simulation-only native laser conversion. No world pose/truth is used.
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_srvs/SetBool.h>
#include <livox_ros_driver2/CustomMsg.h>
#include <gazebo/gazebo_client.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <atomic>
#include <chrono>
#include <cmath>
#include <mutex>
#include <thread>
#include <tuple>
#include <vector>
#include <cstring>
#include <algorithm>
#include <stdexcept>
#include <time.h>

namespace {
double mono() { return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count(); }
double threadCpu() { timespec t{};clock_gettime(CLOCK_THREAD_CPUTIME_ID,&t);return t.tv_sec+t.tv_nsec*1e-9; }
struct Direction { double x,y,z; };
}

class NativeLivoxAdapter {
 public:
  explicit NativeLivoxAdapter(ros::NodeHandle& nh):nh_(nh),private_("~") {
    private_.param("startup_settle_s",settle_,2.0);
    private_.param("scan_period_s",period_,.1);
    private_.param("min_useful_range_m",near_,.45);
    private_.param("max_range_m",far_,40.0);
    private_.param("sample_step",step_,1);step_=std::max(1,step_);
    private_.param("gazebo_world",world_,std::string("inspection_demo"));
    private_.param("gazebo_scan_topic",topic_,std::string("/gazebo/inspection_demo/inspection_quad/base_link/mid360_sim/scan"));
    nh_.param("/drone/record_lio_latency",timing_,false);
    if(!std::isfinite(period_) || period_<=0 || period_>1 || near_<=0 || far_<=near_)throw std::runtime_error("Invalid simulated lidar parameters");
    cloud_pub_=nh_.advertise<livox_ros_driver2::CustomMsg>("/livox/lidar",2);
    rays_pub_=nh_.advertise<sensor_msgs::PointCloud2>("/drone/sim/lidar/mapping_rays",1);
    imu_pub_=nh_.advertise<sensor_msgs::Imu>("/livox/imu",100);
    imu_sub_=nh_.subscribe<sensor_msgs::Imu>("/drone/sim/lidar/imu_raw",100,[this](const sensor_msgs::Imu::ConstPtr& m){
      if(!drop_imu_ && m->header.stamp.toSec()>=settle_)imu_pub_.publish(m);
    });
    lidar_fault_=nh_.advertiseService("/drone/sim/faults/lidar",&NativeLivoxAdapter::lidarFault,this);
    imu_fault_=nh_.advertiseService("/drone/sim/faults/imu",&NativeLivoxAdapter::imuFault,this);
    geometry_fault_=nh_.advertiseService("/drone/sim/faults/lidar_geometry",&NativeLivoxAdapter::geometryFault,this);
    if(timing_) {
      timing_pub_=nh_.advertise<std_msgs::Float64MultiArray>("/drone/latency/lidar_adapter",50);
      raw_timing_pub_=nh_.advertise<std_msgs::Float64MultiArray>("/drone/latency/lidar_transport",50);
    }
  }
  bool lidarFault(std_srvs::SetBool::Request& q,std_srvs::SetBool::Response& r){drop_lidar_=q.data;r.success=true;r.message=q.data?"Lidar stream paused":"Lidar stream restored";return true;}
  bool imuFault(std_srvs::SetBool::Request& q,std_srvs::SetBool::Response& r){drop_imu_=q.data;r.success=true;r.message=q.data?"Lidar IMU stream paused":"Lidar IMU stream restored";return true;}
  bool geometryFault(std_srvs::SetBool::Request& q,std_srvs::SetBool::Response& r){geometry_loss_=q.data;r.success=true;r.message=q.data?"Vertical constraints removed":"Vertical constraints restored";return true;}
  void connect() {
    // ROS/Qt starts before Gazebo in the initial-pose workflow. Use wall time.
    while(ros::ok() && !ros::service::exists("/gazebo/get_world_properties",false))ros::WallDuration(.2).sleep();
    if(!ros::ok())return;
    if(!gazebo::client::setup()) {ROS_FATAL("Gazebo transport initialization failed");ros::shutdown();return;}
    transport_ready_=true;node_.reset(new gazebo::transport::Node());node_->Init(world_);
    scan_sub_=node_->Subscribe(topic_,&NativeLivoxAdapter::scan,this);
    ROS_INFO("Native simulated lidar connected: %s; actual Gazebo measurement timestamps",topic_.c_str());
  }
  void shutdown() {scan_sub_.reset();node_.reset();if(transport_ready_)gazebo::client::shutdown();}
  void scan(ConstLaserScanStampedPtr& message) {
    const double received=mono(),cpu_begin=threadCpu(),received_sim=ros::Time::now().toSec();
    std::lock_guard<std::mutex> guard(scan_mutex_);
    const auto& s=message->scan();const auto& t=message->time();
    if(t.sec()<0 || t.nsec()<0 || t.nsec()>=1000000000)return;
    const ros::Time end(t.sec(),t.nsec());
    if(drop_lidar_ || end.toSec()<settle_ || end<=last_stamp_)return;
    const unsigned int width=s.count(),height=std::max(1u,s.vertical_count());
    const size_t count=size_t(width)*height;
    if(!width || count>1000000 || size_t(s.ranges_size())!=count) {ROS_ERROR_THROTTLE(2.,"Malformed native laser dimensions");return;}
    const double hmin=s.angle_min(),hmax=s.angle_max(),vmin=s.vertical_angle_min(),vmax=s.vertical_angle_max();
    if(!std::isfinite(hmin+hmax+vmin+vmax))return;
    auto geometry=std::make_tuple(width,height,hmin,hmax,vmin,vmax);
    if(geometry!=geometry_ || directions_.size()!=count) {
      geometry_=geometry;directions_.resize(count);
      for(unsigned int y=0;y<height;++y) {
        const double v=vmin+(height>1?(vmax-vmin)*y/(height-1):0);
        for(unsigned int x=0;x<width;++x) {
          const double h=hmin+(width>1?(hmax-hmin)*x/(width-1):0);
          directions_[size_t(y)*width+x]={std::cos(v)*std::cos(h),std::cos(v)*std::sin(h),std::sin(v)};
        }
      }
    }
    if(timing_) {std_msgs::Float64MultiArray m;m.data={end.toSec(),received_sim,received_sim,received,double(count)};raw_timing_pub_.publish(m);}
    livox_ros_driver2::CustomMsg out;out.header.stamp=end-ros::Duration(period_);out.header.frame_id="lidar";
    out.timebase=out.header.stamp.toNSec();out.lidar_id=1;for(auto& v:out.rsvd)v=0;out.points.reserve(count/step_);
    const uint32_t offset=static_cast<uint32_t>(std::llround(period_*1e9));
    sensor_msgs::PointCloud2 rays;rays.header.stamp=end;rays.header.frame_id="lidar";rays.height=1;rays.is_dense=true;
    sensor_msgs::PointCloud2Modifier modifier(rays);
    modifier.setPointCloud2Fields(4,"x",1,sensor_msgs::PointField::FLOAT32,"y",1,sensor_msgs::PointField::FLOAT32,"z",1,sensor_msgs::PointField::FLOAT32,"intensity",1,sensor_msgs::PointField::FLOAT32);
    modifier.resize((count+1)/2);size_t ray_count=0;
    const bool remove_geometry=geometry_loss_;
    for(size_t i=0;i<count;++i) {
      const double range=s.ranges(i);
      const bool miss=(std::isinf(range) && range>0) || (std::isfinite(range) && range>=far_-.001 && range<=far_+.001);
      const bool hit=std::isfinite(range) && range>=near_ && range<far_-.05;
      if(!hit && !miss)continue;
      const double r=miss?far_:range;const auto& d=directions_[i];
      const float x=r*d.x,y=r*d.y,z=r*d.z;
      if(i%2==0) {
        // Explicit max-range misses clear actual rays but never enter LIO.
        float values[4]={x,y,z,hit?1.f:0.f};
        std::memcpy(rays.data.data()+ray_count*16,values,sizeof(values));++ray_count;
      }
      if(!hit || i%step_!=0 || (remove_geometry && std::abs(-.5*x+std::sqrt(.75)*z)>=.20))continue;
      livox_ros_driver2::CustomPoint p;p.x=x;p.y=y;p.z=z;p.offset_time=offset;p.reflectivity=100;p.tag=0x10;p.line=(i/step_)%4;out.points.push_back(p);
    }
    modifier.resize(ray_count);rays.width=ray_count;rays.row_step=ray_count*16;rays_pub_.publish(rays);
    out.point_num=out.points.size();
    if(out.point_num<2)return;
    const double pub_begin=mono();cloud_pub_.publish(out);last_stamp_=end;
    if(timing_) {
      std_msgs::Float64MultiArray m;m.data={out.header.stamp.toSec(),received_sim,received,pub_begin,mono(),ros::Time::now().toSec(),threadCpu()-cpu_begin,double(out.point_num),double(count),-1.,-1.};timing_pub_.publish(m);
    }
  }
 private:
  ros::NodeHandle nh_,private_;ros::Publisher cloud_pub_,rays_pub_,imu_pub_,timing_pub_,raw_timing_pub_;
  ros::Subscriber imu_sub_;ros::ServiceServer lidar_fault_,imu_fault_,geometry_fault_;
  std::atomic<bool> drop_lidar_{false},drop_imu_{false},geometry_loss_{false};
  double settle_,period_,near_,far_;int step_;bool timing_=false,transport_ready_=false;
  std::string world_,topic_;ros::Time last_stamp_;std::mutex scan_mutex_;
  std::vector<Direction> directions_;std::tuple<unsigned int,unsigned int,double,double,double,double> geometry_;
  gazebo::transport::NodePtr node_;gazebo::transport::SubscriberPtr scan_sub_;
};
int main(int argc,char**argv) {
  ros::init(argc,argv,"sim_livox_adapter");ros::NodeHandle nh;
  NativeLivoxAdapter adapter(nh);std::thread connection([&adapter](){adapter.connect();});
  ros::spin();connection.join();adapter.shutdown();return 0;
}
