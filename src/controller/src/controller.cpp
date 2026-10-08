#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/PositionTarget.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PositionCommand.h>  
#include <std_msgs/Bool.h>
#include <cmath>
#include <vector>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "bspline_opt/planning_judgment.h"

enum Mission {
    TAKEOFF,
    NAVIGATE,
    COMPLETE
};

struct Waypoint {
    double x, y, z;
};

class WaypointNavigator {
private:
    ros::NodeHandle nh_;
    
    // 订阅
    ros::Subscriber state_sub_;
    ros::Subscriber odom_sub_;           // MAVROS 本地位置 (odom)
    ros::Subscriber traj_sub_;           // EGO-Planner 轨迹点 (quadrotor_msgs::PositionCommand)
    ros::Subscriber planning_judgment_sub_;  
    ros::Subscriber start_ok_sub_;       // 二维码检测信号
    
    // 发布
    ros::Publisher setpoint_pub_;        // 给 MAVROS 的设定点
    ros::Publisher goal_pub_;            // 给 EGO-Planner 的目标点
    ros::Publisher error_pub_;           // 位置误差

    // 服务
    ros::ServiceClient set_mode_client_;
    ros::ServiceClient arming_client_;
    
    // 定时器
    ros::Timer timer_;
    
    // 状态变量
    mavros_msgs::State current_state_;
    geometry_msgs::PoseStamped current_pose_;
    mavros_msgs::PositionTarget cmd_;     // 当前控制指令
    bool is_planning_ok_ = true;
    
    // 二维码等待标志
    bool start_ok_received_ = false;
    ros::Time start_ok_time_;
    
    // 任务变量
    Mission mission_;
    std::vector<Waypoint> waypoints_;
    int current_wp_idx_;
    bool wp_reached_;
    Waypoint arrival_tolerance_;
    double takeoff_h_;
    double takeoff_yaw_;
    bool auto_arming_;
    bool is_recv_pose_;
    bool is_recv_traj_;
    
public:
    WaypointNavigator(ros::NodeHandle& nh) : nh_(nh), current_wp_idx_(0), wp_reached_(false),
                          is_recv_pose_(false), is_recv_traj_(false) {
        nh_.param("arrival_tolerance/x", arrival_tolerance_.x, 0.1);
        nh_.param("arrival_tolerance/y", arrival_tolerance_.y, 0.1);
        nh_.param("arrival_tolerance/z", arrival_tolerance_.z, 0.1);
        
        nh_.param("takeoff_height", takeoff_h_, 1.5);
        nh_.param("takeoff_yaw", takeoff_yaw_, 0.0);
        nh_.param("auto_arming", auto_arming_, true);
        
        // 初始化控制指令 (使用 FRAME_LOCAL_NED)
        cmd_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        // type_mask 设置：只使用位置和偏航
        cmd_.type_mask = 0x0FFF;   // 等同于 ~uint16_t(0) & ~(uint16_t(0xff) << 12)
        cmd_.header.frame_id = "map";
        // 起飞点设为原点（水平坐标0）
        cmd_.position.x = 0;
        cmd_.position.y = 0;
        cmd_.position.z = takeoff_h_;
        cmd_.yaw = takeoff_yaw_;
        
        // 订阅
        state_sub_ = nh_.subscribe<mavros_msgs::State>("mavros/state", 10,
                        std::bind(&WaypointNavigator::stateCb, this, std::placeholders::_1));
        odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("mavros/local_position/odom", 10,
                        std::bind(&WaypointNavigator::odomCb, this, std::placeholders::_1));
        traj_sub_ = nh_.subscribe<quadrotor_msgs::PositionCommand>("/planning/pos_cmd", 10,
                        std::bind(&WaypointNavigator::trajCb, this, std::placeholders::_1));
        planning_judgment_sub_ = nh_.subscribe<bspline_opt::planning_judgment>("/planning_judgment", 10,
                        std::bind(&WaypointNavigator::planningJudgmentCb, this, std::placeholders::_1));
        start_ok_sub_ = nh_.subscribe<std_msgs::Bool>("/start_ok", 10,
                        std::bind(&WaypointNavigator::startOkCb, this, std::placeholders::_1));
        
        // 发布
        setpoint_pub_ = nh_.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);
        goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal_base", 10); /////////////////////
        error_pub_ = nh_.advertise<geometry_msgs::Vector3>("/position_error_to_goal", 10);
         
        // 服务
        set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
        arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
        
        // 定时器 (30 Hz)
        timer_ = nh_.createTimer(ros::Duration(1.0/30.0),
                                 std::bind(&WaypointNavigator::run, this, std::placeholders::_1));
        ROS_INFO("WaypointNavigator initialized. Waiting for /start_ok signal...");
    }
    
    // 外部接口：传入航点列表并启动任务
    void start(const std::vector<Waypoint>& waypoints) {
        if (waypoints.empty()) {
            ROS_ERROR("No waypoints provided. Abort.");
            return;
        }
        waypoints_ = waypoints;
        mission_ = TAKEOFF;
        ROS_INFO("Started with %lu waypoints.", waypoints_.size());
    }
    
private:
    void stateCb(const mavros_msgs::State::ConstPtr& msg) {
        current_state_ = *msg;
    }
    
    void odomCb(const nav_msgs::Odometry::ConstPtr& msg) {
        current_pose_.pose = msg->pose.pose;
        current_pose_.header = msg->header;
        is_recv_pose_ = true;
    }
    
    void planningJudgmentCb(const bspline_opt::planning_judgment::ConstPtr& msg) {
        is_planning_ok_ = msg->is_planning_ok;
    }

    void trajCb(const quadrotor_msgs::PositionCommand::ConstPtr& msg) {
        cmd_.position.x = msg->position.x;
        cmd_.position.y = msg->position.y;
        cmd_.position.z = msg->position.z;
        cmd_.header.stamp = msg->header.stamp;
        is_recv_traj_ = true;
    }
    
    void startOkCb(const std_msgs::Bool::ConstPtr& msg) {
        if (msg->data && !start_ok_received_) {
            start_ok_received_ = true;
            start_ok_time_ = ros::Time::now();
            ROS_INFO("Received /start_ok = true, waiting 10 seconds before takeoff...");
        }
    }
    
    void sendGoal(const Waypoint& wp, int idx) {
        geometry_msgs::PoseStamped goal;
        goal.header.stamp = ros::Time::now();
        goal.header.frame_id = "map";
        goal.pose.position.x = wp.x;
        goal.pose.position.y = wp.y;
        goal.pose.position.z = wp.z;
        goal.pose.orientation.w = 1.0;
        goal_pub_.publish(goal);
        ROS_INFO("Sent waypoint %d: (%.2f, %.2f, %.2f)", idx, wp.x, wp.y, wp.z);
    }
    
    bool is_arrived(const Waypoint& wp) {
        double dx = current_pose_.pose.position.x - wp.x;
        double dy = current_pose_.pose.position.y - wp.y;
        double dz = current_pose_.pose.position.z - wp.z;
        return (std::abs(dx) < arrival_tolerance_.x &&
                std::abs(dy) < arrival_tolerance_.y &&
                std::abs(dz) < arrival_tolerance_.z);
    }   
    
    void run(const ros::TimerEvent&) {
        if (!current_state_.connected || !is_recv_pose_) return;
        ros::spinOnce();
        
        switch (mission_) {
            case TAKEOFF: {
                // 1. 等待二维码信号
                if (!start_ok_received_) {
                    ROS_INFO_THROTTLE(2, "Waiting for /start_ok signal...");
                    return;  // 不发布任何指令
                }
                // 2. 已收到信号，等待10秒
                if ((ros::Time::now() - start_ok_time_).toSec() < 10.0) {
                    ROS_INFO_THROTTLE(2, "Waiting 10 seconds after QR detection...");
                    return;
                }
                // 3. 10秒后，开始起飞流程（仅执行一次）
                static bool takeoff_initiated = false;
                if (!takeoff_initiated) {
                    // 发送10次悬停指令（当前高度），确保飞控收到setpoint
                    for (int i = 0; i < 10; ++i) {
                        cmd_.position.x = 0;
                        cmd_.position.y = 0;
                        cmd_.position.z = current_pose_.pose.position.z; // 当前高度
                        cmd_.yaw = takeoff_yaw_;
                        cmd_.type_mask &= ~(mavros_msgs::PositionTarget::IGNORE_PX |
                                            mavros_msgs::PositionTarget::IGNORE_PY |
                                            mavros_msgs::PositionTarget::IGNORE_PZ |
                                            mavros_msgs::PositionTarget::IGNORE_YAW);
                        setpoint_pub_.publish(cmd_);
                        ros::Duration(0.1).sleep();
                    }
                    // 切换OFFBOARD模式
                    mavros_msgs::SetMode offb_mode;
                    offb_mode.request.custom_mode = "OFFBOARD";
                    if (set_mode_client_.call(offb_mode) && offb_mode.response.mode_sent) {
                        ROS_INFO("OFFBOARD mode set.");
                    } else {
                        ROS_WARN("Failed to set OFFBOARD mode.");
                    }
                    // 解锁
                    mavros_msgs::CommandBool arm_cmd;
                    arm_cmd.request.value = true;
                    if (arming_client_.call(arm_cmd) && arm_cmd.response.success) {
                        ROS_INFO("Vehicle armed.");
                    } else {
                        ROS_WARN("Failed to arm.");
                    }
                    takeoff_initiated = true;
                    return;
                }
                // 4. 起飞：发送目标高度指令（水平位置固定为原点0,0）
                cmd_.position.x = 0;
                cmd_.position.y = 0;
                cmd_.position.z = takeoff_h_;
                cmd_.yaw = takeoff_yaw_;
                setpoint_pub_.publish(cmd_);
                // 检查是否到达目标高度
                if (std::abs(current_pose_.pose.position.z - takeoff_h_) < 0.1) {
                    ROS_INFO("Takeoff success, switching to NAVIGATE.");
                    mission_ = NAVIGATE;
                    if (!waypoints_.empty()) {
                        current_wp_idx_ = 0;
                        wp_reached_ = false;
                        sendGoal(waypoints_[current_wp_idx_], current_wp_idx_);
                    } else {
                        ROS_ERROR("No waypoints! Stay in TAKEOFF.");
                    }
                }
                break;
            }
            
            case NAVIGATE: {
                // 持续发布控制指令（由轨迹回调更新）
                setpoint_pub_.publish(cmd_);
                geometry_msgs::Vector3 error_vec;
                error_vec.x = current_pose_.pose.position.x - waypoints_[current_wp_idx_].x;
                error_vec.y = current_pose_.pose.position.y - waypoints_[current_wp_idx_].y;
                error_vec.z = current_pose_.pose.position.z - waypoints_[current_wp_idx_].z;
                error_pub_.publish(error_vec);
                
                if (current_wp_idx_ >= (int)waypoints_.size()) {
                    ROS_INFO("All waypoints reached. Mission complete.");
                    mission_ = COMPLETE;
                    break;
                }
                
                const Waypoint& target = waypoints_[current_wp_idx_];
                if (!wp_reached_ && (is_arrived(target) || !is_planning_ok_)) {
                    if(!is_planning_ok_) {
                        ROS_WARN("Goal is not reachable, skip waypoint %d: (%.2f, %.2f, %.2f)", current_wp_idx_, target.x, target.y, target.z);
                        is_planning_ok_ = true;
                    } else {
                        ROS_INFO("Waypoint %d reached successfully.", current_wp_idx_);
                    }
                    ROS_INFO("Now position: (%.2f, %.2f, %.2f)", current_pose_.pose.position.x, current_pose_.pose.position.y, current_pose_.pose.position.z);
                    wp_reached_ = true;
                    
                    if (current_wp_idx_ + 1 < (int)waypoints_.size()) {
                        ROS_INFO("Proceeding to next waypoint %d: (%.2f, %.2f, %.2f)", current_wp_idx_ + 1, waypoints_[current_wp_idx_ + 1].x, waypoints_[current_wp_idx_ + 1].y, waypoints_[current_wp_idx_ + 1].z);
                        current_wp_idx_++;
                        sendGoal(waypoints_[current_wp_idx_], current_wp_idx_);
                        wp_reached_ = false;
                    } else {
                        ROS_INFO("All waypoints reached.");
                        mission_ = COMPLETE;
                    }
                }
                break;
            }
            
            case COMPLETE: {
                static bool land_triggered = false;
                if (!land_triggered) {
                    mavros_msgs::SetMode land_mode;
                    land_mode.request.custom_mode = "AUTO.LAND";
                    if (set_mode_client_.call(land_mode) && land_mode.response.mode_sent) {
                        ROS_INFO("AUTO.LAND mode activated.");
                        land_triggered = true;
                    } else {
                        ROS_WARN("Failed to activate AUTO.LAND, retrying...");
                    }
                }
                // 若仍处于 OFFBOARD 模式，需持续发送指令以防超时
                if (current_state_.mode == "OFFBOARD") {
                    setpoint_pub_.publish(cmd_);
                }
                // 检测落地（可选）
                if (current_pose_.pose.position.z < 0.1 && land_triggered) {
                    ROS_INFO("Landing completed.");
                }
                break;
            }
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "waypoint_navigator");
    ros::NodeHandle nh;
    WaypointNavigator nav(nh);

    // 从外部传入航点列表
    std::vector<Waypoint> waypoints;
    XmlRpc::XmlRpcValue wp_list;
    if (nh.getParam("waypoints", wp_list) && wp_list.getType() == XmlRpc::XmlRpcValue::TypeArray) {
        for (int i = 0; i < wp_list.size(); ++i) {
            XmlRpc::XmlRpcValue wp = wp_list[i];
            if (wp.getType() == XmlRpc::XmlRpcValue::TypeArray && wp.size() == 3) {
                Waypoint wp_struct;
                wp_struct.x = static_cast<double>(wp[0]);
                wp_struct.y = static_cast<double>(wp[1]);
                wp_struct.z = static_cast<double>(wp[2]);
                waypoints.push_back(wp_struct);
                ROS_INFO("Loaded waypoint %d: (%.2f, %.2f, %.2f)", i, wp_struct.x, wp_struct.y, wp_struct.z);
            } else {
                ROS_WARN("Waypoint %d format invalid, skipping", i);
            }
        }
    } else {
        ROS_ERROR("Failed to load waypoints from parameter server.");
    }
    
    nav.start(waypoints);
    ros::spin();
    return 0;
}