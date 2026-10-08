#ifndef _REBO_REPLAN_FSM_H_
#define _REBO_REPLAN_FSM_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <vector>
#include <visualization_msgs/Marker.h>

#include <bspline_opt/bspline_optimizer.h>
#include <plan_env/grid_map.h>
#include <ego_planner/Bspline.h>
#include <ego_planner/DataDisp.h>
#include <std_srvs/Trigger.h>
#include <ego_planner/StartPlanning.h>
#include <ego_planner/GlobalRoute.h>
#include <plan_manage/planner_manager.h>
#include <traj_utils/planning_visualization.h>

using std::vector;

namespace ego_planner
{

  class EGOReplanFSM
  {

  private:
    /* ---------- flag ---------- */
    enum FSM_EXEC_STATE
    {
      INIT,
      WAIT_TARGET,
      GEN_NEW_TRAJ,
      REPLAN_TRAJ,
      EXEC_TRAJ,
      EMERGENCY_STOP
    };
    enum TARGET_TYPE
    {
      MANUAL_TARGET = 1,
      PRESET_TARGET = 2,
      REFENCE_PATH = 3
    };

    /* planning utils */
    EGOPlannerManager::Ptr planner_manager_;
    PlanningVisualization::Ptr visualization_;
    ego_planner::DataDisp data_disp_;

    /* parameters */
    int target_type_; // 1 mannual select, 2 hard code
    double no_replan_thresh_, replan_thresh_;
    double waypoints_[50][3];
    int waypoint_num_;
    double planning_horizen_, planning_horizen_time_;
    double emergency_time_;
    double navigation_speed_{0.5}, terminal_distance_{0.8}, terminal_speed_{0.2}, terminal_deceleration_{0.4}, terminal_settle_time_{3.0};
    bool terminal_approach_{false}, terminal_entry_planned_{false};
    double remainingRouteDistance(const Eigen::Vector3d&) const;

    /* planning data */
    bool trigger_, have_target_, have_odom_, have_new_target_;
    bool planning_enabled_{false};
    bool prepared_start_{false};
    uint64_t session_generation_{0};
    bool use_global_route_{false};
    bool inspection_route_{false}, continuous_route_{false};
    uint64_t route_sequence_{0};
    ros::Time route_stamp_;
    vector<Eigen::Vector3d> route_points_;
    ros::Time odom_stamp_;
    ros::Time last_failed_plan_;
    FSM_EXEC_STATE exec_state_;
    int continously_called_times_{0};

    Eigen::Vector3d odom_pos_, odom_vel_, odom_acc_; // odometry state
    Eigen::Quaterniond odom_orient_;

    Eigen::Vector3d init_pt_, start_pt_, start_vel_, start_acc_, start_yaw_; // start state
    Eigen::Vector3d end_pt_, end_vel_;                                       // goal state
    Eigen::Vector3d local_target_pt_, local_target_vel_;                     // local target state
    int current_wp_;

    bool flag_escape_emergency_;

    /* ROS utils */
    ros::NodeHandle node_;
    ros::Timer exec_timer_, safety_timer_;
    ros::ServiceServer speed_service_, start_service_;
    ros::Subscriber waypoint_sub_, odom_sub_, planning_enabled_sub_;
    ros::Subscriber global_route_sub_;
    ros::Publisher local_target_pub_;
    ros::Publisher replan_pub_, new_pub_, bspline_pub_, data_disp_pub_;

    /* helper functions */
    bool callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj); // front-end and back-end method
    bool callEmergencyStop(Eigen::Vector3d stop_pos);                          // front-end and back-end method
    bool planFromCurrentTraj();

    /* return value: std::pair< Times of the same state be continuously called, current continuously called state > */
    void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call);
    std::pair<int, EGOReplanFSM::FSM_EXEC_STATE> timesOfConsecutiveStateCalls();
    void printFSMExecState();

    void planGlobalTrajbyGivenWps();
    bool getLocalTarget();
    bool getRouteTarget();
    bool installRoute(const nav_msgs::Path &);
    void globalRouteCallback(const ego_planner::GlobalRouteConstPtr &);
    void clearLocalPlan();
    bool startPlanningCallback(ego_planner::StartPlanning::Request&, ego_planner::StartPlanning::Response&);
    bool applySpeedCallback(std_srvs::Trigger::Request&, std_srvs::Trigger::Response&);

    /* ROS functions */
    void execFSMCallback(const ros::TimerEvent &e);
    void checkCollisionCallback(const ros::TimerEvent &e);
    void waypointCallback(const nav_msgs::PathConstPtr &msg);
    void planningEnabledCallback(const std_msgs::BoolConstPtr &msg);
    void odometryCallback(const nav_msgs::OdometryConstPtr &msg);

    bool checkCollision();

  public:
    EGOReplanFSM(/* args */)
    {
    }
    ~EGOReplanFSM()
    {
    }

    void init(ros::NodeHandle &nh);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

} // namespace ego_planner

#endif
