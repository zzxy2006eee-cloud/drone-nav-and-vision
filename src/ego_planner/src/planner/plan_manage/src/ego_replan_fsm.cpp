#include <plan_manage/ego_replan_fsm.h>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace ego_planner
{

  void EGOReplanFSM::init(ros::NodeHandle &nh)
  {
    current_wp_ = 0;
    exec_state_ = FSM_EXEC_STATE::INIT;
    have_target_ = false;
    have_odom_ = false;

    /*  fsm param  */
    nh.param("fsm/flight_type", target_type_, -1);
    nh.param("fsm/thresh_replan", replan_thresh_, -1.0);
    nh.param("fsm/thresh_no_replan", no_replan_thresh_, -1.0);
    nh.param("fsm/planning_horizon", planning_horizen_, -1.0);
    nh.param("fsm/planning_horizen_time", planning_horizen_time_, -1.0);
    nh.param("fsm/emergency_time_", emergency_time_, 1.0);
    nh.param("/drone/terminal_approach_distance_m", terminal_distance_, .8);
    nh.param("/drone/terminal_speed_mps", terminal_speed_, .2);
    nh.param("/drone/terminal_deceleration_mps2", terminal_deceleration_, .4);
    nh.param("/drone/terminal_settle_time_s", terminal_settle_time_, 3.);
    if (!std::isfinite(terminal_distance_) || terminal_distance_ < .3 || terminal_distance_ > 2. ||
        !std::isfinite(terminal_speed_) || terminal_speed_ < .1 || terminal_speed_ > .3 ||
        !std::isfinite(terminal_deceleration_) || terminal_deceleration_ <= 0. || terminal_deceleration_ > .5 ||
        !std::isfinite(terminal_settle_time_) || terminal_settle_time_ < 1. || terminal_settle_time_ > 5.)
      throw std::runtime_error("Invalid terminal approach parameters");

    nh.param("fsm/waypoint_num", waypoint_num_, -1);
    for (int i = 0; i < waypoint_num_; i++)
    {
      nh.param("fsm/waypoint" + to_string(i) + "_x", waypoints_[i][0], -1.0);
      nh.param("fsm/waypoint" + to_string(i) + "_y", waypoints_[i][1], -1.0);
      nh.param("fsm/waypoint" + to_string(i) + "_z", waypoints_[i][2], -1.0);
    }

    /* initialize main modules */
    visualization_.reset(new PlanningVisualization(nh));
    planner_manager_.reset(new EGOPlannerManager);
    planner_manager_->initPlanModules(nh, visualization_);

    start_service_ = nh.advertiseService("/planning/start", &EGOReplanFSM::startPlanningCallback, this);
    nh.param("fsm/use_global_route",use_global_route_,false);
    global_route_sub_=nh.subscribe("/drone/global_route",1,&EGOReplanFSM::globalRouteCallback,this);
    local_target_pub_=nh.advertise<geometry_msgs::PoseStamped>("/planning/local_target",1);
    clearLocalPlan();
    speed_service_ = nh.advertiseService("/planning/apply_speed", &EGOReplanFSM::applySpeedCallback, this);

    /* callback */
    exec_timer_ = nh.createTimer(ros::Duration(0.01), &EGOReplanFSM::execFSMCallback, this);
    safety_timer_ = nh.createTimer(ros::Duration(0.05), &EGOReplanFSM::checkCollisionCallback, this);

    odom_sub_ = nh.subscribe("/odom_world", 1, &EGOReplanFSM::odometryCallback, this);
    planning_enabled_sub_ = nh.subscribe("/drone/planning_enabled", 1, &EGOReplanFSM::planningEnabledCallback, this);

    bspline_pub_ = nh.advertise<ego_planner::Bspline>("/planning/bspline", 10);
    data_disp_pub_ = nh.advertise<ego_planner::DataDisp>("/planning/data_display", 100);

    if (target_type_ == TARGET_TYPE::MANUAL_TARGET && !use_global_route_)
      waypoint_sub_ = nh.subscribe("/waypoint_generator/waypoints", 1, &EGOReplanFSM::waypointCallback, this);
    else if (target_type_ == TARGET_TYPE::PRESET_TARGET)
    {
      ros::Duration(1.0).sleep();
      while (ros::ok() && !have_odom_)
        ros::spinOnce();
      planGlobalTrajbyGivenWps();
    }
    else if (!use_global_route_)
      cout << "Wrong target_type_ value! target_type_=" << target_type_ << endl;
  }

  void EGOReplanFSM::clearLocalPlan()
  {
    planning_enabled_=false; trigger_=false; have_target_=false; have_new_target_=false;
    last_failed_plan_=ros::Time(0);
    auto &local=planner_manager_->local_data_;
    const int id=local.traj_id_; // Preserve monotonically increasing IDs.
    local=LocalTrajData(); local.traj_id_=id; local.duration_=0.;
    local.global_time_offset=0.; local.start_time_=ros::Time(0); local.start_pos_.setZero();
    // Global route belongs to the prepared target. Clear its old local splice
    // and progress; the next start service rebuilds the global polynomial.
    auto &global=planner_manager_->global_data_;
    global.local_traj_.clear(); global.last_progress_time_=0.;
    global.local_start_time_=-1.; global.local_end_time_=-1.;
    global.time_increase_=0.; global.last_time_inc_=0.;
    changeFSMExecState(INIT, "RESET");
  }

  bool EGOReplanFSM::startPlanningCallback(ego_planner::StartPlanning::Request &req,
                                         ego_planner::StartPlanning::Response &reply)
  {
    reply.generation=req.generation;
    reply.next_trajectory_id=planner_manager_->local_data_.traj_id_+1;
    if (req.generation<session_generation_) {
      reply.success=false; reply.message="Obsolete planning session"; return true;
    }
    session_generation_=req.generation;
    prepared_start_=false;
    clearLocalPlan();
    route_points_.clear(); route_stamp_=ros::Time(0);
    terminal_approach_=false;terminal_entry_planned_=false;
    inspection_route_=req.inspection;continuous_route_=req.continuous && req.inspection;
    if (!req.enabled) { reply.success=true; reply.message="Previous local curve cleared"; return true; }
    const auto &p=req.goal.pose.position;
    const double age=(ros::Time::now()-odom_stamp_).toSec();
    if (!have_odom_ || odom_stamp_.isZero() || age<-.02 || age>.7 ||
        req.goal.header.frame_id!="odom" || !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
        fabs(p.x)>8 || fabs(p.y)>8 || p.z<.5 || p.z>2.5 ||
        !std::isfinite(req.speed_mps) || req.speed_mps<.1 || req.speed_mps>1.) {
      reply.success=false; reply.message="Invalid goal, speed or stale start odometry"; return true;
    }
    navigation_speed_=req.speed_mps;
    planner_manager_->setNavigationSpeed(req.speed_mps);
    ros::param::set("/ego_planner_node/manager/max_vel",req.speed_mps);
    ros::param::set("/ego_planner_node/optimization/max_vel",req.speed_mps);
    end_pt_=Eigen::Vector3d(p.x,p.y,p.z); end_vel_.setZero(); init_pt_=odom_pos_;
    route_sequence_=req.goal.header.stamp.toNSec();
    if (!installRoute(req.route)) {
      reply.success=false; reply.message="Fresh complete unified route required"; return true;
    }
    prepared_start_=true;
    // The manager publishes enabled=true only AFTER this ACK. Until then both
    // timers stay disabled, so a collision callback cannot restore an old curve.
    reply.success=true; reply.message="New target prepared from current odometry";
    ROS_INFO("Prepared planning session %lu, next curve >= %d, start [%.3f %.3f %.3f]",
             static_cast<unsigned long>(req.generation),reply.next_trajectory_id,
             odom_pos_.x(),odom_pos_.y(),odom_pos_.z());
    return true;
  }

  bool EGOReplanFSM::applySpeedCallback(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& reply)
  {
    double speed = 0.5;
    ros::param::get("/drone/navigation_speed_request", speed);
    if (!std::isfinite(speed) || speed < 0.1 || speed > 1.0) {
      reply.success = false; reply.message = "Invalid navigation speed"; return true;
    }
    // This service runs on the planner's single callback thread, before the
    // manager sends the new goal; update both cached planning limits together.
    prepared_start_=false;
    clearLocalPlan();
    planner_manager_->setNavigationSpeed(speed);
    ros::param::set("/ego_planner_node/manager/max_vel", speed);
    ros::param::set("/ego_planner_node/optimization/max_vel", speed);
    reply.success = true; reply.message = "Planner speed applied: " + std::to_string(speed);
    ROS_INFO("Navigation speed applied: %.2f m/s", speed);
    return true;
  }

  void EGOReplanFSM::planGlobalTrajbyGivenWps()
  {
    std::vector<Eigen::Vector3d> wps(waypoint_num_);
    for (int i = 0; i < waypoint_num_; i++)
    {
      wps[i](0) = waypoints_[i][0];
      wps[i](1) = waypoints_[i][1];
      wps[i](2) = waypoints_[i][2];

      end_pt_ = wps.back();
    }
    bool success = planner_manager_->planGlobalTrajWaypoints(odom_pos_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), wps, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    for (size_t i = 0; i < (size_t)waypoint_num_; i++)
    {
      visualization_->displayGoalPoint(wps[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
      ros::Duration(0.001).sleep();
    }

    if (success)
    {

      /*** display ***/
      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
      std::vector<Eigen::Vector3d> gloabl_traj(i_end);
      for (int i = 0; i < i_end; i++)
      {
        gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
      }

      end_vel_.setZero();
      have_target_ = true;
      have_new_target_ = true;

      /*** FSM ***/
      // if (exec_state_ == WAIT_TARGET)
      changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      // else if (exec_state_ == EXEC_TRAJ)
      //   changeFSMExecState(REPLAN_TRAJ, "TRIG");

      // visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
      ros::Duration(0.001).sleep();
      visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
      ros::Duration(0.001).sleep();
    }
    else
    {
      ROS_ERROR("Unable to generate global trajectory!");
    }
  }

  void EGOReplanFSM::waypointCallback(const nav_msgs::PathConstPtr &msg)
  {
    if (!planning_enabled_ || msg->poses.empty())
      return;
    if (msg->poses[0].pose.position.z < -0.1)
      return;

    cout << "Triggered!" << endl;
    trigger_ = true;
    init_pt_ = odom_pos_;

    bool success = false;
    //end_pt_ << msg->poses[0].pose.position.x, msg->poses[0].pose.position.y, 1.0;
    double goal_z = 1.0;
    if((msg->poses[0].pose.position.z) == 0)
    {
      goal_z = 1.0;
    }
    else
    {
      goal_z = msg->poses[0].pose.position.z;
    }
    //end_pt_ << msg->poses[0].pose.position.x, msg->poses[0].pose.position.y, msg->poses[0].pose.position.z;
    end_pt_ << msg->poses[0].pose.position.x, msg->poses[0].pose.position.y, goal_z;
    success = planner_manager_->planGlobalTraj(odom_pos_, odom_vel_, Eigen::Vector3d::Zero(), end_pt_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, 0);

    if (success)
    {

      /*** display ***/
      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
      vector<Eigen::Vector3d> gloabl_traj(i_end);
      for (int i = 0; i < i_end; i++)
      {
        gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
      }

      end_vel_.setZero();
      have_target_ = true;
      have_new_target_ = true;

      /*** FSM ***/
      if (exec_state_ == WAIT_TARGET)
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      else if (exec_state_ == EXEC_TRAJ)
        changeFSMExecState(REPLAN_TRAJ, "TRIG");

      // visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
      visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    }
    else
    {
      ROS_ERROR("Unable to generate global trajectory!");
    }
  }

  void EGOReplanFSM::planningEnabledCallback(const std_msgs::BoolConstPtr &msg)
  {
    if (!msg->data) {
      // An earlier STOP topic can arrive after the prepare service on another
      // connection. It clears active state but preserves the prepared target.
      clearLocalPlan();
    } else if (prepared_start_) {
      prepared_start_=false;
      planning_enabled_=true; trigger_=true; have_target_=true; have_new_target_=true;
      changeFSMExecState(GEN_NEW_TRAJ,"ACK_START");
    } // A bare true without an acknowledged target cannot resume any curve.
  }

  void EGOReplanFSM::odometryCallback(const nav_msgs::OdometryConstPtr &msg)
  {
    odom_stamp_=msg->header.stamp;
    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    odom_pos_(2) = msg->pose.pose.position.z;

    odom_vel_(0) = msg->twist.twist.linear.x;
    odom_vel_(1) = msg->twist.twist.linear.y;
    odom_vel_(2) = msg->twist.twist.linear.z;

    //odom_acc_ = estimateAcc( msg );

    odom_orient_.w() = msg->pose.pose.orientation.w;
    odom_orient_.x() = msg->pose.pose.orientation.x;
    odom_orient_.y() = msg->pose.pose.orientation.y;
    odom_orient_.z() = msg->pose.pose.orientation.z;

    have_odom_ = true;
  }

  void EGOReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call)
  {

    if (new_state == exec_state_)
      continously_called_times_++;
    else
      continously_called_times_ = 1;

    static string state_str[7] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};
    int pre_s = int(exec_state_);
    exec_state_ = new_state;
    cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
  }

  std::pair<int, EGOReplanFSM::FSM_EXEC_STATE> EGOReplanFSM::timesOfConsecutiveStateCalls()
  {
    return std::pair<int, FSM_EXEC_STATE>(continously_called_times_, exec_state_);
  }

  void EGOReplanFSM::printFSMExecState()
  {
    static string state_str[7] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};

    cout << "[FSM]: state: " + state_str[int(exec_state_)] << endl;
  }

  void EGOReplanFSM::execFSMCallback(const ros::TimerEvent &e)
  {
    if (!planning_enabled_)
      return;
    
    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 100)
    {
      printFSMExecState();
      if (!have_odom_)
        cout << "no odom." << endl;
      if (!trigger_)
        cout << "wait for goal." << endl;
      fsm_num = 0;
    }

    data_disp_.fsm_state = static_cast<int>(exec_state_);
    data_disp_.header.stamp = ros::Time::now();
    data_disp_pub_.publish(data_disp_);
    if ((exec_state_ == GEN_NEW_TRAJ || exec_state_ == REPLAN_TRAJ) &&
        !last_failed_plan_.isZero() && (ros::Time::now()-last_failed_plan_).toSec() < 0.2)
      return; // Publish heartbeat above even while backing off failed planning.
    switch (exec_state_)
    {
    case INIT:
    {
      if (!have_odom_)
      {
        return;
      }
      if (!trigger_)
      {
        return;
      }
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }

    case WAIT_TARGET:
    {
      if (!have_target_)
        return;
      else
      {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case GEN_NEW_TRAJ:
    {
      start_pt_ = odom_pos_;
      start_vel_ = odom_vel_;
      // New plans start while the manager holds/brakes the aircraft. Small
      // residual EKF velocity must not bend the route backwards at its origin.
      // This is a planning boundary condition, not a change to odometry/fusion.
      if (start_vel_.norm() <= 0.1)
      {
        start_vel_.setZero();
        ROS_INFO_THROTTLE(1.0, "Stationary local-plan start: zero boundary velocity");
      }
      start_acc_.setZero();

      // Eigen::Vector3d rot_x = odom_orient_.toRotationMatrix().block(0, 0, 3, 1);
      // start_yaw_(0)         = atan2(rot_x(1), rot_x(0));
      // start_yaw_(1) = start_yaw_(2) = 0.0;

      bool flag_random_poly_init;
      if (timesOfConsecutiveStateCalls().first == 1)
        flag_random_poly_init = false;
      else
        flag_random_poly_init = true;

      bool success = callReboundReplan(true, flag_random_poly_init);
      if (success)
      {

        changeFSMExecState(EXEC_TRAJ, "FSM");
        flag_escape_emergency_ = true;
      }
      else
      {
        last_failed_plan_ = ros::Time::now();
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case REPLAN_TRAJ:
    {

      if (planFromCurrentTraj())
      {
        changeFSMExecState(EXEC_TRAJ, "FSM");
      }
      else
      {
        last_failed_plan_ = ros::Time::now();
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }

      break;
    }

    case EXEC_TRAJ:
    {
      /* determine if need to replan */
      LocalTrajData *info = &planner_manager_->local_data_;
      ros::Time time_now = ros::Time::now();
      double t_cur = (time_now - info->start_time_).toSec();
      t_cur = min(info->duration_, t_cur);

      Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t_cur);

      // Replan once on entry, before the normal near-goal no-replan branch.
      // Use route arc distance, so a nearby goal across an obstacle is not
      // mistaken for a short final approach.
      const double approach_distance=std::max(terminal_distance_,
          navigation_speed_*navigation_speed_/(2*terminal_deceleration_)+.3);
      if (use_global_route_ && !continuous_route_ && !terminal_entry_planned_ &&
          remainingRouteDistance(pos) <= approach_distance &&
          t_cur < info->duration_ - .1) {
        terminal_approach_=true;terminal_entry_planned_=true;
        have_new_target_=true;
        ROS_INFO("Entering terminal approach; remaining arc %.3f m",remainingRouteDistance(pos));
        changeFSMExecState(REPLAN_TRAJ,"TERMINAL_APPROACH");return;
      }
      if (t_cur > info->duration_ - 1e-2)
      {
        // Let the controller settle on the actual zero-speed final spline.
        // The manager still checks arrival and current tracking safety.
        if (terminal_approach_ && (pos-end_pt_).norm()<=.02 &&
            info->velocity_traj_.evaluateDeBoorT(info->duration_).norm()<=.03 &&
            (time_now-info->start_time_).toSec()<info->duration_+terminal_settle_time_)
          return;
        if (use_global_route_ && (continuous_route_ || (end_pt_-odom_pos_).norm()>.15 || odom_vel_.norm()>.15)) {
          have_new_target_=true;
          changeFSMExecState(GEN_NEW_TRAJ,"ROUTE_CONTINUE");
          return;
        }
        have_target_ = false;

        changeFSMExecState(WAIT_TARGET, "FSM");
        return;
      }
      else if (!continuous_route_ && (end_pt_ - pos).norm() < no_replan_thresh_)
      {
        // cout << "near end" << endl;
        return;
      }
      else if ((info->start_pos_ - pos).norm() < replan_thresh_)
      {
        // cout << "near start" << endl;
        return;
      }
      else
      {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }
      break;
    }

    case EMERGENCY_STOP:
    {

      if (flag_escape_emergency_) // Avoiding repeated calls
      {
        callEmergencyStop(odom_pos_);
      }
      else
      {
        if (odom_vel_.norm() < 0.1)
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }

      flag_escape_emergency_ = false;
      break;
    }
    }
    
  }

  bool EGOReplanFSM::planFromCurrentTraj()
  {

    LocalTrajData *info = &planner_manager_->local_data_;
    if (!planning_enabled_ || !have_target_ || info->start_time_.isZero() || info->duration_<=0 ||
        (exec_state_!=EXEC_TRAJ && exec_state_!=REPLAN_TRAJ)) return false;
    ros::Time time_now = ros::Time::now();
    double t_cur = (time_now - info->start_time_).toSec();
    if (t_cur<0 || t_cur>info->duration_) return false;

    //cout << "info->velocity_traj_=" << info->velocity_traj_.get_control_points() << endl;

    start_pt_ = info->position_traj_.evaluateDeBoorT(t_cur);
    start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
    start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);

    bool success = callReboundReplan(false, false);

    if (!success)
    {
      success = callReboundReplan(true, false);
      //changeFSMExecState(EXEC_TRAJ, "FSM");
      if (!success)
      {
        success = callReboundReplan(true, true);
        if (!success)
        {
          return false;
        }
      }
    }

    return true;
  }

  void EGOReplanFSM::checkCollisionCallback(const ros::TimerEvent &e)
  {
    if (!planning_enabled_)
      return;
    LocalTrajData *info = &planner_manager_->local_data_;
    auto map = planner_manager_->grid_map_;

    if (!have_target_ || (exec_state_ != EXEC_TRAJ && exec_state_ != REPLAN_TRAJ) ||
        info->start_time_.isZero() || info->duration_<=0)
      return;

    /* ---------- check trajectory ---------- */
    constexpr double time_step = 0.01;
    double t_cur = (ros::Time::now() - info->start_time_).toSec();
    double t_2_3 = info->duration_ * 2 / 3;
    for (double t = t_cur; t < info->duration_; t += time_step)
    {
      if (t_cur < t_2_3 && t >= t_2_3) // If t_cur < t_2_3, only the first 2/3 partition of the trajectory is considered valid and will get checked.
        break;

      if (map->getInflateOccupancy(info->position_traj_.evaluateDeBoorT(t)))
      {
        if (planFromCurrentTraj()) // Make a chance
        {
          changeFSMExecState(EXEC_TRAJ, "SAFETY");
          return;
        }
        else
        {
          if (t - t_cur < emergency_time_) // 0.8s of emergency time
          {
            ROS_WARN("Suddenly discovered obstacles. emergency stop! time=%f", t - t_cur);
            changeFSMExecState(EMERGENCY_STOP, "SAFETY");
          }
          else
          {
            //ROS_WARN("current traj in collision, replan.");
            changeFSMExecState(REPLAN_TRAJ, "SAFETY");
          }
          return;
        }
        break;
      }
    }
  }

  bool EGOReplanFSM::callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj)
  {

    if (!getLocalTarget()) return false;

    bool plan_success =
        planner_manager_->reboundReplan(start_pt_, start_vel_, start_acc_, local_target_pt_, local_target_vel_,
            (have_new_target_ || flag_use_poly_init), flag_randomPolyTraj,
            terminal_approach_ && (local_target_pt_-end_pt_).norm()<.01);
    have_new_target_ = false;

    cout << "final_plan_success=" << plan_success << endl;

    if (plan_success)
    {

      auto info = &planner_manager_->local_data_;

      /* publish traj */
      ego_planner::Bspline bspline;
      bspline.order = 3;
      bspline.start_time = info->start_time_;
      bspline.traj_id = info->traj_id_;

      Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
      bspline.pos_pts.reserve(pos_pts.cols());
      for (int i = 0; i < pos_pts.cols(); ++i)
      {
        geometry_msgs::Point pt;
        pt.x = pos_pts(0, i);
        pt.y = pos_pts(1, i);
        pt.z = pos_pts(2, i);
        bspline.pos_pts.push_back(pt);
      }

      Eigen::VectorXd knots = info->position_traj_.getKnot();
      bspline.knots.reserve(knots.rows());
      for (int i = 0; i < knots.rows(); ++i)
      {
        bspline.knots.push_back(knots(i));
      }

      bspline_pub_.publish(bspline);

      visualization_->displayOptimalList(info->position_traj_.get_control_points(), 0);
    }

    return plan_success;
  }

  bool EGOReplanFSM::callEmergencyStop(Eigen::Vector3d stop_pos)
  {

    planner_manager_->EmergencyStop(stop_pos);

    auto info = &planner_manager_->local_data_;

    /* publish traj */
    ego_planner::Bspline bspline;
    bspline.order = 3;
    bspline.start_time = info->start_time_;
    bspline.traj_id = info->traj_id_;

    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
    bspline.pos_pts.reserve(pos_pts.cols());
    for (int i = 0; i < pos_pts.cols(); ++i)
    {
      geometry_msgs::Point pt;
      pt.x = pos_pts(0, i);
      pt.y = pos_pts(1, i);
      pt.z = pos_pts(2, i);
      bspline.pos_pts.push_back(pt);
    }

    Eigen::VectorXd knots = info->position_traj_.getKnot();
    bspline.knots.reserve(knots.rows());
    for (int i = 0; i < knots.rows(); ++i)
    {
      bspline.knots.push_back(knots(i));
    }

    bspline_pub_.publish(bspline);

    return true;
  }

  bool EGOReplanFSM::getLocalTarget()
  {
    if (use_global_route_) return getRouteTarget();
    double t;

    double t_step = planning_horizen_ / 20 / planner_manager_->pp_.max_vel_;
    double dist_min = 9999, dist_min_t = 0.0;
    for (t = planner_manager_->global_data_.last_progress_time_; t < planner_manager_->global_data_.global_duration_; t += t_step)
    {
      Eigen::Vector3d pos_t = planner_manager_->global_data_.getPosition(t);
      double dist = (pos_t - start_pt_).norm();

      if (t < planner_manager_->global_data_.last_progress_time_ + 1e-5 && dist > planning_horizen_)
      {
        // todo
        ROS_WARN_THROTTLE(1.0, "Global progress outside local horizon; requesting a fresh task curve");
        return false;
      }
      if (dist < dist_min)
      {
        dist_min = dist;
        dist_min_t = t;
      }
      if (dist >= planning_horizen_)
      {
        local_target_pt_ = pos_t;
        planner_manager_->global_data_.last_progress_time_ = dist_min_t;
        break;
      }
    }
    if (t >= planner_manager_->global_data_.global_duration_) // Last global point
    {
      local_target_pt_ = end_pt_;
    }

    // A horizon point on the global polynomial may lie inside an obstacle,
    // even when the operator's final goal is free. Select a nearby free local
    // endpoint; the optimizer and continuous-curve guard still check the route.
    auto map = planner_manager_->grid_map_;
    if (!map->isInMap(local_target_pt_) || map->getInflateOccupancy(local_target_pt_) != 0)
    {
      if ((local_target_pt_-end_pt_).norm() < 1e-3)
      {
        ROS_WARN_THROTTLE(1.0, "Final navigation goal is occupied or outside map");
        return false;
      }
      const Eigen::Vector3d desired = local_target_pt_;
      double best = std::numeric_limits<double>::infinity();
      Eigen::Vector3d candidate = desired;
      for (double radius : {0.2, 0.4, 0.8, 1.2})
        for (int direction=0; direction<16; ++direction)
        {
          const double angle=direction*2.0*M_PI/16.0;
          Eigen::Vector3d p=desired+Eigen::Vector3d(radius*cos(angle),radius*sin(angle),0);
          if (fabs(p.x())>8 || fabs(p.y())>8 || p.z()<0.5 || p.z()>2.5 ||
              !map->isInMap(p) || map->getInflateOccupancy(p)!=0 ||
              (p-start_pt_).norm()>planning_horizen_+0.2 ||
              (p-end_pt_).norm() >= (start_pt_-end_pt_).norm()-0.1) continue;
          const double cost=(p-desired).norm()+0.35*(p-end_pt_).norm();
          if (cost<best) { best=cost; candidate=p; }
        }
      if (!std::isfinite(best))
      {
        ROS_WARN_THROTTLE(1.0, "No free local horizon endpoint; retaining final goal for later replan");
        return false;
      }
      local_target_pt_=candidate;
      local_target_vel_=Eigen::Vector3d::Zero();
      ROS_INFO_THROTTLE(1.0, "Occupied local horizon endpoint adjusted to [%.2f %.2f %.2f]",
                        candidate.x(),candidate.y(),candidate.z());
      return true;
    }
    t=std::min(t,planner_manager_->global_data_.global_duration_);
    if ((end_pt_ - local_target_pt_).norm() < (planner_manager_->pp_.max_vel_ * planner_manager_->pp_.max_vel_) / (2 * planner_manager_->pp_.max_acc_))
    {
      // local_target_vel_ = (end_pt_ - init_pt_).normalized() * planner_manager_->pp_.max_vel_ * (( end_pt_ - local_target_pt_ ).norm() / ((planner_manager_->pp_.max_vel_*planner_manager_->pp_.max_vel_)/(2*planner_manager_->pp_.max_acc_)));
      // cout << "A" << endl;
      local_target_vel_ = Eigen::Vector3d::Zero();
    }
    else
    {
      local_target_vel_ = planner_manager_->global_data_.getVelocity(t);
      if (local_target_vel_.norm()>planner_manager_->pp_.max_vel_)
        local_target_vel_=local_target_vel_.normalized()*planner_manager_->pp_.max_vel_;
      // cout << "AA" << endl;
    }
    return true;
  }

  bool EGOReplanFSM::installRoute(const nav_msgs::Path &msg)
  {
    const double age=(ros::Time::now()-msg.header.stamp).toSec();
    if (msg.header.frame_id!="odom" ||
        msg.poses.size()<2 || msg.poses.size()>2048 || age<-.02 || age>2.) return false;
    vector<Eigen::Vector3d> points;
    for (const auto &pose:msg.poses) {
      const auto &p=pose.pose.position;
      Eigen::Vector3d v(p.x,p.y,p.z);
      if (!v.allFinite() || fabs(v.x())>8 || fabs(v.y())>8 || v.z()<.5 || v.z()>2.5) return false;
      if (points.empty() || (points.back()-v).norm()>1e-5) points.push_back(v);
    }
    if (points.size()<2 || (points.back()-end_pt_).norm()>.02) return false;
    route_points_=std::move(points);route_stamp_=msg.header.stamp;
    return true;
  }

  void EGOReplanFSM::globalRouteCallback(const ego_planner::GlobalRouteConstPtr &msg)
  {
    if (!use_global_route_ || (!planning_enabled_ && !prepared_start_) || msg->task_id!=route_sequence_) return;
    if (inspection_route_ && !msg->inspection)return;
    if (inspection_route_ && msg->blocked) {route_points_.clear();route_stamp_=ros::Time(0);return;}
    // Empty/partial routes mean the front end is searching. Suspend target
    // generation until a complete replacement arrives; safety still checks
    // the currently active local curve against the latest occupied map.
    if (msg->path.poses.size()<2) {
      route_points_.clear();route_stamp_=ros::Time(0);return;
    }
    const Eigen::Vector3d old_end=end_pt_;
    if (inspection_route_ && msg->inspection && !msg->path.poses.empty()) {
      const auto &p=msg->path.poses.back().pose.position;
      end_pt_=Eigen::Vector3d(p.x,p.y,p.z);
    }
    if (!installRoute(msg->path)) {route_points_.clear();end_pt_=old_end;}
    else if (inspection_route_ && msg->inspection) {
      continuous_route_=msg->continuous;
      if (continuous_route_) {terminal_approach_=false;terminal_entry_planned_=false;}
    }
  }

  double EGOReplanFSM::remainingRouteDistance(const Eigen::Vector3d& position) const
  {
    if (route_points_.size()<2 || route_stamp_.isZero() ||
        (ros::Time::now()-route_stamp_).toSec() < -.02 ||
        (ros::Time::now()-route_stamp_).toSec() > 2.) return std::numeric_limits<double>::infinity();
    size_t segment=0;double gap=std::numeric_limits<double>::infinity();Eigen::Vector3d projection;
    for (size_t i=0;i+1<route_points_.size();++i) {
      const Eigen::Vector3d d=route_points_[i+1]-route_points_[i];
      if (d.squaredNorm()<1e-12) continue;
      const double u=std::max(0.,std::min(1.,(position-route_points_[i]).dot(d)/d.squaredNorm()));
      const Eigen::Vector3d q=route_points_[i]+u*d;
      if ((q-position).norm()<=gap+1e-8) {gap=(q-position).norm();segment=i;projection=q;}
    }
    if (!std::isfinite(gap) || gap>.75) return std::numeric_limits<double>::infinity();
    double remaining=gap;Eigen::Vector3d from=projection;
    for (size_t i=segment+1;i<route_points_.size();++i) {remaining+=(route_points_[i]-from).norm();from=route_points_[i];}
    return remaining;
  }

  bool EGOReplanFSM::getRouteTarget()
  {
    const double age=(ros::Time::now()-route_stamp_).toSec();
    if (route_points_.size()<2 || age<-.02 || age>2.) {
      ROS_WARN_THROTTLE(1.,"Waiting for fresh complete unified global route");return false;
    }
    // Project the planning start onto the current polyline, then stop the
    // local target at its next bend. A direct horizon chord must not cut
    // across a corner that the global A* deliberately routed around.
    size_t segment=0;double best=1e9;Eigen::Vector3d projection;
    for(size_t i=0;i+1<route_points_.size();++i) {
      Eigen::Vector3d d=route_points_[i+1]-route_points_[i];
      double u=std::max(0.,std::min(1.,(start_pt_-route_points_[i]).dot(d)/d.squaredNorm()));
      Eigen::Vector3d q=route_points_[i]+u*d;double gap=(q-start_pt_).norm();
      if(gap<=best+1e-8) {best=gap;segment=i;projection=q;}
    }
    if(best>.75) {ROS_WARN_THROTTLE(1.,"Planning start too far from unified route");return false;}
    const double approach_distance=std::max(terminal_distance_,
        navigation_speed_*navigation_speed_/(2*terminal_deceleration_)+.3);
    const double arc_distance=remainingRouteDistance(start_pt_);
    if (!continuous_route_ && arc_distance <= approach_distance) terminal_approach_=true;
    // Retain the current boundary speed while braking. Reducing the velocity
    // limit below it would make a continuous start boundary infeasible.
    const double speed=terminal_approach_ ? std::min(navigation_speed_,std::max(terminal_speed_,start_vel_.norm())) : navigation_speed_;
    planner_manager_->setNavigationSpeed(speed);
    double remaining=planning_horizen_;Eigen::Vector3d from=projection;
    bool stop=false;Eigen::Vector3d end_tangent=Eigen::Vector3d::Zero();
    vector<Eigen::Vector3d> seed;seed.push_back(start_pt_);
    if((projection-start_pt_).norm()>.02) seed.push_back(projection);
    local_target_pt_=projection;
    for(size_t i=segment+1;i<route_points_.size();++i) {
      Eigen::Vector3d d=route_points_[i]-from;double length=d.norm();
      if(length<1e-6) {from=route_points_[i];continue;}
      end_tangent=d/length;
      if(length>=remaining) {
        local_target_pt_=from+end_tangent*remaining;seed.push_back(local_target_pt_);break;
      }
      remaining-=length;local_target_pt_=route_points_[i];seed.push_back(local_target_pt_);
      if(i+1==route_points_.size()) {stop=!continuous_route_;break;}
      // Dense smooth samples are a continuous reference: do not stop at each
      // tiny chord or replace the curve by a start-to-horizon straight seed.
      from=route_points_[i];
    }
    auto map=planner_manager_->grid_map_;
    if(!map->isInMap(local_target_pt_) || map->getInflateOccupancy(local_target_pt_)!=0) return false;
    if((local_target_pt_-start_pt_).norm()<(terminal_approach_ && stop ? .02 : .1) || seed.size()<2) return false;
    planner_manager_->route_seed_=std::move(seed);
    local_target_vel_=stop?Eigen::Vector3d::Zero().eval():(end_tangent*planner_manager_->pp_.max_vel_).eval();
    geometry_msgs::PoseStamped target;target.header.frame_id="odom";target.header.stamp=ros::Time::now();
    target.pose.position.x=local_target_pt_.x();target.pose.position.y=local_target_pt_.y();target.pose.position.z=local_target_pt_.z();target.pose.orientation.w=1;
    local_target_pub_.publish(target);
    ROS_INFO_THROTTLE(1.,"Unified route local target [%.3f %.3f %.3f], endpoint_stop=%d",local_target_pt_.x(),local_target_pt_.y(),local_target_pt_.z(),stop);
    return true;
  }

} // namespace ego_planner
