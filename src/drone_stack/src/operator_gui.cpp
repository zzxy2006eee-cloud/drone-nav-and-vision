#include <plan_env/MapArchive.h>
#include <drone_stack/MapSession.h>
#include <QFileDialog>
#include <QDir>
#include <QLineEdit>
#include <drone_stack/QueueGoal.h>
#include <drone_stack/SetMaxFlightHeight.h>
#include <drone_stack/SetNavigationSpeed.h>
#include <drone_stack/Takeoff.h>
#include <drone_stack/PlanInspection.h>
#include <drone_stack/InspectionCommand.h>
#include <drone_stack/InspectionRegions.h>
#include <drone_stack/AiText.h>
#include <drone_stack/AiGate.h>
#include <drone_stack/AiCancel.h>
#include <QSignalBlocker>
#include <QShortcut>
#include <QUuid>
#include <QTabBar>
#include <QStackedWidget>
#include <QComboBox>
#include <QJsonArray>
#include <QFutureWatcher>
#include <QtConcurrent/QtConcurrentRun>
#include <geometry_msgs/PointStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/master.h>
#include <ros/ros.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <QCheckBox>
#include <QMouseEvent>
#include <QWheelEvent>
#include <OgreCamera.h>
#include <OgreManualObject.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreMaterialManager.h>
#include <OgreTechnique.h>
#include <OgrePass.h>
#include <OgreRay.h>
#include <OgrePlane.h>
#include <visualization_msgs/MarkerArray.h>
#include <algorithm>
#include <vector>
#include <QDateTime>
#include <QJsonDocument>
#include <QJsonObject>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Header.h>
#include <std_msgs/String.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>

#include <rviz/display.h>
#include <rviz/display_group.h>
#include <rviz/properties/property_tree_model.h>
#include <rviz/render_panel.h>
#include <rviz/tool.h>
#include <rviz/tool_manager.h>
#include <rviz/view_manager.h>
#include <rviz/view_controller.h>
#include <rviz/visualization_manager.h>
#include <rviz/properties/property.h>
#include <rviz/properties/vector_property.h>

#include <QApplication>
#include <QDoubleSpinBox>
#include <QFrame>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QImage>
#include <QLabel>
#include <QMainWindow>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QPixmap>
#include <QPushButton>
#include <QScrollArea>
#include <QSizePolicy>
#include <QStringList>
#include <QSplitter>
#include <QTimer>
#include <QVBoxLayout>

#include <cmath>
#include <map>
#include <mutex>
#include <string>

class OperatorWindow : public QMainWindow {
public:
  explicit OperatorWindow(ros::NodeHandle& nh) : nh_(nh) {
    setWindowTitle(QStringLiteral("巡检四旋翼 · 综合操作台"));
    resize(1600, 960);
    setMinimumSize(1100, 700);
    setStyleSheet(QStringLiteral(
      "QMainWindow,QWidget#root,QWidget#side,QScrollArea{background:#101721;color:#e7edf5;}"
      "QLabel,QCheckBox{color:#dce5f0;}"
      "QLabel#title{font-size:22pt;font-weight:700;color:#f3f7fb;}"
      "QLabel#card{background:#17212e;border:1px solid #405067;border-radius:7px;"
      "padding:8px;color:#dce5f0;font-size:11pt;}"
      "QGroupBox{background:#17212e;border:1px solid #2b3a4e;border-radius:7px;"
      "margin-top:20px;padding:14px 8px 8px;color:#dce5f0;font-size:13pt;}"
      "QGroupBox::title{subcontrol-origin:margin;left:12px;padding:0 5px;}"
      "QPushButton{background:#245d87;color:white;border:1px solid #347bb0;"
      "border-radius:5px;min-height:34px;padding:3px 9px;}"
      "QPushButton:disabled{background:#293442;color:#6f7b8a;}"
      "QPushButton#land{background:#8b4e1f;border-color:#b87132;}"
      "QDoubleSpinBox{background:#0d141d;color:white;border:1px solid #405067;"
      "min-height:30px;}QPlainTextEdit{background:#0d141d;color:#d4deea;}"));
    nh_.param("/use_sim_time", use_sim_time_, false);
    buildUi();

    auth_client_ = nh_.serviceClient<std_srvs::SetBool>("/drone/set_authorized");
    arm_client_ = nh_.serviceClient<std_srvs::Trigger>("/drone/arm");
    takeoff_client_ = nh_.serviceClient<drone_stack::Takeoff>("/drone/takeoff");
    hold_client_ = nh_.serviceClient<std_srvs::Trigger>("/drone/hold");
    inspection_plan_client_=nh_.serviceClient<drone_stack::PlanInspection>("/drone/inspection/plan");
    inspection_command_client_=nh_.serviceClient<drone_stack::InspectionCommand>("/drone/inspection/command");
    inspection_regions_client_=nh_.serviceClient<drone_stack::InspectionRegions>("/drone/inspection/regions");
    ai_text_client_=nh_.serviceClient<drone_stack::AiText>("/drone/ai/submit_text");
    ai_gate_client_=nh_.serviceClient<drone_stack::AiGate>("/drone/ai/set_gate");
    ai_cancel_client_=nh_.serviceClient<drone_stack::AiCancel>("/drone/ai/cancel");
    ai_session_id_=QUuid::createUuid().toString(QUuid::WithoutBraces).toStdString();
    ai_heartbeat_pub_=nh_.advertise<std_msgs::String>("/drone/ai/client_heartbeat",1);
    ai_status_sub_=nh_.subscribe<std_msgs::String>("/drone/ai/status",1,[this](const std_msgs::String::ConstPtr& m){
      std::lock_guard<std::mutex> guard(mutex_);ai_snapshot_=QJsonDocument::fromJson(QByteArray::fromStdString(m->data)).object();ai_receipt_=ros::WallTime::now();
    });
    ai_events_sub_=nh_.subscribe<std_msgs::String>("/drone/ai/events",20,[this](const std_msgs::String::ConstPtr& m){
      const auto data=QJsonDocument::fromJson(QByteArray::fromStdString(m->data)).object();
      std::lock_guard<std::mutex> guard(mutex_);ai_pending_events_<<QStringLiteral("[%1 / 步骤%2] %3").arg(data["state"].toString()).arg(data["step"].toInt()+1).arg(data["message"].toString());
      if(ai_pending_events_.size()>200)ai_pending_events_.removeFirst();
    });
    inspection_draft_pub_=nh_.advertise<visualization_msgs::MarkerArray>("/drone/inspection/draft",1,true);
    inspection_sub_=nh_.subscribe<std_msgs::String>("/drone/inspection/status",1,[this](const std_msgs::String::ConstPtr& m){
      std::lock_guard<std::mutex> guard(mutex_);inspection_snapshot_=QJsonDocument::fromJson(QByteArray::fromStdString(m->data)).object();inspection_receipt_=ros::WallTime::now();
    });
    cancel_client_ = nh_.serviceClient<std_srvs::Trigger>("/drone/cancel_goal");
    land_client_ = nh_.serviceClient<std_srvs::Trigger>("/drone/land");
    direct_land_client_ = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    goal_client_ = nh_.serviceClient<drone_stack::QueueGoal>("/drone/queue_goal");
    speed_client_ = nh_.serviceClient<drone_stack::SetNavigationSpeed>("/drone/set_navigation_speed");
    map_session_client_ = nh_.serviceClient<drone_stack::MapSession>("/drone/map_session");
    map_session_sub_ = nh_.subscribe<std_msgs::String>("/drone/map_session_status",1,[this](const std_msgs::String::ConstPtr& m){
      const auto object=QJsonDocument::fromJson(QByteArray::fromStdString(m->data)).object();
      std::lock_guard<std::mutex> guard(mutex_);map_session_mode_=object["mode"].toString();map_session_message_=object["message"].toString();alignment_ready_=object["ready"].toBool();
    });
    map_transform_sub_=nh_.subscribe<geometry_msgs::PoseStamped>("/drone/map_transform",1,[this](const geometry_msgs::PoseStamped::ConstPtr& m){
      std::lock_guard<std::mutex> guard(mutex_);map_transform_=*m;map_transform_changed_=true;
    });
    map_mode_sub_ = nh_.subscribe<std_msgs::String>("/drone/map_mode",1,[this](const std_msgs::String::ConstPtr& m){std::lock_guard<std::mutex> guard(mutex_);map_mode_=QString::fromStdString(m->data);});
    map_status_sub_ = nh_.subscribe<std_msgs::String>("/drone/map_status",1,[this](const std_msgs::String::ConstPtr& m){std::lock_guard<std::mutex> guard(mutex_);map_status_=QString::fromStdString(m->data);});
    ceiling_client_ = nh_.serviceClient<drone_stack::SetMaxFlightHeight>("/drone/set_max_flight_height");
    queue_sub_ = nh_.subscribe<nav_msgs::Path>("/drone/goal_queue", 1, [this](const nav_msgs::Path::ConstPtr& msg) {
      std::lock_guard<std::mutex> guard(mutex_); queue_message_ = *msg; have_queue_update_ = true;
    });
    ceiling_sub_ = nh_.subscribe<std_msgs::Float64>("/drone/max_flight_height", 1, [this](const std_msgs::Float64::ConstPtr& msg) {
      if (!std::isfinite(msg->data)) return;
      std::lock_guard<std::mutex> guard(mutex_); ceiling_value_ = msg->data; have_ceiling_update_ = true;
    });
    state_sub_ = nh_.subscribe("/mavros/state", 5, &OperatorWindow::onState, this);
    odom_sub_ = nh_.subscribe("/mavros/local_position/odom", 5, &OperatorWindow::onOdom, this);
    battery_sub_ = nh_.subscribe("/mavros/battery", 5, &OperatorWindow::onBattery, this);
    lio_quality_sub_ = nh_.subscribe<std_msgs::String>("/drone/flight_health_snapshot", 1, [this](const std_msgs::String::ConstPtr& msg) {
      const auto doc = QJsonDocument::fromJson(QByteArray::fromStdString(msg->data));
      if (!doc.isObject()) return;
      const auto obj = doc.object();
      if (!obj.value("valid").isBool() || !obj.value("quality").isString()) return;
      std::lock_guard<std::mutex> guard(mutex_);
      lio_quality_ = obj.value("quality").toString().toStdString();
      lio_valid_ = obj.value("valid").toBool();
      lio_receipt_ = ros::WallTime::now(); lio_ros_receipt_ = ros::Time::now();
    });
    heartbeat_sub_ = nh_.subscribe<std_msgs::Header>("/drone/manager_heartbeat", 1, [this](const std_msgs::Header::ConstPtr&) {
      std::lock_guard<std::mutex> guard(mutex_); manager_receipt_ = ros::WallTime::now(); manager_ros_receipt_ = ros::Time::now();
    });
    cloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("/drone/cloud_fcu_world", 1, [this](const sensor_msgs::PointCloud2::ConstPtr& msg) {
      std::lock_guard<std::mutex> guard(mutex_);
      cloud_stamp_ = msg->header.stamp; cloud_receipt_ = ros::WallTime::now();
    });
    map_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("/grid_map/occupancy_inflate", 1, [this](const sensor_msgs::PointCloud2::ConstPtr& msg) {
      std::lock_guard<std::mutex> guard(mutex_);
      map_stamp_ = msg->header.stamp; map_receipt_ = ros::WallTime::now();
    });
    phase_sub_ = nh_.subscribe("/drone/flight_state", 5, &OperatorWindow::onPhase, this);
    auth_sub_ = nh_.subscribe("/drone/authorized", 5, &OperatorWindow::onAuth, this);
    error_sub_ = nh_.subscribe("/drone/flight_error", 5, &OperatorWindow::onError, this);
    front_sub_ = nh_.subscribe("/drone/front/image_processed", 1, &OperatorWindow::onFront, this);
    down_sub_ = nh_.subscribe("/drone/down/image_processed", 1, &OperatorWindow::onDown, this);
    navigation_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/drone/gui/navigation_markers", 1, true);
    active_goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/drone/gui/active_goal", 1, false);
    global_path_sub_ = nh_.subscribe<nav_msgs::Path>("/drone/global_path", 1, [this](const nav_msgs::Path::ConstPtr& msg) {
      std::lock_guard<std::mutex> guard(mutex_); global_path_message_ = *msg; have_global_path_ = true;
    });
    global_marker_sub_ = nh_.subscribe<visualization_msgs::MarkerArray>("/drone/global_path_markers", 1, [this](const visualization_msgs::MarkerArray::ConstPtr& msg) {
      std::lock_guard<std::mutex> guard(mutex_); yellow_markers_ = *msg; have_yellow_markers_ = true;
      yellow_receipt_ = ros::WallTime::now();
      yellow_stamp_ = msg->markers.empty() ? ros::Time(0) : msg->markers.front().header.stamp;
    });
    local_path_sub_ = nh_.subscribe<nav_msgs::Path>("/drone/executed_local_path", 1, [this](const nav_msgs::Path::ConstPtr& msg) {
      std::lock_guard<std::mutex> guard(mutex_); local_path_message_ = *msg; have_local_path_ = true;
      local_path_receipt_ = ros::WallTime::now();
    });
    global_status_sub_ = nh_.subscribe<std_msgs::String>("/drone/global_path_status", 1, [this](const std_msgs::String::ConstPtr& msg) {
      std::lock_guard<std::mutex> guard(mutex_); global_status_ = QString::fromStdString(msg->data);
    });
    navigation_wait_sub_ = nh_.subscribe<std_msgs::String>("/drone/navigation_wait_reason", 1, [this](const std_msgs::String::ConstPtr& msg) {
      std::lock_guard<std::mutex> guard(mutex_); navigation_wait_ = QString::fromStdString(msg->data);
    });
    clicked_sub_ = nh_.subscribe("/clicked_point", 3, &OperatorWindow::onClicked, this);
    auto* timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, [this] { refresh(); });
    timer->start(150);
  }

private:
  enum class CardColor { Green, Yellow, Red };

  void setCardColor(const QString& key, CardColor color) {
    auto* card = cards_.at(key);
    const int value = static_cast<int>(color);
    if (card->property("healthState").isValid() && card->property("healthState").toInt() == value) return;
    card->setProperty("healthState", value);
    const QString background = color == CardColor::Green ? "#205c3b" : color == CardColor::Yellow ? "#765b19" : "#7b2c36";
    const QString border = color == CardColor::Green ? "#53b880" : color == CardColor::Yellow ? "#e7bd45" : "#e16875";
    card->setStyleSheet(QStringLiteral("QLabel#card{background:%1;border:2px solid %2;border-radius:7px;padding:8px;color:#ffffff;font-size:11pt;}").arg(background, border));
  }

  void showTopDown() {
    two_d_view_ = true;
    manager_->getViewManager()->setCurrentViewControllerType("rviz/TopDownOrtho");
    auto* view = manager_->getViewManager()->getCurrent();
    view->subProp("Scale")->setValue(std::max(10.0, std::min(render_->width(), render_->height())/18.0));
    view->subProp("Angle")->setValue(0.0);
    nav_msgs::Odometry odom;
    { std::lock_guard<std::mutex> guard(mutex_); odom = odom_; }
    const auto center=mapFromOdom(odom.pose.pose.position);
    view->subProp("X")->setValue(center.x);
    view->subProp("Y")->setValue(center.y);
    manager_->getToolManager()->setCurrentTool(manager_->getToolManager()->getDefaultTool());
  }

  geometry_msgs::Point mapFromOdom(const geometry_msgs::Point& p) const {
    geometry_msgs::Point out;const double c=std::cos(map_yaw_view_),sn=std::sin(map_yaw_view_);
    out.x=c*p.x-sn*p.y+map_translation_view_.x;out.y=sn*p.x+c*p.y+map_translation_view_.y;out.z=p.z+map_translation_view_.z;return out;
  }
  geometry_msgs::Point odomFromMap(const geometry_msgs::Point& p) const {
    geometry_msgs::Point out;const double c=std::cos(map_yaw_view_),sn=std::sin(map_yaw_view_);
    const double x=p.x-map_translation_view_.x,y=p.y-map_translation_view_.y;
    out.x=c*x+sn*y;out.y=-sn*x+c*y;out.z=p.z-map_translation_view_.z;return out;
  }
  void publishInitialPose(bool show) {
    visualization_msgs::MarkerArray markers;visualization_msgs::Marker m;
    m.header.frame_id="map";m.header.stamp=ros::Time::now();m.ns="initial_pose";m.id=0;
    m.type=visualization_msgs::Marker::ARROW;m.action=show?visualization_msgs::Marker::ADD:visualization_msgs::Marker::DELETE;
    m.pose.position.x=map_dx_->value();m.pose.position.y=map_dy_->value();m.pose.position.z=map_dz_->value()+.1;
    const double yaw=map_yaw_->value()*std::acos(-1.)/180.;m.pose.orientation.z=std::sin(yaw/2);m.pose.orientation.w=std::cos(yaw/2);
    m.scale.x=.8;m.scale.y=.12;m.scale.z=.12;m.color.r=1.;m.color.a=1.;markers.markers.push_back(m);navigation_pub_.publish(markers);
  }

  bool eventFilter(QObject* watched, QEvent* event) override {
    if (watched == render_ && two_d_view_) {
      if(picking_initial_pose_ && event->type()==QEvent::MouseMove){
        if(initial_drag_active_){
          auto* mouse=static_cast<QMouseEvent*>(event);
          const auto ray=manager_->getViewManager()->getCurrent()->getCamera()->getCameraToViewportRay(
            mouse->localPos().x()/std::max(1,render_->width()),mouse->localPos().y()/std::max(1,render_->height()));
          const auto hit=ray.intersects(Ogre::Plane(Ogre::Vector3::UNIT_Z,0.));
          if(hit.first){const auto p=ray.getPoint(hit.second);map_dx_->setValue(initial_drag_start_.x);map_dy_->setValue(initial_drag_start_.y);
            if(std::hypot(p.x-initial_drag_start_.x,p.y-initial_drag_start_.y)>.1)map_yaw_->setValue(std::atan2(p.y-initial_drag_start_.y,p.x-initial_drag_start_.x)*180./std::acos(-1.));
            publishInitialPose(true);}
        }
        return true;
      }
      if (event->type() == QEvent::Wheel) {
        auto* wheel = static_cast<QWheelEvent*>(event);
        auto* scale = manager_->getViewManager()->getCurrent()->subProp("Scale");
        const double steps = wheel->angleDelta().y()/120.0;
        scale->setValue(std::max(2.0, std::min(2000.0, scale->getValue().toDouble()*std::pow(1.2, steps))));
        event->accept(); return true;
      }
      if (event->type() == QEvent::MouseButtonPress || event->type() == QEvent::MouseButtonRelease) {
        auto* mouse = static_cast<QMouseEvent*>(event);
        if (inspection_picking_ && mouse->button()==Qt::LeftButton) {
          const auto ray=manager_->getViewManager()->getCurrent()->getCamera()->getCameraToViewportRay(
              mouse->localPos().x()/std::max(1,render_->width()),mouse->localPos().y()/std::max(1,render_->height()));
          const auto hit=ray.intersects(Ogre::Plane(Ogre::Vector3::UNIT_Z,0.));
          if(hit.first){
            const auto point=ray.getPoint(hit.second);
            if(event->type()==QEvent::MouseButtonPress && inspection_picking_==1)inspection_drag_start_=point;
            if(event->type()==QEvent::MouseButtonRelease){
              if(inspection_picking_==1){
                const auto a=inspection_drag_start_;
                inspection_polygon_={{a.x,a.y},{point.x,a.y},{point.x,point.y},{a.x,point.y}};
                inspection_picking_=0;
              }else if(inspection_polygon_.size()<64)inspection_polygon_.push_back({point.x,point.y});
              inspection_dirty_=true;publishInspectionDraft();
            }
          }
          return true;
        }
        if (picking_initial_pose_ && mouse->button()==Qt::LeftButton) {
          const auto ray=manager_->getViewManager()->getCurrent()->getCamera()->getCameraToViewportRay(
              mouse->localPos().x()/std::max(1,render_->width()),mouse->localPos().y()/std::max(1,render_->height()));
          const auto hit=ray.intersects(Ogre::Plane(Ogre::Vector3::UNIT_Z,0.));
          if(hit.first){
            const auto p=ray.getPoint(hit.second);
            if(event->type()==QEvent::MouseButtonPress){initial_drag_start_=p;initial_drag_active_=true;}
            else if(initial_drag_active_){
              map_dx_->setValue(initial_drag_start_.x);map_dy_->setValue(initial_drag_start_.y);
              if(std::hypot(p.x-initial_drag_start_.x,p.y-initial_drag_start_.y)>.1)
                map_yaw_->setValue(std::atan2(p.y-initial_drag_start_.y,p.x-initial_drag_start_.x)*180./std::acos(-1.));
              initial_drag_active_=false;picking_initial_pose_=false;publishInitialPose(true);
              log(QStringLiteral("初始位置与朝向已选择；点击确认初始位姿并加载导航。"));
            }
          }
          return true;
        }
        if (picking_goal_ && mouse->button() == Qt::LeftButton) {
          if (event->type() == QEvent::MouseButtonRelease) {
            const auto ray = manager_->getViewManager()->getCurrent()->getCamera()->getCameraToViewportRay(
                mouse->localPos().x()/std::max(1,render_->width()), mouse->localPos().y()/std::max(1,render_->height()));
            const auto hit = ray.intersects(Ogre::Plane(Ogre::Vector3::UNIT_Z, 0.0));
            if (hit.first) {
              const auto point = ray.getPoint(hit.second);
              std::lock_guard<std::mutex> guard(mutex_);
              if (!have_click_) {
                pending_click_.header.frame_id = "map"; pending_click_.header.stamp = ros::Time::now();
                pending_click_.point.x = point.x; pending_click_.point.y = point.y;
                have_click_ = true; pick_button_->setEnabled(false);
              }
            }
          }
          return true;
        }
      }
    }
    return QMainWindow::eventFilter(watched, event);
  }

  void publishNavigation() {
    visualization_msgs::MarkerArray array;
    for (int id=0; id<3; ++id) {
      visualization_msgs::Marker m;
      m.header.frame_id = "odom"; m.header.stamp = ros::Time::now();
      m.ns = "operator_navigation"; m.id = id; m.pose.orientation.w = 1.0;
      m.action = have_target_marker_ ? visualization_msgs::Marker::ADD : visualization_msgs::Marker::DELETE;
      m.color.a = 1.0;
      if (id == 0) {
        m.type = visualization_msgs::Marker::CYLINDER; m.pose.position = selected_goal_;
        m.pose.position.z += .08; m.scale.x = .30; m.scale.y = .30; m.scale.z = .04; m.color.r = 1.0;
      } else {
        m.type = visualization_msgs::Marker::LINE_STRIP; m.scale.x = .055;
        m.color.r = id == 1 ? 1.0 : .10; m.color.g = 1.0; m.color.b = id == 1 ? 0.0 : .15;
        m.points = id == 1 ? planned_path_ : actual_path_;
        // Yellow reference segments are owned by the map-aware route node,
        // which distinguishes observed solid lines from unknown dashed lines.
        if (id == 1 || m.points.size()<2) m.action = visualization_msgs::Marker::DELETE;
        for (auto& point : m.points) point.z += .06;
      }
      array.markers.push_back(m);
    }
    const size_t count = queued_points_.size();
    for (size_t i = 0; i < std::max(count, rendered_queue_count_); ++i) {
      for (int label = 0; label < 2; ++label) {
        visualization_msgs::Marker m;
        m.header.frame_id = "odom"; m.header.stamp = ros::Time::now();
        m.ns = "operator_goal_queue"; m.id = 2*i + label; m.pose.orientation.w = 1.0;
        m.action = i < count ? visualization_msgs::Marker::ADD : visualization_msgs::Marker::DELETE;
        m.color.r = 1.0; m.color.a = 1.0;
        if (i < count) m.pose.position = queued_points_[i];
        if (label) {
          m.type = visualization_msgs::Marker::TEXT_VIEW_FACING; m.scale.z = .32;
          m.pose.position.z += .35; m.text = std::to_string(i+1);
        } else {
          m.type = visualization_msgs::Marker::CYLINDER;
          m.pose.position.z += .08; m.scale.x = .30; m.scale.y = .30; m.scale.z = .04;
          // The active point is already drawn by operator_navigation/0.
          if (i == 0) m.action = visualization_msgs::Marker::DELETE;
        }
        array.markers.push_back(m);
      }
    }
    rendered_queue_count_ = count;
    navigation_pub_.publish(array);
  }

  Ogre::ManualObject* createRouteLayer(const std::string& name, unsigned char queue) {
    const std::string materialName = "drone_layer_" + name;
    auto material = Ogre::MaterialManager::getSingleton().create(materialName, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    material->setLightingEnabled(false);
    material->setDepthCheckEnabled(false); material->setDepthWriteEnabled(false);
    material->setCullingMode(Ogre::CULL_NONE);
    material->getTechnique(0)->getPass(0)->setVertexColourTracking(Ogre::TVC_DIFFUSE);
    auto* object = manager_->getSceneManager()->createManualObject(materialName);
    object->setDynamic(true); object->setRenderQueueGroup(queue);
    overlay_node_->attachObject(object);
    return object;
  }

  void drawRouteLayer(Ogre::ManualObject* object, const std::vector<geometry_msgs::Point>& points,
                      bool pairs, const Ogre::ColourValue& color, double width) {
    object->clear(); if (points.size()<2) return;
    object->begin(object->getName(), Ogre::RenderOperation::OT_TRIANGLE_LIST);
    const auto vertex = [&](const Ogre::Vector3& p) { object->position(p); object->colour(color); };
    for (size_t i=0;i+1<points.size();i+=pairs?2:1) {
      const auto& a=points[i];const auto& b=points[i+1];
      const double length=std::hypot(b.x-a.x,b.y-a.y);
      if(length<1e-8 && std::abs(b.z-a.z)<1e-8) continue;
      Ogre::Vector3 offset = length<1e-8 ? Ogre::Vector3(width/2,0,0) :
          Ogre::Vector3(-(b.y-a.y)*width/(2*length),(b.x-a.x)*width/(2*length),0);
      Ogre::Vector3 start(a.x,a.y,a.z),end(b.x,b.y,b.z);
      vertex(start+offset);vertex(start-offset);vertex(end-offset);
      vertex(start+offset);vertex(end-offset);vertex(end+offset);
    }
    object->end(); object->setBoundingBox(Ogre::AxisAlignedBox::BOX_INFINITE);
  }

  void drawHeadingLayer(const nav_msgs::Odometry& odom, bool valid) {
    heading_layer_->clear(); if (!valid) return;
    const auto& p=odom.pose.pose.position;const auto& q=odom.pose.pose.orientation;
    const double yaw=std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
    Ogre::Vector3 origin(p.x,p.y,p.z+.1),forward(std::cos(yaw),std::sin(yaw),0),side(-std::sin(yaw),std::cos(yaw),0);
    heading_layer_->begin(heading_layer_->getName(),Ogre::RenderOperation::OT_TRIANGLE_LIST);
    const auto vertex=[&](double x,double y) {heading_layer_->position(origin+forward*x+side*y);heading_layer_->colour(Ogre::ColourValue(1,.05,.05,1));};
    vertex(-.12,.05);vertex(-.12,-.05);vertex(.32,-.05);
    vertex(-.12,.05);vertex(.32,-.05);vertex(.32,.05);
    vertex(.24,.14);vertex(.24,-.14);vertex(.62,0);
    heading_layer_->end();heading_layer_->setBoundingBox(Ogre::AxisAlignedBox::BOX_INFINITE);
  }

  void buildUi() {
    auto* root = new QWidget(this);
    root->setObjectName("root");
    auto* main = new QVBoxLayout(root);
    main->setContentsMargins(12, 10, 12, 10);
    auto* title = new QLabel(QStringLiteral("巡检四旋翼 · 综合操作台"), root);
    title->setObjectName("title");
    main->addWidget(title);
    auto* cards = new QHBoxLayout();
    for (const QString& key : {"ROS", "PX4", "授权/ARM", "模式", "电池", "LIO", "雷达/EGO", "双相机", "告警"}) {
      auto* label = new QLabel(key + QStringLiteral("\n--"), root);
      label->setObjectName("card");
      label->setAlignment(Qt::AlignCenter);
      label->setFixedHeight(64);
      label->setWordWrap(true);
      label->setSizePolicy(QSizePolicy::Ignored,QSizePolicy::Fixed);
      cards->addWidget(label, 1);
      cards_[key] = label;
    }
    main->addLayout(cards);
    telemetry_ = new QLabel(QStringLiteral("等待位姿与飞行状态"), root);
    readiness_ = new QLabel(QStringLiteral("等待系统就绪"), root);
    telemetry_->setFixedHeight(24);
    telemetry_->setSizePolicy(QSizePolicy::Ignored,QSizePolicy::Fixed);
    readiness_->setWordWrap(true);
    readiness_->setFixedHeight(40);
    readiness_->setSizePolicy(QSizePolicy::Ignored,QSizePolicy::Fixed);
    main->addWidget(telemetry_);
    main->addWidget(readiness_);
    page_tabs_=new QTabBar(root);page_tabs_->addTab(QStringLiteral("综合页面"));page_tabs_->addTab(QStringLiteral("区域巡检"));page_tabs_->addTab(QStringLiteral("AI文本控制"));
    page_tabs_->setExpanding(false);main->addWidget(page_tabs_);
    auto* horizontal = new QSplitter(Qt::Horizontal, root);
    auto* left = new QWidget(horizontal);
    auto* leftLayout = new QVBoxLayout(left);
    auto* viewTools = new QHBoxLayout();
    auto* view3d = new QPushButton(QStringLiteral("三维视图"), left);
    auto* view2d = new QPushButton(QStringLiteral("俯视图"), left);
    auto* pickGoal = new QPushButton(QStringLiteral("点选 XY 目标"), left);
    viewTools->addWidget(view3d);
    viewTools->addWidget(view2d);
    viewTools->addWidget(pickGoal);
    pick_button_ = pickGoal;
    auto* resetView = new QPushButton(QStringLiteral("复位视角"), left);
    viewTools->addWidget(resetView);
    viewTools->addStretch();
    leftLayout->addLayout(viewTools);
    render_ = new rviz::RenderPanel(left);
    leftLayout->addWidget(render_, 1);
    manager_ = new rviz::VisualizationManager(render_);
    render_->initialize(manager_->getSceneManager(), manager_);
    render_->setAutoRender(true);
    render_->installEventFilter(this);
    manager_->initialize();
    manager_->getRootDisplayGroup()->setEnabled(true);
    manager_->setFixedFrame("map");
    overlay_node_=manager_->getSceneManager()->getRootSceneNode()->createChildSceneNode();
    manager_->getDisplayTreeModel()->getRoot()->subProp("Global Options")->subProp("Frame Rate")->setValue(15);
    manager_->startUpdate();
    manager_->getViewManager()->setCurrentViewControllerType("rviz/Orbit");
    auto* view = manager_->getViewManager()->getCurrent();
    view->subProp("Distance")->setValue(14.0);
    view->subProp("Pitch")->setValue(0.75);
    view->subProp("Yaw")->setValue(0.8);
    static_cast<rviz::VectorProperty*>(view->subProp("Focal Point"))->setVector(Ogre::Vector3(1.5, 0, 1));
    auto* grid = manager_->createDisplay("rviz/Grid", "ENU 地面网格", true);
    grid->subProp("Plane Cell Count")->setValue(30);
    grid->subProp("Cell Size")->setValue(0.5);
    // Use the cloud registered into PX4's local ENU frame.  This keeps the
    // lidar map, EGO voxels and MAVROS odometry aligned in the same RViz view.
    addDisplay("rviz/PointCloud2", "MID360 点云", "/drone/cloud_fcu_world", "Points");
    addDisplay("rviz/PointCloud2", "EGO 体素", "/grid_map/occupancy_inflate", "Boxes");
    addDisplay("rviz/PointCloud2", "预建地图预览", "/drone/prior_map_preview", "Boxes");
    // Explicit render queues implement the requested ordering independently
    // of camera angle and physical altitude. Geometry comes from real routes.
    yellow_layer_ = createRouteLayer("global_yellow", 95);
    purple_layer_ = createRouteLayer("executed_purple", 96);
    heading_layer_ = createRouteLayer("heading_red", 97);
    for (const auto& title : {QStringLiteral("MID360 点云"), QStringLiteral("EGO 体素"), QStringLiteral("EGO 轨迹")}) {
      auto* toggle = new QCheckBox(title, left);
      toggle->setChecked(true);
      viewTools->addWidget(toggle);
      connect(toggle, &QCheckBox::toggled, this, [this, title](bool enabled) {
        if (displays_.count(title)) displays_[title]->setEnabled(enabled);
        if (title == QStringLiteral("EGO 轨迹")) purple_layer_->setVisible(enabled);
      });
    }
    connect(resetView, &QPushButton::clicked, this, [this] {
      two_d_view_ = false; picking_goal_ = false;
      manager_->getViewManager()->setCurrentViewControllerType("rviz/Orbit");
      auto* view = manager_->getViewManager()->getCurrent();
      view->subProp("Distance")->setValue(14.0);
      view->subProp("Pitch")->setValue(0.75);
      view->subProp("Yaw")->setValue(0.8);
      static_cast<rviz::VectorProperty*>(view->subProp("Focal Point"))->setVector(Ogre::Vector3(1.5, 0, 1));
    });
    addDisplay("rviz/MarkerArray", "目标与路径", "/drone/gui/navigation_markers", "");
    addDisplay("rviz/MarkerArray", "巡检区域草稿", "/drone/inspection/draft", "");
    addDisplay("rviz/MarkerArray", "巡检条带预览", "/drone/inspection/markers", "");
    addDisplay("rviz/MarkerArray", "巡检覆盖统计", "/drone/inspection/coverage", "");

    navigation_hint_ = new QLabel(QStringLiteral("红点：目标  |  黄实线：已观测自由  |  黄虚线：待观测参考  |  紫线：执行中的EGO曲线  |  绿线：实际轨迹"), left);
    navigation_hint_->setWordWrap(true);
    navigation_hint_->setFixedHeight(76);
    navigation_hint_->setSizePolicy(QSizePolicy::Ignored,QSizePolicy::Fixed);
    leftLayout->addWidget(navigation_hint_);
    connect(view3d, &QPushButton::clicked, this, [this] {
      two_d_view_ = false; picking_goal_ = false;
      manager_->getViewManager()->setCurrentViewControllerType("rviz/Orbit");
    });
    connect(view2d, &QPushButton::clicked, this, [this] { picking_goal_ = false; showTopDown(); });
    connect(pickGoal, &QPushButton::clicked, this, [this] {
      picking_initial_pose_=false;
      inspection_picking_=0;
      picking_goal_ = !picking_goal_;
      if (picking_goal_) showTopDown();
      log(picking_goal_ ? QStringLiteral("连续左键点选目标，按编号依次执行；滚轮缩放，Z使用右侧高度。再次点击按钮结束点选。") : QStringLiteral("已结束点选，现有目标队列继续执行。"));
    });
    horizontal->addWidget(left);

    side_pages_=new QStackedWidget(horizontal);
    auto* sideScroll = new QScrollArea(side_pages_);
    sideScroll->setWidgetResizable(true);
    sideScroll->setFixedWidth(420);
    sideScroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    sideScroll->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOn);
    auto* side = new QWidget(sideScroll);
    side->setObjectName("side");
    auto* sideLayout = new QVBoxLayout(side);
    sideLayout->setSizeConstraint(QLayout::SetMinAndMaxSize);
    front_image_ = cameraBox(sideLayout, side, QStringLiteral("前视处理画面 · 仿真目标"));
    down_image_ = cameraBox(sideLayout, side, QStringLiteral("下视处理画面 · 仿真目标"));
    auto* ops = new QGroupBox(QStringLiteral("飞行操作"), side);
    auto* opsLayout = new QGridLayout(ops);
    ops->setFocusPolicy(Qt::StrongFocus);
    auto* auth = new QPushButton(QStringLiteral("操作授权"), ops);
    auto* arm = new QPushButton(QStringLiteral("PX4 解锁 ARM"), ops);
    arm->setFocusPolicy(Qt::NoFocus);
    auto* takeoff = new QPushButton(QStringLiteral("起飞"), ops);
    auto* hold = new QPushButton(QStringLiteral("悬停"), ops);
    auto* cancel = new QPushButton(QStringLiteral("取消目标"), ops);
    auto* land = new QPushButton(QStringLiteral("一键降落"), root);
    auto* disarm = new QPushButton(QStringLiteral("地面上锁"), ops);
    arm_button_ = arm; takeoff_button_ = takeoff; hold_button_ = hold;
    cancel_button_ = cancel; land_button_ = land; disarm_button_ = disarm;
    auth_button_ = auth;
    land->setObjectName("land");
    takeoff_height_ = spin(1.2, ops);
    takeoff_height_->setRange(0.2, 2.0);
    opsLayout->addWidget(auth, 0, 0);
    opsLayout->addWidget(arm, 0, 1);
    opsLayout->addWidget(new QLabel(QStringLiteral("相对起飞高度 m")), 1, 0);
    opsLayout->addWidget(takeoff_height_, 1, 1);
    navigation_speed_ = spin(0.5, ops);
    navigation_speed_->setRange(0.1, 1.0);
    navigation_speed_->setSingleStep(0.05);
    navigation_speed_->setSuffix(QStringLiteral(" m/s"));
    navigation_speed_->setToolTip(QStringLiteral("下一次导航任务采用此规划速度；当前任务及其重规划保持原速度。"));
    opsLayout->addWidget(new QLabel(QStringLiteral("移动速度 m/s")), 2, 0);
    opsLayout->addWidget(navigation_speed_, 2, 1);
    maximum_height_ = spin(2.5, ops);
    maximum_height_->setRange(.8, 2.5); maximum_height_->setSingleStep(.1);
    maximum_height_->setSuffix(QStringLiteral(" m"));
    maximum_height_->setToolTip(QStringLiteral("机体中心最高 ENU Z，与点选绝对Z同一基准；规划与目标保留0.20m裕量。可在未解锁地面或无任务的健康HOLD时修改。"));
    ceiling_button_ = new QPushButton(QStringLiteral("最高Z（ENU）设置"), ops);
    opsLayout->addWidget(ceiling_button_, 3, 0);
    opsLayout->addWidget(maximum_height_, 3, 1);
    opsLayout->addWidget(takeoff, 4, 0, 1, 2);
    opsLayout->addWidget(hold, 5, 0);
    opsLayout->addWidget(cancel, 5, 1);
    opsLayout->addWidget(new QLabel(QStringLiteral("降落按钮位于底部固定操作栏")), 6, 0, 1, 2);
    opsLayout->addWidget(disarm, 7, 0, 1, 2);
    sideLayout->addWidget(ops);
    auto* mapping = new QGroupBox(QStringLiteral("建图模式 / 巡检模式"),side);
    auto* mappingLayout = new QGridLayout(mapping);
    map_mode_label_ = new QLabel(QStringLiteral("地图模式等待中"),mapping);
    map_mode_label_->setWordWrap(true);
    map_mode_label_->setFixedHeight(64);
    map_mode_label_->setSizePolicy(QSizePolicy::Ignored,QSizePolicy::Fixed);
    mappingLayout->addWidget(map_mode_label_,0,0,1,2);
    std::string startupMap;nh_.param<std::string>("/drone/prebuilt_map_path",startupMap,"");
    map_file_ = new QLineEdit(startupMap.empty()?QDir::homePath()+QStringLiteral("/robotproject/project0/start/maps/inspection.dmap"):QString::fromStdString(startupMap),mapping);
    mappingLayout->addWidget(map_file_,1,0,1,2);
    auto* browseMap = new QPushButton(QStringLiteral("选择地图文件"),mapping);
    mappingLayout->addWidget(browseMap,2,0,1,2);
    connect(browseMap,&QPushButton::clicked,this,[this]{
      const auto path=QFileDialog::getOpenFileName(this,QStringLiteral("选择预建导航地图"),map_file_->text(),QStringLiteral("导航地图 (*.dmap);;所有文件 (*)"));
      if(!path.isEmpty())map_file_->setText(path);
    });
    map_dx_=spin(1.01,mapping);map_dy_=spin(.98,mapping);map_dz_=spin(.17,mapping);map_yaw_=spin(0.,mapping);
    map_dx_->setRange(-30.,30.);map_dy_->setRange(-30.,30.);map_dz_->setRange(-5.,5.);map_yaw_->setRange(-180.,180.);
    for(auto* input:{map_dx_,map_dy_,map_dz_})input->setSingleStep(.1);
    mappingLayout->addWidget(new QLabel(QStringLiteral("初始地图位置 X m")),3,0);mappingLayout->addWidget(map_dx_,3,1);
    mappingLayout->addWidget(new QLabel(QStringLiteral("初始地图位置 Y m")),4,0);mappingLayout->addWidget(map_dy_,4,1);
    mappingLayout->addWidget(new QLabel(QStringLiteral("初始地图位置 Z m")),5,0);mappingLayout->addWidget(map_dz_,5,1);
    mappingLayout->addWidget(new QLabel(QStringLiteral("初始机头朝向 yaw °")),6,0);mappingLayout->addWidget(map_yaw_,6,1);
    auto* mappingNote=new QLabel(QStringLiteral("默认预建地图：先点出生位置并拖朝向，确认后启动Gazebo。平地机体Z设0.17m。在线模式自动设本次起点。"),mapping);mappingNote->setWordWrap(true);mappingNote->setFixedHeight(80);mappingNote->setSizePolicy(QSizePolicy::Ignored,QSizePolicy::Fixed);mappingLayout->addWidget(mappingNote,7,0,1,2);
    initial_pose_button_=new QPushButton(QStringLiteral("点选初始位置并拖动朝向"),mapping);
    mappingLayout->addWidget(initial_pose_button_,8,0,1,2);
    connect(initial_pose_button_,&QPushButton::clicked,this,[this]{picking_goal_=false;showTopDown();picking_initial_pose_=true;log(QStringLiteral("在地图上按下鼠标选位置，拖动并松开设置机头朝向。"));});
    const std::vector<std::pair<QString,std::string>> operations={{QStringLiteral("开始新建图"),"new"},{QStringLiteral("保存地图"),"save"},{QStringLiteral("打开预建地图预览"),"prepare"},{QStringLiteral("确认初始位姿并加载导航"),"load"},{QStringLiteral("返回在线建图导航"),"online"}};
    for(size_t i=0;i<operations.size();++i){
      auto* button=new QPushButton(operations[i].first,mapping);map_buttons_.push_back(button);
      mappingLayout->addWidget(button,9+int(i)/2,int(i)%2);
      if(operations[i].second=="load")initial_confirm_button_=button;
      const auto action=operations[i].second;
      connect(button,&QPushButton::clicked,this,[this,action]{
        QString path=map_file_->text();
        if(action=="save"){
          QDir().mkpath(QDir::homePath()+QStringLiteral("/robotproject/project0/start/maps"));
          path=QFileDialog::getSaveFileName(this,QStringLiteral("保存导航地图"),path,QStringLiteral("导航地图 (*.dmap)"));
          if(path.isEmpty())return;
          if(!path.endsWith(QStringLiteral(".dmap")))path+=QStringLiteral(".dmap");
          map_file_->setText(path);
        }
        drone_stack::MapSession service;service.request.action=action;service.request.path=path.toStdString();
        service.request.x=map_dx_->value();service.request.y=map_dy_->value();service.request.z=map_dz_->value();service.request.yaw_deg=map_yaw_->value();
        if(!map_session_client_.call(service))log(QStringLiteral("地图会话服务不可用"));
        else {
          log(QString::fromStdString(service.response.message));
          if(service.response.success && action=="prepare") {picking_goal_=false;showTopDown();manager_->getViewManager()->getCurrent()->subProp("X")->setValue(0.);manager_->getViewManager()->getCurrent()->subProp("Y")->setValue(0.);}
          if(service.response.success && action!="prepare"){picking_initial_pose_=false;publishInitialPose(false);}
        }
      });
    }
    sideLayout->addWidget(mapping);
    auto* goals = new QGroupBox(QStringLiteral("局部三维目标 · ROS ENU"), side);
    auto* goalLayout = new QGridLayout(goals);
    relative_x_ = spin(1.0, goals);
    relative_y_ = spin(0.0, goals);
    relative_z_ = spin(0.0, goals);
    clicked_height_ = spin(1.2, goals);
    clicked_height_->setRange(.5, 2.3);
    goalLayout->addWidget(new QLabel(QStringLiteral("前向 ΔX m")), 0, 0);
    goalLayout->addWidget(relative_x_, 0, 1);
    goalLayout->addWidget(new QLabel(QStringLiteral("左向 ΔY m")), 1, 0);
    goalLayout->addWidget(relative_y_, 1, 1);
    goalLayout->addWidget(new QLabel(QStringLiteral("上向 ΔZ m")), 2, 0);
    goalLayout->addWidget(relative_z_, 2, 1);
    auto* sendRelative = new QPushButton(QStringLiteral("发送相对目标"), goals);
    goal_button_ = sendRelative;
    goalLayout->addWidget(sendRelative, 3, 0, 1, 2);
    goalLayout->addWidget(new QLabel(QStringLiteral("点选目标地图 Z m")), 4, 0);
    goalLayout->addWidget(clicked_height_, 4, 1);
    queue_status_ = new QLabel(QStringLiteral("剩余目标：0（含当前）"), goals);
    goalLayout->addWidget(queue_status_, 5, 0, 1, 2);
    sideLayout->addWidget(goals);
    auto* inspectionScroll=new QScrollArea(side_pages_);inspectionScroll->setWidgetResizable(true);
    inspectionScroll->setMinimumWidth(310);inspectionScroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    auto* inspectionSide=new QWidget(inspectionScroll);inspectionSide->setObjectName("side");
    auto* inspectionLayout=new QVBoxLayout(inspectionSide);inspectionLayout->setSizeConstraint(QLayout::SetMinAndMaxSize);
    auto* overviewLink=new QPushButton(QStringLiteral("返回综合页：授权 / 解锁 / 起飞"),inspectionSide);
    inspectionLayout->addWidget(overviewLink);connect(overviewLink,&QPushButton::clicked,this,[this]{page_tabs_->setCurrentIndex(0);});
    auto* knownBox=new QGroupBox(QStringLiteral("已知巡检区域 · 当前地图"),inspectionSide);
    auto* kl=new QGridLayout(knownBox);
    known_regions_=new QComboBox(knownBox);known_regions_->setSizeAdjustPolicy(QComboBox::AdjustToMinimumContentsLengthWithIcon);
    known_regions_->setMinimumContentsLength(12);kl->addWidget(known_regions_,0,0,1,2);
    known_name_=new QLineEdit(knownBox);known_name_->setPlaceholderText(QStringLiteral("输入区域名称（支持中文）"));known_name_->setMaxLength(64);
    kl->addWidget(known_name_,1,0,1,2);
    known_load_=new QPushButton(QStringLiteral("加载为选区"),knownBox);known_refresh_=new QPushButton(QStringLiteral("刷新列表"),knownBox);
    known_save_=new QPushButton(QStringLiteral("保存为新区域"),knownBox);known_update_=new QPushButton(QStringLiteral("更新所选区域"),knownBox);
    known_delete_=new QPushButton(QStringLiteral("删除所选区域"),knownBox);
    kl->addWidget(known_load_,2,0);kl->addWidget(known_refresh_,2,1);kl->addWidget(known_save_,3,0);kl->addWidget(known_update_,3,1);kl->addWidget(known_delete_,4,0,1,2);
    inspectionLayout->addWidget(knownBox);
    connect(known_refresh_,&QPushButton::clicked,this,[this]{requestKnownRegions("list");});
    connect(known_load_,&QPushButton::clicked,this,[this]{loadKnownRegion();});
    connect(known_save_,&QPushButton::clicked,this,[this]{requestKnownRegions("save");});
    connect(known_update_,&QPushButton::clicked,this,[this]{requestKnownRegions("update");});
    connect(known_delete_,&QPushButton::clicked,this,[this]{requestKnownRegions("delete");});
    auto* inspectionBox=new QGroupBox(QStringLiteral("下视区域巡检 · 预建地图专用"),inspectionSide);
    auto* il=new QGridLayout(inspectionBox);
    inspection_status_=new QLabel(QStringLiteral("等待巡检管理器"),inspectionBox);
    inspection_status_->setWordWrap(true);inspection_status_->setFixedHeight(95);inspection_status_->setSizePolicy(QSizePolicy::Ignored,QSizePolicy::Fixed);
    il->addWidget(inspection_status_,0,0,1,2);
    inspection_alarm_=new QLabel(QStringLiteral("无巡检异常"),inspectionBox);
    inspection_alarm_->setWordWrap(true);inspection_alarm_->setFixedHeight(100);inspection_alarm_->setSizePolicy(QSizePolicy::Ignored,QSizePolicy::Fixed);
    il->addWidget(inspection_alarm_,1,0,1,2);
    auto* rectangle=new QPushButton(QStringLiteral("框选矩形"),inspectionBox);
    auto* polygon=new QPushButton(QStringLiteral("绘制多边形"),inspectionBox);
    auto* closePolygon=new QPushButton(QStringLiteral("完成多边形"),inspectionBox);
    il->addWidget(rectangle,2,0);il->addWidget(polygon,2,1);il->addWidget(closePolygon,3,0,1,2);
    inspection_edit_buttons_={rectangle,polygon,closePolygon};
    for(auto pair:std::vector<std::pair<QPushButton*,int>>{{rectangle,1},{polygon,2}})
      connect(pair.first,&QPushButton::clicked,this,[this,pair]{
        picking_goal_=false;picking_initial_pose_=false;showTopDown();inspection_picking_=pair.second;
        inspection_polygon_.clear();inspection_dirty_=true;publishInspectionDraft();
        log(pair.second==1?QStringLiteral("拖动鼠标框选矩形巡检区"):QStringLiteral("左键逐点绘制多边形，再点击完成多边形"));
      });
    connect(closePolygon,&QPushButton::clicked,this,[this]{inspection_picking_=0;publishInspectionDraft();});
    inspection_auto_spacing_=new QCheckBox(QStringLiteral("按下视相机自动计算间距"),inspectionBox);
    il->addWidget(inspection_auto_spacing_,4,0,1,2);
    auto spin=[inspectionBox](double low,double high,double value,double step){auto* w=new QDoubleSpinBox(inspectionBox);w->setRange(low,high);w->setDecimals(2);w->setValue(value);w->setSingleStep(step);return w;};
    inspection_spacing_=spin(.1,5.,.5,.1);inspection_overlap_=spin(0.,.8,.2,.05);
    inspection_speed_=spin(.1,1.,.5,.1);inspection_angle_=spin(0.,179.,0.,5.);
    inspection_height_=spin(.5,2.3,1.2,.1);
    inspection_auto_angle_=new QCheckBox(QStringLiteral("自动选择扫描方向"),inspectionBox);inspection_auto_angle_->setChecked(true);
    inspection_entry_=new QComboBox(inspectionBox);inspection_entry_->addItems({QStringLiteral("自动选择可达入口"),QStringLiteral("候选入口1"),QStringLiteral("候选入口2"),QStringLiteral("候选入口3"),QStringLiteral("候选入口4")});
    il->addWidget(new QLabel(QStringLiteral("手动间距 m")),5,0);il->addWidget(inspection_spacing_,5,1);
    il->addWidget(new QLabel(QStringLiteral("自动重叠率")),6,0);il->addWidget(inspection_overlap_,6,1);
    il->addWidget(new QLabel(QStringLiteral("巡检速度 m/s")),7,0);il->addWidget(inspection_speed_,7,1);
    il->addWidget(inspection_auto_angle_,8,0,1,2);
    il->addWidget(new QLabel(QStringLiteral("扫描角度 °")),9,0);il->addWidget(inspection_angle_,9,1);
    il->addWidget(inspection_entry_,10,0,1,2);
    il->addWidget(new QLabel(QStringLiteral("巡检高度 地图Z m")),11,0);il->addWidget(inspection_height_,11,1);
    auto* ihint=new QLabel(QStringLiteral("青色条带、橙色连接为预览；黄色/紫色为真实控制参考/轨迹。加载已知区域后需重新生成路线。"),inspectionBox);
    ihint->setWordWrap(true);ihint->setFixedHeight(70);ihint->setSizePolicy(QSizePolicy::Ignored,QSizePolicy::Fixed);il->addWidget(ihint,12,0,1,2);
    inspection_plan_button_=new QPushButton(QStringLiteral("生成巡检路线"),inspectionBox);
    inspection_start_button_=new QPushButton(QStringLiteral("开始巡检"),inspectionBox);
    inspection_pause_button_=new QPushButton(QStringLiteral("暂停巡检"),inspectionBox);
    inspection_resume_button_=new QPushButton(QStringLiteral("继续巡检"),inspectionBox);
    inspection_cancel_button_=new QPushButton(QStringLiteral("取消巡检"),inspectionBox);
    il->addWidget(inspection_plan_button_,13,0);il->addWidget(inspection_start_button_,13,1);
    il->addWidget(inspection_pause_button_,14,0);il->addWidget(inspection_resume_button_,14,1);il->addWidget(inspection_cancel_button_,15,0,1,2);
    auto dirty=[this]{inspection_dirty_=true;};
    for(auto* w:{inspection_spacing_,inspection_overlap_,inspection_speed_,inspection_angle_,inspection_height_})
      connect(w,QOverload<double>::of(&QDoubleSpinBox::valueChanged),this,dirty);
    connect(inspection_height_,QOverload<double>::of(&QDoubleSpinBox::valueChanged),this,[this]{if(!inspection_polygon_.empty())publishInspectionDraft();});
    connect(inspection_entry_,QOverload<int>::of(&QComboBox::currentIndexChanged),this,dirty);
    connect(inspection_auto_spacing_,&QCheckBox::toggled,this,dirty);connect(inspection_auto_angle_,&QCheckBox::toggled,this,dirty);
    connect(inspection_plan_button_,&QPushButton::clicked,this,[this]{
      inspection_picking_=0;
      if(inspection_polygon_.size()<3){log(QStringLiteral("请先框选矩形或完成多边形"));return;}
      drone_stack::PlanInspection call;call.request.region.header.frame_id="map";call.request.region.header.stamp=ros::Time::now();
      for(auto p:inspection_polygon_){geometry_msgs::Point32 v;v.x=p.first;v.y=p.second;v.z=inspection_height_->value();call.request.region.polygon.points.push_back(v);}
      call.request.altitude=inspection_height_->value();call.request.auto_spacing=inspection_auto_spacing_->isChecked();
      call.request.spacing=inspection_spacing_->value();call.request.overlap=inspection_overlap_->value();call.request.speed=inspection_speed_->value();
      call.request.angle_deg=inspection_auto_angle_->isChecked()?-1.:inspection_angle_->value();call.request.entry=inspection_entry_->currentIndex();
      asyncInspection(inspection_plan_client_,call,[this](bool ok,const drone_stack::PlanInspection& result){
        log(ok?QString::fromStdString(result.response.message):QStringLiteral("巡检规划服务不可用"));
        if(ok&&result.response.success){inspection_plan_id_=result.response.plan_id;inspection_dirty_=false;}
      });
    });
    for(auto pair:std::vector<std::pair<QPushButton*,std::string>>{{inspection_start_button_,"start"},{inspection_pause_button_,"pause"},{inspection_resume_button_,"resume"},{inspection_cancel_button_,"cancel"}})
      connect(pair.first,&QPushButton::clicked,this,[this,pair]{
        drone_stack::InspectionCommand call;call.request.action=pair.second;
        {std::lock_guard<std::mutex> guard(mutex_);call.request.plan_id=inspection_snapshot_["plan_id"].toString().toStdString();}
        asyncInspection(inspection_command_client_,call,[this](bool ok,const drone_stack::InspectionCommand& result){log(ok?QString::fromStdString(result.response.message):QStringLiteral("巡检操作服务不可用"));});
      });
    inspectionLayout->addWidget(inspectionBox);
    inspection_front_image_=cameraBox(inspectionLayout,inspectionSide,QStringLiteral("前视处理画面"));
    inspection_down_image_=cameraBox(inspectionLayout,inspectionSide,QStringLiteral("下视巡检处理画面"));
    inspectionLayout->addStretch();inspectionScroll->setWidget(inspectionSide);
    auto* aiPage=new QWidget(side_pages_);auto* al=new QVBoxLayout(aiPage);
    auto* aiTitle=new QLabel(QStringLiteral("DeepSeek 文本任务控制"),aiPage);aiTitle->setStyleSheet("font-size:20px;font-weight:bold;");al->addWidget(aiTitle);
    auto* aiAuth=new QHBoxLayout();ai_parse_gate_=new QCheckBox(QStringLiteral("允许云端解析（文字/状态/区域名）"),aiPage);
    ai_control_gate_=new QCheckBox(QStringLiteral("授权AI执行动作"),aiPage);aiAuth->addWidget(ai_parse_gate_);aiAuth->addWidget(ai_control_gate_);aiAuth->addStretch();al->addLayout(aiAuth);
    ai_status_label_=new QLabel(QStringLiteral("等待AI服务"),aiPage);ai_status_label_->setWordWrap(true);ai_status_label_->setFixedHeight(80);al->addWidget(ai_status_label_);
    ai_regions_label_=new QLabel(aiPage);ai_regions_label_->setWordWrap(true);ai_regions_label_->setFixedHeight(55);al->addWidget(ai_regions_label_);
    ai_input_=new QPlainTextEdit(aiPage);ai_input_->setPlaceholderText(QStringLiteral("例如：起飞1米，然后巡检设备区，最后降落。\n或：前往地图坐标(3, 2, 1.2)。仅使用已知区域名称，不使用语音。"));ai_input_->setMinimumHeight(110);ai_input_->setMaximumHeight(160);al->addWidget(ai_input_);
    auto* aiButtons=new QHBoxLayout();ai_submit_button_=new QPushButton(QStringLiteral("发送文字（Ctrl+Enter）"),aiPage);
    ai_cancel_button_=new QPushButton(QStringLiteral("停止AI任务"),aiPage);auto* clearAi=new QPushButton(QStringLiteral("清空显示"),aiPage);
    aiButtons->addWidget(ai_submit_button_);aiButtons->addWidget(ai_cancel_button_);aiButtons->addWidget(clearAi);aiButtons->addStretch();al->addLayout(aiButtons);
    al->addWidget(new QLabel(QStringLiteral("AI答复与完整计划（实际完成情况看任务状态）"),aiPage));
    ai_reply_=new QPlainTextEdit(aiPage);ai_reply_->setReadOnly(true);ai_reply_->setMinimumHeight(120);al->addWidget(ai_reply_,1);
    al->addWidget(new QLabel(QStringLiteral("执行与异常记录"),aiPage));ai_event_view_=new QPlainTextEdit(aiPage);ai_event_view_->setReadOnly(true);ai_event_view_->setMaximumBlockCount(200);ai_event_view_->setMinimumHeight(120);al->addWidget(ai_event_view_,1);
    auto* aiNote=new QLabel(QStringLiteral("解析授权与执行授权独立。飞行仍需综合页授权及定位/地图安全门。撤权或停止会中止后续步骤；已有降落继续执行。紧急情况可直接使用下方一键降落。"),aiPage);
    aiNote->setWordWrap(true);aiNote->setFixedHeight(60);al->addWidget(aiNote);
    for(auto pair:std::vector<std::pair<QCheckBox*,std::string>>{{ai_parse_gate_,"parse"},{ai_control_gate_,"control"}})
      connect(pair.first,&QCheckBox::toggled,this,[this,pair](bool enabled){
        drone_stack::AiGate call;call.request.session_id=ai_session_id_;call.request.gate=pair.second;call.request.enabled=enabled;
        ai_call_pending_=true;
        asyncInspection(ai_gate_client_,call,[this](bool ok,const drone_stack::AiGate& result){ai_call_pending_=false;log(ok?QString::fromStdString(result.response.message):QStringLiteral("AI授权服务不可用"));});
      });
    connect(ai_submit_button_,&QPushButton::clicked,this,[this]{
      drone_stack::AiText call;call.request.session_id=ai_session_id_;call.request.text=ai_input_->toPlainText().toStdString();ai_call_pending_=true;
      asyncInspection(ai_text_client_,call,[this](bool ok,const drone_stack::AiText& result){ai_call_pending_=false;log(ok?QString::fromStdString(result.response.message):QStringLiteral("AI文本服务不可用"));});
    });
    auto* submitShortcut=new QShortcut(QKeySequence(Qt::CTRL+Qt::Key_Return),aiPage);
    connect(submitShortcut,&QShortcut::activated,this,[this]{if(ai_submit_button_->isEnabled())ai_submit_button_->click();});
    connect(ai_cancel_button_,&QPushButton::clicked,this,[this]{
      drone_stack::AiCancel call;call.request.session_id=ai_session_id_;ai_call_pending_=true;
      asyncInspection(ai_cancel_client_,call,[this](bool ok,const drone_stack::AiCancel& result){ai_call_pending_=false;log(ok?QString::fromStdString(result.response.message):QStringLiteral("AI停止服务不可用"));});
    });
    connect(clearAi,&QPushButton::clicked,this,[this]{ai_input_->clear();ai_event_view_->clear();});
    sideLayout->addStretch();
    sideScroll->setWidget(side);
    side_pages_->addWidget(sideScroll);side_pages_->addWidget(inspectionScroll);side_pages_->addWidget(aiPage);
    connect(page_tabs_,&QTabBar::currentChanged,this,[this,left,title,horizontal]{
      const bool wasAi=side_pages_->currentIndex()==2;
      if(page_tabs_->currentIndex()==2&&!wasAi)saved_split_sizes_=horizontal->sizes();
      side_pages_->setCurrentIndex(page_tabs_->currentIndex());picking_goal_=false;picking_initial_pose_=false;inspection_picking_=0;
      left->setVisible(page_tabs_->currentIndex()!=2);
      if(wasAi&&page_tabs_->currentIndex()!=2&&!saved_split_sizes_.isEmpty())horizontal->setSizes(saved_split_sizes_);
      title->setText(page_tabs_->currentIndex()==2?QStringLiteral("巡检四旋翼 · AI文本控制"):page_tabs_->currentIndex()==1?QStringLiteral("巡检四旋翼 · 区域巡检"):QStringLiteral("巡检四旋翼 · 综合操作台"));
      pick_button_->setVisible(page_tabs_->currentIndex()==0);
      if(page_tabs_->currentIndex()==1)showTopDown();
    });
    horizontal->addWidget(side_pages_);
    horizontal->setStretchFactor(0, 4);
    horizontal->setStretchFactor(1, 1);
    main->addWidget(horizontal, 1);
    auto* emergency = new QHBoxLayout();
    auto* emergencyHint = new QLabel(QStringLiteral("一键降落：优先管理器，管理器不可用时请求 PX4 AUTO.LAND"), root);
    emergency->addWidget(emergencyHint, 1);
    land->setMinimumWidth(230);
    emergency->addWidget(land);
    main->addLayout(emergency);
    events_ = new QPlainTextEdit(root);
    events_->setReadOnly(true);
    events_->setMaximumBlockCount(200);
    events_->setMaximumHeight(110);
    main->addWidget(events_);
    for(auto* button:root->findChildren<QPushButton*>())button->setFocusPolicy(Qt::NoFocus);
    setCentralWidget(root);

    connect(auth, &QPushButton::clicked, this, [this] {
      std_srvs::SetBool srv;
      srv.request.data = !authorized_;
      if (!auth_client_.call(srv)) log(QStringLiteral("授权服务不可用"));
      else log(QString::fromStdString(srv.response.message));
    });
    connect(arm, &QPushButton::clicked, this, [this, ops] {
      ops->setFocus(Qt::MouseFocusReason);
      trigger(arm_client_, QStringLiteral("解锁"));
    });
    connect(navigation_speed_, &QDoubleSpinBox::editingFinished, this, [this] { configureSpeed(); });
    connect(ceiling_button_, &QPushButton::clicked, this, [this] {
      drone_stack::SetMaxFlightHeight srv; srv.request.height_m = maximum_height_->value();
      if (!ceiling_client_.call(srv)) { log(QStringLiteral("最高高度设置服务不可用")); return; }
      log(QString::fromStdString(srv.response.message));
      maximum_height_->setValue(srv.response.height_m);
      clicked_height_->setMaximum(srv.response.height_m-.20+map_translation_view_.z);
    });
    connect(takeoff, &QPushButton::clicked, this, [this] {
      drone_stack::Takeoff srv;
      srv.request.height_m = takeoff_height_->value();
      if (!takeoff_client_.call(srv)) log(QStringLiteral("起飞服务不可用"));
      else log(QString::fromStdString(srv.response.message));
    });
    connect(hold, &QPushButton::clicked, this, [this] { trigger(hold_client_, QStringLiteral("悬停")); });
    connect(cancel, &QPushButton::clicked, this, [this] { trigger(cancel_client_, QStringLiteral("取消目标")); });
    connect(land, &QPushButton::clicked, this, [this] {
      std_srvs::Trigger request;
      if (land_client_.call(request) && request.response.success) {
        log(QString::fromStdString(request.response.message));
        return;
      }
      mavros_msgs::SetMode fallback;
      fallback.request.custom_mode = "AUTO.LAND";
      const bool sent = direct_land_client_.call(fallback) && fallback.response.mode_sent;
      log(sent ? QStringLiteral("管理器不可用，已向 PX4 请求降落") : QStringLiteral("降落请求失败，请检查飞控连接"));
    });
    connect(disarm, &QPushButton::clicked, this, [this] {
      auto client = nh_.serviceClient<std_srvs::Trigger>("/drone/disarm");
      trigger(client, QStringLiteral("地面上锁"));
    });
    connect(sendRelative, &QPushButton::clicked, this, [this] { sendRelativeGoal(); });
  }

  static QDoubleSpinBox* spin(double value, QWidget* parent) {
    auto* out = new QDoubleSpinBox(parent);
    out->setDecimals(2);
    out->setRange(-1000000.0, 1000000.0);
    out->setValue(value);
    out->setSuffix(QStringLiteral(" m"));
    return out;
  }

  QLabel* cameraBox(QVBoxLayout* layout, QWidget* parent, const QString& name) {
    auto* box = new QGroupBox(name, parent);
    auto* body = new QVBoxLayout(box);
    auto* image = new QLabel(QStringLiteral("等待画面"), box);
    image->setAlignment(Qt::AlignCenter);
    image->setFixedHeight(180);
    // A live pixmap must never participate in layout size negotiation.
    image->setSizePolicy(QSizePolicy::Ignored,QSizePolicy::Fixed);
    image->setStyleSheet("background:#080d13;color:#8fa0b5;border:1px solid #405067;");
    body->addWidget(image);
    layout->addWidget(box);
    return image;
  }

  void addDisplay(const QString& cls, const QString& title, const QString& topic, const QString& style) {
    auto* display = manager_->createDisplay(cls, title, true);
    if (!display) { log(QStringLiteral("RViz 无法加载：") + title); return; }
    displays_[title] = display;
    // rviz/Marker names its topic property differently from PointCloud2 and
    // Odometry.  Accessing "Topic" here creates an RViz undefined-property
    // error and leaves the trajectory unsubscribed.
    const char* topic_property = (cls == "rviz/Marker" || cls == "rviz/MarkerArray") ? "Marker Topic" : "Topic";
    if (auto* property = display->subProp(topic_property)) property->setValue(topic);
    if (!style.isEmpty()) if (auto* property = display->subProp("Style")) property->setValue(style);
    if (cls == "rviz/Odometry") display->subProp("Keep")->setValue(1);
    if (cls == "rviz/PointCloud2") {
      display->subProp("Decay Time")->setValue(0.0);
      // Simulation intensity is constant, so use explicit colours rather
      // than automatically normalising an empty or zero-width intensity range.
      display->subProp("Color Transformer")->setValue("FlatColor");
      display->subProp("Color")->setValue(style == "Boxes" ? QColor(255, 170, 55) : QColor(75, 190, 255));
      if (style == "Boxes") {
        display->subProp("Size (m)")->setValue(0.1);
        display->subProp("Alpha")->setValue(0.4);
      }
    }
  }

  void trigger(ros::ServiceClient& client, const QString& name) {
    std_srvs::Trigger srv;
    if (!client.call(srv)) log(name + QStringLiteral("服务不可用"));
    else log(name + QStringLiteral(": ") + QString::fromStdString(srv.response.message));
  }

  bool submit(const geometry_msgs::PoseStamped& goal) {
    if (!configureSpeed()) return false;
    drone_stack::QueueGoal srv; srv.request.goal = goal;
    if (!goal_client_.call(srv)) { log(QStringLiteral("目标队列服务不可用")); return false; }
    log(QString::fromStdString(srv.response.message));
    navigation_hint_->setText(srv.response.success ?
      QStringLiteral("已加入队列，剩余%1个目标；仅规划当前目标，可继续点选。绿线为实际轨迹。").arg(srv.response.queued_count) :
      QStringLiteral("目标被拒绝，可重新点选；现有队列保持执行。"));
    return srv.response.success;
  }

  bool configureSpeed() {
    drone_stack::SetNavigationSpeed srv;
    srv.request.speed_mps = navigation_speed_->value();
    if (!speed_client_.call(srv)) { log(QStringLiteral("移动速度设置服务不可用")); return false; }
    log(QString::fromStdString(srv.response.message));
    return srv.response.success;
  }

  void sendRelativeGoal() {
    nav_msgs::Odometry odom;
    {
      std::lock_guard<std::mutex> guard(mutex_);
      if (!have_odom_ || (ros::Time::now() - odom_.header.stamp).toSec() > 0.35) {
        log(QStringLiteral("位姿过期，目标未发送")); return;
      }
      odom = odom_;
    }
    const auto& q = odom.pose.pose.orientation;
    const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                  1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    const double f = relative_x_->value(), l = relative_y_->value();
    geometry_msgs::PoseStamped goal;
    goal.header.frame_id = "odom";
    goal.header.stamp = ros::Time::now();
    goal.pose = odom.pose.pose;
    goal.pose.position.x += std::cos(yaw) * f - std::sin(yaw) * l;
    goal.pose.position.y += std::sin(yaw) * f + std::cos(yaw) * l;
    goal.pose.position.z += relative_z_->value();
    submit(goal);
  }

  void onClicked(const geometry_msgs::PointStamped::ConstPtr& point) {
    if (point->header.frame_id != "odom" || !std::isfinite(point->point.x) || !std::isfinite(point->point.y)) return;
    std::lock_guard<std::mutex> guard(mutex_);
    // Target clicks are generated by the XY render-panel event filter.
    // Ignore external /clicked_point publishers to avoid unintended commands.
    (void)point;
  }

  void onState(const mavros_msgs::State::ConstPtr& msg) { std::lock_guard<std::mutex> guard(mutex_); state_ = *msg; state_receipt_ = ros::WallTime::now(); state_ros_receipt_ = ros::Time::now(); }
  void onOdom(const nav_msgs::Odometry::ConstPtr& msg) { std::lock_guard<std::mutex> guard(mutex_); odom_ = *msg; have_odom_ = true; }
  void onBattery(const sensor_msgs::BatteryState::ConstPtr& msg) { std::lock_guard<std::mutex> guard(mutex_); battery_ = *msg; battery_receipt_ = ros::WallTime::now(); battery_ros_receipt_ = ros::Time::now(); }
  void onLio(const std_msgs::Bool::ConstPtr& msg) { std::lock_guard<std::mutex> guard(mutex_); lio_valid_ = msg->data; lio_receipt_ = ros::WallTime::now(); lio_ros_receipt_ = ros::Time::now(); }
  void onPhase(const std_msgs::String::ConstPtr& msg) { std::lock_guard<std::mutex> guard(mutex_); phase_ = msg->data; if (phase_ == "NAVIGATING") ++navigation_epoch_; }
  void onAuth(const std_msgs::Bool::ConstPtr& msg) { std::lock_guard<std::mutex> guard(mutex_); authorized_ = msg->data; }
  void onError(const std_msgs::String::ConstPtr& msg) { std::lock_guard<std::mutex> guard(mutex_); pending_error_ = QString::fromStdString(msg->data); }
  void onFront(const sensor_msgs::Image::ConstPtr& msg) { onImage(msg, true); }
  void onDown(const sensor_msgs::Image::ConstPtr& msg) { onImage(msg, false); }

  void onImage(const sensor_msgs::Image::ConstPtr& msg, bool front) {
    QImage image;
    if (msg->encoding == "rgb8" || msg->encoding == "bgr8") {
      image = QImage(msg->data.data(), msg->width, msg->height, msg->step, QImage::Format_RGB888).copy();
      if (msg->encoding == "bgr8") image = image.rgbSwapped();
    } else if (msg->encoding == "mono8") {
      image = QImage(msg->data.data(), msg->width, msg->height, msg->step, QImage::Format_Grayscale8).copy();
    }
    std::lock_guard<std::mutex> guard(mutex_);
    if (front) { front_ = image; front_stamp_ = msg->header.stamp; front_receipt_ = ros::WallTime::now(); }
    else { down_ = image; down_stamp_ = msg->header.stamp; down_receipt_ = ros::WallTime::now(); }
  }

  void refresh() {
    nav_msgs::Odometry odom;
    QImage front, down;
    ros::Time frontStamp, downStamp, stateRos, lioRos, managerRos, cloudStamp, mapStamp, batteryRos;
    ros::WallTime batteryReceipt;
    ros::WallTime cloudReceipt, mapReceipt;
    ros::WallTime stateReceipt, lioReceipt, managerReceipt, frontReceipt, downReceipt;
    geometry_msgs::PointStamped clicked;
    bool click = false, authorized, lio;
    mavros_msgs::State state;
    sensor_msgs::BatteryState battery;
    std::string phase, lioQuality;
    QString error, mapMode, mapStatus,sessionMode,sessionMessage;
    bool alignmentReady=false,transformChanged=false;geometry_msgs::PoseStamped mapTransform;
    nav_msgs::Path queue;
    bool queueUpdate = false, ceilingUpdate = false;
    double ceiling = 2.5;
    {
      std::lock_guard<std::mutex> guard(mutex_);
      mapMode=map_mode_;mapStatus=map_status_;sessionMode=map_session_mode_;sessionMessage=map_session_message_;alignmentReady=alignment_ready_;mapTransform=map_transform_;transformChanged=map_transform_changed_;map_transform_changed_=false;
      state = state_; battery = battery_; phase = phase_; odom = odom_;
      front = front_; down = down_; frontStamp = front_stamp_; downStamp = down_stamp_;
      authorized = authorized_; lio = lio_valid_;
      lioQuality = lio_quality_;
      batteryRos = battery_ros_receipt_; batteryReceipt = battery_receipt_;
      stateRos = state_ros_receipt_; lioRos = lio_ros_receipt_; managerRos = manager_ros_receipt_;
      cloudStamp = cloud_stamp_; mapStamp = map_stamp_; cloudReceipt = cloud_receipt_; mapReceipt = map_receipt_;
      stateReceipt = state_receipt_; lioReceipt = lio_receipt_; managerReceipt = manager_receipt_;
      frontReceipt = front_receipt_; downReceipt = down_receipt_;
      click = have_click_; clicked = pending_click_; have_click_ = false;
      error = pending_error_; pending_error_.clear();
      queueUpdate = have_queue_update_; queue = queue_message_; have_queue_update_ = false;
      ceilingUpdate = have_ceiling_update_; ceiling = ceiling_value_; have_ceiling_update_ = false;
    }
    if(transformChanged){
      map_translation_view_=mapTransform.pose.position;const auto& q=mapTransform.pose.orientation;
      map_yaw_view_=std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
      overlay_node_->setPosition(Ogre::Vector3(map_translation_view_.x,map_translation_view_.y,map_translation_view_.z));
      overlay_node_->setOrientation(Ogre::Quaternion(Ogre::Radian(map_yaw_view_),Ogre::Vector3::UNIT_Z));
      clicked_height_->setRange(.5+map_translation_view_.z,ceiling_value_-.2+map_translation_view_.z);
      clicked_height_->setToolTip(QStringLiteral("地图坐标Z；控制时转换为当前ENU。允许范围随初始位姿和ENU天花板变化。"));
    }
    const bool showPrior=sessionMode=="ALIGNING" || sessionMode=="WAITING_POSE" || sessionMode=="STARTING_SIM";
    displays_["预建地图预览"]->setEnabled(showPrior);
    const bool showLive=!showPrior;
    // Respect existing visibility toggles; only force-hide while aligning.
    if(showLive!=live_view_enabled_){
      displays_["MID360 点云"]->setEnabled(showLive);displays_["EGO 体素"]->setEnabled(showLive);live_view_enabled_=showLive;
    }
    if (!error.isEmpty()) log(error);
    if (!error.isEmpty()) last_error_ = error;
    const auto now = ros::WallTime::now();
    const auto rosNow = ros::Time::now();
    if (use_sim_time_) {
      if (!last_clock_.isZero() && rosNow < last_clock_) clock_fault_ = true;
      if (rosNow > last_clock_) clock_progress_ = now;
      last_clock_ = rosNow;
    }
    const bool clockHealthy = !use_sim_time_ || (!clock_fault_ && !rosNow.isZero() &&
        !clock_progress_.isZero() && (now-clock_progress_).toSec() < 10.0);
    // Simulation freshness uses ROS time; a separate wall watchdog catches
    // a paused/dead clock. Real hardware keeps the original wall deadlines.
    const auto fresh = [&](const ros::Time& stamp, const ros::WallTime& receipt, double limit) {
      if (receipt.isZero() || !clockHealthy) return false;
      const double wallAge = (now-receipt).toSec();
      if (stamp.isZero()) return false;
      const double age = (rosNow-stamp).toSec();
      return wallAge >= 0 && wallAge < (use_sim_time_ ? 10.0 : limit) && age >= -0.02 && age < limit;
    };
    const bool stateFresh = fresh(stateRos, stateReceipt, 2.0);
    const bool managerFresh = fresh(managerRos, managerReceipt, 0.75);
    lio = lio && fresh(lioRos, lioReceipt, 0.75);
    const bool cloudFresh = fresh(cloudStamp, cloudReceipt, 1.0);
    const bool mapFresh = fresh(mapStamp, mapReceipt, 2.0);
    const double poseAge = (rosNow-odom.header.stamp).toSec();
    const bool poseFresh = !odom.header.stamp.isZero() && clockHealthy && poseAge >= -0.02 && poseAge < 0.35;
    state.connected = state.connected && stateFresh;
    const bool flightReady = state.connected && lio && managerFresh && poseFresh;
    auth_button_->setEnabled(state.connected && managerFresh);
    if (ceilingUpdate) {
      maximum_height_->setValue(ceiling); clicked_height_->setMaximum(ceiling-.20+map_translation_view_.z);
      maximum_height_->setToolTip(QStringLiteral("当前已生效天花板：%1 m ENU Z；目标及规划上限：%2 m。地面或无任务的健康HOLD时可修改。")
        .arg(ceiling,0,'f',2).arg(ceiling-.20,0,'f',2));
    }
    if (queueUpdate && queue.header.frame_id == "odom") {
      const bool wasBusy = goal_busy_;
      queued_points_.clear();
      for (const auto& pose : queue.poses) queued_points_.push_back(pose.pose.position);
      goal_busy_ = !queued_points_.empty(); have_target_marker_ = goal_busy_;
      if (goal_busy_) {
        const auto& next = queue.poses.front();
        const bool changed = !wasBusy || next.header.stamp != goal_start_ ||
          std::hypot(next.pose.position.x-selected_goal_.x,next.pose.position.y-selected_goal_.y) > 1e-5 ||
          std::abs(next.pose.position.z-selected_goal_.z) > 1e-5;
        if (changed) {
          selected_goal_ = next.pose.position; goal_start_ = next.header.stamp;
          planned_path_.clear(); active_goal_pub_.publish(next);
          if (!wasBusy) { actual_path_.clear(); last_trace_stamp_ = ros::Time(0); }
        }
      } else {
        planned_path_.clear(); actual_path_.clear(); last_trace_stamp_ = ros::Time(0);
        navigation_hint_->setText(QStringLiteral("队列已结束；目标与路径已清除，可继续点选。"));
      }
      queue_status_->setText(QStringLiteral("剩余目标：%1（含当前）").arg(queued_points_.size()));
      publishNavigation();
    }
    if (goal_busy_ && (!authorized || !state.armed || !managerFresh ||
        phase == "LANDING" || phase == "DESCENDING" || phase == "FAILSAFE")) {
      goal_busy_ = false; have_target_marker_ = false; queued_points_.clear();
      planned_path_.clear(); actual_path_.clear(); publishNavigation();
      queue_status_->setText(QStringLiteral("剩余目标：0（任务中止）"));
    }
    goal_button_->setEnabled(alignmentReady && flightReady && authorized && state.armed &&
                            (phase == "HOLD" || phase == "NAVIGATING") && queued_points_.size() < 100);
    refreshInspection(alignmentReady,flightReady,authorized,state.armed,phase,sessionMode);
    refreshAi();
    ceiling_button_->setEnabled(state.connected && managerFresh &&
                               (!state.armed || (flightReady && phase == "HOLD" && !goal_busy_)));
    const QString modeTitle=(sessionMode=="PRIOR_NAV" || sessionMode=="WAITING_POSE" || sessionMode=="ALIGNING")
        ?QStringLiteral("巡检模式 · 预建地图")
        :sessionMode=="ONLINE"?QStringLiteral("建图模式 · 在线地图"):QStringLiteral("地图模式初始化中");
    map_mode_label_->setText(modeTitle+QStringLiteral("\n")+(alignmentReady?QStringLiteral("地图坐标就绪"):QStringLiteral("等待初始化，导航锁定")));
    map_mode_label_->setToolTip(sessionMessage+QStringLiteral("\n")+mapStatus);
    const bool bootPending=sessionMode=="WAITING_POSE";
    const bool mapEditable=bootPending || (flightReady && !state.armed && phase=="READY" && lioQuality=="HEALTHY");
    for(auto* button:map_buttons_)button->setEnabled(mapEditable);
    initial_pose_button_->setEnabled(mapEditable && (sessionMode=="ALIGNING" || bootPending));
    initial_confirm_button_->setEnabled(mapEditable && (sessionMode=="ALIGNING" || bootPending));
    initial_confirm_button_->setText(bootPending?QStringLiteral("确认初始位姿并启动仿真"):QStringLiteral("确认初始位姿并加载导航"));
    if(bootPending && map_buttons_.size()>1)map_buttons_[1]->setEnabled(false);
    if(!mapEditable)picking_initial_pose_=false;
    arm_button_->setEnabled(alignmentReady && flightReady && authorized && !state.armed && phase == "READY");
    takeoff_button_->setEnabled(flightReady && authorized && state.armed && phase == "ARMED");
    const bool canHold = flightReady && state.armed &&
        (phase == "TAKEOFF" || phase == "HOLD" || phase == "NAVIGATING");
    hold_button_->setEnabled(canHold); cancel_button_->setEnabled(canHold);
    land_button_->setEnabled(state.connected && state.armed);
    disarm_button_->setEnabled(state.connected && managerFresh && state.armed && phase == "ARMED");
    auth_button_->setText(authorized ? QStringLiteral("撤销授权") : QStringLiteral("操作授权"));
    pick_button_->setEnabled(!inspection_reserved_ui_ && (picking_goal_ || (goal_button_->isEnabled() && !click)));
    pick_button_->setText(picking_goal_ ? QStringLiteral("结束连续点选") : QStringLiteral("点选XY目标"));
    if (click && picking_goal_) {
      if (!goal_button_->isEnabled()) {
        log(QStringLiteral("系统状态不允许导航，点选目标未发送"));
      } else {
      geometry_msgs::PoseStamped goal;
      goal.header.frame_id = "odom";
      goal.header.stamp = ros::Time::now();
      auto mapPoint=clicked.point;mapPoint.z=clicked_height_->value();
      goal.pose.position = odomFromMap(mapPoint);
      goal.pose.orientation = odom.pose.pose.orientation;
      submit(goal);
      }
    }
    bool overlayChanged = false;
    {
      std::lock_guard<std::mutex> guard(mutex_);
      if (have_global_path_) {
        const auto& m = global_path_message_;
        if (m.poses.empty()) { planned_path_.clear(); overlayChanged = true; }
        else if (have_target_marker_ && m.header.stamp >= goal_start_ && m.header.frame_id == "odom" && m.poses.size()>=2) {
          const auto& end = m.poses.back().pose.position;
          if (std::hypot(end.x-selected_goal_.x,end.y-selected_goal_.y)<.01 && std::abs(end.z-selected_goal_.z)<.01) {
            planned_path_.clear();
            for (const auto& pose : m.poses) planned_path_.push_back(pose.pose.position);
            overlayChanged = true;
            global_path_receipt_ = now; global_path_stamp_ = m.header.stamp;
          }
        }
        have_global_path_ = false;
      }
    }
    if (!planned_path_.empty() && !fresh(global_path_stamp_, global_path_receipt_, 2.0)) {
      planned_path_.clear(); overlayChanged = true;
    }
    QString globalStatus, navigationWait;
    { std::lock_guard<std::mutex> guard(mutex_); globalStatus = global_status_; navigationWait = navigation_wait_; }
    if (goal_busy_) navigation_hint_->setText(QStringLiteral("红点编号：执行顺序（剩余%1个） | 黄线：当前目标路线 | 紫线：执行中的EGO曲线 | 绿线：实际轨迹\n").arg(queued_points_.size()) + navigationWait + QStringLiteral("\n") + globalStatus);
    if (goal_busy_ && poseFresh && odom.header.stamp > last_trace_stamp_) {
      const auto& p = odom.pose.pose.position;
      if (actual_path_.empty() || std::hypot(p.x-actual_path_.back().x,p.y-actual_path_.back().y)>.02 || std::abs(p.z-actual_path_.back().z)>.02) {
        actual_path_.push_back(p); overlayChanged = true;
        if (actual_path_.size()>20000) {
          std::vector<geometry_msgs::Point> reduced;
          for (size_t i=0;i<actual_path_.size();i+=2) reduced.push_back(actual_path_[i]);
          reduced.push_back(actual_path_.back()); actual_path_.swap(reduced);
        }
      }
      last_trace_stamp_ = odom.header.stamp;
    }
    {
      std::lock_guard<std::mutex> guard(mutex_);
      if (have_yellow_markers_) {
        std::vector<geometry_msgs::Point> points;
        for(const auto& marker:yellow_markers_.markers)
          if(marker.action==visualization_msgs::Marker::ADD && marker.header.frame_id=="odom")
            points.insert(points.end(),marker.points.begin(),marker.points.end());
        drawRouteLayer(yellow_layer_,points,true,Ogre::ColourValue(1,1,0,1),.055);
        have_yellow_markers_=false;
      }
      if (have_local_path_) {
        std::vector<geometry_msgs::Point> points;
        if(local_path_message_.header.frame_id=="odom" && phase=="NAVIGATING")
          for(const auto& pose:local_path_message_.poses) points.push_back(pose.pose.position);
        drawRouteLayer(purple_layer_,points,false,Ogre::ColourValue(.8,.12,1,1),.065);
        have_local_path_=false;
      }
      if(phase!="NAVIGATING" || !managerFresh ||
         (!local_path_message_.poses.empty() && !fresh(local_path_message_.header.stamp,local_path_receipt_,.75)))
        purple_layer_->clear();
      if(phase!="NAVIGATING" || !managerFresh || !fresh(yellow_stamp_,yellow_receipt_,2.0)) yellow_layer_->clear();
    }
    drawHeadingLayer(odom,poseFresh && !showPrior);
    manager_->queueRender();
    if (overlayChanged) publishNavigation();
    const bool rosOnline = ros::master::check();
    cards_["ROS"]->setText(QStringLiteral("ROS\n") + (rosOnline ? QStringLiteral("在线") : QStringLiteral("离线")));
    cards_["PX4"]->setText(QStringLiteral("PX4\n") + (state.connected ? QStringLiteral("已连接") : QStringLiteral("断连")));
    cards_["授权/ARM"]->setText(QStringLiteral("授权/ARM\n%1 / %2").arg(authorized ? QStringLiteral("是") : QStringLiteral("否"), state.armed ? QStringLiteral("已解锁") : QStringLiteral("锁定")));
    cards_["模式"]->setText(QStringLiteral("模式\n") + QString::fromStdString(state.mode));
    const bool batteryValid = fresh(batteryRos, batteryReceipt, 5.0) &&
        std::isfinite(battery.percentage) && battery.percentage >= 0 && battery.percentage <= 1;
    const double batteryPercent = 100.0*static_cast<double>(battery.percentage);
    // The small tolerance accounts for FLOAT32 representations of 0.2/0.6.
    const CardColor batteryColor = !batteryValid || batteryPercent < 20.0-0.0001 ? CardColor::Red :
        (batteryPercent > 60.0+0.0001 ? CardColor::Green : CardColor::Yellow);
    cards_["电池"]->setText(QStringLiteral("电池\n") + (batteryValid ? QString::number(batteryPercent, 'f', 0) + "%" : QStringLiteral("无有效数据")));
    cards_["电池"]->setToolTip(QStringLiteral("大于60%：绿色；20%至60%（含边界）：黄色；低于20%或数据失效：红色"));
    const bool lioWarning = lioQuality == "WARNING" || lioQuality == "HOLD";
    const bool lioSevere = lioQuality == "SEVERE";
    cards_["LIO"]->setText(QStringLiteral("LIO\n") +
        (lioSevere ? QStringLiteral("严重异常") : !lio ? QStringLiteral("失效") :
         lioQuality == "HOLD" ? QStringLiteral("健康异常 / HOLD") :
         lioWarning ? QStringLiteral("健康异常警告") : QStringLiteral("有效")));
    cards_["雷达/EGO"]->setText(QStringLiteral("雷达/EGO\n%1 / %2").arg(cloudFresh ? QStringLiteral("点云在线") : QStringLiteral("点云断流"), mapFresh ? QStringLiteral("体素在线") : QStringLiteral("体素断流")));
    const auto& q = odom.pose.pose.orientation;
    const double yaw = std::atan2(2.0*(q.w*q.z+q.x*q.y), 1.0-2.0*(q.y*q.y+q.z*q.z))*180.0/3.141592653589793;
    const auto mapPosition=mapFromOdom(odom.pose.pose.position);
    telemetry_->setText(QStringLiteral("飞行阶段：%1  |  地图 X %2  Y %3  Z %4 m  |  地图航向 %5°  |  %6")
        .arg(managerFresh ? QString::fromStdString(phase) : QStringLiteral("管理器离线"))
        .arg(poseFresh ? QString::number(mapPosition.x, 'f', 2) : "--")
        .arg(poseFresh ? QString::number(mapPosition.y, 'f', 2) : "--")
        .arg(poseFresh ? QString::number(mapPosition.z, 'f', 2) : "--")
        .arg(poseFresh ? QString::number(std::remainder(yaw+map_yaw_view_*180./std::acos(-1.),360.), 'f', 1) : "--")
        .arg(use_sim_time_ ? QStringLiteral("仿真") : QStringLiteral("实机连接")));
    QString reason;
    if (bootPending) reason = QStringLiteral("请在右侧地图栏设置出生位置和朝向，点击确认后才启动Gazebo");
    else if (sessionMode=="STARTING_SIM") reason = QStringLiteral("正在按所设位姿启动仿真，等待定位和地图就绪");
    else if (!clockHealthy) reason = clock_fault_ ? QStringLiteral("仿真时钟倒退，请重启界面并检查仿真") : QStringLiteral("仿真时钟未开始或已冻结");
    else if (!state.connected) reason = QStringLiteral("等待 PX4 连接");
    else if (!managerFresh) reason = QStringLiteral("飞行管理器离线；普通操作禁用");
    else if (!lio) reason = QStringLiteral("等待有效 LIO 定位");
    else if (!poseFresh) reason = QStringLiteral("等待飞控有效位姿");
    else if (!authorized) reason = QStringLiteral("请先操作授权，再解锁与起飞");
    else if (!state.armed) reason = QStringLiteral("已授权；READY 时可解锁，解锁后显式起飞");
    else if (phase == "ARMED") reason = QStringLiteral("已解锁；设置相对高度后起飞，或地面上锁");
    else if (phase == "HOLD" || phase == "NAVIGATING") reason = QStringLiteral("可发送局部目标、悬停或一键降落");
    else reason = QStringLiteral("当前阶段：") + QString::fromStdString(phase);
    readiness_->setText(reason);
    readiness_->setToolTip(reason);
    for (auto* button : {auth_button_, arm_button_, takeoff_button_, hold_button_, cancel_button_, goal_button_, pick_button_, disarm_button_})
      button->setToolTip(button->isEnabled() ? QString() : reason);
    land_button_->setToolTip(QStringLiteral("已连接且已解锁时可请求降落；管理器不可用时直接请求 PX4 AUTO.LAND"));
    const bool frontFresh = fresh(frontStamp, frontReceipt, 1.0);
    const bool downFresh = fresh(downStamp, downReceipt, 1.0);
    cards_["双相机"]->setText(QStringLiteral("双相机\n%1 / %2").arg(frontFresh ? QStringLiteral("前在线") : QStringLiteral("前断流"), downFresh ? QStringLiteral("下在线") : QStringLiteral("下断流")));
    QStringList alarms;
    if (!rosOnline) alarms << QStringLiteral("ROS离线");
    if (!clockHealthy) alarms << QStringLiteral("仿真时钟异常");
    if (!state.connected) alarms << QStringLiteral("PX4断连");
    if (!managerFresh) alarms << QStringLiteral("飞行管理器离线");
    if (!lio) alarms << QStringLiteral("LIO无效");
    if (!poseFresh) alarms << QStringLiteral("飞控位姿过期");
    if (!cloudFresh || !mapFresh) alarms << QStringLiteral("点云或体素断流");
    if (!frontFresh || !downFresh) alarms << QStringLiteral("相机断流");
    if (!batteryValid) alarms << QStringLiteral("电池数据失效");
    else if (batteryColor == CardColor::Red) alarms << QStringLiteral("电量低于20%");
    if (phase == "FAILSAFE") alarms << QStringLiteral("飞行保护状态");
    const bool normalLanding = last_error_ == "Operator requested landing" || last_error_ == "Operator revoked authorization";
    if (!last_error_.isEmpty() && !normalLanding && phase != "READY") alarms << last_error_;
    const bool alarmActive = !alarms.isEmpty();
    cards_["告警"]->setText(QStringLiteral("告警\n") + (alarmActive ? QStringLiteral("异常，见提示") : QStringLiteral("无")));
    QString alarmTip = alarms.join(QStringLiteral("\n"));
    if (!last_error_.isEmpty()) alarmTip += QStringLiteral("\n最近管理器消息（历史）：") + last_error_;
    cards_["告警"]->setToolTip(alarmTip);
    setCardColor("ROS", rosOnline ? CardColor::Green : CardColor::Red);
    setCardColor("PX4", state.connected ? CardColor::Green : CardColor::Red);
    // Ground locked/unauthorized is a normal operating state.
    const bool authHealthy = state.connected && managerFresh &&
        (!state.armed || authorized || phase == "LANDING" || phase == "DESCENDING");
    setCardColor("授权/ARM", authHealthy ? CardColor::Green : CardColor::Red);
    const bool modeHealthy = state.connected && managerFresh && !state.mode.empty() &&
        phase != "FAILSAFE" && (!state.armed || state.mode == "OFFBOARD" || state.mode == "AUTO.LAND");
    setCardColor("模式", modeHealthy ? CardColor::Green : CardColor::Red);
    setCardColor("电池", batteryColor);
    setCardColor("LIO", !lio || lioSevere ? CardColor::Red :
        lioWarning ? CardColor::Yellow : CardColor::Green);
    setCardColor("雷达/EGO", cloudFresh && mapFresh ? CardColor::Green : CardColor::Red);
    setCardColor("双相机", frontFresh && downFresh ? CardColor::Green : CardColor::Red);
    setCardColor("告警", alarmActive ? CardColor::Red : CardColor::Green);
    for(auto* label:{front_image_,inspection_front_image_})if(label->isVisible()){
      if(frontFresh&&!front.isNull())label->setPixmap(QPixmap::fromImage(front).scaled(label->contentsRect().size(),Qt::KeepAspectRatio,Qt::SmoothTransformation));
      else label->setText(QStringLiteral("前视画面等待中"));
    }
    for(auto* label:{down_image_,inspection_down_image_})if(label->isVisible()){
      if(downFresh&&!down.isNull())label->setPixmap(QPixmap::fromImage(down).scaled(label->contentsRect().size(),Qt::KeepAspectRatio,Qt::SmoothTransformation));
      else label->setText(QStringLiteral("下视画面等待中"));
    }
  }

  template<class Service,class Done>
  void asyncInspection(ros::ServiceClient client,Service service,Done done){
    inspection_call_pending_=true;
    auto* watcher=new QFutureWatcher<std::pair<bool,Service>>(this);
    connect(watcher,&QFutureWatcher<std::pair<bool,Service>>::finished,this,[this,watcher,done]{
      const auto result=watcher->result();inspection_call_pending_=false;done(result.first,result.second);watcher->deleteLater();
    });
    watcher->setFuture(QtConcurrent::run([client,service]() mutable {const bool ok=client.call(service);return std::make_pair(ok,service);}));
  }

  void refreshAi(){
    const auto wall=ros::WallTime::now();
    if(ai_last_heartbeat_.isZero()||(wall-ai_last_heartbeat_).toSec()>=.5){
      QJsonObject pulse;pulse["session_id"]=QString::fromStdString(ai_session_id_);pulse["takeoff_height_m"]=takeoff_height_->value();
      std_msgs::String m;m.data=QJsonDocument(pulse).toJson(QJsonDocument::Compact).toStdString();ai_heartbeat_pub_.publish(m);ai_last_heartbeat_=wall;
    }
    QJsonObject data;ros::WallTime receipt;QStringList events;
    {std::lock_guard<std::mutex> guard(mutex_);data=ai_snapshot_;receipt=ai_receipt_;events.swap(ai_pending_events_);}
    for(const auto& event:events)ai_event_view_->appendPlainText(QDateTime::currentDateTime().toString("HH:mm:ss ")+event);
    const bool fresh=!receipt.isZero()&&(wall-receipt).toSec()<2.;
    const bool mine=data["session_id"].toString()==QString::fromStdString(ai_session_id_);
    const bool connected=data["client_sessions"].toArray().contains(QString::fromStdString(ai_session_id_));
    const bool parse=mine&&data["parse_authorized"].toBool(),control=mine&&data["control_authorized"].toBool();
    {QSignalBlocker a(ai_parse_gate_),b(ai_control_gate_);ai_parse_gate_->setChecked(parse);ai_control_gate_->setChecked(control);}
    ai_parse_gate_->setEnabled(fresh&&connected&&data["configured"].toBool()&&!ai_call_pending_&&
      (mine||(!data["parse_authorized"].toBool()&&!data["control_authorized"].toBool())));
    ai_control_gate_->setEnabled(fresh&&parse&&!ai_call_pending_);
    ai_submit_button_->setEnabled(fresh&&parse&&!data["busy"].toBool()&&!ai_call_pending_);
    ai_cancel_button_->setEnabled(fresh&&mine&&!ai_call_pending_&&(data["busy"].toBool()||data["control_authorized"].toBool()));
    const QString state=data["state"].toString(),error=data["error"].toString();
    const std::map<QString,QString> labels={{"IDLE",QStringLiteral("待输入")},{"PARSING",QStringLiteral("云端解析中")},{"EXECUTING",QStringLiteral("等待实际执行结果")},
      {"PARSED",QStringLiteral("仅解析，未执行")},{"NEEDS_INPUT",QStringLiteral("需补充指令信息")},{"SUCCEEDED",QStringLiteral("任务完成")},
      {"COMPLETED_PARTIAL",QStringLiteral("部分完成")},{"CANCELED",QStringLiteral("已停止")},{"FAILED",QStringLiteral("任务失败")}};
    const auto steps=data["steps"].toArray();
    ai_status_label_->setText(fresh?QStringLiteral("%1 · 密钥%2\n云端解析：%3 / AI执行：%4\n%5 · 步骤 %6/%7\n%8")
      .arg(data["model"].toString(),data["configured"].toBool()?QStringLiteral("已配置"):QStringLiteral("未配置"),parse?QStringLiteral("开启"):QStringLiteral("关闭"),control?QStringLiteral("开启"):QStringLiteral("关闭"),
           labels.count(state)?labels.at(state):state).arg(std::max(0,data["step"].toInt()+1)).arg(steps.size()).arg(error):QStringLiteral("AI服务离线；授权与发送已禁用"));
    ai_status_label_->setToolTip(error);
    ai_status_label_->setStyleSheet(QStringLiteral("QLabel{background:%1;padding:5px;border-radius:4px;color:#152020;}")
      .arg(!fresh||!error.isEmpty()||state=="FAILED"?"#ef7777":state=="SUCCEEDED"||state=="IDLE"?"#77cb8a":"#efcf68"));
    QStringList names;for(const auto& item:known_records_)names<<item.toObject()["name"].toString();
    ai_regions_label_->setText(QStringLiteral("当前地图已知区域：")+(names.isEmpty()?QStringLiteral("无，请到区域巡检页保存区域"):names.join(QStringLiteral("、"))));
    QString reply=data["answer"].toString()+QStringLiteral("\n\n完整计划：\n");
    for(int i=0;i<steps.size();++i){const auto step=steps[i].toObject();reply+=QStringLiteral("%1. %2 [%3] %4\n").arg(i+1).arg(step["description"].toString(),step["tool"].toString(),QString::fromUtf8(QJsonDocument(step["arguments"].toObject()).toJson(QJsonDocument::Compact)));}
    if(reply!=ai_reply_->toPlainText())ai_reply_->setPlainText(reply);
  }

  void requestKnownRegions(const std::string& action){
    if(known_map_digest_.isEmpty()){log(QStringLiteral("预建地图标识尚未就绪"));return;}
    QJsonObject data;data["map_digest"]=known_map_digest_;
    const QString selected=known_regions_->currentData().toString();
    if(action=="delete"||action=="update")data["id"]=selected;
    if(action=="save"||action=="update"){
      if(inspection_polygon_.size()<3||known_name_->text().trimmed().isEmpty()){log(QStringLiteral("请先选区并输入区域名称"));return;}
      QJsonArray polygon;for(auto point:inspection_polygon_)polygon.append(QJsonArray{point.first,point.second});
      data["name"]=known_name_->text().trimmed();data["polygon"]=polygon;data["altitude"]=inspection_height_->value();
      data["spacing"]=inspection_spacing_->value();data["overlap"]=inspection_overlap_->value();data["speed"]=inspection_speed_->value();
      data["auto_spacing"]=inspection_auto_spacing_->isChecked();data["angle_deg"]=inspection_auto_angle_->isChecked()?-1.:inspection_angle_->value();data["entry"]=inspection_entry_->currentIndex();
    }
    drone_stack::InspectionRegions service;service.request.action=action;
    service.request.payload=QJsonDocument(data).toJson(QJsonDocument::Compact).toStdString();
    const QString expected=known_map_digest_,name=data["name"].toString();
    asyncInspection(inspection_regions_client_,service,[this,expected,selected,name,action](bool ok,const drone_stack::InspectionRegions& result){
      if(expected!=known_map_digest_)return;
      if(!ok||!result.response.success){log(ok?QString::fromStdString(result.response.message):QStringLiteral("已知区域服务不可用"));return;}
      const auto payload=QJsonDocument::fromJson(QByteArray::fromStdString(result.response.payload)).object();
      if(payload["map_digest"].toString()!=known_map_digest_)return;
      known_records_=payload["regions"].toArray();known_regions_->clear();known_regions_->addItem(QStringLiteral("请选择已知区域（%1个）").arg(known_records_.size()));
      int choose=0;
      for(const auto& item:known_records_){const auto r=item.toObject();const QString id=r["id"].toString();
        known_regions_->addItem(r["name"].toString(),id);const int index=known_regions_->count()-1;
        known_regions_->setItemData(index,QStringLiteral("地图Z %1m / 间距 %2m / 速度 %3m/s").arg(r["altitude"].toDouble()).arg(r["spacing"].toDouble()).arg(r["speed"].toDouble()),Qt::ToolTipRole);
        if((action=="save"&&r["name"].toString()==name)||(action!="save"&&action!="delete"&&id==selected))choose=index;
      }
      known_regions_->setCurrentIndex(choose);log(QString::fromStdString(result.response.message));
    });
  }

  void loadKnownRegion(){
    const QString id=known_regions_->currentData().toString();
    for(const auto& item:known_records_){const auto r=item.toObject();if(r["id"].toString()!=id)continue;
      const double height=r["altitude"].toDouble();
      if(height<inspection_height_->minimum()||height>inspection_height_->maximum()){log(QStringLiteral("已知区域高度超出当前天花板/导航范围，请调整后再加载"));return;}
      inspection_polygon_.clear();for(const auto& point:r["polygon"].toArray()){const auto a=point.toArray();inspection_polygon_.push_back({a[0].toDouble(),a[1].toDouble()});}
      inspection_height_->setValue(height);inspection_spacing_->setValue(r["spacing"].toDouble());inspection_overlap_->setValue(r["overlap"].toDouble());
      inspection_speed_->setValue(r["speed"].toDouble());inspection_auto_spacing_->setChecked(r["auto_spacing"].toBool());
      const double angle=r["angle_deg"].toDouble();inspection_auto_angle_->setChecked(angle<0);inspection_angle_->setValue(std::max(0.,angle));
      inspection_entry_->setCurrentIndex(r["entry"].toInt());known_name_->setText(r["name"].toString());
      inspection_picking_=0;picking_goal_=false;inspection_dirty_=true;inspection_plan_id_.clear();showTopDown();publishInspectionDraft();
      log(QStringLiteral("已加载区域“%1”，请重新生成巡检路线；未开始飞行。").arg(r["name"].toString()));return;
    }
    log(QStringLiteral("请选择已知区域"));
  }

  void publishInspectionDraft(){
    visualization_msgs::MarkerArray array;visualization_msgs::Marker m;
    m.header.frame_id="map";m.header.stamp=ros::Time::now();m.ns="inspection_draft";m.id=0;
    m.type=visualization_msgs::Marker::LINE_STRIP;m.action=inspection_polygon_.empty()?visualization_msgs::Marker::DELETE:visualization_msgs::Marker::ADD;
    m.pose.orientation.w=1.;m.scale.x=.04;m.color.g=1.;m.color.b=1.;m.color.a=1.;
    for(auto p:inspection_polygon_){geometry_msgs::Point v;v.x=p.first;v.y=p.second;v.z=inspection_height_->value();m.points.push_back(v);}
    if(!inspection_picking_ && m.points.size()>=3)m.points.push_back(m.points.front());
    array.markers.push_back(m);inspection_draft_pub_.publish(array);
  }

  void refreshInspection(bool aligned,bool healthy,bool authorized,bool armed,const std::string& phase,const QString& mapMode){
    QJsonObject snapshot;ros::WallTime receipt;
    {std::lock_guard<std::mutex> guard(mutex_);snapshot=inspection_snapshot_;receipt=inspection_receipt_;}
    const bool fresh=!receipt.isZero()&&(ros::WallTime::now()-receipt).toSec()<2.;
    const QString state=snapshot["state"].toString();
    const bool active=state=="TRANSITING"||state=="SCANNING"||state=="TURNING"||state=="REPLANNING";
    const bool reserved=active||state=="PAUSED";
    inspection_reserved_ui_=reserved;
    const bool editable=fresh&&aligned&&mapMode=="PRIOR_NAV"&&!reserved&&state!="PLANNING"&&!inspection_call_pending_;
    if(!editable)inspection_picking_=0;
    for(auto* button:inspection_edit_buttons_)button->setEnabled(editable);
    inspection_plan_button_->setEnabled(editable&&inspection_polygon_.size()>=3);
    inspection_spacing_->setEnabled(editable&&!inspection_auto_spacing_->isChecked());
    inspection_overlap_->setEnabled(editable&&inspection_auto_spacing_->isChecked());
    inspection_speed_->setEnabled(editable);inspection_angle_->setEnabled(editable&&!inspection_auto_angle_->isChecked());
    inspection_height_->setRange(clicked_height_->minimum(),clicked_height_->maximum());inspection_height_->setEnabled(editable);
    inspection_auto_angle_->setEnabled(editable);inspection_auto_spacing_->setEnabled(editable);inspection_entry_->setEnabled(editable);
    const bool flying=fresh&&aligned&&healthy&&authorized&&armed&&phase=="HOLD"&&mapMode=="PRIOR_NAV";
    inspection_start_button_->setEnabled(flying&&state=="READY"&&!inspection_dirty_&&!inspection_call_pending_);
    inspection_resume_button_->setEnabled(flying&&state=="PAUSED"&&!inspection_call_pending_);
    inspection_pause_button_->setEnabled(fresh&&active&&!inspection_call_pending_);
    inspection_cancel_button_->setEnabled(fresh&&state!="IDLE"&&state!="CANCELED"&&!inspection_call_pending_);
    const QString digest=snapshot["map_digest"].toString();
    if(digest!=known_map_digest_){
      if(!known_map_digest_.isEmpty()){
        inspection_polygon_.clear();inspection_plan_id_.clear();inspection_dirty_=true;inspection_picking_=0;known_name_->clear();publishInspectionDraft();
      }
      known_map_digest_=digest;known_records_=QJsonArray();known_regions_->clear();known_regions_->addItem(QStringLiteral("请选择已知区域"));known_loaded_digest_.clear();
    }
    const bool library=editable&&!digest.isEmpty();
    known_regions_->setEnabled(library);known_name_->setEnabled(library);
    known_refresh_->setEnabled(library);known_load_->setEnabled(library&&!known_regions_->currentData().toString().isEmpty());
    known_save_->setEnabled(library&&inspection_polygon_.size()>=3);
    known_update_->setEnabled(library&&inspection_polygon_.size()>=3&&!known_regions_->currentData().toString().isEmpty());
    known_delete_->setEnabled(library&&!known_regions_->currentData().toString().isEmpty());
    if(library&&snapshot["available"].toBool()&&known_loaded_digest_!=digest){known_loaded_digest_=digest;requestKnownRegions("list");}
    if(reserved){goal_button_->setEnabled(false);pick_button_->setEnabled(false);picking_goal_=false;}
    const std::map<QString,QString> names={{"IDLE",QStringLiteral("待选区")},{"PLANNING",QStringLiteral("生成路线中")},{"READY",QStringLiteral("预览就绪")},
      {"TRANSITING",QStringLiteral("入口/转场导航")},{"SCANNING",QStringLiteral("扫描中")},{"TURNING",QStringLiteral("换行/转向")},
      {"REPLANNING",QStringLiteral("重新连接路线")},{"PAUSED",QStringLiteral("已暂停")},{"COMPLETED",QStringLiteral("完成")},
      {"COMPLETED_PARTIAL",QStringLiteral("部分完成")},{"FAILED",QStringLiteral("失败")},{"CANCELED",QStringLiteral("已取消")}};
    const auto metrics=snapshot["metrics"].toObject();
    inspection_status_->setText(fresh?QStringLiteral("%1 · %2条扫描线\n间距 %3m / 方向 %4° / 入口%5\n覆盖 %6%（%7/%8 m²）%9")
      .arg(names.count(state)?names.at(state):state).arg(snapshot["lines"].toInt()).arg(snapshot["spacing"].toDouble(),0,'f',2)
      .arg(snapshot["selected_angle"].toDouble(),0,'f',0).arg(snapshot["selected_entry"].toInt())
      .arg(metrics["ratio"].toDouble()*100.,0,'f',1).arg(metrics["covered_m2"].toDouble(),0,'f',2).arg(metrics["eligible_m2"].toDouble(),0,'f',2)
      .arg(inspection_dirty_&&state=="READY"?QStringLiteral("\n参数已改，请重新生成"):QString()):QStringLiteral("巡检管理器离线或数据过期"));
    inspection_status_->setToolTip(QStringLiteral("记录目录：")+snapshot["record_dir"].toString());
    QStringList alerts,eventKeys,diagnostics;bool red=false;
    for(const auto& item:snapshot["issues"].toArray()){
      const auto issue=item.toObject();red|=issue["level"].toString()=="ERROR";
      eventKeys<<issue["code"].toString()+issue["level"].toString()+issue["reason"].toString()+issue["action"].toString();
      if(!issue["detail"].toString().isEmpty())diagnostics<<issue["code"].toString()+": "+issue["detail"].toString();
      alerts<<QStringLiteral("[%1 · %2s · %3次] %4\n%5").arg(issue["code"].toString())
        .arg(issue["first_time"].toDouble(),0,'f',1).arg(issue["count"].toInt())
        .arg(issue["reason"].toString(),issue["action"].toString());
    }
    if(!fresh)alerts.prepend(QStringLiteral("巡检管理器离线；禁止开始或恢复，请查看节点日志"));
    else if(!snapshot["available"].toBool())alerts.prepend(QStringLiteral("巡检需要已加载并对齐的预建地图"));
    const QString alarm=alerts.join(QStringLiteral("\n"));
    const QString eventKey=alerts.isEmpty()?QString():eventKeys.join("|")+QString::number(fresh)+QString::number(snapshot["available"].toBool());
    inspection_alarm_->setText(alarm.isEmpty()?QStringLiteral("无巡检异常"):alarm);
    inspection_alarm_->setToolTip(alarm+QStringLiteral("\n")+diagnostics.join(QStringLiteral("\n")));
    inspection_alarm_->setStyleSheet(QStringLiteral("QLabel{background:%1;color:#152020;border-radius:4px;padding:4px;}")
      .arg(red?"#ef7777":alerts.isEmpty()?"#77cb8a":"#efcf68"));
    if(eventKey!=inspection_last_alarm_){
      if(!alarm.isEmpty())log(QStringLiteral("巡检：")+alarm);
      else if(!inspection_last_alarm_.isEmpty())log(QStringLiteral("巡检异常已恢复；历史记录保留"));
      inspection_last_alarm_=eventKey;
    }
  }

  void log(const QString& text) { if (events_) events_->appendPlainText(QDateTime::currentDateTime().toString("HH:mm:ss ") + text); }

  ros::NodeHandle nh_;
  ros::Publisher navigation_pub_, active_goal_pub_;
  ros::Subscriber global_marker_sub_, local_path_sub_;
  ros::Subscriber global_path_sub_, global_status_sub_, lio_quality_sub_, navigation_wait_sub_;
  std::string lio_quality_ = "UNKNOWN";
  ros::Subscriber cloud_sub_, map_sub_, heartbeat_sub_, state_sub_, odom_sub_, battery_sub_, lio_sub_, phase_sub_, auth_sub_, error_sub_, front_sub_, down_sub_, clicked_sub_;
  ros::ServiceClient auth_client_, arm_client_, takeoff_client_, hold_client_, cancel_client_, land_client_, goal_client_;
  ros::ServiceClient map_session_client_;
  ros::Subscriber map_session_sub_,map_transform_sub_;
  QString map_session_mode_,map_session_message_;
  geometry_msgs::PoseStamped map_transform_;
  geometry_msgs::Point map_translation_view_;
  double map_yaw_view_=0.;
  bool alignment_ready_=false,map_transform_changed_=false,picking_initial_pose_=false,initial_drag_active_=false,live_view_enabled_=true;
  Ogre::Vector3 initial_drag_start_;
  Ogre::SceneNode* overlay_node_=nullptr;
  QPushButton* initial_pose_button_=nullptr;
  QPushButton* initial_confirm_button_=nullptr;
  ros::Subscriber map_mode_sub_,map_status_sub_;
  QString map_mode_,map_status_;
  QLabel* map_mode_label_=nullptr;
  QLineEdit* map_file_=nullptr;
  QDoubleSpinBox *map_dx_=nullptr,*map_dy_=nullptr,*map_dz_=nullptr,*map_yaw_=nullptr;
  std::vector<QPushButton*> map_buttons_;
  ros::ServiceClient direct_land_client_;
  ros::ServiceClient speed_client_, ceiling_client_;
  ros::Subscriber queue_sub_, ceiling_sub_;
  nav_msgs::Path queue_message_;
  bool have_queue_update_ = false, have_ceiling_update_ = false;
  double ceiling_value_ = 2.5;
  std::vector<geometry_msgs::Point> queued_points_;
  size_t rendered_queue_count_ = 0;
  QLabel* queue_status_ = nullptr;
  QPushButton* ceiling_button_ = nullptr;
  QDoubleSpinBox* maximum_height_ = nullptr;
  std::mutex mutex_;
  mavros_msgs::State state_;
  nav_msgs::Odometry odom_;
  sensor_msgs::BatteryState battery_;
  geometry_msgs::PointStamped pending_click_;
  bool have_odom_ = false, have_click_ = false, authorized_ = false, lio_valid_ = false;
  std::string phase_;
  QString pending_error_;
  QString last_error_;
  ros::ServiceClient inspection_plan_client_,inspection_command_client_;
  ros::ServiceClient inspection_regions_client_;
  ros::ServiceClient ai_text_client_,ai_gate_client_,ai_cancel_client_;
  ros::Subscriber ai_status_sub_,ai_events_sub_;ros::Publisher ai_heartbeat_pub_;
  std::string ai_session_id_;QJsonObject ai_snapshot_;QStringList ai_pending_events_;
  ros::WallTime ai_receipt_,ai_last_heartbeat_;bool ai_call_pending_=false;
  QCheckBox *ai_parse_gate_=nullptr,*ai_control_gate_=nullptr;
  QPushButton *ai_submit_button_=nullptr,*ai_cancel_button_=nullptr;
  QLabel *ai_status_label_=nullptr,*ai_regions_label_=nullptr;
  QPlainTextEdit *ai_input_=nullptr,*ai_reply_=nullptr,*ai_event_view_=nullptr;
  QTabBar* page_tabs_=nullptr;QStackedWidget* side_pages_=nullptr;
  QList<int> saved_split_sizes_;
  QLabel *inspection_front_image_=nullptr,*inspection_down_image_=nullptr;
  QDoubleSpinBox* inspection_height_=nullptr;
  QComboBox* known_regions_=nullptr;QLineEdit* known_name_=nullptr;
  QPushButton *known_load_=nullptr,*known_refresh_=nullptr,*known_save_=nullptr,*known_update_=nullptr,*known_delete_=nullptr;
  QJsonArray known_records_;QString known_map_digest_,known_loaded_digest_;
  ros::Subscriber inspection_sub_;
  ros::Publisher inspection_draft_pub_;
  QJsonObject inspection_snapshot_;ros::WallTime inspection_receipt_;
  QLabel *inspection_status_=nullptr,*inspection_alarm_=nullptr;
  QDoubleSpinBox *inspection_spacing_=nullptr,*inspection_overlap_=nullptr,*inspection_speed_=nullptr,*inspection_angle_=nullptr;
  QCheckBox *inspection_auto_spacing_=nullptr,*inspection_auto_angle_=nullptr;
  QComboBox* inspection_entry_=nullptr;
  QPushButton *inspection_plan_button_=nullptr,*inspection_start_button_=nullptr,*inspection_pause_button_=nullptr,
      *inspection_resume_button_=nullptr,*inspection_cancel_button_=nullptr;
  std::vector<QPushButton*> inspection_edit_buttons_;
  std::vector<std::pair<double,double>> inspection_polygon_;
  Ogre::Vector3 inspection_drag_start_;
  int inspection_picking_=0;bool inspection_dirty_=true,inspection_call_pending_=false,inspection_reserved_ui_=false;
  std::string inspection_plan_id_;QString inspection_last_alarm_;
  QImage front_, down_;
  bool two_d_view_ = false, goal_busy_ = false, navigation_seen_ = false, have_target_marker_ = false, have_global_path_ = false;
  ros::Time goal_start_, last_trace_stamp_;
  size_t navigation_epoch_ = 0, goal_navigation_epoch_ = 0;
  geometry_msgs::Point selected_goal_;
  std::vector<geometry_msgs::Point> planned_path_, actual_path_;
  nav_msgs::Path global_path_message_, local_path_message_;
  visualization_msgs::MarkerArray yellow_markers_;
  bool have_yellow_markers_ = false, have_local_path_ = false;
  ros::WallTime local_path_receipt_, yellow_receipt_;
  ros::Time yellow_stamp_;
  Ogre::ManualObject *yellow_layer_ = nullptr, *purple_layer_ = nullptr, *heading_layer_ = nullptr;
  QString global_status_, navigation_wait_;
  ros::Time global_path_stamp_;
  ros::WallTime global_path_receipt_;
  QLabel* navigation_hint_ = nullptr;
  bool use_sim_time_ = false, clock_fault_ = false, picking_goal_ = false;
  ros::Time battery_ros_receipt_;
  ros::WallTime battery_receipt_;
  ros::Time last_clock_, state_ros_receipt_, lio_ros_receipt_, manager_ros_receipt_, cloud_stamp_, map_stamp_;
  ros::WallTime clock_progress_, cloud_receipt_, map_receipt_;
  ros::Time front_stamp_, down_stamp_;
  ros::WallTime state_receipt_, lio_receipt_, manager_receipt_, front_receipt_, down_receipt_;
  rviz::RenderPanel* render_ = nullptr;
  rviz::VisualizationManager* manager_ = nullptr;
  std::map<QString, QLabel*> cards_;
  std::map<QString, rviz::Display*> displays_;
  QLabel *telemetry_ = nullptr, *readiness_ = nullptr;
  QPushButton* pick_button_ = nullptr;
  QLabel *front_image_ = nullptr, *down_image_ = nullptr;
  QPushButton *arm_button_ = nullptr, *takeoff_button_ = nullptr, *hold_button_ = nullptr,
      *cancel_button_ = nullptr, *land_button_ = nullptr, *disarm_button_ = nullptr,
      *auth_button_ = nullptr, *goal_button_ = nullptr;
  QDoubleSpinBox *takeoff_height_ = nullptr, *navigation_speed_ = nullptr, *relative_x_ = nullptr, *relative_y_ = nullptr, *relative_z_ = nullptr, *clicked_height_ = nullptr;
  QPlainTextEdit* events_ = nullptr;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "drone_operator_gui");
  QApplication app(argc, argv);
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(2);
  spinner.start();
  OperatorWindow window(nh);
  window.show();
  return app.exec();
}
