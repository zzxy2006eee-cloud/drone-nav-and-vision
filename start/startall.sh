#!/bin/bash

#杀死会话
tmux kill-session -t drone
#可创建tmux会话
tmux new -s drone

roslaunch mavros px4.launch fcu_url:=/dev/ttyACM0:921600 gcs_url:=udp://:14556@10.42.0.103:14550

#启动激光雷达SDK
tmux new-window -t drone -n lidar "cd /home/d/livox_ws && source devel/setup.bash && export DISPLAY=:0 && roslaunch livox_ros_driver2 msg_MID360.launch"

#启动下视相机
tmux new-window -t drone -n camera "rosrun usb_cam usb_cam_node _video_device:=\"/dev/v4l/by-id/usb-HJ_USB_2.0_Camera_HJ_USB_2.0_Camera_SN0001-video-index0\" _pixel_format:=mjpeg"
#(如果是C70)
tmux new-window -t drone -n camera "rosrun usb_cam usb_cam_node _video_device:=\"/dev/v4l/by-id/usb-Generic_Integrated_Webcam_200901010001-video-index0\" _pixel_format:=mjpeg"

# 运行 Faster-LIO
tmux new-window -t drone -n lio "cd /home/d/robotproject/project1 && source devel/setup.bash && export DISPLAY=:0 && roslaunch faster_lio mapping_mid360.launch"
#tmux new-window -t drone -n lio "cd /home/d/robotproject/project0 && source devel/setup.bash && export DISPLAY=:0 && roslaunch fast_lio_localization localization_mid360.launch "


#运行ego_planner轨迹规划
tmux new-window -t drone -n planner "cd /home/d/robotproject/project1 && source devel/setup.bash && roslaunch ego_planner real_drone.launch"

#运行杂物识别
tmux new-window -t drone -n vision "cd /home/d/robotproject/project0/src/controller/scripts && python3 vision.py"

#在地面电脑中运行rviz查看处理后的视频流
# rosrun rviz rviz -d /home/ccl4/robot_project/remote_rviz/vision0.rviz

#在地面电脑终端中运行rviz
# rosrun rviz rviz -d /home/ccl4/robot_project/remote_rviz/default.rviz


# cd /root/robot_project/project0 && source devel/setup.bash && export DISPLAY=:0 && roslaunch fast_lio_localization localization_rflysim.launch

tmux new-window -t drone -n controller "cd /home/d/robotproject/project0 &&  source devel/setup.bash &&  roslaunch controller controll.launch"


tmux new-window -t drone -n record "rosbag record -o flight_4  /vision/processed_image/compressed /cloud_registered  /ego_planner_node/goal_point  /ego_planner_node/optimal_list  /grid_map/occupancy_inflate   /ego_planner/odom  /tf /tf_static"

tmux new-window -t drone -n record "rosbag record -o  lidar3 /livox/imu /livox/lidar"


tmux select-window -t 5