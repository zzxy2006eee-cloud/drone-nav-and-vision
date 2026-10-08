xterm -hold -e "cd /root/robot_project/project0/src/cruise_task/sensor_pkg && python3 main.py" &
sleep 20

xterm -hold -e "cd /root/robot_project/project1/ros_ws && source devel/setup.bash && export DISPLAY=:0 && roslaunch faster_lio_main rflysim.launch" &
sleep 5

xterm -hold -e "cd /root/robot_project/project0 && source devel/setup.bash && roslaunch ego_planner rflysim.launch" &
sleep 10

xterm -hold -e "cd /root/robot_project/project0 &&  source devel/setup.bash &&  roslaunch controller controll.launch" 
