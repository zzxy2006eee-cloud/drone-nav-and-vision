# 巡检功能离线检查阶段

日期：2026-10-07。

- `catkin_make -j1 -l1 --pkg ego_planner drone_stack`：退出码0，EGO与Qt均编译成功。
- `python3 src/drone_stack/scripts/test_inspection.py`：31项通过，约1.3秒；不创建ROS节点、不发送运动指令。
- 新增/修改Python模块语法检查、git差异空白检查、文档本地链接核对通过。
- 测试输出见 [offline_tests.log](offline_tests.log)，构建末尾输出见 [build_summary.log](build_summary.log)。

检查覆盖区域与相机几何、实际参考路线绕柱、窗口语义、旧会话拒绝、受阻前段隔离、终点邻近防误到达和故障状态。这里的柱体来自离线体素夹具，**不是实际Gazebo飞行实验**。

运行环境探测：创建本机TCP监听返回 `PermissionError: [Errno 1] Operation not permitted`。当前工具环境无法启动ROS master，因此未进行Qt渲染、PX4跟踪、相机同步及动态障碍的在线验收，也没有CPU/磁盘实飞性能数据。

下一阶段须完整重启新消息版本，在预建地图下手动设置初始位姿、授权、起飞，再从Qt开始巡检；实验使用 `DRONE_SHOW_GAZEBO=0`，关闭Gazebo图形界面。测试场景及故障清单见 [INSPECTION.md](../../INSPECTION.md)。
