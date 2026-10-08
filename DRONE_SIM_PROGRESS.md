## 2026-10-07 用户启动等待与devel模块导入修复（最新）

- 用户启动world_20261007_181232、inspection_20261007_181247后询问PX4断连/LIO失效。检查实际日志及startup_check.json：WAITING_POSE、无/clock、没有spawn_request.json和px4_gazebo.log，PX4/Gazebo尚未生成，需综合页确认出生位姿。
- 同时发现AI与巡检节点真实ImportError：catkin devel辅助脚本relay不导出执行context内函数，入口未将真实源码目录置于sys.path首位。已为drone_ai_node.py、inspection_manager.py添加入口源码路径，与原flight_manager/safe_global_path一致。
- 两个devel入口分别在独立Python进程使用runpy（非__main__，不初始化ROS/发送控制）检查通过，语法及差异检查通过。未重启当前用户进程，当前工具不能在其ROS网络上启动节点。
- 用户需要先Ctrl-C导航终端B并重新执行bash start/start_navigation.sh inspection，保留世界终端A等待，然后在综合页选初始位置/朝向并确认启动仿真。再次核对时仍无出生请求；不可据旧日志宣称飞行已开始。

## 2026-10-07 AI仿真实际启动尝试（最新）

- 用户要求尝试AI控制仿真，保留后台Gazebo、不打开其图形界面的约束。
- 先探测TCP监听，返回PermissionError/Operation not permitted。实际启动独立端口11421的roscore：首次默认ROS_HOME只读；改用/tmp后仍因XML-RPC监听Operation not permitted失败，退出码1。
- 当前工具可见相关进程未检测到；工具进程命名空间可能隔离，不能据此判断宿主机仿真是否运行。未停止已有任务、未启动新Gazebo/Qt，未发送授权/ARM/起飞/导航/降落指令。
- 使用私有密钥尝试真实DeepSeek纯解析调用（不执行动作），返回ProxyError，未获得计划。不能验证真实AI控制链路或密钥有效性。
- 日志保存于docs/simulation/2026-10-07-ai-start-attempt/。下一步需在允许本地TCP/UDP的运行环境启动分离脚本，先进行只解析验证，再做起飞/导航/已知区域/降落；此前70项离线检查不能替代本次飞行验收。

## 2026-10-06 当前版本源码与新README已推送GitHub

- 用户明确要求将这版代码附新README推送。源码提交9a9667f39443fbbe596aa7bd789fc65504c5722d，origin/main已推送成功并ls-remote核对；修改前基线11c9129作为历史记录保留。
- 新README重新整理当前真实实现、依赖/编译、无图与带图出生位姿流程、原生C++雷达与仅位姿融合、增量地图/缓存/显示、1s HOLD/2s降落与estimator_age不变、验证范围和未完成验收。docs/simulation/2026-10-06附小型格式/融合参数/地面计时摘要。
- 共41个源码/配置/文档文件提交，包括必要PX4启动与模板镜像；未提交大型录包、点云、地图、构建目录或本地凭据。发布操作没有停止仿真、发送飞行控制或改变运行参数。
- 下面各部署阶段的“未提交/未推送”描述是当时状态；当前源码备份已在GitHub。飞行回归和实机验收仍未全部完成。

## 2026-10-06 原生C++雷达前端已部署；用户选择1s HOLD/2s降落，estimator_age保持

- 用户纠正不改estimator_age，并明确选择持续2秒降落、1秒先HOLD、严重立即降落；管理器/bridge/launch统一1/2，estimator_age仍2秒，EGO心跳1秒。健康边界表达式测试及运行参数核对通过。
- sim_livox_native_adapter.cpp直接原生Gazebo LaserScanStamped.Time，C++构造CustomMsg+真实hit/miss映射射线和IMU转发，旧ROS block-laser插件从generate_model.py删除（仅改生成模板会被该脚本写回，已修）；设备几何/传感器参数/ENU/pose-only保持。当前SDK流不使用world_pose/真值。
- 启动出现Qt/Gazebo OGRE1.9崩溃，core空指针来自无物理显示器的RandR模式列表；workspace兼容lib及启动器条件启用处理，GPU与系统显示配置保持。完整编译和最终原生目标构建退出0；Qt和Gazebo后台现已稳定。
- 部署/实验start/qt_work/native_lidar_final_20261006_181818（两个current别名），日志start/logs/native_lidar_final_20261006_181818。地面验证README/报告native_frontend_deployment_report.md：299帧前端构造0.525ms、位姿年龄22ms sim/p95 30/max46，前端进程约4.1%单核；格式9061点/100ms offset正确，mapping4531hit+5540miss，融合EV11/GPS0/BARO0/HGT3。
- 当前用户手动飞行进行中：20:46:56起飞，20:47:18导航；20:47:50在线目标被观测占据，单点跳过保留后续，20:47:51新点导航。近期位姿年龄中位28ms/p95 46/max60、健康HEALTHY；20:47:27局部前瞻碰撞等待不应称已完全无导航问题。没有自动飞行控制，记录未停止，新的飞行验收仍进行。
- 当前进程：launcher=46011, monitor.py=47024, reference_observer.py=47025, heartbeat_observer.py=47026, resource_observer.py=47027, voxel_observer.py=47028, lio_latency_observer.py=47029, rosbag=47030, qt=46085, adapter=46106, lio=46107, bridge=46113, ego=46121, manager=46168, map_session=46170, global=46176, px4=46644, gzserver=46510. README更新，estimator_age原样；当前未提交/推送，GitHub仍11c9129。恢复用native_lidar_latest检查点及这些日志/进度；勿因当前飞行直接重启。

## 2026-10-06 定位链路分段计时已部署，地面300帧测得前端瓶颈与源时间戳滞后

- 新增全局可选/drone/record_lio_latency（DRONE_RECORD_LIO_TIMING=true）；C++雷达MessageEvent接收时刻、预处理/缓冲/等IMU/去畸变/降采样/IEKF/地图/位姿及点云发布的墙钟/进程CPU/主线程CPU；Python适配器接收/反序列化CPU和逐点转换/发布，桥接锁等待/处理；独立CPP原始雷达传输收包时刻。默认关闭，算法/队列/时间戳/融合不改。
- 确认落地上锁后停止旧实验，计时完整编译两次退出0并重启。初步阶段发现诊断包起点/终点域错配，原始packet保留并用lio_latency_join.py按CPP BEGIN/END准确关联重建；最终四路探针完整关联。当前start/qt_work/lio_segment_timing_final_20261006_165403（两个current别名），startup sim70.190 READY/HEALTHY、MAPPING、unarmed/landed1，已有50完整健康帧。记录持续，未自动飞行。
- sim65～95地面300帧：适配器回调前CPU50ms、构造90ms/发布8.4ms；LIO计算至位姿10.2ms（p95 20.7），缓冲等0.016ms、IMU等0、桥接锁0.010ms。LIO队列深度1；报告位姿年龄中位0.230sim秒。地面常态主要是雷达前端CPU/时间标签，不能替代旧飞行峰值分段结果。
- 仿真点云头到CPP接收中位0.132sim秒。额外nativeLaser与ROSCloud同帧接收配对5帧：原生测量时间比ROS头新0.128～0.154sim秒。安装插件二进制LastUpdateTime→PutLaserData与官方Gazebo回调/lastUpdateTime更新顺序吻合，存在上一周期旧时间戳；不把全部差值叫纯固定100ms，不直接刷新/加常数伪造时间。源时间戳修复/C++前端替换尚未实施。
- 报告lio_segment_timing_analysis.md、timing_schema.json、latency_ground_summary.json、native_lidar_timestamp_comparison.json及CSV/packet保存。下一步用户手动相近路线复现0.47s峰值；后台记录包含/Odometry、/mavros/odometry/out、/mavros/estimator_status和四路计时诊断；不录原始点云。当前进程：launcher=32066, monitor.py=33070, reference_observer.py=33071, heartbeat_observer.py=33072, resource_observer.py=33073, voxel_observer.py=33074, lio_latency_observer.py=33075, rosbag=33076, qt=32140, adapter=32161, lio=32162, bridge=32168, ego=32177, manager=32220, map_session=32221, global=32223, px4=32708, gzserver=32576.

## 2026-10-06 EKF取消原始ULog根因已取证（只分析）

- ULog08_11_51.ulg sim602.788 vision_data_stopped、EV位置/高度/航向停融，solution831→129；603.276恢复位置/高度，实际停融0.488s。ROS标志602.808无效→603.806有效，低频报告延长到近1s，603.368的0.5s健康保护取消任务发生在原始EKF恢复之后。
- 对应定位样本在ROS诊断已老0.448～0.474s，PX4接收年龄0.445～0.473s。实际EKF延迟时域316ms；RingBuffer只接受该时域前100ms内样本，过旧数据无法及时融合，继而400ms EV融合样本超时。EKF2_DELAY_MAX=400配置值≠实测时域316ms；未归因到具体LIO函数。
- filter_fault_flags0、IMU错误/削顶0、几何支持>500，外部速度融合关闭（EV_CTRL11/cs_ev_vel0）。报告start/qt_work/spline_cache_fixed_20261006_155431/ekf_health_protection_analysis.md及ekf_raw_incident_603.json保存原始窗口。建议减延迟、核对实际EKF时域、提高ESTIMATOR_STATUS5～10Hz；均未实施，未重启/发送控制。

## 2026-10-06 SciPy样条只读缓存错误已修复部署，录制曲线回归通过

- 用户确认降落后再次读取FCU connected=true/armed=false/landed_state1。停止旧记录/运行，rosbag SIGINT完成索引；修复BSpline使用独立可写knots/points副本，缓存原始几何保持只读。纯Python改动无需C++编译；新launcher源快照与当前trajectory_guard.py逐字节一致，deployment.json保存SHA256。
- 使用旧实验录制的27条真实Bspline回归：每条128采样位置、速度、加速度与未缓存BSpline逐项一致（最大差0）；确切出错的起点标量求值通过，validate_curve数值检查通过；32次4线程并发求值一致，27条derivative(1/2)对象求值也通过。结果spline_cache_fix_verification.json。这是数值回归，不是自动飞行或真实障碍绕行验收。
- 新运行start/logs/spline_cache_fixed_20261006_155431，部署/记录start/qt_work/spline_cache_fixed_20261006_155431（current_feature_deployment/current_manual_observer）。sim65.190地面READY/HEALTHY、OFFBOARD未进入、FCU AUTO.LOITER/connected/armed=false/landed1、map MAPPING/alignment true，manager/global voxel有效且resync0。运行日志无新read-only/bad callback报错；没有ARM/起飞/目标指令。
- 心跳1.0sim秒、增量地图/缓存与4Hz/15FPS显示保持；Qt已打开，heartbeat/resource/voxel/常规观察及轻量rosbag已开启。进程：launcher=18700, monitor.py=19695, reference_observer.py=19696, heartbeat_observer.py=19697, resource_observer.py=19698, voxel_observer.py=19699, rosbag=19700, qt=18777, ego=18808, manager=18857, map_session=18860, global=18863, px4=19338, gzserver=19211. 下一步用户手动复试飞行，观察on_spline及紫线/终点，不宣称其他飞行问题已全部验收。

## 2026-10-06 飞行实验发现样条缓存只读兼容问题，源码修正待地面部署

- 本轮当前NAVIGATING/SPACE_WAIT、FCU connected/armed/OFFBOARD、HEALTHY，仍保持等待位姿。15:50:17等on_spline回调ValueError buffer source array is read-only；不是本次心跳超时。新缓存BSpline别名引用了被setflags(write=False)的原始knots/points，安装SciPy的Cython求值接口要求可写缓冲，造成轨迹读数异常。
- trajectory_guard.py已修正：BSpline使用独立knots.copy()/points.copy()可写数组，缓存原始几何仍只读。Python AST通过；当前飞行进程仍是旧内存代码，未重启、未发控制，不可宣称修复已部署/飞行验收。待确认落地未解锁再部署，复查样条求值、终点控制和轨迹显示。
- 增量地图manager/global有效、无断序；当前CPU管理器约16%、Qt14%、EGO75%（飞行附近抽样）。记录继续于start/qt_work/cpu_optimized_20261006_154027。

## 2026-10-06 CPU优化1/2/3与1秒心跳门槛已编译部署，地面只读核对完成

- 按用户选择落实：新增plan_env/VoxelUpdate、GetVoxelSnapshot，/grid_map/voxel_delta每有效观测发布epoch/revision/base_revision、占据/自由增删ID；数据不变仍发小型空增量保留真实扫描采集时间。地图/原点初始化完成前仍清图发full；READY锁定地图帧后仅扫描dirty体素，reset/加载更换epoch。保留盲区与射线确认清除/膨胀引用逻辑。
- 管理器/全局节点共用voxel_map_client.py，占据/自由同版本位图，在控制锁外解码、变更层copy-on-write，锁内原子交换。断序/格式错误先使本地地图时间失效再完整快照恢复，旧epoch/revision不覆盖新状态；地图时间不随缓存重放刷新。两端不再订阅安全/自由PointCloud2（readonly rostopic info确认Subscribers None）。后台兼容点云仅有订阅时生成；Qt占据体素4Hz，安全增量仍每扫描更新。
- trajectory_guard缓存24组几何内容对应的BSpline/PPoly，manager和global复用；不缓存时间/地图/碰撞结论，保持每次检查当前输入。执行紫线显示刷新4Hz、RViz15FPS；默认只开Qt，Gazebo物理/传感器服务器仍运行，需要窗口用DRONE_SHOW_GAZEBO=1。
- max_planner_age_s默认、允许上限和launch值改1.0sim秒，运行rosparam核对1.0。2秒轨迹墙钟预算等其余保护不变。新增显式online_mode，修复DRONE_PREBUILT_MAP空字符串被roslaunch回退地图默认值；本轮自动无图启动，无需点击online服务。
- 编译完整catkin_make -j1 -l1退出0，Python AST/XML/bash -n/git diff --check通过。旧实验确认connected/unarmed/landed1后停止，rosbag用SIGINT完成索引（旧flight_diagnostics_0.bag约20MB）。新运行start/logs/cpu_optimized_20261006_154027，部署与观察start/qt_work/cpu_optimized_20261006_154027（两个current别名）。sim65.560 READY/HEALTHY、ONLINE/MAPPING、alignment true、FCU未解锁落地；启动JSON中客户端仍为冷启动full，后续已确认同一epoch连续递增增量。
- sim75～95地面抽样196个增量，体素ID载荷平均546.8B、最大3236B（不含ROS消息头）；CPU单核百分比：管理器12.78、Qt13.53、EGO66.12、LIO25.93、Gazebo服务101.23。这与之前飞行/不同地点不是同条件对照，不应宣称确定降幅；ground_profile.json保存数字。
- 增量接收与权威完整快照按同一epoch/revision680核对：occupied122264/free265306完全一致、无交集。第三只读晚接入客户端自然断序后通过服务成功恢复；正常manager/global未见断序。voxel_snapshot_consistency.json保存证据。未发送任何ARM/起飞/目标，缓存/增量飞行安全与效果仍需手动回归。
- 当前进程：launcher=15344, monitor.py=16337, reference_observer.py=16338, heartbeat_observer.py=16339, resource_observer.py=16340, voxel_observer.py=16341, rosbag=16342, qt=15418, lio=15438, ego=15451, manager=15497, map_session=15499, global=15501, gzserver=15852, px4=15977. monitor/reference/heartbeat/resource/voxel观察和轻量rosbag持续运行，记录含voxel_delta；不录原始点云/大体素/视频。源码改动未提交/推送，GitHub仍为11c9129。README已补接口/启动/性能改动。

## 2026-10-06 本轮心跳超时已取证分析（未修改）

- sim137.276新规划ACK/enabled=true，137.582（差0.306s）触发心跳超时HOLD；首DataDisp直到137.672才发出（ACK后0.396s），137.718管理器接受（age0.046s），此前已取消。EGO有ACK_START及后续final_plan_success=1，不是死进程证据；故障窗口没有先收到首心跳却因future/stale拒绝的记录。
- 超时附近整机CPU88.7～90.8%，8逻辑核load1约16.8～19.2，EGO约单核97～102%，计算压力与单线程回调共享均存在。直接机制是启动期未等首心跳就套0.3s门槛；未记录逐C++回调耗时，具体阻塞函数尚不能确定。整轮另有future拒绝，与本次首心跳迟发分开分析。
- 分析报告start/qt_work/heartbeat_online_20261006_151124/ego_timeout_analysis.md。建议启动/运行心跳分阶段、独立心跳与规划进度、降低地图处理拥堵，时钟超前另修；均未实施。本轮监测/录包继续，未发送控制或重启。

## 2026-10-06 无图在线实验重启就绪，心跳诊断与轻量录包持续记录

- 按用户要求无地图重启，确认启动前无ROS/PX4/Gazebo旧进程。权限受限时的尝试因本机TCP/UDP禁止失败；本轮权限恢复后成功启动。记录目录start/qt_work/heartbeat_online_20261006_151124（current_feature_deployment/current_manual_observer），运行日志start/logs/heartbeat_online_20261006_151124.
- 明确通过地图会话online启动，spawn_request模式ONLINE，未导入预建地图。环境空DRONE_PREBUILT_MAP被roslaunch默认地图参数回退，此次启动使用/drone/map_session action online完成无图切换；空参数默认回退仍需后续修正。本轮未修改导航/健康判定或0.3sim秒心跳阈值。
- flight_manager增加可选record_heartbeat_diagnostics（默认false），本次true：在原心跳判定位置发布/drone/planner_heartbeat_diagnostic数组[callback_now,source_stamp,age,accepted,last_accepted,goal_start,navigating]，不改变接受条件。launch/start_simulation新增诊断参数/DRONE_RECORD_HEARTBEAT，Python AST及shell/XML通过，纯Python配置改动无需C++编译。
- heartbeat_observer记录原始DataDisp接收、管理器接受/拒绝、planning_enabled/阶段/健康/握手日志、按进程CPU/RSS/线程及仿真墙钟倍率；resource_observer额外记录整机/逐核CPU、负载和可用内存。原monitor/reference_observer记录位置/朝向/队列/全局与执行曲线。CSV和JSONL持续落盘。
- rosbag记录clock、DataDisp、判定诊断、planning_enabled、manager heartbeat、phase/health/stage、队列/目标/GlobalRoute/Bspline、FCU里程计/setpoint及rosout/LIOhealth，128MB分卷；没有相机、原始点云或巨大体素话题。录包active应在用户实验完成后SIGINT关闭再分析，防止索引未落盘。记录启动覆盖解锁/飞行前，整机资源从中段启动阶段开始。
- startup_ready.json确认sim65.330 READY/HEALTHY、connected=true/armed=false、landed_state1、map_session ONLINE ready=true、map backend MAPPING、alignment=true。只读topic info核对心跳诊断和原始心跳均被observer及rosbag订阅。没有自动ARM、起飞或发送目标。
- 当前进程：launcher=8883, monitor.py=9329, reference_observer.py=9330, heartbeat_observer.py=9331, rosbag=10726, qt=8960, lio=8978, ego=8991, manager=9033, map_session=9037, gzserver=10015, gzclient=9916, px4=10355, resource_observer.py=11259. 用户可手动实验，后台持续记录。后续先看heartbeat_decisions.csv中接受/未来/过期/重复时间、heartbeat_arrivals与启动握手和CPU同步关系，避免仅凭负载猜原因。

## 2026-10-05 默认带图先选出生位姿；在线障碍目标跳点已编译部署

- 用户反馈手动修改地图初始位姿未改变实际Gazebo位置，以及在线目标进入障碍会清空全部队列。本轮默认先启动ROS+Qt并预览场景地图，WAITING_POSE时不启动Gazebo/PX4；Qt出生位置/拖动航向确认经/drone/map_session生成/drone/simulation_spawn_request，启动器以墙钟等待，随后才启动PX4/Gazebo。spawn_model接收X/Y/Z及-Y弧度yaw，prepare_px4_sitl.py将补丁持久化。Gazebo GUI也在确认后启动并归启动器清理；等待进程纳入独立受管进程组，TERM可以中断等待。
- 默认地图start/maps/inspection_demo_scene.dmap使用场景世界ENU。Qt默认出生[1.01,0.98,0.17]、yaw0，可点击XY并拖朝向；平地Z限制0.15～0.20m、XY±8m、距地图原始障碍至少0.55m。定位就绪后核对实际出生与请求（位置差≤0.1m、yaw差≤5°），加载对齐地图。已经初始化的受管仿真不允许仅改TF重设出生位姿，需重启。LIO持续在线定位，真值只用于初始对齐/核对，不进入EKF。Faster-LIO不进行预建点云匹配重定位。
- 支持Qt启动前“返回在线建图导航”/“开始新建图”，或DRONE_PREBUILT_MAP=直接在线启动。MapArchive新增clip_to_navigation_grid，仅仿真自动加载启用：变换原始体素盒与当前网格取交，远于导航范围的地图部分可裁剪；有效飞行体积在网格内部，保守膨胀重建，历史自由非对齐不导入。手动加载默认仍拒绝越界。
- flight_manager订阅实际/drone/map_mode；只有MAPPING在线模式跳过障碍点。入队已知占据点拒绝但不清其余队列；下一点派发时最新地图判占据则逐点跳过；当前目标被实测占据则invalidate旧规划/清旧曲线、HOLD保留后续队列，0.3sim秒后依次派发（全跳完则HOLD）。定位/地图过期、OFFBOARD丢失等安全保护仍可取消全队列。带图障碍点规则保持原状。
- Python AST、XML、bash -n及git diff --check通过；完整catkin_make -j1 -l1退出0，随后Qt预览状态补齐的make -C build drone_operator_gui -j1退出0。Qt在WAITING_POSE/STARTING_SIM显示地图预览、隐藏实时层，初始位姿按钮在未连接FCU时可用；启动提示不再将尚未启动的时钟说成冻结。
- 当前运行日志start/logs/20261005_222419，部署/只读记录start/qt_work/pose_spawn_queue_final_20261005_222214（current_feature_deployment/current_manual_observer）。只读核对WAITING_POSE、ready=false、53392预览点/frame map、没有spawn_request且gzserver/PX4未运行；Qt截图已看到场景墙/柱预览（可能被终端遮挡）。进程：launcher=2251158, qt=2251240, manager=2251316, map_session=2251325, ego=2251273, monitor.py=2251656, reference_observer.py=2251657.
- 没有代用户选位姿/确认启动、没有解锁/起飞，也未运行导航跳点飞行回归。下一步用户在Qt右侧地图栏选择出生点和朝向后确认，观察实际Gazebo生成位姿/带图就绪，再手动实验；只读观察已预先运行。当前修改未提交/推送，GitHub仍为11c9129基线。README最新流程取代前面的自动真值带图启动描述。

## 2026-10-05 当前代码编译完成，场景预建地图已加载启动

- 用户要求编译当前代码、建立当前仿真场景地图并带图启动。完整catkin_make -j1 -l1退出0，首次启动发现地图底部体素的微小坐标变换越界；修复后再次完整编译退出0。后端仅裁掉中心有效的底部体素落在网格下方的部分，XY/顶部越界及体素中心越界仍拒绝。地图预览改批量坐标转换，减少持锁时间和回调阻塞。
- 新增build_simulation_scene_map.py：仅仿真、连接落地未解锁时，根据当前Gazebo静态方盒碰撞几何生成V2原始占据。资产start/maps/inspection_demo_scene.dmap及同名json，10cm分辨率，53392原始占据、7碰撞盒（四墙/两柱/黄色设备）、无伪造自由证据。地图是仿真几何先验，不是飞行雷达扫描图；无碰撞的视觉靶和导航高度以下地板不纳入。实测雷达继续确认/清除，观测自由安全检查保留。
- 新启动方式DRONE_PREBUILT_MAP=/home/d/robotproject/project0/start/maps/inspection_demo_scene.dmap bash src/drone_stack/scripts/start_simulation.sh。launch传入地图会话及Qt默认文件；READY/HEALTHY落地上锁后用Gazebo机体真值做一次地图初始对齐、加载。该真值不进入Faster-LIO、EKF或持续控制；实机手动点选初始位置/拖朝向流程保留。
- 最终运行start/logs/20261005_213729；部署及只读观察start/qt_work/premap_final_20261005_213729（current_feature_deployment/current_manual_observer）。sim66.902确认READY/HEALTHY、FCU connected=true/armed=false、landed_state1、alignment=true、地图后端/会话PRIOR_NAV且ready=true。没有解锁、起飞或发送导航目标。
- sim88.010只读抽样Qt同源膨胀显示182940体素；转换到地图坐标后，七处障碍中心到最近膨胀体素均小于5.9cm，确认所有场景障碍已进入地图。此项仅核对加载与坐标，不代表飞行规划/动态清除已验收。startup_ready.json/map_geometry_observation.json/deployment.json保存证据与指纹；Qt及Gazebo窗口存在，Qt已置于前面。
- 进程：launcher=2243204, gzclient=2244199, monitor.py=2244200, reference_observer.py=2244201, qt=2243282, manager=2243393, map_session=2243397, ego=2243337, lio=2243318, gzserver=2243661, px4=2243844. 只读观察持续运行；当前修改未提交、未推送，GitHub仍为修改前基线11c9129。README补充地图生成及带图启动命令。

## 2026-10-05 21:14 三项修改已编译部署，地面就绪；修改前基线已推送GitHub

- 按用户顺序先更新根README并提交现有全部项目源码/小型分析文档，GitHub origin/main推送成功且ls-remote核对为11c91294e3933209a7ef0f9dcdff9343eeaae8f5。该提交是三个修复之前的基线。后续修复当前为本地未提交改动，未推送；README/无人机栈说明已补新流程。
- 终点保持最终15cm三维球、≤0.15m/s、稳定1sim秒。确认阶段仅使用同一批准零速度末端（没有合格末端则测得位姿）；瞬时越界/速度/时间抖动重置确认计时，不立即转向。退出半径20cm、持续0.5sim秒或收敛超过4sim秒才重试。每周期独立检查碰撞/自由空间/飞行范围，健康保护不变。
- 地图新增仿真实际射线流：有效回波hit=1，明确40m无回波miss=0；近场/异常不造自由证据，实机缺失点不是自由射线。桥接按采集时刻传感器外参/FCU位姿发布/drone/mapping/rays，EGO与同步cloud_body_pose配对。先登记命中，再清除实测射线穿越原始占据；只保护命中体素，移除历史15cm邻域保护带，当前膨胀引用随原始占据删除。盲区保留，Qt体素Decay Time0。
- 首次启动检查无回波会产生56万自由体素；最终版本自由证据仅记录配置导航体积，范围外射线仍删除旧原始占据，降到约16.4万，不增加无用自由传输。地图完整性和动态清除效果仍需飞行回归，不能宣称所有残留已消除。
- map_session.py及/drone/map_session服务维护地面map→odom对齐；在线READY自动原点/初始机头+X。预建地图Qt先预览、点位置拖朝向（XYZ/yaw可输入）、确认加载，再允许ARM和导航。地图文件V2保存map_from_odom元数据，兼容V1；地图坐标目标/高度换回控制ENU，所有显示层使用同一变换，不改LIO/EKF原点或ENU/NED。手动初始位姿不是自动重定位。
- Python AST/XML和git diff --check通过，catkin_make -j1 -l1、Qt目标编译及最终完整编译退出0。源码/二进制指纹与日志start/qt_work/three_fixes_final_20261005_211423（current_feature_deployment）；sim65.176只读地面READY/HEALTHY、connected=true、armed=false、AUTO.LOITER、landed_state1、alignment_ready=true、ONLINE。
- 最终运行start/logs/20261005_211423；launcher2233736、Qt2233825、manager2233891、map_session2233892、global2233895、EGO2233847、LIO2233833、gzserver2234202、gzclient2234701。新地图流10071点：4531回波+5540明确无回波，observed_free163996、inflated127381，采集年龄约0.97sim秒（地面抽样）。没有ARM/起飞/导航或地图存取/切换测试，三个问题尚未飞行验收。
- 旧实验监测已在确认落地上锁后停止。最终只读monitor2234721、reference_observer2234722持续记录，current_manual_observer指向最终部署目录；没有启动额外原始点云订阅诊断（避免重复解码负担）。Qt/Gazebo打开，GUI screenshot可能被终端覆盖，不能作为地图初始位姿交互验收证据。下一步由用户手动复试终点、多点、地图动态清除和预建初始位姿。

## 2026-10-05 本轮终点/地图残留/初始位姿需求已记录，尚未修改

- 首点终点问题已定位：sim126.492距目标10.6cm进入确认，sim127.39距15.0000205cm速度仅0.0094m/s，硬15cm门槛失效，127.408重规划、机头反向至-132.32度，132.126才到达。需要确认阶段滞回和安全末端控制，不能通过放宽最终到达容差掩盖。
- 地图有射线清除和膨胀引用扣减；原始占据残留、射线覆盖/回波保护带、FCU/LIO逐帧配准与Qt历史显示均需区分。尚未证明具体根因，不得写成无实时更新。初始位姿建议map/odom显式对齐、Qt地图初始位置/朝向设置，并保持控制ENU及飞控内部连续。
- 分析建议start/qt_work/lio_diagnosis_20261005_202824/three_issue_review.md。用户已选择：在线自动设起点；预建地图手动点位置并拖动朝向。当前飞机已手动降落上锁，后台诊断仍在运行。没有控制或算法修改。

## 2026-10-05 20:28 定位异常复现实验版本重启完成，只读诊断持续记录

- 按用户指令，重启前新鲜确认connected=true、armed=false、landed_state1；停止上一轮launcher2221110、Qt2221186、gzclient2222031。新日志start/logs/20261005_202841；launcher2222913、Qt2222991、manager2223096、global2223099、EGO2223050、LIO2223035、gzserver2223360、gzclient2223864。
- sim65.050地面READY/HEALTHY、connected=true、armed=false、AUTO.LOITER、landed_state1；2秒曲线墙钟预算及30约束阈值保持，导航/健康策略没有修改，没有自动飞行。
- 新只读监测start/qt_work/lio_diagnosis_20261005_202824（current_manual_observer），monitor2223884、reference_observer2223885、cloud_diagnostic2223886。记录几何三特征值/匹配数、定位与真值/实际朝向/轨迹队列/健康和黄线出现；额外每约1墙钟秒记录原始与适配点数、距离/机体坐标方向分布、三套位姿。弱约束时每5墙钟秒最多30帧压缩原始点云，限制数据量；不发布任何控制/真值输入。
- ground_alignment_baseline.json保存地面三套坐标初始对齐基准；真值world与LIO/FCU ENU原点不同，不能直接相减称漂移。具体弱方向/环境表面需复现后基于捕获帧进一步分析。现在等待用户手动相近点实验，监测后台持续，进程/deployment/pids/startup_ready均已保存。

## 2026-10-05 20:25 本轮第一点完成，下一点规划被几何健康保护中止（只分析）

- 第一目标[5.758,6.407,1.2]于sim150.562 / 20:21:45.063完成。近终点最弱平移约束反复低于30，区域采样最低23.226、匹配仍459点以上，没有雷达停发证据。
- 下一点派发要求HEALTHY；sim152.082 / 20:21:50.087恢复，152.128开始下一点。因此实际约5.08墙钟秒等待主要是健康恢复门槛，不是全程A*计算。随后154.522取消任务HOLD，155.014降落，20:22:15落地上锁。下一点没有规划ACK记录；未记录精确全局搜索扩展及黄色路线首次发布时刻，不能断言已成功算出第二点完整路线或耗尽15000节点。
- 当前LOCALIZING/SEVERE、armed=false、landed_state1。几何分数已恢复约69，但bridge.pose_fault在连续弱约束1.012仿真秒后锁定，停止外部位姿输出，需重启恢复；没有自动解除锁定。没有改程序、重启或发送控制。
- 证据start/qt_work/wall_budget_2s_restart/first_goal_health_incident.json及本轮ROS节点/stack日志。几何弱约束具体对应哪个表面尚未取证，不应称定位跳变或雷达损坏。

## 2026-10-05 20:13 按用户指令重启完成，2秒轨迹校验墙钟限制已部署

- 重启前新鲜FCU状态确认connected=true、armed=false、landed_state=1；停止旧launcher2217592、Qt2219809、Gazebo客户端2218504及本轮只读观察2220076/2220077，旧ROS/PX4/Gazebo退出后启动新完整程序。
- 当前日志start/logs/20261005_201347，launcher2221110、Qt2221186、管理器2221276、gzserver2221543、gzclient2222031。纯Python改动无需C++编译，新管理器加载trajectory_guard.py的wall_budget=2.0；CPU0.05秒、校验10000节点及其他保护保持。
- sim65.104只读启动状态READY/HEALTHY、connected=true、armed=false、AUTO.LOITER、landed_state=1、map_mode=MAPPING；全局节点上限15000。Qt/Gazebo进程均在线，没有发送ARM/起飞/导航命令。旧轮监测已停止，尚未开启新飞行监测。
- 证据start/qt_work/wall_budget_2s_restart/{deployment.json,startup_ready.json,launcher.log,gzclient.log}；原pending.json已标记runtime_applied=true。本次验证为地面启动，未做飞行实验。

## 2026-10-05 用户指定曲线校验墙钟上限改为2秒，源码完成待重启生效

- trajectory_guard.py validate_curve默认wall_budget由0.25改为2.0；flight_manager完整曲线与执行前瞻均使用该默认值。线程CPU预算0.05秒与递归10000节点上限保持，全局参考显式0.15/0.5秒预算保持，未改变碰撞/体积/新鲜度/超时后取消策略。
- Python AST及git diff --check通过；纯Python改动无需C++编译。当前运行管理器2217793已经导入旧默认值，尚未应用2秒；当前飞机仍HOLD、armed=true、Z约1.87m，不能通过空中重启管理器中断OFFBOARD。落地上锁后按用户重启指令部署；Qt无需为此重编译。没有发送飞行指令或改变运行参数。

## 2026-10-05 当前手动三点导航被整段曲线墙钟预算取消

- 用户重新起飞后HOLD高度约1.90m，低起点问题已消失；sim334.448提交首点[5.654,6.068,1.2]并连续排入3点。全局完整路线与EGO新会话ACK均已生成，机头对准后新曲线id2在sim337.390触发完整轨迹校验超预算，进入HOLD并清空3点队列；当前armed=true、OFFBOARD、HEALTHY，无自动降落。
- /tmp/drone_ros_home/log/70463b98-c0b2-11f1-b884-a1c2669bfe23/drone_flight_manager-16.log第161行记录wall=0.2514s、threadCPU=0.0059s、nodes=10；命中0.25s墙钟限制，而非0.05s CPU或10000校验节点限制。高调度等待/抢占是可疑原因，尚未进一步取调度证据；预算耗尽不能证明轨迹碰撞或LIO漂移。该预算独立于全局A*15000。
- 证据start/qt_work/manual_map_modes_20261005_195912/trajectory_validation_wall_timeout.json及原始logs/reference_observer。Qt按空队列清线源于取消，未到达。只分析和保存，没有修改参数/代码，后台观察继续运行。

## 2026-10-05 本轮手动实验发现起点高度低于规划范围

- 监测已记录TAKEOFF→HOLD→NAVIGATING；用户提交两个1.2m高目标[5.468,5.894]和[3.959,-3.603]。当前sim313.034实际ENU Z=0.12656226754188538，低于全局/管理器默认最低Z0.5m，规划起点落在允许飞行体积之外，六邻域A*无法连接可用起点，黄色完整路径为空。此原因与15000节点预算无关，定位仍HEALTHY。
- 证据start/qt_work/manual_map_modes_20261005_195912/low_start_height.json和telemetry/events；只分析和记录，没有调整高度参数、发送导航/降落命令或重启。建议用户先取消导航并降落，上锁后设置相对起飞高度1.2m重新起飞再测试；原相对起飞高度可调最小0.2m与导航最低绝对Z0.5m的兼容性需后续单独修改。

## 2026-10-05 19:59 地图模式版本手动实验监测已接入

- 仿真继续start/logs/20261005_194642，Qt2219809，未重启或修改控制。监测目录start/qt_work/manual_map_modes_20261005_195912，monitor2220076、reference_observer2220077，current_manual_observer指向本轮。
- 接入sim270.998时已TAKEOFF、armed=true、OFFBOARD、HEALTHY，地图模式MAPPING，地图状态“已清除旧地图，开始在线建图”；队列为空。PX4日志已记录Takeoff detected。本次只读监测，没有发送ARM/起飞/导航/地图切换指令。
- 记录地图模式/存取状态、队列、终点阶段、实际位置/速度/目标距离、全局路径、真实EGO曲线边界与紫线同源、实际朝向相对有向切线、健康/几何约束、OFFBOARD间隔、FCU/LIO/真值。没有录整幅点云/图像/rosbag；监测后台持续运行，后续需结合最新日志判断。

## 2026-10-05 Qt右侧自动放大/移动布局修复，仅重启Qt

- 相机QLabel原按自身size缩放图片后setPixmap，默认sizeHint再参与滚动内容的高度/宽度协商，存在持续撑大布局的反馈；动态多行状态文字也改变布局。现相机固定180px高度、QSizePolicy::Ignored/Fixed并按contentsRect绘制，右侧固定420px宽、垂直滚动条常驻；状态卡/就绪提示/导航提示/地图模式说明固定高度，长文提供tooltip，避免刷新改变控件位置。
- 操作按钮NoFocus，避免点击或健康状态使按钮启停时焦点自动跳到输入框/滚动位置；原ARM显式焦点留在操作组逻辑保留。右栏仍支持手动滚动。
- catkin_make drone_operator_gui -j1 -l1编译退出0、git diff --check通过。停止旧Qt2217671，单独启动新Qt2219809；仿真/管理器未重启（launcher2217592、管理器2217793、gzserver2218011），没有飞行控制指令。Qt为独立进程组，后续完整重启时还应按PID/命令核对停止该Qt。
- 界面已打开并查看截图qt_after.png，READY、定位有效、未解锁、相机/体素在线，尺寸正常。证据start/qt_work/map_modes_20261005/ui_fix含build.log、deployment.json及前后截图；before抓图因窗口被覆盖显示桌面，不能作为布局变化测量。当前部署指纹已更新Qt源码与二进制；未执行额外自动GUI测试或飞行实验。

## 2026-10-05 15000节点与Qt地图模式版本已编译部署，地面启动通过

- 重启前只读确认sim917.810 armed=false，sim919.010 landed_state1；旧仿真launcher2210827、gzclient2211755及只读观察2211751/2211752均已停止。
- catkin_make -j1 -l1编译退出0，地图后端、规划器、Qt均通过；日志start/qt_work/map_modes_20261005/build.log。已重启start/logs/20261005_194642，launcher2217592、Qt2217671、EGO2217725、管理器2217793、全局路径2217801、Gazebo客户端2218504；地图服务类型plan_env/MapArchive在线，max_search_nodes=15000，map_mode=MAPPING。
- Python AST、launch/package XML、git diff --check及部署源码/二进制指纹核对通过，Qt/Gazebo窗口存在；sim65.120只读确认READY/HEALTHY、connected=true、armed=false、AUTO.LOITER、landed_state1、map_mode=MAPPING、max_search_nodes=15000；地面启动检查通过，startup_ready.json与Qt/Gazebo截图已保存。本轮只编译/启动，没有调用地图切换/保存/加载服务进行测试，不自动ARM或飞行实验。地图存取和飞行效果仍需用户手动验证。

- 当前部署指纹、编译日志、启动状态与截图位于start/qt_work/map_modes_20261005；current_feature_deployment指向该目录。旧manual_terminal_retry只读观察已停止，新一轮飞行未启动。工作区改动未提交或推送GitHub。

## 2026-10-05 15000节点与Qt地图模式改造：源码完成，待落地后编译部署

- 用户要求提高节点至15000，新增建图保存、预建地图导航并同步Qt。已写入全局max_search_nodes参数15000，保留同任务PLANNING/SPACE_WAIT阶段切换的A*搜索，完整全局路径等待45秒、管理器ACK窗口50秒墙钟；其余体积/碰撞/目标代次保护保留。
- plan_env新增MapArchive.srv及/drone/map_archive服务，action=new/save/load/online；保存完整记忆原始占据证据与已观测自由体素，含网格/ENU元数据的.dmap稀疏存档，临时写入后原子替换；加载先校验全部记录，再重建当前膨胀与天花板。实时射线仍更新/清除占据。
- 导航/Qt体素发布改为稀疏已知单元集合，覆盖完整记忆和加载范围，不局限当前扫描窗口；动态维护膨胀引用与自由空间集合。存档不保存膨胀/虚拟天花板，避免重复膨胀。
- Qt右侧增加地图模式/状态、地图路径、文件选择、ENU dx/dy/dz/yaw校准及开始新建图、保存地图、加载地图导航、返回在线建图导航；存取/切换限定地面未解锁、READY/HEALTHY/空队列。加载不自动重定位，不改善Faster-LIO几何约束，必须与当前ENU正确对齐。非零yaw或非整格平移的历史自由空间不导入，占据按变换体素包络保守栅格化。
- Python AST/launch和package XML及git diff格式检查通过；直接cmake -S src -B build配置生成退出0（证据start/qt_work/map_modes_20261005/configure_final.log），尚未C++编译、部署或飞行验证，不得把源码当成已运行功能。当前仍旧版本start/logs/20261005_185449，飞机空中armed=true、landed_state2、HOLD（最新sim688.076），没有停止或修改运行控制。
- 已向用户说明并通过异步输入工具请求手动降落上锁，收到落地未解锁状态后再停止旧launcher2210827、gzclient2211755及monitor2211751/2211752；单任务编译降低内存占用，再重启Qt和仿真并只读检查READY。用户此轮未授权自动飞行，不自动执行飞行验收。新的service需要plan_env消息生成依赖与drone_stack对plan_env依赖，编译时继续排查。

## 2026-10-05 最后目标黄色路径缺失，只读诊断确认搜索预算与重试冲突

- 本轮前三目标到达；第三点[5.284,-4.946,1.2]于sim197.370完成，最后点[-5.433,2.731,1.2]于197.670启动，目标距机体约13.2m。后续反复PLANNING/SPACE_WAIT，错误Unified global route not available yet；定位HEALTHY、目标保留，OFFBOARD悬停流持续。
- 管理器等待完整全局路径15秒墙钟，超时进入2秒仿真SPACE_WAIT再重试；全局on_stage对每个非TRACKING阶段变化将search=None，长搜索的已探索状态反复丢失。A*分批CPU预算0.025秒、循环sleep0.1秒、六邻域，节点上限10000。
- 当前地图只读离线复算：起点/目标均安全，直连线被障碍挡住；原10000扩展上限用3.617CPU秒/143批后退出无路结果。隔离诊断副本仅把扩展上限改成60000，未修改源码/运行节点/飞行命令，10602扩展后找到4折点路径，并按原平滑/碰撞算法生成350点完整曲线，证明有安全可行路线。实时15秒等待及10000上限足以阻碍发布。
- 证据start/qt_work/manual_terminal_retry_20261005_185449/last_goal_readonly_diagnosis.json；本轮只分析，没有修复或部署。后续方案：保留同任务悬停重试的搜索状态；优化/增加搜索预算并把搜索进度纳入等待判定，维持碰撞安全检查。飞行监测仍后台运行。

## 2026-10-05 18:54 用户要求重新启动并手动复试

- 前一轮第一点在接近终点时最低位置观测支持连续低于30，sim1745.700 WARNING、1746.238 HOLD取消6点队列、1746.750 LANDING、1750.810落地上锁；触发前目标误差约0.0227m、速度0.243m/s，没有满足速度与1秒到达条件。终点样条id7精确末端与速度/加速度全零已记录，但收敛被健康保护打断，不能判通过。证据manual_terminal_20261005_185111/incident_first_goal_health.json。
- 按用户要求只读确认armed=false、landed_state=1后停止旧仿真launcher2207394及只读监测2210240/2210243；保持源码和参数，重启完整仿真与Qt，并开启新轮独立监测。
- 当前仿真日志start/logs/20261005_185449；launcher2210827、Qt2210900、管理器2211031、EGO2210956、FasterLIO2210937、Gazebo客户端2211755。只读监测目录start/qt_work/manual_terminal_retry_20261005_185449，monitor2211751、reference_observer2211752，最新指针current_manual_observer。
- 启动状态已保存于本轮startup_status.json：sim65.028，阶段READY、健康HEALTHY、飞控{'connected': True, 'armed': False, 'mode': 'AUTO.LOITER'}，落地状态{'landed_state': 1}。不自动ARM、起飞或导航，后续由用户手动操作；监测保留持续后台记录。

## 2026-10-05 18:51 用户手动终点收敛实验监测启动

- 不重启、不修改控制、不发送飞行指令。仿真继续start/logs/20261005_174011，监测目录start/qt_work/manual_terminal_20261005_185111，monitor PID2210240、reference_observer PID2210243。
- 接入sim1729.156时已NAVIGATING/TRACKING/HEALTHY，队列6个1.2m高目标：约[5.974,6.546]、[6.200,-6.873]、[-6.357,-6.648]、[-6.034,6.449]、[2.520,-0.261]、[3.424,-4.035]。第一点任务sim1721.746开始，接入前部分需从原始stack日志补充。
- 持续采集终点阶段/管理器内部阶段、10Hz实际位置/速度/目标距离、队列、健康、几何约束、OFFBOARD目标间隔、FCU/LIO/真值、实际批准样条端点及末端速度/加速度、紫线同源与真实切线朝向。未录整幅点云/图像/rosbag。
- 此记录是监测开始，不是实验完成或验收通过。监测为后台只读进程，停止/总结时重新检查最新日志。

## 2026-10-05 终点接近与收敛已编译部署，待手动飞行验证

- 已补齐管理器统一到达检查：在OFFBOARD、地图与目标占据安全门之后、规划/等待/转向提前返回之前检查原15cm三维球、0.15m/s和连续1秒；确认期间冻结测得的悬停位置及实际yaw，失效旧规划代次，拒绝旧曲线/指令覆盖，离开容差再规划。正常到达保留后续队列。
- EGO按剩余全局弧长进入默认0.8m接近阶段，按制动距离扩大；末段0.2m/s目标速度与0.4m/s²减速度参数，保留运动起始边界，五次弧长时间轮廓、精确样条终点及零末端速度/加速度；重分配后再次检查物理可行性。拒绝非单调短距离运动边界。终点规划和路线种子下限统一为0.02m，其余普通短轨迹门槛保持。
- EGO与管理器共用3秒末端收敛窗口，避免末段刚结束就立即补规划。仅跟踪已批准样条的真实零速度终点，最新地图、跟踪距离、体积、视野与方向限制仍检查；超时回到保守等待/重规划。不会按5cm误差向量触发转向，也不制造显示专用路径。
- 新增latched状态/drone/navigation_terminal_stage；Qt其他操作保持。末端样条取值缓存减少重复构造。`catkin_make -j2 -l2`最终编译退出0，Python AST、launch XML、diff格式及部署源码/二进制指纹核对通过。
- 本轮未飞行：重启前只读确认connected=true、armed=false、landed_state=1，然后停止旧launcher2199255及gzclient2200161以编译部署。本次不宣称飞行验收通过；用户手动实验仍需验证短末段、弯道末端、等待/转向进入到达范围、顺序队列与取消。
- 修改与构建证据目录：start/qt_work/terminal_convergence_20261005。此前规划检查超预算、Qt失败状态保留等未选方案仍未修改。历史“尚未开始管理器修改”记录只代表上一次保存时状态。

- 当前运行：`start/logs/20261005_174011`；launcher PID2207394、Qt PID2207468、管理器PID2207604、EGO PID2207525、Gazebo客户端PID2208289。sim65.128只读确认READY/HEALTHY、connected=true、armed=false、AUTO.LOITER、landed_state=1，终点状态IDLE；Qt/Gazebo窗口、双相机/点云/体素正常，未发飞行指令。部署指纹与启动结果见证据目录deployment.json、startup_ready.json，构建日志build_final.log。
- 实现备份：`start/checkpoints/terminal_convergence_implemented_latest.tar.gz`；保存进度后重新打包当前未提交改动，未推送GitHub。下一步由用户手动飞行并观察终点收敛是否减少SPACE_WAIT与重复转向，不能将本轮地面启动检查算作飞行通过。

## 2026-10-05 终点收敛改造进行中：用户要求保存并暂停

### 当前状态

- 用户最新授权范围：增加“终点接近与收敛”、提前减速与平稳停止；等待/规划/转向期间都检查到达。仍保持三维球半径15cm、速度<=0.15m/s、连续1秒，碰撞/已观测自由空间/±60度朝向及健康保护保持。
- **只完成EGO侧第一轮源码修改，尚未编译、尚未部署、尚未飞行验证；管理器修改尚未开始。** 用户额度不足要求先保存，到此暂停实施。
- 当前运行仍是`start/logs/20261005_163410`对应的连续朝向/天花板/多点队列版本，launcher PID2199255、Qt PID2199336、Gazebo客户端PID2200161。最后手动实验已落地上锁；只读观察PID2200755/2200756已停止。恢复时重新核对实时状态，再决定部署，禁止把在磁盘上的半成品当作当前已运行功能。
- 修改前GitHub基线89e79fa2c65abfe8bb13a7bbac7802d170a7a8a3，当前工作区还包含该基线之后先前功能与报告，均未提交/推送。本次备份覆盖所有当前改动文件和未跟踪服务/报告。

### 本轮已写入的源码（待审查、编译）

1. `src/drone_stack/launch/stack.launch`添加共享参数：终点接近距离0.8m、末段速度0.2m/s、减速度0.4m/s²。
2. `ego_replan_fsm.h/.cpp`：计算到终点的剩余全局曲线弧长，避免隔障碍的近距离误判；按制动距离扩大接近距离，接近入口主动触发一次真实EGO重规划；末段速度限制保留当前起始速度以确保制动连续性；末端0.15m与0.15m/s条件；短末段目标允许>=0.02m。
3. `planner_manager.h/.cpp`：`reboundReplan`与`refineTrajAlgo`加terminal_stop参数（默认false）；只有终点收敛轨迹将原0.20m拒绝阈值降低到0.02m。沿真实全局几何曲线用五次弧长时间轮廓初始化末段；固定三次样条起点位姿/速度/加速度，末三控制点精确设为目标，从而终点位置精确、速度/加速度为零；时间重分配后的优化同样恢复精确边界。沿用完整连续碰撞检查，不生成独立显示轨迹。

### 下一次必须继续的工作

1. 审查EGO第一轮改动：五次弧长轮廓在较大起始速度与极短距离时是否单调/物理可行（当前采样夹到单调长度并不能代替可行性验证）；检查固定边界在时间重分配后是否还满足速度/加速度上限、接口签名和头文件；terminal_entry_planned标记与重复会话/替换路线的关系。不要因未编译而声称功能完成。
2. 修改`flight_manager.py`：将到达判断移动到NAVIGATING的OFFBOARD/地图/目标障碍安全门之后、PLANNING/SPACE_WAIT/TURNING等提前返回之前；全部内部阶段统一计时，取消begin_heading_alignment对到达计时的无条件重置。到达确认时冻结已测得的到达位置/当前实际yaw、继续发送OFFBOARD悬停；防止规划回调在1秒确认期间覆盖状态。任务切换/取消正确清理计时，正常到达继续保留并分派剩余队列。
3. 完成管理器“接近/收敛”状态（可独立terminal_stage字段/话题，避免直接更名TRACKING影响全局节点现有分支），与EGO末段匹配。在终点曲线结束后，如果末端确实是目标、当前到目标跟踪段通过最新障碍/自由空间/体积检查，可短时持续跟踪同一批准样条零速度末端以让位置控制收敛；设置有界收敛时间，失败才重新规划，避免直接进入通用2秒SPACE_WAIT。若需要新轨迹或超出安全视野，仍由真实EGO曲线与原保护处理；不要重新引入5cm位置误差方向触发转向。
4. 到达检查与回调并发/序列失效要保持30Hz目标连续性；检查goal_reached_since存在时旧样条/旧ACK不会取消已稳定的到达，位置离开容差或健康安全门失效应正确处理。
5. 完成后编译、语法检查与进度说明；用户本轮没有要求自动飞行，不自动起飞。仅在确认落地未解锁后部署/重启供手动实验，若需飞行验证等待相应授权。原“前向校验超预算取消”、Qt异常提示、未来时钟警告等其他方案本轮未选，不擅自加入。

### 保存位置

- 工作区所有源码已落盘，进度文件为本文件。
- 本次源码备份：`start/checkpoints/terminal_convergence_wip_20261005_172434.tar.gz`；对应目录包含git_diff.patch、保存元数据和源码校验和。该压缩包是相对Git基线的工作中修改文件集合，不含完整依赖或新编译产物。
- 最新本轮备份链接：`start/checkpoints/terminal_convergence_wip_latest.tar.gz`，独立于历史完整仿真检查点。
- 实验复盘：docs/simulation/2026-10-05/queue_yaw_experiment_review.md、queue_yaw_experiment_metrics.json；原始手动数据start/qt_work/manual_queue_yaw_20261005_164041。

## 2026-10-05 手动两组多点实验结束复盘，等待用户补充方案

- 用户已结束实验；只读观察PID2200755/2200756已停止，仿真/Qt保留。最后已记录READY、未解锁；降落来自用户sim370.844，375.820 READY。
- 两组三点共6项，5项完成；最后项前向校验超预算取消。第二组第二点50.444仿真秒，仅18.798秒TRACKING，3次SPACE_WAIT、2次TURNING；终点等待/重规划问题确认。277次紫线同源采样误差0，参考进度无回退；最小几何约束201.61，无健康HOLD/SEVERE。最高实际Z1.586m、最后项全局参考最高Z2.075m，天花板2.5m未调整，尚未完整验收。
- 完整证据/方案：docs/simulation/2026-10-05/queue_yaw_experiment_review.md，统计queue_yaw_experiment_metrics.json；原始数据start/qt_work/manual_queue_yaw_20261005_164041。问题包含终点收敛、转向前后曲线一致性及曲率限速、检查缓存/线程与三态结果、Qt异常状态保留诊断、黄色参考安全前段、少量未来时钟样本处理。
- 本次只分析日志、保存报告及停止只读观察，没有修改控制源代码、参数或启动新飞行。方案等待用户补充。

## 2026-10-05 16:50:43 手动队列实验最后一项安全检查超预算取消

- 第一组三点全部完成；第二组前两点完成，最后目标[-5.894250,0.524309,1.2]执行中sim328.012前向可执行曲线检查返回`Full EGO trajectory validation exceeded its bounded budget; executable lookahead`，管理器取消任务、进入HOLD、清空队列；Qt据空队列删除红点和绿线，规划节点删除黄线，执行曲线清空紫线。用户见到目标/线路消失的直接原因已确认。
- 取消后距最终目标约5.19m，未到达；仍armed=true、悬停，未触发降落。该错误证明计算预算保护触发，不能证明轨迹碰撞或LIO失效。检查限额为wall0.25s/threadCPU0.05s/10000节点，本次错误没有记录具体越界项。
- 详细事件`start/qt_work/manual_queue_yaw_20261005_164041/cancellation_328012.json`，原始events/reference_audit/telemetry继续记录。仅分析，没有修改控制代码或发送指令。后续需分析前向检查耗时并区分故障取消与到达在Qt的显示提示。

## 2026-10-05 16:41 用户手动多点实验只读监测已启动

- 仿真继续`start/logs/20261005_163410`，不重启、不修改控制程序、不发送飞行指令。
- 记录目录`start/qt_work/manual_queue_yaw_20261005_164041`，monitor PID2200755、reference_observer PID2200756；持续记录队列/天花板、健康、几何约束、OFFBOARD间隔、FCU/LIO/真值、全局进度、真实紫色曲线与样条一致性，以及实际/指令朝向相对同一执行样条切线的偏差。样条缓存仅保留32条，未录整张点云/图像/rosbag。
- 监测接入时已NAVIGATING/TRACKING，用户已提交三个1.2m高目标：[3.298058,-0.000580]、[5.100377,-2.527462]、[1.710939,-5.134988]。第一点开始sim171.856，早于监视接入sim185.866；更早的详细轨迹不能凭本次CSV还原，原始stack/PX4日志仍保留。初始最高Z为2.5m。
- 该节是持续观察起点，不是最终验收结论；后续读取目录事件和采样给出结果。参考观察日志可能出现两个线程的JSON对象紧邻在同一行，分析时用JSONDecoder.raw_decode逐个解析；reference_audit.jsonl单次写入不受打印交错影响。

## 2026-10-05 连续朝向、可调最高高度、多点顺序队列已部署

- 用户请求的三项改动已实现：机头在移动中按获准执行的真实EGO样条有向XY切线连续跟随，最多30度/秒；保留起步悬停对准和超出±60度后的刹停转向；不使用位置误差方向。低水平速度/纯垂直段保留上一朝向。
- Qt右侧飞行操作新增“最高Z（ENU）设置”，0.8～2.5 m、默认2.5 m；相对起飞高度仍独立。最高高度是机体中心绝对ENU Z，与点选Z同基准。起飞终点、全局路线、整段EGO批准曲线、执行点均受设置值减0.20 m裕量限制；实际超过上限0.05 m时触发降落保护。未解锁地面或无队列的健康OFFBOARD HOLD可更改，禁止降到当前高度加0.20 m以下。动态修改清除旧虚拟天花板整个平面，按真实障碍膨胀引用计数恢复该层占据，再加入新层，防止合成障碍残留。
- `/drone/queue_goal`接收连续目标，管理器最多保存100点（含当前）；Qt持续点选、红点编号和剩余数量，第二个及以后不发布给全局/EGO规划。当前点到达原15cm/0.15m/s容差并保持1秒后移除；HOLD过渡0.30秒后重新验证最新地图，再发布/规划下一点。未知区域可加入，已知膨胀障碍拒绝；下一点新发现被占据时取消剩余队列并HOLD。取消、悬停、降落、健康HOLD取消均清空队列，包括两个目标之间HOLD窗口内的健康异常。
- 黄色全局路线、紫色真实控制曲线只对应当前目标；排队点之间不生成显示专用路线。绿色实际轨迹保留到整次队列结束，最后完成清空。再次点击“结束连续点选”只是退出点选模式，不取消已排队任务。相对目标也加入同一队列，旧`/drone/local_goal`接口保留。
- 已通过`catkin_make -j2 -l2`编译、Python AST语法、launch XML解析、`git diff --check`。最终源码/二进制指纹、编译日志、界面截图和只读启动结果位于`start/qt_work/yaw_ceiling_queue_final_20261005_163410`。初次启动检查发现旧天花板残留风险后补修并重新编译/部署，初次运行不是最终版本。
- 最终运行`start/logs/20261005_163410`；launcher PID2199255，Gazebo客户端PID2200161。READY/HEALTHY已在sim70.228确认，connected=true、armed=false、landed_state1，队列为空；Qt与Gazebo窗口正常，两个新增service类型已确认。旧只读监视PID2191192/2191193已停止；本轮没有发授权、ARM、起飞或导航指令，没有执行自动飞行实验。
- 本轮为实现、编译与地面启动检查，**没有飞行验收结果**。下一步由用户手动测试弯道中连续朝向、低天花板下绕柱、连续多点顺序执行和中途取消。此前末段曲线结束后的SPACE_WAIT补规划、转向后新曲线方向变化问题尚未新增专用收敛/一致性机制，不能宣称终点停转全部解决。
- 运行说明见`src/drone_stack/README.md`新增章节。GitHub基线仍为89e79fa2c65abfe8bb13a7bbac7802d170a7a8a3、tag baseline-20261005-before-yaw-terminal-fix；本轮新增代码尚未提交或推送。

## 2026-10-05 GitHub修改前基线保存

- 按用户要求保存当前全部程序改动、配置和进度到origin/main，并建立baseline-20261005-before-yaw-terminal-fix标签，供后续修改前回退。
- 实验结论摘要纳入docs/simulation/2026-10-05/；完整Qt运行日志、录包、源码临时备份仍保留本机，start/qt_work与start/cleanup_records加入忽略项。
- 仍保持现有“起步转向、移动维持朝向、超过±60度悬停转向”逻辑。尚未实施移动中连续朝向跟随、终点收敛阶段或新曲线转向一致性修复；已知问题详见手动实验分析。
- 本次为版本保存，不代表全部安全验收通过，也没有改动运行中的控制算法。

## 2026-10-05 15:30 手动三目标问题分析（未修改程序）

- 首两目标sim146.938/188.704到达；第三[-3.854017,-0.120992,1.2]未完成，sim250.406用户请求降落、256.044落地。无健康保护降落记录。
- 首点局部末段结束时还差0.221m，进入2秒SPACE_WAIT再规划/转向，缺少终点继续收敛机制。第三sim231.056更新膨胀体素阻挡前缀，234.510 EGO紧急停止恢复；三维全局换路高度1.625再1.875m，PX4收到高度目标最高1.93360m，故确实由规划/控制指令升高，不能归咎LIO高度失效。
- 第三转向44.88度后立刻用刹停新边界重规划，新曲线方向116.28度，相差71.4>60，再次转向。主要问题是转向前后没有固定同一条将执行的曲线，加上终点水平很短/高度差约0.4m仍走完整水平转向。
- 黄线sim231.040~231.272短暂缺失，247.320再次清空至降落前无恢复；仅能确认无可通过检查的参考前段，缺少地图/实际样条快照与具体拒绝原因，不能证明实体障碍或地图鬼影。
- 全程几何最小101.94>30；控制已记录最大间隔0.140sim秒，356次紫线匹配误差0，参考进度无回退，曲线最大密集采样角3.982度。之前两点通过不足以代表这轮第三点通过。
- 完整分析及待选改进start/qt_work/manual_experiment_20261005_152102/ANALYSIS.md，指标与ULog高度对照同目录。仅写分析记录，未改算法或发飞行指令；只读观察仍运行。

## 2026-10-05 15:21 手动实验实时观察启动

- 用户手动实验，代理仅监测；不发飞行指令、不重启、不修改算法。
- 轻量日志start/qt_work/manual_experiment_20261005_152102，monitor PID2191192、reference_observer PID2191193。记录健康/告警/几何约束/位置真值/OFFBOARD间隔/全局路线进度与紫色执行曲线同源对照，无整图录包。
- 仿真仍start/logs/20261005_151634。首个手动目标[5.269694,-0.178527,1.2]最终sim146.938到达HOLD；此前sim135.772还差0.221m时局部曲线结束，SPACE_WAIT保留任务、补规划并转向-106.70度后完成，定位健康，无保护降落。第二目标[-6.636519,6.056249,1.2]sim151.354提交，正在执行。部分审计128次紫线误差0、进度无回退；observations.json记录发现，后台观察仍运行。

## 2026-10-05 15:16 用户手动实验前完整重启

- 按用户请求，在确认armed=false、landed_state1后停止上一套运行，重启仿真/PX4/ROS/Qt/Gazebo。
- 当前日志start/logs/20261005_151634；独立launcher PID2189992、gzclient PID2190911。Qt/Gazebo窗口已确认打开。未启动自动飞行脚本，由用户手动操作。
- 启动观察结果记录start/qt_work/manual_restart_20261005_1516/ready.json；若尚无该文件则定位初始化仍在进行。

## 2026-10-05 01:20 最终部署两目标回归通过，Qt/Gazebo保持打开

- 最终运行start/logs/20261005_011035，独立启动器PID2170199、Gazebo客户端2171136。飞行复现已结束，只读观察进程已停止；当前READY/HEALTHY/armed=false/landed_state1，紫线已清空，无后续自动飞行指令。
- 最终源码包含单调参考进度、碰撞检查的真实平滑全局曲线、EGO整段曲线初始化及参考跟随代价、紫色同源执行曲线、显式黄/紫/红渲染顺序，以及真实浮点射线遍历、按实际距离缩短终点接入控制柄与最大4度Bezier采样。
- attempt_03自动相对起飞1.2m、0.5m/s依次到达[5.330093860626221,-2.9184792041778564,1.2]和[-0.7135858535766602,-5.078310966491699,1.2]，误差0.09932/0.04668m、导航耗时23.890/26.330sim秒。只有每次起步必要转向；无SPACE_WAIT/健康HOLD/保护降落。测试结束主动降落上锁。
- 173次紫线采样与实际执行B样条同一时刻匹配误差0；路线进度无回退；全局密集曲线最大相邻方向变化3.96417度（终点小回弯问题不再重现）；最大OFFBOARD目标间隔0.130sim秒。
- 1516个Gazebo真值位置+完整姿态采样，以保守机体半包络[0.32,0.37,0.18]m再加0.03m检查，未发现与场景障碍包络重叠；最小分离轴间隙0.24686m，pillar_a。此数不是欧氏距离，采样不等同连续碰撞证明。
- PX4 ULog17_10_53.ulg融合审计通过：65个空中状态样本全程EV位置/高度/航向，EV速度/GPS/气压计/测距/光流/磁融合计数均0，EV_CTRL11/HGT_REF3/GPS_CTRL0/BARO_CTRL0。没有手工在ROS端重复转换NED。
- 构建通过，7项本次平滑参考单测、23项连续曲线保护单测、1007个解析射线面穿越对照通过。较早test_flight_safety.py的75项尚不兼容当前状态机：原测试66个旧fixture字段错误/2失败/1跳过；单独工作副本改为完整初始化后仍16项旧契约预期不匹配（如未知空间保留任务SPACE_WAIT、忽略旧样条、地图单调戳、异步规划、到达捕获）。未改生产测试来掩盖结果，不能声称全部安全验收通过；后续应逐条对照现有需求更新测试契约并复核保护时序。
- 详细记录start/qt_work/smooth_route_20261005/attempt_03/{verification,flight_result,geometry_audit,fusion_audit}.json、reference_audit.jsonl、truth_envelope.csv。实际Qt紫线/红箭头图像保存在attempt_02/qt_local_curve.png（Qt二轮/最终同一二进制）。第一次实验中断记录及第二轮成功数据保留。raycast.cpp编译后仅恢复原CRLF换行，运行代码语义不变，指纹说明line_ending_preservation.json。
- 可验证结论：用户记录的两目标和本次修改回归通过；没有宣称所有场景、所有安全故障或实机验收完成。

## 2026-10-05 01:04 两点复现通过；射线与终点连接进一步修正，待最终重测

- attempt_01首点到达，第二段反复SPACE_WAIT（未观测前缀），sim126.056启动器收到停止信号退出130，ROS/PX4/Gazebo整体结束；无法记录降落完成，不将此轮算通过。日志与interruption.json保留。改为独立进程启动，避免实验生命周期受命令会话限制。
- attempt_02使用同一已部署算法，从初始场景起飞1.2m，依次到达[5.330094,-2.918479,1.2]、[-0.713586,-5.078311,1.2]。到达误差0.08809/0.04393m，耗时24.208/26.044sim秒，未出现SPACE_WAIT或异常保护降落；最终landed1/未解锁。最大控制目标间隔0.120sim秒；173次紫线起点与同一B样条同一时刻对照误差0，参考进度无回退。记录start/qt_work/smooth_route_20261005/attempt_02/flight_result.json，Qt真实紫线/红箭头截图qt_local_curve.png。
- 仍不能宣称最终全部通过：日志量化显示终点处短接入出现118.58度采样拐角，固定最小0.08m控制柄在仅几厘米连接上产生回弯。源码移除最小柄长，按连接实际距离/3设置；Bezier距离采样外增加最大4度角度采样及端切线检查，超过1024段不能满足则拒绝。新增短接入测试，七项Python检查通过。
- 源码发现RayCaster用取整体素差作为射线方向，已用解析体素面穿越对照复现：示例26个真实体素漏9个、误记10个。类遍历改用真实浮点方向，零方向设无穷、负方向面边界正确、同时跨越重合体素面以避免仅触碰边角的零长度空闲票。示例修正后无漏记/误记；此缺陷是已证实的地图几何错误，尚未证明是attempt_01停滞唯一原因。不会扩大空闲区域或降低已观测安全门。
- 当前运行start/logs/20261005_005603，已落地READY；独立进程PID见attempt_02/pids.json。射线修正与终点细化源码正在构建，部署后重跑两目标，尚未算最终验证完成。

## 2026-10-05 全局曲线、路线进度和紫色实际局部轨迹（正在复现用户目标）

- 用户授权修复、重启Qt，并在最终部署后自动按其目标点做仿真实验。记录的目标为[5.330093860626221,-2.9184792041778564,1.2]和[-0.7135858535766602,-5.078310966491699,1.2]，默认0.5m/s、相对起飞1.2m；额外目标已询问，暂无补充。
- 全局路线以不可变版本和单调弧长进度维护，不再反复把机体接回已越过拐点。每次地图改变需要换路时，从当前位置安装新版本；/drone/global_route_progress记录task/version/arc供审计。
- 全局A*结果每个拐弯采用切线连续的三次Bezier连接，经连续碰撞和离散闭合体素检查，缩小圆滑范围仍不能安全时拒绝，禁止发布尖角备用路径。机体偏离时采用经检查的平滑前向接入。相同密集曲线同时发布到Qt与EGO，最多2048点。
- EGO使用完整曲线前向片段作为B样条初值，目标速度沿曲线端部切线，取消旧15度拐点停步及小于0.2m跳样。rebound优化增加全局参考曲线跟随代价，避免仅初始化后又被拉直；局部避障/可行性和连续检查保留。
- 初次部署后的用户手动飞行暴露优化反复失败：全局只检查实际膨胀体素而EGO另检查0.05m余量，且rebound无参考约束。最终源码将全局查询增加1个0.1m邻接体素的保守余量（覆盖既有0.05m检查，原地图膨胀参数不变），采用按查询缓存而非再扩展整张地图；优化全局参考权重8，严重保护未降低。该手动飞行在用户请求降落后READY/未解锁。
- 飞行管理器在执行目标真正下发后，直接采样同一approved_curve发布/drone/executed_local_path；暂停/转向/取消清空，不另建显示规划器。Qt紫线直接读取它，显式Ogre渲染队列95黄/96紫/97红朝向箭头，关闭这些层的深度遮挡，避免按高度伪装图层。
- 最终catkin_make通过；新增test_smooth_global_path.py六项离线检查通过，覆盖平滑角度、前进进度、已过节点、碰撞拒绝、余量查询、紫线来源一致。
- 当前最终运行start/logs/20261005_004301，launcher39479/Gazebo27959/飞行复现63173。轻量记录start/qt_work/smooth_route_20261005/attempt_01/。自动等待65秒初始化后解锁、起飞、依次两目标、最后降落；尚未宣称飞行通过。部署指纹deployment.json，源码备份与编译日志同目录。

## 2026-10-04 健康快照与障碍清除修复（用户手动飞行）

- 用户授权修复两项并重开仿真/Qt，明确禁止代理自行开始飞行实验。
- LIO发布带采样时间、位姿时间、几何时间、valid/quality/severe的统一JSON健康快照 `/drone/lio/health`；管理器只从该快照更新LIO有效性和质量，旧Bool/String仅兼容显示。
- 管理器统一判定LIO/FCU数据新鲜度和EKF有效标志：短暂异常WARNING，持续0.5仿真秒取消任务/HOLD，持续1秒降落；LIO严重锁定/时钟倒退/连接丢失仍立即保护。HOLD使用最后健康位姿，持续发布OFFBOARD目标；恢复后不自动恢复已取消任务。Qt从 `/drone/flight_health_snapshot` 一次更新有效性和颜色，降落保护期间红色锁定至上锁。
- EGO地图新增原始障碍观测证据，每个体素每帧只更新一次；新命中立即阻挡，重复命中增强确认，同帧命中优先于空闲射线。至少两次独立空闲扫描可清除旧障碍；回波周围0.15m量化保护带不作空闲穿越，回波后方/未观测盲区不清除。
- 膨胀层用当前原始占据体素的精确引用计数维护：原始障碍删除时撤销其膨胀贡献，其他障碍共同覆盖部分继续保留。效果等同从当前原始地图重建，避免每帧整图膨胀开销。水平0.4m/竖直配置0.25m不变。
- 备份与构建日志：start/qt_work/health_map_fix_20261004/。catkin_make退出0，Python语法通过；Gazebo/Qt窗口已打开，sim65.092秒确认READY/HEALTHY/connected=true/armed=false，安全体素消息持续更新。没有开展飞行实验。
- 最新运行 start/logs/20261004_233130，launcher会话51206、Gazebo GUI35390。健康判定排除地面65秒初始化等待，防止将初始化误报为严重健康异常；因此管理器在未解锁状态下单独重启到会话16670，日志 start/qt_work/health_map_fix_20261004/manager.log。下次停止需同时停止该独立管理器和Gazebo GUI。最新源快照已更新，部署哈希/只读启动状态保存在deployment.json/startup_status.json。此前连续导航验收仍未全部通过，不能将本次修复当成飞行验收通过。

## 2026-10-04 23:10 统一路线修改完成并验证；两轮连续飞行均受健康门中断，已落地

- 有效运行start/logs/20261004_225127，路线改造全部加载：权威navigation_goal → 全局3D A* GlobalRoute → EGO折线局部目标/优化 → 执行器；黄线同源，独立全局多项式输入关闭。转向/重试保留任务纳秒ID、过滤旧任务和过期/部分路径，全局节点不再消费EGO原始曲线。编译/Python/节点连接通过。
- 36个局部目标到当前黄色折线最大差9.16e-16m。首轮两个目标到达误差0.14795/0.06490m，第三反向已沿黄线执行至绕行拐点附近，旧规划卡住未重现；但sim140.524 LIO valid=false触发保护降落，约束状态瞬间UNKNOWN后HEALTHY，而valid滞后恢复，失效根源未完全归因。
- 减载对照仅关闭gzclient，Qt/仿真/传感器/算法/门槛保持；从前轮落点重新飞三目标，第一到达误差0.10058m，第二sim236.956 estimator_ok=false触发降落。故不能宣称两轮连续三目标通过或Gazebo渲染为唯一根因。没有放宽安全门。
- 两轮均确认未解锁/landed_state1/READY，maxsetpointgap0.136/0.126sim秒。测试驱动已退出；对照后Gazebo渲染窗口恢复（68756）。launcher91979、只读观察99219、路线审核20177持续。当前机体地面在约[-0.44,-1.90]附近，实时状态需再读。
- 记录start/qt_work/manual_monitor_20261004_unified_route/CHANGES.md与verification.json，首轮flight_result.json、retry_no_gazebo_gui/flight_result.json分别保留失败原因；source/deployment/参数/节点连接/CSV齐全。飞行探针finally已修正，保护正在降落时不重复land覆盖原因，未修改本次实验结果。
- 下一项为健康时效/valid与quality发布时序、PX4 EKF状态瞬态诊断；本次用户授权路线统一已完成，不将健康保护取消当作修复。

## 2026-10-04 23:05 统一路线首轮：前两目标通过，反向路线已执行但健康失效保护降落，低负载对照进行中

- 同运行start/logs/20261004_225127，source/build未再改动。第一/二目标到达误差0.14795/0.06490m；第三反向目标已沿统一路线移动至绕行拐点附近（约[2.7,-1.3]），没有旧规划卡住问题。
- sim140.492 LIO quality短暂UNKNOWN、valid=false，同一sim恢复HEALTHY但valid至140.592才恢复；sim140.524执行器保护降落。没有位姿跳变/退化锁存日志，时效判断具体触发源未完全归因。所有安全门保留，不宣称首轮三目标整体通过。
- 首轮自动落地READY/未解锁/landed_state1，maxsetpointgap0.136sim秒。证据根目录flight_result.json/monitor.log。
- 地面确认后仅关闭Gazebo渲染窗口（gzclient1802450）降低负载，gzserver/PX4/传感器/算法/Qt与阈值保持；从落地点重新重复三个目标作对照。记录retry_no_gazebo_gui，飞行会话86356；parent只读监测99219和路线目标审核20177持续，launcher91979。尚未出对照结论。

## 2026-10-04 22:55 统一全局路线已部署，三目标复测准备中

- 用户授权统一黄线与EGO执行参考：管理器发布/drone/navigation_goal，独立3D A*生成/drone/global_route（task_id+Path）并同源显示黄色路线；后台新规划确认包含同一完整路径，EGO局部终点沿该折线选择，遇明显拐点停在拐点，不再从起点到最终目标另造全局多项式。EGO内部A*/B样条优化继续局部避障及动力学处理。
- 全局路径不再订阅原始EGO曲线，避免循环依赖；Qt目标话题只作界面标记，不能修改权威全局任务。独立任务ID用提交时ROS纳秒戳，避免Header.seq在ROS发布时被自动改写；转向/SPACE_WAIT保留原任务戳，旧任务路径拒绝。
- EGO当前任务订阅完整路径更新，过期/部分路径停止生成新局部目标；局部曲线结束但最终目标未到达则从当前位置继续下一段。安全审批/持续OFFBOARD/姿态规则/融合参数保留。
- Qt/EGO/traj_server构建成功，Python语法解析成功；新运行start/logs/20261004_225127，launcher91979/Gazebo60729/只读观察99219/就绪7562/飞行45031。准备起飞连续三个目标，第三为旧故障反向[-2.214404,-3.188609,1.2]，尚未宣称飞行通过。
- 记录start/qt_work/manual_monitor_20261004_unified_route。飞行探针过早在/use_sim_time设置前启动已停止、在地面重新启动使用仿真时间；旧运行在已落地未解锁确认后停止。

## 2026-10-04 22:24 三项修改完成，连续两个目标仿真回归通过，已降落READY

- 最新有效运行 start/logs/20261004_221414。旧轨迹清理/确认接口、删除转向后0.25s延时与Qt等待原因、黄线分批搜索与保留安全前段已编译部署。
- 起飞1.2m后连续到达[-2.520226,-2.153642,1.2]与[5.215438,1.191749,1.2]，最终误差0.1240/0.1299m，含转向耗时14.576/29.056仿真秒。第二目标原旧轨迹恢复故障触发过程通过，未出现跳变取消或未经请求降落。
- 4次新会话曲线起点与当前机体最大差0.00165m，确认到曲线0.002～0.008仿真秒。武装期间最大setpoint采样间隔0.148仿真秒，OFFBOARD连续保持。
- 黄线移动中持续可用，到达后清理；转向保留/障碍截断分批A*通过focused regression。在线黄线记录从第二目标移动中开始，未注入动态障碍、未做Qt鼠标端到端验收，不扩大结论。
- 测试主动/drone/land后确认未解锁、landed_state=1、READY。当前飞机地面在第二目标附近，Qt/Gazebo仍打开；launcher10499/Gazebo39476/只读监测20750；飞行59684退出0。
- 记录 start/qt_work/manual_monitor_20261004_planning_resume/CHANGES.md、flight_result.json、verification.json、regression_result.json、编译/参数/部署哈希/轻量CSV。EV_CTRL11/HGT_REF3/GPS0/BARO0、LIO30/.5sHOLD/1sLAND、ENU、±60度、0.75m跳变门保留。
- 自动回归结束后外部再次ARM/起飞（22:23:11/22:23:20），22:24:02进入HOLD。最后地面断言采样与外部起飞相遇，断言退出1，仅只读确认脚本，未发送控制命令；不影响已完成自动回归的ground_unarmed=true证据。最新状态需读实时日志，不能假定仍在地面。
- 后续用户可通过Qt点选导航，重点观察转向期间黄线、安全前段与完整新线替换、导航等待原因显示。当前验证覆盖本次改动与两目标复现，不能据此宣称全部仿真/实机验收完成。

## 2026-10-04 22:18 三项修改已部署，连续目标回归正在进行

- 用户“你开始把”授权按确认方案修改、完整重启与仿真复测。旧机先降落确认未解锁/landed_state=1再停止。
- 新增 /planning/start 确认接口：目标/速度/当前起点一起准备；旧local_data曲线及global局部拼接清理，collision回调仅允许当前有效EXEC/REPLAN，恢复不再从INIT追旧曲线；收到ACK后才enabled=true。网络调用在后台串行线程，30Hz悬停发布不阻塞。新会话曲线起点误差超过0.25m拒绝，原0.75m位置跳变门保留。
- 删除转向结束后的固定0.25s目标话题延时，保留朝向/速度/定位稳定与新扫描条件；Qt显示等待原因。
- 黄线3D A*按25ms线程CPU分批续算；转向暂停不推进旧局部曲线，已有路线每轮按最新地图重验，遇障碍仅保留连接的安全前段，完整替代路线通过最新地图后原子替换；地图/位姿失效仍清线。
- Qt/EGO/traj_server编译成功，Python解析和生成服务导入成功；focused regression通过（绕障跨2批/4路线点；被障碍截断到x=-0.15；转向清局部曲线但保留显示参考线）。
- 新运行 start/logs/20261004_221414；launcher10499/Gazebo39476/观察20750/readiness96793/飞行回归59684。记录 start/qt_work/manual_monitor_20261004_planning_resume。当前尚在启动就绪等待/飞行回归执行过程中，不宣称连续两目标已通过。

## 2026-10-04 21:45 手动实验：首目标到达，第二目标转向后旧曲线重规划触发位置跳变保护

- 本阶段只读日志，未改源码/参数/节点，未发送控制命令。
- 第一目标[-2.52023,-2.15364,1.2]首次朝向-136.02度，21:42:31进入TRACKING，21:43:17无flight_error转HOLD（达到到达门）；没有此前的反复转向。
- 第二目标[5.21544,1.19175,1.2]，朝向9.00度后21:44:12完成转向；21:44:13触发Trajectory setpoint jump exceeds tracking limit，导航取消HOLD，未记录LIO/EKF失效或自动降落。
- 已定位EGO状态缺陷：禁用规划保留local_data_旧曲线；转向后重新enable时还处INIT、尚未收到延迟0.25s的新目标，checkCollisionCallback只排除WAIT_TARGET，仍能检查旧曲线并planFromCurrentTraj，日志SAFETY: INIT -> EXEC_TRAJ。
- 该回调按旧曲线已经前进的时间取起点[0.753,-1.29,1.15]与速度[0.51,0.155,0.00637]，实际飞机约[-2.516,-2.193,1.074]仍悬停，起点水平差约3.39m。新的源码曲线虽有新时间/编号，却不属于当前悬停状态，因此被0.75m跳变门拦住。
- 后续需在规划禁用/任务代次切换清理旧local_data_，碰撞回调仅在有效当前任务的EXEC_TRAJ/REPLAN_TRAJ运行，INIT/WAIT_TARGET/GEN_NEW_TRAJ禁止恢复旧曲线，并加强新曲线起点连续性审批。不得通过放宽0.75m安全门解决。尚未实施。

## 2026-10-04 21:39 路线有向切线朝向修复已加载，地面READY

- 用户要求已实施：朝向只取实际审批EGO局部曲线递增参数的一阶导数XY有向切线；新曲线起步向前约10cm弧长找切线，正常运动取当前执行参数处切线；竖直/退化/结束保留航向。
- EGO从静止开始的GEN_NEW_TRAJ在<=0.1m/s采用零速度边界，避免悬停估计残速折弯起步路线，运动中的重规划连续性及EKF/IMU反馈不改。没有审批曲线保持悬停重试，不按直指目标方向转向。切线/弧长缓存每条曲线重建一次。
- 旧机通过/drone/land降落，确认未解锁且landed_state=1后完整重启；EGO编译与Python语法解析完成。新运行start/logs/20261004_213408，部署执行器hash一致、Qt/Gazebo窗口存在。
- 最终启动状态：{"sim": 65.02, "phase": "READY", "lio_valid": true, "lio_quality": "HEALTHY", "connected": true, "armed": false, "ekf_valid": true, "path_odom_frame": "odom", "idle_command_count": 0}
- 会话启动16350/Gazebo30020/只读监测61250持续运行，就绪85961完成退出0；PIDs/源码前后备份/编译日志/参数/轻量CSV记录在start/qt_work/manual_monitor_20261004_curve_tangent。
- ±60度超界悬停转向、LIO支持30/0.5s HOLD/1s LAND、ROS ENU及EV位姿融合/禁速度、Qt速度设置保持。未运行本版本自动飞行或测试套件，反复转向修复仍待用户手动飞行复测，不宣称已飞行验证通过。

## 2026-10-04 21:37 路线切线朝向已部署，等待READY

- EGO编译、Python语法解析完成；新运行start/logs/20261004_213408，执行器部署源码hash一致，Qt/Gazebo窗口已打开。
- 航向使用实际审批EGO曲线递增参数的有向切线，初始零切线时向前约10cm弧长选择切线；瞬时指令速度/漂移方向不参与朝向。EGO静止起步采用零速度边界，运动中重规划保持连续。没有曲线时悬停重试。
- 当前未解锁，LIO HEALTHY/EKF有效，等待65秒仿真启动保护。±60度超界转向及安全/融合/Qt设置保持，PX4参数读取正确。
- 会话启动16350/Gazebo30020/只读监测61250/就绪85961，PIDs和运行记录在start/qt_work/manual_monitor_20261004_curve_tangent。尚未进行本版本飞行复测，监测持续。

## 2026-10-04 按实际规划路线有向切线修复，编译/部署中

- 用户要求按规划路线有向切线决定朝向，已实现Bspline一阶导数XY方向，取消所有瞬时指令速度/漂移速度方向来源；新曲线起步向前约10cm弧长找有效方向，移动阶段取当前曲线切线，竖直/结束保持航向。
- EGO从静止开始的GEN_NEW_TRAJ在<=0.1m/s采用零速度边界，避免FCU微小残速折弯初始路线；运动中重规划及EKF速度反馈不改。没有审批曲线不再直指目标转向。
- 原机空中HOLD已请求降落，确认未解锁且landed_state=1后停止旧栈，准备完整重启；不在飞行中替换节点。
- 记录start/qt_work/manual_monitor_20261004_curve_tangent，含修改前备份/构建日志/监测。±60度、LIO30/0.5HOLD/1LAND、ENU/EV禁速度、Qt调速保持。
- Python语法解析成功，EGO编译中；尚未飞行复测，不宣称本问题已验证消除。

## 2026-10-04 21:21 手动实验：起步残余速度导致反复转向，待修复

- 用户要求解释又转回去，本阶段只读查看日志与航向采样，未改运行程序或发送控制命令。
- 用户目标[-1.76596,0.000496,1.2]；首次对准163.19度。21:20:09转向完成后，新曲线起始速度朝43.4度，与当前实际残余速度方向约43.25度一致，水平残速约0.067m/s，角差119.8度，触发±60度规则。
- 后续43.38 -> -20.91 -> 176.43度，导航几乎未进入持续运动。转向完成允许速度<=0.1，但motion_heading从>0.025即取瞬时规划速度；EGO起始速度继承FCU估计速度，因此悬停残速被误当作下一段路线方向。移除5cm分支已加载，仍存在起步速度判定缺陷。
- 需在起步阶段使用稳定的后续曲线方向/抑制残余速度，并协调转向结束的静止门与方向门；正常运动±60度规则保留。尚未实施本项修复，不宣称飞行通过。

## 2026-10-04 21:04 Qt/导航升级最终版地面READY

- 最终启动start/logs/20261004_210123，仿真65.032秒地面READY，connected=true、armed=false、LIO valid/HEALTHY、EKF位置/高度/速度状态有效，空闲EGO命令0。
- 最终修改已完整加载：取消5cm位置误差触发转向；Qt移动速度0.1～1.0m/s（默认0.5，下一任务）；ARM焦点修复；已知障碍目标拒绝；未知区域黄色虚线参考，已观测黄色实线；执行前确认自由空间。新障碍/未观测/局部规划无解或结束未到最终目标时保留任务、持续悬停并重规划，通信/旧轨迹/定位失效保护保留。
- 起步悬停转向及±60度、LIO最低支持30、0.5s HOLD/1s LAND、EV位姿融合/速度禁融、ENU/MAVROS自动转换保持。
- GUI/EGO/traj_server编译成功，Python/XML解析成功；部署hash一致，Qt截图确认速度行、Gazebo窗口存在。仅地面启动观察，未进行本版本飞行验证，不宣称所有轨迹问题已消除。
- 当前运行会话：启动78116、Gazebo18240、只读监测29191；就绪68954完成退出0。PIDs/参数/轻量CSV/JSON/截图见start/qt_work/manual_monitor_20261004_navigation_upgrade；监测持续、用户可手动实验。

## 2026-10-04 21:03 最终导航升级部署，等待READY

- 最终运行start/logs/20261004_210123，Qt和Gazebo已打开，执行器与黄线路径源码hash和部署快照一致；GUI截图确认移动速度0.50m/s行。
- 最后复核补齐：当前曲线因新发现膨胀障碍被拒时悬停、保留目标再规划；只有最终目标入已知障碍才取消。新局部曲线零初速度阶段使用曲线前瞻判断是否需转向，避免自由空间门在转向前阻断；局部曲线结束未到最终目标时重新规划，不持续追踪旧终点。
- 参数读回EV_CTRL11/HGT_REF3/GPS0/BARO0，LIO支持阈值30和0.5s HOLD/1s LAND保留；当前未解锁、LIO HEALTHY/EKF有效，等待65秒启动保护。
- 最终会话：启动78116、Gazebo18240、只读监测29191、地面就绪68954。PIDs/运行参数/部署hash见start/qt_work/manual_monitor_20261004_navigation_upgrade。
- 编译和语法解析完成；尚未进行本版本飞行验证，不宣称轨迹失败全部消除。只读观测持续。

## 2026-10-04 20:55 Qt与导航升级完成，完整重启等待地面READY

- 用户最终决定：已知膨胀障碍内的目标拒绝，未知区域允许黄色参考路径；其余按已确认方案实施，修改后完整重启。
- 移除5cm位置误差方向触发转向，低速保持航向；起步悬停转向、前方±60度规则保留。
- Qt在相对起飞高度下增加移动速度0.1～1.0m/s、默认0.5、步长0.05，下次任务生效。新增/drone/set_navigation_speed，任务提交时同步EGO管理器/优化器缓存速度限值，内部重规划保持任务速度。服务调用移出管理器锁，保持OFFBOARD定时发布可运行。
- ARM按钮不接收焦点，点击后聚焦操作容器，防止自动跳到高度输入。
- 黄实线为已观测自由、黄虚线为未观测参考，避开已知膨胀障碍；实际执行启用观测自由前缀检查，至少3s且随速度/时延增加制动裕量。
- 未观测执行路段进入内部SPACE_WAIT，NAVIGATING/Qt目标保留，连续悬停；等待2s、速度<=0.1且新地图/定位健康后重试。心跳正常但5s无可用轨迹时保留任务重试；旧轨迹、心跳/地图/定位异常保护仍有效。若新地图发现最终目标进入障碍则取消到HOLD。
- EGO局部视距终点落障碍时尝试附近自由局部端点，最终目标不变；失败重试间隔0.2s、警告节流，完整连续曲线碰撞检查保留。
- GUI/EGO/traj_server编译成功，Python和XML解析成功，未运行飞行或测试套件。本轮功能尚待用户手动飞行验证，不宣称此前轨迹失败已经全部复现消除。
- 已确认旧实验20:14由操作员请求降落并地面未解锁后停止旧栈；新运行start/logs/20261004_205238，完整Qt/Gazebo窗口打开，当前启动定位健康但等待65s仿真启动保护。
- 记录start/qt_work/manual_monitor_20261004_navigation_upgrade，启动会话99233/Gazebo51093/只读监测59956/就绪49353。初始监测因进程文件未就绪退出，已修复文件缺失处理并重启；未影响仿真与控制节点。
- LIO最低约束30、0.5s HOLD/1s LAND、EKF EV_CTRL11/HGT_REF3/GPS0/BARO0读取正确，EV速度禁融与ROS ENU保留；仅轻量CSV/JSON记录。

## 2026-10-04 20:13 实时监视：轨迹终点被拒后超时HOLD

- 用户要求只看实时日志，未改源码/参数/运行节点，未发送飞行指令。
- 20:10:49用户选择新目标[4.71766,-1.06669,1.2]，仍出现反复转向；20:12:35最后对准-36.33度后等待轨迹，20:12:57轨迹超时HOLD。
- 20:13:08再次选择[4.97121,-0.63011,1.2]，EGO反复报告terminal point of current trajectory is in obstacle, skip planning；黄色全程已知自由路径不可用，20:13:31再次超时HOLD，Qt标记删除。
- 未记录本阶段LIO或EKF失效，也未记录自动LAND；机体仍armed/OFFBOARD，悬停指令持续，最近20s统计最大间隔0.05s仿真。
- EGO警告是内部当前轨迹终点的地图判定，不能据此直接断言用户目标处存在真实障碍；需要进一步核对局部终点/占用与未知体素。
- 航向位置误差分支导致过频转向问题仍待修正。实时监测持续运行，导航未完成。

## 2026-10-04 20:09 航向实验出现频繁转向，待修复

- 用户问当前状态，只读日志：首转158.27度成功，随后158->98->158->-81->59->146频繁切换TURNING/TRACKING，尚未到达目标，仍空中NAVIGATING/OFFBOARD。
- 最新几何支持约588、LIO/EKF未记录失效，问题不是本轮定位退化。
- 118.84s进入TURNING前实际yaw约59.9度，最后轨迹速度方向85.1度，差约25度处于±60内，但转向目标146.74度。对应motion_heading里的位置误差>5cm且其方向超出实际机头±60时优先采用位置误差方向的额外分支。
- 该位置误差分支容易把延迟/跟踪修正误认为下一段运动方向，与用户按运动方向保持朝向的规则不一致，应移除或改为有效合成控制速度检查；用户选定主规则按局部轨迹速度/切线决定，低速保持朝向。
- 当前只读排查，未改运行节点、重启或发送飞行指令；待修正，不宣称航向飞行验证通过。

## 2026-10-04 前方120度航向逻辑已加载，地面READY

- 用户选定逻辑已实现：首次水平移动前按审批局部曲线方向减速悬停、转向；随后保持该航向。运动方向超出固定/实际机头±60度时，再次悬停转向并请求新轨迹。
- 执行器内部WAITING/TURNING/TRACKING保持NAVIGATING及同一Qt目标；任务代次递增拒绝旧异步审批，旧曲线停发，30Hz OFFBOARD悬停连续。低速使用曲线0.5s前瞻处理零初速。
- 转向先速度<=0.1m/s稳定0.2s，航向限速30度/s；实际误差<=10度、速度<=0.1且稳定0.3s、有该稳定起点后的地图观测才恢复。20s转向超时取消到HOLD。到达容差仍15cm/速度门/1s，保持到达位置，不后退修正精确坐标。
- Python语法解析成功，部署源快照hash一致。完整重启start/logs/20261004_200050，sim65.010地面READY、LIO HEALTHY/valid、EKF有效、connected、未解锁；空闲指令0。Qt/Gazebo窗口已确认。
- 运行会话：启动97647/Gazebo18819/只读监测45074；就绪60367已退出0。进程gui1237464/ego1237499/traj1237509/manager1237536/path1237545/px41239321。
- 记录start/qt_work/manual_monitor_20261004_heading120，新增navigation_stage与heading_samples.csv。LIO阈值30、0.5s HOLD/1s LAND、EV速度禁融、ENU转换、其他Qt保持；管理器require_observed_free配置未改。
- 未运行自动飞行或测试套件，首次转向/跨±60度/重新规划/到达行为还待用户手动飞行验证，不宣称碰撞风险已全部解决。

## 2026-10-04 用户选定前方120度固定航向逻辑，源码完成并部署中

- 起步先悬停转向局部轨迹方向，保持航向移动；超出±60度重复悬停转向并重新规划，保留同一Qt任务。
- 实现于flight_manager.py，增加内部WAITING/TURNING/TRACKING阶段，不采用恒定traj_server yaw；低速/纯升降/角度跨±pi处理，20s转向超时HOLD。
- 确认地面未解锁后停止旧栈，准备完整重启。LIO阈值30、0.5s HOLD/1s LAND、EV速度禁止融合、ENU和其他Qt保持。管理器观测自由配置未改，不宣称朝向检查解决全部盲区风险。
- 记录start/qt_work/manual_monitor_20261004_heading120，新增航向采样和内部阶段日志；Python语法解析通过，未进行自动飞行或测试套件。

## 2026-10-04 航向固定与前倾雷达覆盖问题已定位，待修正

- 航向更新门槛0.1m，而前瞻0.1s、规划0.5m/s形成约0.05m位移，导致traj_server保留旧yaw。
- EGO已要求观测自由，但管理器require_observed_free=false，执行检查缺口需要一起处理。
- 当前空中HOLD/armed，未改运行节点/发送指令；诊断和转向后重规划方案记录start/qt_work/manual_monitor_20261004_support30/HEADING_DIAGNOSIS.md，未实施。

## 2026-10-04 阈值30已生效，地面READY等待用户实验

- mapping/min_translation_support=30.0运行参数已读回；HOLD=0.5s、LAND=1.0s、恢复=0.5s。其他Qt功能、ENU转换与外部速度融合禁用保持。
- 完整重启start/logs/20261004_190521，sim65.008 READY、LIO HEALTHY/valid、EKF有效、connected、未解锁，空闲规划指令0，路径位姿frame odom。Qt、Gazebo显示已打开。
- 会话：启动75465，Gazebo26081，只读监测89166，就绪58986已退出0。进程gui1091427/ego1091464/traj1091465/manager1091508/path1091512/px41093314。
- 记录start/qt_work/manual_monitor_20261004_support30（运行参数/源配置hash/状态/逐帧几何/轻量轨迹/真值LIO采样）。监视阈值同步为30。用户手动飞行，尚未验证阈值30的空中定位质量和任务完成，不宣称验收通过。

## 2026-10-04 阈值30实验准备，重启初始化中

- 用户授权最低平移约束50降到30，源码配置已修改；0.5s HOLD、1s LAND、0.5s健康恢复保持，其他飞行参数和Qt不变。
- 已地面上锁完整重启start/logs/20261004_190521，启动器75465、Gazebo客户端26081（本地资源）、只读监测89166、就绪观察58986。
- 记录start/qt_work/manual_monitor_20261004_support30，新增机体真值/桥接位姿轻量CSV，比较误差需校正初始坐标偏移并配对时间。
- 当前初始化，LIO HEALTHY，EKF已有效，未解锁，尚未过65s启动门限；等待用户手动实验，不自动发送飞行指令。

## 2026-10-04 18:29 本轮分级几何保护实际触发

- 18:23:37另一次导航因estimator_ok=False降落，当时LIO HEALTHY；不能混为几何异常，本次未完成该EKF事件ULog诊断。
- 18:29:16最弱支持48.48011，WARNING；下一帧51.87182短暂恢复取消计时，381.324收到48.06126重新计时。
- 持续低约束到381.898进入HOLD，381.902取消导航；382.360持续1.036s触发SEVERE，382.378 LANDING，触发时estimator_ok=True。
- 支持由48附近降到35.1344，之后最低已观察32.3682。383.352恢复55.6359，但严重保护按设计锁存。
- 18:29:45落地上锁LOCALIZING；当前原生FasterLIO继续输出、几何约束约307，但桥接SEVERE停发EV，所以Qt仍显示LIO异常。
- 导航1Hz采样最近目标0.13025m，未完成到达判定。分级时序已在本轮发生，不代表定位精度或整套仿真验收通过。
- 用户询问原因，本次只读排查未改参数/重启/发送指令。

## 2026-10-04 定时LIO保护已编译并完整重启，等待用户手动实验

- 按用户选择：约束低于50先WARNING，持续0.5s取消任务HOLD，持续1s或严重异常LAND；Qt仅修改顶端LIO卡片文字与颜色，健康绿、WARNING/HOLD黄、严重/失效红。
- 连续健康0.5s解除轻度警告，HOLD取消任务不自动续飞；严重故障保持锁存。几何标记与原生估计协方差分离，新鲜位姿在短暂轻度异常期间继续通过原检查后发送。
- Python语法解析及drone_operator_gui/run_mapping_online编译成功，完整重启start/logs/20261004_181114。启动器8672、Gazebo客户端8800、只读监测97934；就绪观察10129已完成。
- sim65.102 READY，LIO HEALTHY/valid、EKF有效、PX4 connected、未解锁；黄线路径位姿odom，空闲规划指令0。Gazebo和Qt桌面窗口已确认。
- 参数只读确认EKF2_EV_CTRL=11、HGT_REF=3、GPS_CTRL=0、BARO_CTRL=0、EV_DELAY=0，外部速度融合仍禁用，ENU转换不变。
- 记录start/qt_work/manual_monitor_20261004_timed_geometry（编译日志、源/二进制哈希、参数、状态、逐帧几何与轻量轨迹）。未进行自动飞行或测试套件；0.5s/1s触发与恢复行为仍待用户实验验证。

## 2026-10-04 用户选定定时LIO保护，部署进行中

- 用户授权修改并重启，随后由用户手动实验。轻度低于50警告，持续0.5s取消任务HOLD，持续1s或严重异常LAND；Qt仅顶端LIO卡片绿/黄/红及对应文字。
- 源码已修改，解除几何标记注入原生协方差，由桥接按ROS时间判定；轻度恢复需连续健康0.5s，取消后不自动续飞。外部速度禁止融合、ENU转换、其他Qt功能不变。
- 旧栈已在地面未解锁时完整停止，正在编译。新栈尚未确认就绪，不能飞行验收声明。
- 记录start/qt_work/manual_monitor_20261004_timed_geometry；本轮没有测试套件或自动飞行。

## 2026-10-04 17:49 手动导航再现短暂退化保护

- 只读监测本轮飞行，EGO执行与黄色完整路线有效；导航采样最近距目标0.11336m，尚未完成到达判定。
- 最弱约束49.55479、49.81681两帧低于50，首低sample210.306，210.532恢复56.30243（约0.226s仿真）。桥接永久锁存、停发EV，触发时EKF仍有效；17:49:55落地上锁LOCALIZING。
- 报告start/qt_work/manual_monitor_20261004_retry/ROUND_REPORT.md。监测继续，未重启/改参数/发指令。本轮ULog尚未复核。
- 用户希望选择降低误触发的保护方案；方案待选，不自行实施。

## 2026-10-04 用户重新实验：重启并加强只读监测

- 用户先要求不重启，停止命令当时已执行，已明确告知旧仿真退出；随后用户明确允许重启，由用户自行飞行，助手只监测。
- 当前新仿真日志start/logs/20261004_173901，统一启动器会话37117，Gazebo客户端15397，监视28820，启动就绪观察56233。Qt799635、EGO799657、traj_server799667、manager799716、黄线路径799720、PX4801595。
- 配置与代码未再修改，退化阈值仍50。监测新增逐帧translation_observability（6个数字）记录lio_geometry.csv和低于50事件，保留状态/目标/融合/警告/轻量轨迹与CPU记录，不录大bag/图片/点云，不发送控制指令。记录start/qt_work/manual_monitor_20261004_retry。
- 65.082s已进入READY，LIO/EKF有效、未解锁，最小约束指标约434。已通知用户可操作；助手继续只读观测，尚未构成飞行验收。

## 2026-10-04 17:27 用户二次起飞后的自动降落

- 导航编号修正后spline1完整审批并执行，17:25:54转HOLD且Qt覆盖层清除，随后用户请求降落。后续17:26:45二次起飞，17:27:23自加LIO退化门触发自动LAND，17:27:42上锁。
- 233.684s support最小49.66686低于50，协方差被加1e6，bridge trace>1e5永久锁存pose_fault。下一帧54.558恢复但锁存不恢复；当时EKF有效，随后因为停发外部位姿才无效。不能说LIO大误差或EKF先失效。
- 当前LOCALIZING/unarmed，需后续处理桥接故障锁存与退化判定敏感性；本轮只读排查，未重启/改参数。证据start/qt_work/manual_monitor_20261004_restart/LIO_DEGENERACY_172723.md。

## 2026-10-04 导航超时修正与完整冷启动完成

- 管理器编号只跟踪带新鲜原始生成时间的Bspline；旧PositionCommand不再污染下一目标编号。曲线必须不早于目标；检测编号重置时取消至HOLD并要求重新选目标。独立navigation_generation防止旧异步验证跨任务提交。traj_server导航禁用时清空/停发旧轨迹，并等待新曲线。
- 黄色路径输入改为/drone/fcu/odom，与cloud_fcu_world同FCU ENU数值，未增加NED转换。冷启动发现catkin relay导入问题，已修正safe_global_path源模块搜索路径；第一次171100失败启动已完整关闭，最终启动为start/logs/20261004_171509。
- 编译traj_server与Python语法检查通过。最终65.014s状态READY、LIO有效、EKF位置/速度有效、connected、未解锁；路径位姿frame=odom，空闲pos_cmd数0。所有核心进程存活；未执行飞行测试/测试套件，不能宣称导航已到达验证。
- 统一启动器会话99911：Qt716950、EGO717057、traj_server717068、管理器717132、黄线路径717133、PX4718870。这些节点均由统一启动器拥有。独立Gazebo客户端会话38186；只读监视会话97981记录start/qt_work/manual_monitor_20261004_restart。停止整栈时同时清理独立客户端/观察器。
- 操作授权关闭，地面上锁；用户可手动授权/解锁/起飞继续实验。备份/编译日志/源SHA/进程与启动状态：start/qt_work/20261004_navigation_fix。EKF参数与外部速度禁止融合保持。

## 2026-10-04 导航超时根因已确认（尚未修复）

- 只读读取管理器运行字段latest_command_id=4、required_trajectory_id=5、approved_spline_id=None；实际新EGO位置指令id=2。上次部分部署遗漏重启traj_server，它残留旧id4抬高管理器门槛；新EGO从id1/2计数，两次均在编号检查处拒收，5秒仿真等待后HOLD。
- 是部署生命周期错误，不是EKF故障/5秒门槛过短。应地面同步重启规划执行整条链，并长期增加会话/目标代次；不得删编号保护。黄线map/odom frame拒收独立待修。证据start/qt_work/manual_monitor_20261004/NAVIGATION_TIMEOUT_CAUSE.md、manager_runtime_counters.json。
- 本轮只读诊断，未重启节点/改源码/发飞行指令。监视会话96687仍在记录。

## 2026-10-04 用户手动实验监视

- 两次导航因EGO-Planner trajectory timed out切HOLD，没有到目标；16:35:42用户请求降落，16:35:57落地上锁READY。ULog08_33_26未记录vision_data_stopped，40个空中flags样本EV位置/高度/航向开启，EV速度/GPS/气压关闭，输入年龄最大394ms。
- 已发现黄线节点拒绝frame_id=map的MAVROS里程计（代码只接受odom）；需修复。轨迹等待超时拒收原因仍待捕获，注意此次部分重启后的轨迹ID生命周期；不可称导航已正常。
- 任务取消后Qt三个marker均DELETE，尚未到达验证。只读观察会话96687仍记录start/qt_work/manual_monitor_20261004/events.jsonl和telemetry.csv；没有发送控制指令、没有改参数、没有大bag。报告ROUND_REPORT.md与ulog_summary.json。验收清单仍停止。

## 2026-10-04 地图去重与Qt任务结束清除已部署

- 按用户选定范围：EGO按新观测发布一次latched地图；占用/自由地图字节与布局相同时复用体素转换，原采集时间和安全检查保留。黄线路径无目标时跳过地图转换与规划，新任务等待新观测，旧任务转换结果不能覆盖新任务。
- Qt到达/取消/保护结束时删除红点、黄色路径与绿色轨迹，禁止旧路径恢复结束任务覆盖层；HOLD健康满足时恢复点选。
- Python语法检查、ego_planner_node和drone_operator_gui编译通过。未执行测试套件或飞行验收，不宣称EKF问题已消除。
- 地面READY且未解锁后替换节点；当前EGO567897、管理器567898、黄线路径567899、Qt567901，运行状态READY/connected/unarmed。管理器重启授权已关闭。LIO/PX4/Gazebo保持运行，启动器会话33463、gzclient会话78771。
- 这四个新节点为脱离启动器的独立进程，停止整栈必须同时结束它们；旧PID298735/298789/466013/472569已退出。下一次常规启动包含相同修改。
- 备份、编译日志、进程信息、状态与改动说明：start/qt_work/20261004_map_dedup。没有发送飞行指令。

## 2026-10-04 EKF深入只读诊断与方案待选

- 地面18秒墙钟配对采样：原始雷达年龄中位200ms，adapter新增46ms，LIO新增61ms，位姿桥接新增中位0ms。仿真RTF约0.18；CPU busy90～93%，地图20Hz仿真频率重复发布约6MB。PX4时间偏差约2.8ms，不是数百毫秒固定偏差。旧悬停已有同类超时，新增黄线路径节点是额外负担而非唯一原因。
- 已向用户提供源头减负A、A后EKF缓冲B、A后分级保护C供选择。未改代码、参数或节点，未发送飞行命令；测量/边界见start/qt_work/manual_goal_20261004/readonly_timing_probe.json和EKF_IMPROVEMENT_OPTIONS.md。

## 2026-10-04 EKF输入时间安全门诊断

- 最新导航降落已确认vision_data_stopped事件；外部位姿三帧接收年龄465～473ms，相对EKF融合时刻落后157～165ms，超过源码100ms取样窗口，随后400ms无有效样本保护关闭EV。位置/高度280ms后恢复，管理器单次无效即LAND。未放宽门槛、未启动新飞行。
- 各段延迟或时间映射归因仍待统一时间诊断，不能宣称修复。报告与离线时间重构：start/qt_work/manual_goal_20261004/EKF_DIAGNOSIS.md、ekf_input_timing.json。

# 巡检四旋翼仿真进度

更新时间：2026-10-04 15:45:02 CST。二维点选/目标锁定/红黄绿路径显示已编译并打开新Qt472569，安全路径显示节点466013运行；未执行本轮飞行复测，验收仍停止。

## 2026-10-04 15:50 用户点选目标未到达

- 用户点选ENU[-4.353531,-2.023357,1.2]，15:50:28.711管理器HOLD→NAVIGATING，EGO进入EXEC_TRAJ；15:50:43.965因estimator_ok=False自动LAND，15:51:10回READY。
- 健康日志odom_age=.022、estimator_age=.004、health_age=.062s，LIO有效/PX4连接正常。ULog07_49_21.ulg外部位置/高度1028.928s停止融合、1029.208s恢复（280ms），yaw1029.600s恢复；管理器1029.062s已请求降落。原生位置/高度失效flags和电量warning未变，1029.616s OFFBOARD丢失在切LAND之后。
- 15:48:22此前一次起飞也因EKF有效性失败提前降落，属于重复问题，不判修复。证据start/qt_work/manual_goal_20261004/diagnosis.json。融合中断原因未确立，本次只查日志，不改保护参数/不开始飞行测试。

## 2026-10-04 二维点选及安全全局路径显示

- 用户要求点击点选后二维视图、滚轮缩放、红色实心目标、单一任务锁定、黄色当前安全完整路线及绿色实际轨迹，完成后重新打开Qt。
- Qt新增二维平面射线点选（不依赖点击雷达表面）、缩放、红色CYLINDER、单次任务锁定。只在授权/空中HOLD允许发送目标，到达/取消/保护结束恢复；相对目标也锁定。红点与绿色MAVROS里程计轨迹保留，当前黄线结束时清除。
- 新只读节点safe_global_path.py：最新完整膨胀地图+observed_free、全线段闭体素DDA、全曲线安全前缀检查、0.25m格点有界3D A*连接终点；未知自由空间/碰撞/过期/无法连接清除黄线。地图变化和EGO局部曲线变化重算，发布前重检最新地图。只显示安全候选完整路线，不替换EGO执行、不新增飞控目标发送者。未知物体和传感器误差不由地图证明安全。
- 新话题/drone/gui/active_goal、/drone/global_path、/drone/global_path_status、/drone/gui/navigation_markers。不使用EGO未经避障的全局参考曲线冒充安全路径；保持ROS ENU。
- 编译通过，备份/日志/SHA/真实GUI截图在start/qt_work/20261004_xy_navigation，流程说明src/drone_stack/QT_XY_NAVIGATION.md。新Qt PID472569（旧395841已终止），安全路径节点PID466013，均独立会话；stack.launch已加入节点供下次整栈启动。停止整栈时应同时停止这两个独立PID，禁止重复启动。
- 本轮只编译、启动及查看界面，没有发送飞行指令或执行测试/飞行验收；红点/黄绿轨迹的在线交互与复杂路线尚未复测。不可声称路线或新版本飞行验收已过，EKF96ms瞬态降落仍待诊断。

## 2026-10-04 Qt顶部状态颜色

- 用户要求健康绿色、异常红色；电池>60%绿色、20%～60%含边界黄色、<20%红色，电池无效/过期数据红色。保留文字与提示，不依靠颜色唯一表达状态。
- 授权未开启/地面锁定视为正常状态；告警卡片反映当前健康，历史消息保留日志/tooltip，正常操作降落不持续标红。
- operator_gui.cpp编译通过，旧Qt298710已停止。新GUI395841由独立会话脱离临时命令启动，进程记录/编译日志/源码备份/真实截图 `start/qt_work/20261004_status_colors`。截图显示9张绿色健康卡片，双相机、蓝色点云与橙色体素正常。
- 本次只改Qt并替换界面，未发送授权/解锁/起飞/降落请求，保持用户已有状态（截图：已授权、锁定、READY）。未运行测试/验收；黄色/红色分支未通过故障注入在线验证。
- 当前仿真启动器仍会话33463，管理器298789；新Qt不在原启动器记录的进程组内，停止整栈时需同时结束395841，后续避免重复GUI。

## 2026-10-04 14:58 用户手动悬停自动降落核查

- 14:57:11用户请求起飞，14:57:53到达HOLD；14:58:57管理器因estimator_ok=False请求降落，14:59:21回到READY。并非Qt操作空闲计时或演示脚本请求。
- 原日志健康：odom_age=0.028、estimator_age=0.014、health_age=0.070s，lio_valid=True、connected=True。所谓stale提示实际是EKF有效性标志不满足，不是这些消息过期。
- 原生ULog06_57_02.ulg：380.976s外部位置/高度融合暂关，solution_status_flags831→129；381.072s恢复，持续96ms。管理器381.068s单次无效即LAND。381.640s OFFBOARD信号丢失发生在管理器切LAND之后；电量警告/原生位置高度速度失效flags未变。
- 证据 `start/qt_work/manual_hover_20261004/ekf_transition.json`。EV融合短暂中断原因和保护对单次标志的敏感性尚待诊断，不放宽安全门槛、不声称悬停问题已解决；验收仍停止。

## 2026-10-04 用户请求：1米起飞旋转建图展示

- 首次展示起飞后控制演示/管理器/Qt进程退出，演示工具exit143，未完成整圈；PX4保护降落。独立原生心跳确认unarmed、真值地面Z=0.168073m后重启整栈，未把失败算成功。后续核查：14:28:35管理器ROS记录signal-15，Qt/演示脚本退出143；ULog432.904s OFFBOARD信号丢失，电量/位置/高度失效标志未触发。信号发送来源未查明，详见首次目录EXIT_ANALYSIS.md。
- 添加 `/drone/rotate_once` Trigger：只允许已授权、空中HOLD、新鲜位姿及OFFBOARD，位置保持、航向目标10deg/s转一圈；原管理器30Hz持续发送目标，定位/模式丢失保护和撤销授权降落保留。源码备份 `start/qt_work/20261004_ui_phase1/flight_manager.before_rotation.py`。Qt旋转时暂不开放普通目标操作，一键降落仍可用。
- 重做展示 `start/qt_work/rotation_demo_20261004_retry` success=true。相对起飞目标1.0m，79.196s开始旋转，115.196s目标满360°，118.264s请求降落，123.054s原生心跳确认落地上锁，123.060s撤销授权；实际累计航向变化359.99°。
- 本次是用户请求的展示飞行，未恢复清单验收，不新增整套验收通过项。telemetry/events/native_heartbeats/result与截图保留，未录大体积bag。Qt图中已出现三处障碍体素。
- 当前统一启动器工具会话33463，GUI PID298710、管理器298789、gzserver299085；独立gzclient会话78771。原独立GUI/管理器已退出，不再按旧PID处理。地面保持，后续避免重复启动。

## 2026-10-04 工作切换：Qt功能完善

- 用户明确停止之前清单中的验收，直接跳转Qt功能完善；无验收调度器运行，不恢复剩余扩展/压力飞行。
- 当前工作清单 `src/drone_stack/QT_TASKS.md`，第一轮状态新鲜度/固定降落/数据健康/位姿提示/显示开关与单次点选已完成，最终编译通过并替换GUI，真实截图已查看。
- 备份及编译日志、源码/二进制SHA、状态快照与截图 `start/qt_work/20261004_ui_phase1`。状态connected=true、armed=false、authorized=false、phase=READY；不请求授权/解锁/起飞，未运行测试或飞行/故障验收。
- 新Qt为单独运行进程244309（工具会话6808），原Qt201921已停止；仿真主启动201535和gzclient204648继续运行。停止仿真时还应结束新Qt独立进程。当前模式OFFBOARD但未解锁，地面READY。

## 2026-10-04 手动启动界面查看

- 按用户要求完整启动 `src/drone_stack/scripts/start_simulation.sh`，日志 `start/logs/20261004_135722`；Qt、内嵌RViz、Faster-LIO、EGO、MAVROS、PX4和Gazebo服务运行。
- 单独启动gzclient显示同一仿真，未创建第二个仿真实例；Qt置于前台。完整图形显示时实时倍率约0.20，初始化等待按仿真时间进行。
- 启动核验：sim_time=67.658s，phase=READY，PX4 connected=true、armed=false、mode=AUTO.LOITER，LIO有效；此前确认两路处理图像640×480，Qt显示在线及橙色EGO体素/蓝色点云。
- 展示时发现Qt的PX4连接卡片会间歇“断连”：GUI使用2秒WallTime消息新鲜度，图形仿真实时倍率约0.20时心跳间隔可能超过门槛；ROS快照connected=true、READY。待修复仿真时间与独立墙钟看门狗的显示判定，当前不开展飞行。
- 本次只启动展示，未授权/解锁/起飞，未启动实验调度器，未增加验收通过项。进程保持运行，后续启动前先检查现有实例。主启动进程201535，Qt201921，gzserver202329，PX4 203807，gzclient204648（PID仅本次会话有效）。

## 2026-10-04 01:39 续作核验

- goal_hold_scenes_01 四项 three_d、blocked、multi_goal、corridor 全部整轮通过，全部闭包/降落上锁/融合/包络审计通过。原唯一调度器已正常结束，无残留仿真。
- 补齐生产指纹及启动快照：EGO三个动态库、traj_server、drone_stack patches和实际Gazebo IMU插件；下一完整核心使用新指纹，不拼接旧版结果。
- 新增追加实验 high_lio_loss（ENU高度2.45m、20s悬停后停止Faster-LIO）与 gui_land（实际Qt无障碍动作点击一键降落、原生心跳确认落地上锁）；先验证再进行core_live_20和全部18扩展。未执行项不计通过，实机准备仍false。

## 2026-10-04 01:43 高位故障失败与修复

- additional_safety_01/high_lio_loss 整轮失败：ENU 2.45m目标到达后中断LIO；ULog外部位姿最后100.024s、100.356s保护LAND、104.432s失效、104.976s终止（armed仍true）；105.072s真值高度0.347m时SITL停滞。融合及包络通过不能抵消未落地上锁，原始bag/ULog/心跳与noaid_failure_analysis.json保留。
- 默认EKF2_NOAID_TOUT=5000000us不足覆盖允许高度范围内完整下降。仿真配置改为10000000us（本版PX4参数最大值），仅有界IMU传播等待降落，管理器仍立即LAND，不伪造EV/不融合速度/GPS/气压计。需高位复测证明落地时间/漂移满足门槛，实机须独立验证惯性漂移与降落时间。
- gui_land 尚未执行；启动additional_safety_02同两项复测。完整核心与18扩展仍pending。

## 2026-10-04 01:52 高位第二轮因果审计

- additional_safety_02仍失败并保留原始数据。10s窗口实际生效；真值105.034s已低于0.25m、106s稳定接地0.1685m，但EKF惯性VZ仍0.255m/s，108s增至0.361m/s，超过原生LND阈值0.25，接地检测始终false；109.242s失效、109.894s终止，未上锁。
- 原生传感器Z约-9.75m/s²、偏置估计约+0.008m/s²，剩余重力方向加速度误差约0.05m/s²；模拟偏置随机游走0.006而EKF默认过程噪声0.003。下一轮仅增加EKF2_ACC_B_NOISE到文档范围内0.01以跟踪已有偏置，保留实际IMU噪声、接地阈值、所有融合源及验收门槛。参数作用必须靠追加03高位实验验证。

## 2026-10-04 01:56 高位故障第三轮通过

- additional_safety_03/high_lio_loss整轮通过：ENU目标2.45m，20s悬停后停止真实LIO节点，保护反应0.384s、无反升、最大水平漂移0.142329m（限0.30），融合/包络/落地上锁全部通过。
- 原生ULog：最后EV101.044s、接地106.516s、landed107.204s、上锁107.716s，停止EV至上锁6.672s，终止状态样本0。high_altitude_landing_analysis.json保存。实际NOAID10s、ACC_B_NOISE0.01，传感器与接地阈值未变。
- 唯一飞行追加03正在gui_land，完成后才能汇总两项。下一新版本core_live_20完整核心，再extended_live_09全部18项；旧版核心/定向通过不替代新指纹，实机准备仍false。

## 2026-10-04 01:59 GUI追加测试

- additional_safety_03/high_lio_loss整轮通过，但gui_land点击工具仅匹配小写press/click，返回No accessible landing action，因此追加03整体false。实际Qt一键降落按钮存在且enabled，截图gui_before_land.png已查看，RViz体素及双相机正常。安全清理降落/融合/包络通过不能冒充GUI点击通过。
- 修改gui_snapshot动作名称casefold匹配，并在未知动作报错记录原始名称；10项GUI单测通过（含Press与未知toggle拒绝）。唯一飞行gui_land_01重测，仅工具变化，生产指纹不变。若再失败读取真实动作名称继续修复。

## 2026-10-04 02:04 两项追加安全阶段完成

- 高位LIO停止additional_safety_03/high_lio_loss与实际Qt一键降落gui_land_01/gui_land均整轮通过、融合/包络/落地上锁通过，生产指纹一致。03追加目录整体仍false（原GUI工具失败），不重写原报告；最终引用各真实通过的闭合子轮次。
- Qt实际动作Press通过真实doAction调用，按钮降落没有替换为CLI服务；截图已查看，但gui_land_01图中用户终端覆盖中央RViz，图像证据有遮挡，先前追加03截图清晰展示完整界面。无障碍实际控件与原生心跳证据独立保存。
- 当前生产指纹：`4baa223d07bee7e13dc39b15a9de69e8fcb8155ae0edf1e43e07cd0a396fd68e`。开始新core_live_20三冷导航+五基础故障；完成后同指纹extended_live_09全部18项。生产及start/tools全部冻结，实机准备仍false。

## 2026-10-04 02:21 新版本三冷导航通过

- core_live_20/three_cold_navigation_attempt_01 三轮独立冷启动整轮通过：起飞、双相机、非法操作、前后60s悬停、实际绕障、落地上锁、原生融合审计全部通过。
- 当前唯一飞行核心会话9128转五故障，从fault_lidar开始；监督71147等待同版本核心通过再启动extended_live_09全部18项，日志/tmp/drone_core_live_20.log及/tmp/drone_campaign_09.log。生产和工具冻结。尚不能宣布完整仿真通过，实机准备false。
- 补充fixture后72飞行保护单测全部通过无跳过，记录high_altitude_analysis/flight_safety_with_cpp_fixture.log。三次高位参数对比图及CSV已生成并实际查看。

## 2026-10-04 02:43 core_live_20全核心通过

- 同一生产指纹4baa223d07bee7e13dc39b15a9de69e8fcb8155ae0edf1e43e07cd0a396fd68e：三次冷启动导航及五项基础故障全整轮通过。雷达0.480s/漂移0.016102m；IMU0.610s/0.020189m；撤销授权同采样时刻/0.020977m；OFFBOARD原生0.560s/0.030352m；低电量2.784s/0.064292m。均无反升、降落上锁、禁止融合源全0。
- core_metrics.csv、STAGE_REPORT.md、core_performance.png/pdf已生成。核心会话9128正常结束；监督71147已启动extended_live_09全部18项，当前唯一飞行multi_goal。日志/tmp/drone_campaign_09.log，生产/工具继续冻结。实机准备仍false。

## 2026-10-04 03:13 扩展规划器退出失败及修复

- extended_live_09前5项整轮通过，kill_planner第6项失败：进入HOLD后20s真值X峰峰0.265599m（限0.20）、实时相对LIO误差0.170218m（限0.10），清理降落/融合/包络通过但整轮false。监督71147结束，无残留飞行。剩12项未执行。
- planner_stop_reanalysis.json/npz保留：NAV83.79s、HOLD86.01s，HOLD切入真值速度[0.5932,0.2633,0.0003]m/s；规划器退出后traj_server仍继续旧已批准轨迹，管理器仅监控命令新鲜度，导致制动过晚。采集时刻对齐后LIO绝对误差最高0.07724m，实时悬停统计还混入扫描延迟；保持所有悬停门槛不变，不改测试以跳过制动。
- 管理器增加EGO现有/planning/data_display生产心跳监控，NAV阶段0.3s上限，短启动宽限，重复/倒退/未来/过期消息不刷新健康，超时HOLD并持续目标。75保护检查（含C++fixture）全部通过；扩展bag新增原生心跳话题。生产指纹已变，core_live_20不能代替最终版。
- 下一定向planner_heartbeat_01：kill_planner、kill_traj、multi_goal、three_d；通过后新core_live_21完整核心，监督extended_live_10执行18原扩展+高位LIO中断+真实GUI降落共20项，确保追加项也同最终指纹。实机准备false。

## 2026-10-04 03:34 心跳修复定向四项全过

- planner_heartbeat_01 四项kill_planner/kill_traj/multi_goal/three_d全部整轮通过。EGO退出0.372s保护，20s悬停X峰峰0.067298m/LIO误差0.015114m；traj_server退出0.536s，X峰峰0.052495m/LIO误差0.014349m；正常连续及复杂规划无心跳误中断。融合/机体包络/落地上锁全过，STAGE_REPORT.md保存。
- 旧core_live_20完整核心过，但新管理器指纹变化必须重测。启动core_live_21三冷导航+五基础故障，之后监督extended_live_10全部20项（原18+高位LIO停止+实际Qt降落），追加两项也重新用最终版本验证。唯一核心日志/tmp/drone_core_live_21.log，监督/tmp/drone_campaign_10.log；生产/工具冻结，实机准备false。

## 2026-10-04 04:14 最终core_live_21全部核心通过

- 生产指纹1ebdd48c0f66383d1528416fbba940ece38588ed778b1d1888736c956897bcd5。三次冷启动导航与五基础故障全部整轮通过，core_metrics.csv及STAGE_REPORT.md保存。
- 核心36047结束0；监督62466已自动开始extended_live_10共20项，当前multi_goal为唯一飞行。日志/tmp/drone_campaign_10.log，生产和工具继续冻结。此前19/20核心及09扩展仅历史，不拼接替代最终版。
- 完整扩展仍未通过，实机准备false。最终完成后再统一原始数据/原生审计/GUI图片/统计图与新检查点；剩余失败继续保留并修复。

## 2026-10-04 13:41 额度中断后状态核验

- 最终core_live_21完整核心通过，指纹1ebdd48c0f66383d1528416fbba940ece38588ed778b1d1888736c956897bcd5。
- extended_live_10正式整轮通过8/20：multi_goal、corridor、three_d、blocked、boundaries、kill_planner、kill_traj、kill_lio。kill_lio含实际GUI控件审计，截图还需完整最终图像汇总复核。
- 第9项kill_bridge scenario真实飞行保护通过（反应0.444s、反升0.000052m、漂移0.038255m、独立心跳最终unarmed），但gui_fault trigger_observed=false。GUI观察只等待Bool false，桥接节点退出后不再发布该Bool，须增加消息断流检测；观察进程20s等待/TERM后5s再次超时未捕获，调度器异常退出，未完成第9项整轮审计，不计通过。后11项未执行。
- 调度器退出后残留仿真持续到本次核查；13:39:51连接下再次验证unarmed且真值Z0.168073m，通过安全收尾。向原启动器73772发送TERM清理其进程组；late_cleanup_verification.log与flight_late_cleanup.ulg另存，不覆盖原始失败报告。
- 下一步：修复GUI对桥接健康消息断流的触发与有界进程清理，审计本次闭包数据，定向复测kill_bridge后继续同生产版本剩余验收（工具版本变化需独立新报告，不伪造原整轮成功）。当前磁盘仅约56GiB空闲，须先核对异常长留ULog/包体积并保留数据归档以保障后续空间。完整仿真未通过，实机准备false。

## 2026-10-04 13:47:23 CST 已通过实验原始数据清理

- 按用户指示，仅删除明确整轮通过的录包、ULog及运行日志：126个已通过目录、1851个文件，释放147.701GiB。系统盘使用率88%→54%，空闲56GiB→203GiB。
- 通过状态JSON报告、图表/CSV、源码与配置快照、回归fixtures、检查点均保留；全部删除路径不存在且81份通过证据报告SHA核对未变。失败/未完成轮次保留，包括kill_bridge约122GiB异常录包、旧规划器失败、高位失败原始数据。
- 清理记录：start/cleanup_records/20261004_passed_sim_data.json，包含逐文件路径、大小、对应通过证据及删除后校验。已删除通过项的原始bag/ULog不能再从工作区重新审计，旧检查点可能保留部分历史原始数据；现有通过报告代表当时真实运行结果，后续最终完整验收仍需继续。
- 未修改生产代码、仿真传感器或飞控参数，未改变8/20扩展通过状态，当前无仿真运行，实机准备仍false。

## 最新运行状态（优先于下方历史记录）

- extended_live_08 multi_goal及corridor整轮通过，corridor最小包络间隙0.210807m；three_d第3目标EGO无法规划5s被取消，清理降落/融合/包络通过，但整轮失败。监督55687已退出1；后续15项未执行。desktop_after_scenario_failure.png只拍到关机后桌面，不作为GUI证据。
- 起点回放证明116.02s估计位置[2.511335,0.043066,1.440218]已进入原始膨胀占据cell(175,150,9)，不是仅新增5cm储备。前一目标停在当时位置（误差0.09184m）并在悬停波动中侵蚀横梁余量；starting_clearance_reanalysis.json保留。
- 修复成功到达时保留当前yaw，完整线段/时效/体积检查后悬停精确原目标；仅成功到达使用，手动HOLD/故障停止仍当前位姿。72管理器检查通过，安全膨胀及整曲线检查未放宽。生产已变，旧完整核心不能替代最终版本，之后需新core_live_20+全部扩展。
- 当前唯一飞行goal_hold_scenes_01定向复测three_d→blocked→multi_goal→corridor，日志/tmp/drone_goal_hold_scenes_01.log；生产/工具再次冻结。若失败继续因果审计修复，不跳过失败。

- 本次恢复已完成完整core_live_19：三冷导航+雷达/IMU/撤销授权/OFFBOARD丢流/低电量全部整轮通过。新增IMU反应0.672s/漂移0.017744m；撤销同一仿真时刻/漂移0.017861m；OFFBOARD原生0.564s/漂移0.023500m；低电量2.394s/漂移0.055105m；均无反升/最终上锁。core_metrics.csv与STAGE_REPORT.md已生成。
- 核心会话16117已退出0。唯一飞行由扩展监督会话55687拥有，extended_live_08从multi_goal起执行全部18项；日志/tmp/drone_campaign_08_resume.log，生产与工具仍冻结。当前扩展不是已通过；所有既有历史暂停/会话编号仅作历史。

- 本轮已恢复实验：启动前无残留ROS/PX4/Gazebo，生产fingerprint与3个补充EGO动态库SHA全部匹配；core_live_19从fault_imu继续，监督器仅在核心通过后启动extended_live_08全部18项。日志/tmp/drone_core_live_19_resume.log、/tmp/drone_campaign_08_resume.log。生产与工具冻结；旧暂停说明仅作历史，不再代表当前状态。

- **保存完成**：2223主检查点1,242,214,497字节、2083文件SHA256读回一致、latest已更新。另有drone_sim_runtime_2026-10-03_2223.tar.gz补充3个EGO动态库，全部SHA验证；当前core_live_19/runtime_library_sha256.json记录它们（既有生产fingerprint未覆盖这3库，恢复前另行核对，不能只核对主fingerprint）。本次修改后的最新进度另存检查点旁.progress.md。

- 本次重启保存的新检查点：start/checkpoints/drone_sim_checkpoint_2026-10-03_2223.tar.gz。独立.manifest.json、.sha256、.verification.json记录归档范围及逐文件校验，latest只在验证成功后更新。保存本版源码/二进制、core_live_19全部闭合数据、连续曲线回归fixture、core_live_18失败第3轮原始bag与报告；较早几何故障/离线校准原始输入在已验证的2134检查点，历史原始大bag仍保留工作区。

- **用户要求暂停并重启终端：当前无ROS/PX4/Gazebo/rosbag/测试调度器，全部当前bag已闭合，无.bag.active。** 主68671与监督62848已停止；此前会话86749/63108、10202/24147也都结束。不要轮询这些旧会话。
- 当前生产指纹：`7693fdb664c29acc9101d84692edbc944de60abbd9ae03d3fdcfb9f2f4507f8c`。`core_live_19` 三次冷启动正常导航整轮及 `fault_lidar_attempt_01` 整轮通过。雷达故障反应0.508s、无反升、水平漂移0.015631m、确认上锁；bag/flight.ulg/融合与包络审计已保存。
- 停止时核心父进程已暂停，但故障子轮次实际完成落地、闭包与审计；根据原始cold_start_result.json成功将已完成fault_lidar记录补入父汇总，未伪造/跳过未完成故障。父报告status=paused_by_user、passed=false，保持用户暂停历史。
- **续作从fault_imu开始**：余下 IMU中断、撤销授权、真实OFFBOARD目标流丢失、低电量；通过后执行extended_live_08全部18项。新扩展目录尚未创建，未计通过。真实GUI点击降落、较高高度定位丢失的追加验证可在主回归后补齐，不能仅凭正常1.2m故障结论推广到整个高度范围。
- 从工作区恢复命令：
```bash
cd /home/d/robotproject/project0
source devel/setup.bash
unset ROS_IP
export ROS_HOSTNAME=127.0.0.1 ROS_MASTER_URI=http://127.0.0.1:11311 ROS_HOME=/tmp/drone_ros_home OPENBLAS_NUM_THREADS=1
python3 src/drone_stack/scripts/run_core_acceptance.py start/experiments/20261003_core_live_19 --resume > /tmp/drone_core_live_19_resume.log 2>&1
```
- 上述核心在一个受控会话运行后，另一个仅监督会话可启动：
```bash
source devel/setup.bash
unset ROS_IP
export ROS_HOSTNAME=127.0.0.1 ROS_MASTER_URI=http://127.0.0.1:11311 ROS_HOME=/tmp/drone_ros_home OPENBLAS_NUM_THREADS=1
python3 start/tools/run_sim_campaign.py start/experiments/20261003_core_live_19 start/experiments/20261003_extended_live_08 > /tmp/drone_campaign_08_resume.log 2>&1
```
- 恢复前核对fingerprint与实际进程；若改生产文件必须新目录完整回归，不能用--resume拼版本。不要再次启动第二套ROS。所有尚未执行项目保持pending，实机准备false。

- core_live_19三次冷启动正常导航整轮全部通过：每轮前后60s悬停、双相机、规划/避障、降落上锁、融合审计。第3轮实际避障通过，新连续曲线检查没有再漏出造成取消的正常候选。原生B-spline与Bezier转换546位置误差最大3.21006e-15m，日志已存continuous_planner/native_conversion_test.log。当前主68671继续五基础故障；监督62848等待后自动extended_live_08全部18项。生产/工具仍冻结。

- core_live_18整轮失败：前两冷启动整轮通过；第三轮正常候选曲线被管理器碰撞保护停止，清理降落上锁/融合通过。原始候选10万采样1165碰撞点，首点约[1.83988,0.799995,0.901886]、曲线比例0.61554，complete_curve_reanalysis.json保留。不是放宽执行保护的问题。原监督63108退出，extended_live_07未启动。
- 根因：EGO反弹/细化旧后检仅前2/3稀疏采样，细小穿入体素片段漏检。修复Bezier凸包递归完整曲线/闭合体素面检查，优化碰撞重试、细化后和最终发布前均复核；增加5cm规划储备，管理器独立门槛不变；初始化/反弹约束检查延伸完整局部曲线，预算10000节点/50ms。C++15项（含本次真实曲线fixture）及70管理器/23曲线/26契约/4适配器/17探针通过，catkin构建通过。
- 当前唯一飞行core_live_19会话68671，监督62848同版本核心通过后自动extended_live_08全部18项。生产指纹7693fdb664c29acc9101d84692edbc944de60abbd9ae03d3fdcfb9f2f4507f8c。日志/tmp/drone_core_live_19.log、/tmp/drone_campaign_08.log；生产/工具冻结。21:34检查点早于本次规划修复，最新工作区/记录优先，最终须重新归档。

- 21:38：新检查点drone_sim_checkpoint_2026-10-03_2134.tar.gz完成，1,393,640,163字节，1773文件逐项SHA256与gzip/tar读回校验通过，latest已原子更新。范围为当前源码/运行二进制、geometry_loss_04闭合原始数据、离线校准输入与四场景报告/ULog；四场景13GiB原始bag仍在工作区未放入此紧凑检查点，当前活动核心/扩展未归档。manifest与verification文件写明范围，不能当全工作区备份。

- 21:33：geometry_loss_04整轮通过，5飞行检查+7观测证据+GUI+融合+包络全部通过；反应0.110s、反升0.002517m、漂移0.010958m，native心跳证实上锁。生产指纹87c535dab794535d034edb5f0f319c586c11417890b127404bb801c861e8836d，截图已查看，STAGE_REPORT.md已保存。
- 当前唯一飞行拥有者：core_live_18会话86749，三次冷启动导航/前后60s悬停，再5基础故障；监督63108等待同版本核心通过后自动运行extended_live_07全部18项。日志/tmp/drone_core_live_18.log、/tmp/drone_campaign_07.log。生产/全部工具冻结，不启动第二套ROS。若失败先审计修复，旧版通过不得拼接替代。

- geometry_loss_03实际飞行5检查、GUI、ULog融合/包络全部通过；反应0.162s、反升0.002247m、漂移0.013294m，独立心跳87.75证实unarmed，截图已查看。原审计错误要求采集时间不早于回调注入时间（首坏采集83.608，注入83.762，坏帧接收83.828），导致整轮仍失败，原报告保留。
- 修正审计以过滤点云与坏位姿原始采集时间严格匹配，并要求坏位姿接收发生在注入后1.5s内；不改时间字段。geometry_loss_04唯一会话24147，源码/工具冻结。第三轮离线重审另存文件，不能回写原整轮报告。

- geometry_loss_02仍失败，GUI实际控件审计已通过。独立心跳88.754仍armed，89.034时SITL停滞；ULog证明接地87.19、默认COM_DISARM_LAND=2、无EV五秒超时引发88.43/88.974后续failsafe，未实际上锁。旧清理land因MAVROS disconnected/armed=false误判不作为验收依据。
- 修复：仿真COM_DISARM_LAND=0.5（仅PX4确认接地后自动上锁，无强制空中上锁）；管理器LANDING/下降接地阶段保持到确实unarmed；land验收要求connected且地面，不接受失联默认false。70管理器/26契约通过。第三轮geometry_loss_03唯一会话10202，生产/工具冻结。

- geometry_loss_01未通过：原生退化诊断、bridge停止EV、实际LANDING→READY、ULog融合/包络通过，截图已查看，但独立心跳观测墙钟2s超时与AT-SPI桌面控件查询15s超时使整轮失败。保留数据，不计通过。
- 修复验收工具：独立心跳使用2s仿真时间新鲜度与10s墙钟冻结看门狗，保存native_heartbeats.json；AT-SPI每次D-Bus查询限定500ms，跳过死应用查询异常。17探针/8GUI检查通过。生产飞行门槛未变。当前geometry_loss_02唯一会话97549，源码/工具冻结；通过后再启动新完整核心与18扩展。

- 续接核验：observability_scenes_01 的 blocked、multi_goal、corridor、three_d 全部整轮通过，生产指纹cc01aef56cd3f9a55467f6d2e6b364c9f110bc6c2a2a4e23bcec00c816fa57c4；无残留飞行进程。
- 新增 geometry_loss 第18项：模拟适配器保留标定水平带点云、独立IMU和原始瞬时扫描时间；正常模式默认关闭，无真值参与算法。4适配器/26契约/16探针检查通过。在线实验20261003_geometry_loss_01，会话76435，必须原生弱信息矩阵、协方差、EV停止、持续输入、降落、GUI、融合和包络同时证明；当前源码/工具冻结。
- 下一步：该故障通过后新版本core_live_18全核心及extended_live_07全18项；失败修复后重新冷启动。旧版本结果不能代替本版。

- 20:49：唯一飞行会话71864：observability_scenes_01，blocked整轮通过（完整曲线体素碰撞，20s Z范围0.066606m/LIO误差0.040792m，融合/包络/降落上锁通过）；multi_goal当前执行五目标，之后corridor/three_d。源码/工具冻结。
- 原始传感器离线校准完成：nominal1653帧最弱支持量最低347.127、无低于50；原blocked首次低于50在85.292s，当时原在线高度残差约0.004544m，早于后续大误差。选择50替代待定20以提前制动；无需真值作为算法输入。calibration_report.json含原始输入SHA，blocked_observability_calibration.png已查看。
- 最新编译成功；68管理器/23曲线与编码/26桥接模型/16探针/7 C++观测数学检查通过。默认严格自由前瞻关闭，整曲线碰撞/边界保护保持；弱观测方向协方差1e6使桥接锁定停止外部位姿、管理器降落。blocked测试新增仅在真实退化协方差+桥接日志+相位事件均证实后允许保护降落，反应≤1.5s/反升≤0.15m/水平漂移≤0.30m/地面上锁门槛不变。
- 尚待真正在线触发观测退化（当前blocked因几何提前拒绝，未触发该支路），须增加保留水平墙体返回、移除高度约束的几何输入故障，证明检测/EV停止/实际降落。源码/工具当前不可改，先等四场景结束；此项可加入最终扩展。新完整核心与全部扩展仍未开始。

- 20:28：无ROS/PX4/Gazebo飞行；构建最终版会话93658、正常输入提取44558。observed_free_scenes_03 blocked整轮通过（曲线已知膨胀占据碰撞，Z范围0.052710m/LIO误差0.030171m），multi_goal第二个抬高目标被3s未知空间前瞻拒绝；全局无顶场景向上射线无返回，不可把缺失返回当自由空间。会话95833已退出1，降落/融合/包络通过，走廊/三维未执行。
- 最新默认策略：require_observed_free=false（源码仍保留可选3s严格模式与射线/自身空间记忆，正常仿真需验证）。整条B-spline已知占据碰撞/完整体积检查保持。新增Faster-LIO当前点面法向信息矩阵特征值诊断，弱位置方向协方差标1e6；bridge原生位置协方差trace>1e5停止EV并锁定无效，继续PX4 IMU状态传播并由管理器保护降落。mapping/min_translation_support=20为待校准阈值，尚未证明仿真正常/故障都适用。
- 修复Faster-LIO里程计先发布后填写协方差的原始顺序；offline初始化样本补齐与ROS配置一致，原来offline未使用400。7项C++数学测试、26桥接/模型测试通过；68管理器/23曲线与编码旧检查需要本轮复核。
- lio_observability/blocked_inputs.bag已仅提取原失败的/livox/lidar、/livox/imu；nominal_inputs从原多高度诊断提取中。回放不能用真值当输入，需要对照正常支持量与退化支持量后定阈值，再复测blocked/multi_goal/corridor/three_d，之后新完整核心+17扩展。
- 生产指纹及启动二进制SHA新增libfaster_lio.so和PX4实际bin/px4；历史生产指纹不覆盖本次变更，旧核心/扩展通过不能替代最终新版本。

- 20:02：唯一会话95833，observed_free_scenes_03（blocked→multi_goal→corridor→three_d）。生产/工具冻结。observed_free_scenes_02 blocked因目标未知而整轮通过，但multi_goal首条曲线被未知体素取消，停止/降落/融合/包络通过，整轮失败保留。
- 起点未知体素定位到当前机身内的盲区（曲线前移4～9cm、上升2～4cm）；按实际0.47×0.47×0.11m机身标记自身空间，保留占据优先。Faster-LIO dense_publish_en=true提供完整去畸变返回，内部滤波和降采样不变，避免质心伪射线。
- 最新策略替代上方历史的整曲线未知禁入/未知目标拒绝：允许未观测的有限ENU目标排队；整条曲线仍必须无已知膨胀占据碰撞/越界，但未知空间检查针对接下来3s的执行前瞻及当前线段，每0.1s随控制指令重检。未观测尾段不提前执行；规划器可生成未来候选路线。检查在锁外，连续OFFBOARD目标保持。前瞻不能配置小于3s。
- 68管理器/23曲线与编码/25配置检查通过，prefix_build构建通过；observed_free_callback_03实际EGO回调7项通过，包括机身盲区自身空间、ExactTime错时拒绝、返回后未知与READY记忆。最新策略尚待在线证明；新完整核心和17扩展仍未开始。

- 19:44：observed_free_scenes_01结束1，blocked工具旧判passed但实际仅因自由地图不新鲜而拒绝，不能算未知空间保护覆盖；multi_goal首目标同样因该原因拒绝。20.8万点逐点Python地图解码CPU0.731s，影响地图时效；不放宽2s门槛。
- 修复管理器批量NumPy读取并编码为有界整数体素集合，实测同样数据0.04976s CPU、全部成员一致，约14.7倍；防止越界编码别名，支持大小端/行填充/去重/非有限点过滤。66保护/20曲线与编码/24配置通过。探针15项通过，blocked仅接受未知/占据拒绝或曲线几何保护，拒绝其它准备失败蒙混通过。
- 当前唯一会话88261：observed_free_scenes_02（blocked→multi_goal→corridor→three_d），生产/工具冻结；在线管理器平均CPU已从约74%降到约25%，实机准备仍false。

- 19:33：唯一飞行会话30433，observed_free_scenes_01，blocked→multi_goal→corridor→three_d；生产/工具冻结。full_curve_scenes_04的multi_goal整轮通过，blocked仍失败（Z范围1.226928m/LIO误差1.171865m），会话17988退出1，尚未执行走廊/三维。
- 新根因证据：blocked_curve_audit_fast.json，第一完整曲线最高2.259989m、前3曲线未碰到当前占据体素，但走过未被扫描的墙体上方/背后；第4最高2.556472m才因越界停止。占据为空不等于观测自由，整曲线碰撞预检单独不足。
- 修复：桥接发布每帧点云配套的同时间FCU机体位姿；EGO ExactTime同步，按机体旋转/雷达平移确定射线原点，用DDA仅标记返回前的遍历体素。首次READY清占据与自由记忆。无人机配置将未知体素视为规划障碍，管理器要求目标/整曲线/即时线段均在已观测自由空间，并检查自由地图时效；其它上游默认兼容。
- 构建成功；66保护/24配置/16曲线检查通过。真实EGO回调6项全过（observed_free_callback_02）。callback_01测试对象共享Header导致所谓错时仍相同的夹具失败保留，已深拷贝修复探针，不是放宽同步要求。
- 当前尚未证明正常路线在保守自由地图下仍可执行，不能声称本修复完成。若四场景全过，须新完整核心+17扩展及界面/压力/输入故障，旧版本结果不可替代。

- 19:15：full_curve_scenes_04/multi_goal五目标/各20s悬停整轮通过，最大悬停Z范围0.080805m，融合/包络/降落上锁通过。当前blocked，之后corridor/three_d；唯一会话17988。生产指纹54e300cdb9fb05b53d4b6ec44fae1b610498e393633146575ebd3e39e8df9583，源码/工具冻结。

- 19:11：full_curve_scenes_03 blocked整轮通过，但multi_goal第五目标发生整曲线时间门控取消（前四目标和悬停通过），最终降落/融合/几何通过、整轮失败；会话23686结束。bag中第五曲线150.294s，地图149.996s，消息新鲜，管理器时钟到达顺序尚未直接记录，不能以bag时钟替代本节点时钟。已为失败日志增加具体age/map_age。
- 修复稍早到达的整曲线：几何检查后进入待激活，时钟达到起始时间且地图仍新鲜才允许执行；明显未来、激活前过期、目标代次改变均拒绝。62保护/13曲线/24配置测试通过（完整C++地图fixture，无跳过）。当前唯一会话17988：full_curve_scenes_04，顺序multi_goal、blocked、corridor、three_d。生产/工具冻结；若四项全过，重新跑完整核心和17项扩展。

- 当前唯一飞行会话23686：full_curve_scenes_03，blocked已整轮通过，正在multi_goal；之后corridor/three_d。生产/工具冻结。blocked在执行前拒绝完整曲线，理由为膨胀体素相交，20s悬停Z范围0.066555m、LIO位移误差0.043325m，降落/融合/几何均通过。整曲线日志wall=0.1168s、CPU=0.0082s、26节点。
- 当前预算为wall0.25s、线程CPU0.05s、10000节点；空间/地图/跟踪门槛未变。修复catkin入口辅助模块导入；不可达探针等待实际NAV再HOLD，避免不同话题连接的旧状态竞态。14探针/13曲线/57管理器检查通过。full_curve_scenes_01启动导入失败与02探针竞态失败均保留。
- 若四项定向复测全过，启动core_live_18完整核心及extended_live_07全部17项；旧ec4版本核心不能代表新整曲线版本。节点故障/输入故障/GUI/压力项仍待执行。

- extended_live_06会话27332已停止：corridor、three_d整轮通过；blocked失败，进入地图超时HOLD后20s真值Z范围0.884818m、LIO位移误差0.583569m。最后清理降落与融合/几何通过，但整轮仍失败。所有后续节点/输入/UI/压力项未执行。
- 三维最小包络间隙0.151790m、最大跟踪0.222434m；走廊最小间隙0.212833m、最大跟踪0.188566m。均为旧ec4b6f...版本。
- 原始传感器复现blocked_input_diagnostic_01同样失败；地面点从80s约288/扫描降至86s约22，88s后零，随后高度残差扩大。旧EGO第一条完整曲线已有10个体素碰撞片段，第四/第五曲线最高2.602/2.527m超过2.5m执行边界；旧控制器只逐条验证即时目标，未预检完整曲线。详见full_trajectory_guard/old_blocked_path_audit.log与blocked_ground_visibility.json/png。
- 修复：flight_manager订阅B-spline，通过逐段Bezier凸包递归检查连续曲线与膨胀体素/完整飞行体积；默认最多10000子节点、0.1s墙钟、非法输入拒绝。只有获批曲线ID的PositionCommand可以执行；验证在锁外，30Hz定时目标继续。源码与依赖已更新，24模型/57保护/13曲线检查通过，构建完成。
- 当前唯一飞行full_curve_scenes_01（最新会话见工具调用），顺序blocked、multi_goal、corridor、three_d。若全过，须新版本完整核心+全部扩展；旧核心通过不能代替新版本。禁止同时启动第二套ROS/PX4/Gazebo或修改源码/工具。

- 断电中止extended_live_05/corridor，无完整scenario/降落/ULog审计，不算通过。原telemetry.bag.active保留；power_loss_recovery_1549副本已reindex并恢复，部分几何审计在离线会话79572执行。没有修改旧通过/失败报告。
- 权限短暂变为managed时TCP/UDP均EPERM，无法飞行；用户随后恢复danger-full-access，已再次实测TCP/UDP可用，无残留仿真。24项模型、54项保护、12项工具复核通过。
- 当前唯一飞行会话27332：extended_live_06剩余16项，顺序corridor、three_d、blocked、boundaries、kill_planner、kill_traj、kill_lio、kill_bridge、kill_manager、kill_mavros、mavlink_drop、pose_jump、timestamp_regression、clock_reset、gui_camera_loss、stress。生产ec4b6f871ac03d373f0ab5f383a90eafc7420f0ced97be7c1c7a4570ac750def与通过核心/多高度一致，工具SHA也一致。源码/工具冻结，出现失败先审计修复，不跳过失败。
- 最终汇总须合并同版本core_live_17、extended_live_05/multi_goal和extended_live_06完成结果，不把已中止走廊bag当成完整通过。

- core_live_17主18277已退出0，完整核心全部通过；雷达0.466s/反升0.000362m/漂移0.007292m；IMU0.664s/无反升/漂移0.021773m；撤销0.068s/无反升/漂移0.028221m；OFFBOARD原生0.596s/反升0.004628m/漂移0.035608m；低电量3.292s/反升0.005297m/漂移0.034429m。全部最终上锁及融合审计通过。
- 原等待监督54549意外退出143且无输出，扩展目录未创建，原因未证实；当前重新启动run_sim_campaign同一核心版本直接进入extended_live_05（最新会话见工具调用）。生产当前指纹ec4b6f...和通过核心一致，未改源码，不重新解释旧失败。当前禁止第二套仿真和源码/工具变化。

- core_live_17三轮整轮导航、融合/包络均通过，优化图表生成，最大跟踪0.181943/0.204246/0.188999m；前后60s Z范围0.085710/0.110769、0.083566/0.113512、0.085849/0.104370m。已查看gui_nominal.png，体素/轨迹与两路处理图像可见。当前fault_lidar_attempt_01，主18277/监督54549不变，生产ec4b6f...冻结。

- 最新主会话18277 / 监督器54549：core_live_17新版本完整核心运行中，通过后执行extended_live_05全部17项。生产指纹ec4b6f871ac03d373f0ab5f383a90eafc7420f0ced97be7c1c7a4570ac750def。唯一仿真，源码与工具冻结。
- ros_spawn_low_battery_short会话53154整轮通过，飞前只有20s悬停；低电量3.160s保护、反升0.002569m、漂移0.049623m、最终上锁，ULog通过，不代替完整60s回归。
- ROS生成分支成功/明确失败/挂起三个实际shell分支检查通过（模拟ROS命令，测试超时缩短）；失败后TERM、2s后KILL并wait，防止世界进程令清理无限等待。prepare最终幂等与bash语法通过，24项配置与54项保护通过，ros_model_spawn_fix证据已保存。
- 新实机交接文件src/drone_stack/HARDWARE_ACCEPTANCE.md，记录实机外参、时钟同步、真实点云/IMU格式、电池补偿和动力控制等必需差异，仍标未验证。

- 主会话22339已失败退出，监督器53479因生产变化退出；extended_live_05未启动。low_battery_attempt_01世界在跑但模型缺失，gz model发现/生成请求无响应，PX4未启动。无飞行，ground_safety和land均失败，证据保留。已TERM停止拥有该世界的启动器，ROS/Gazebo已退出。
- 生产prepare_px4_sitl对inspection_quad/ROS1使用timeout60s的gazebo_ros spawn_model服务调用，失败停止世界并退出；其它模型保持上游路径。验收timeout若无无人机记录模型名而不再次抛ValueError。prepare两次幂等和bash语法通过，24模型/54保护复核中；当前ros_spawn_low_battery_short仅20s悬停，是诊断不是完整回归。
- live_16已通过：雷达0.524s/无反升/漂移0.012413m；雷达IMU0.768s/无反升/漂移0.026012m；授权撤销0.014s/无反升/漂移0.019281m；OFFBOARD原生0.608s/无反升/漂移0.027960m。整轮仍失败，不能宣称完整核心通过。

- 当前主会话22339 / 监督器53479；生产d205216d...未变。core_live_16三轮整轮导航和ULog通过，三轮包络通过，最大跟踪0.176860/0.180261/0.193264m。前后60s Z范围：0.087548/0.107748、0.089866/0.109454、0.087303/0.109898m。当前fault_lidar_attempt_01，17项扩展仍待核心结束。不可修改源码或工具/启动第二套仿真。

- ready_gate_scenes_01两场景完整通过，会话47676结束，无残留仿真。走廊28111真值样本、零包络重叠、最小间隙0.213581m、跟踪最大0.181671m；三维零重叠、最小间隙0.140671m、跟踪最大0.228032m，融合/降落上锁通过。修复前后地图切片已保存cloud_memory_ready_fix/ready_gate_occupancy_comparison.png并查看。
- 最新唯一飞行是core_live_16，生产指纹d205216d4f5c080ab741ed31f336f924f35e1630d8e30e37b3a505022695d35d；监督器等待同版本核心全部通过后执行extended_live_05全部17项。源码与工具冻结，实机准备仍false。

- 新构建生产指纹d205216d4f5c080ab741ed31f336f924f35e1630d8e30e37b3a505022695d35d；构建和24项模型/54项保护/12项工具检查通过。真实EGO回调ready_gate通过，证明首次READY清初始化残留，之后HOLD/READY保留盲区旧障碍。实验工具默认无门控参数时仍可对照retain/replace。
- 当前唯一在线会话47676：ready_gate_scenes_01顺序corridor/three_d；走廊场景导航往返已通过，正在降落及整轮审计。禁止同时启动另一套仿真或修改源码/工具。完成两场景后执行core_live_16与extended_live_05全部17项。
- core_live_15三轮导航包络审计全部通过，最大跟踪0.192850/0.132558/0.174318m；optimization_comparison图表已生成。旧通过只代表旧版本。

- 2026-10-03恢复核对：原主会话13796/监督器35395均已结束，无残留ROS/PX4/Gazebo。core_live_15整轮通过，原生产指纹4e9644da...；extended_live_04/corridor首次目标无轨迹，5秒触发EGO-Planner trajectory timed out，地图在82.654s/FCU高度0.897m存在入口与中心虚假占据。失败bag/ULog/几何通过记录保留。
- 当前正在增加地图初始化门控：drone配置订阅flight_state，首个READY清除初始化阶段地图并开始永久静态记忆；之前按当帧替换，之后HOLD/READY不得清除。其它上游应用无门控参数时保持原行为。生产已变化，旧核心通过不能代表新配置全回归通过。

- **地图记忆修复后的三维短复测会话32143已整轮通过并退出。新完整核心`20261003_core_live_15`和监督器`run_sim_campaign.py ... extended_live_04`已启动（exec会话见最新调用）；生产版本4e9644da3bb8b8000095183bf91bc39d0ac1cdad2837c6b1631df21748cce489，当前核心首轮。禁止第二组仿真与源码变更；核心全过后监督器重跑全部17项扩展。**
- 新安装位置输入诊断对照图已保存head_lidar_fix/head_mount_ground.png与ground_visibility_comparison.json：飞行真值高度≥1.5m的68个扫描，地面点至少1428、中央値1773；LIO与真值高度诊断残差最大0.05708m。旧中央安装的同类扫描中央値12、最小0，最大残差2.414m；这是近似时间配对诊断，不代替验收。
- 新地图记忆三维复测`20261003_cloud_memory_three_d_01`整轮通过：四目标/各20s悬停/降落上锁/融合审计通过；44076个真值样本零包络重叠，最小分离轴间隙0.121477m（上方梁），最大跟踪偏差0.207185m，真值最大间隔0.052s。旧缺陷已实测修复，但新配置完整核心与全部扩展仍待结果。
- live_03三维复测前三目标/悬停通过，但低位返程遭轨迹跳变保护，且包络审计有1219个low_barrier重叠样本，不能只报“保护有效”。根因：EGO cloudCallback每帧resetBuffer清掉旧障碍；已经扫描的低障碍转到前倾雷达后方盲区后被遗忘。旧失败bag/ULog/GUI完整保留，最终清理降落通过。
- 修复EGO：可选retain_cloud_obstacles，drone配置true，缺少自由空间证据时保留静态占据；publish安全地图包含当前局部窗口所有已记忆障碍，不因当帧点云边界窄而遗漏。上游默认false，当前没有自由空间射线清除/动态障碍移除模型，重启地图清空。地图freshness仍来自实际新点云，未改变超时和膨胀/跟踪门槛。
- 实际EGO回调两模式对照（无飞行）均通过：retain保留旧障碍，replace删除旧障碍；旧时间戳和空点云不刷新地图。固定构建指纹一致；23项配置/模型与54项保护检查通过。完整编译完成。
- 扩展live_02：multi_goal/corridor整轮通过；three_d第二高位目标近上方梁体膨胀体素，触发保护HOLD但误被工具以goal_error<0.2认作到达，后悬停XY=0.205m/LIO误差0.113m超限，整轮失败保留。降落/融合/几何通过，不能把保护取消当成功。
- 工具修复：记录flight_error事件序号，到达需HOLD/OFFBOARD并无本目标的新保护原因，等待0.25s跨话题原因传递；12项检查通过。三维测试高位路线改[.5,0,.45]、[2.7,0,.45]后再低位穿梁，场景/障碍/保护门槛未改变。当前15项使用新工具快照，生产版本未变。
- live_14基础故障全部通过并有ULog：雷达0.554s、反升0.004723m、漂移0.018519m；雷达IMU0.720s、无反升、漂移0.008984m；授权撤销同一个仿真时间刻保护、漂移0.020513m；原生OFFBOARD失流0.544s、无反升、漂移0.034654m；低电量2.494s、反升0.007347m、漂移0.038644m。最终均上锁。核心status=core_passed_remaining_acceptance_pending。
- live_14三轮完整导航和ULog均通过；三轮前后60s Z峰峰值0.085992/0.103084、0.086698/0.107802、0.082995/0.108825m，LIO最大位移误差均≤0.015240m。前两轮全场景包络审计通过，最大跟踪偏差0.190024/0.183260m；第三轮包络审计也通过，最大跟踪偏差0.189507m；图表已完成。五种基础故障现已通过，扩展仍待结果。
- live_14 round_01完整导航/降落/ULog通过：前后60s悬停Z范围0.085992/0.103084m，LIO位移误差0.009681/0.014077m；全场景包络审计另行进行。
- 多高度短复测head_lidar_multi_goal_02：五目标/每次20s悬停/降落/融合均通过，悬停Z范围0.044282～0.086289m、LIO误差最大0.022353m；落地漂移0.038519m。但额外原始传感器密集录制时真值最大记录间隔0.112s超过0.1s，整轮严格保留失败。工具增加--diagnostic-sensors选项，标准验收用原轻量记录量，未放宽几何门槛。
- head_lidar_multi_goal_01因测试服务响应和相位话题不同连接，旧HOLD被误判取消；bag证明管理器从NAV直接收到工具LAND请求，不是飞行保护HOLD。工具新增等待NAV转换，10项检查通过。旧失败保留。
- 当前指纹/启动快照新增Faster-LIO源码与二进制，以完整记录算法版本；54项飞行保护和22项模型/坐标离线检查通过。第一次保护检查误用了不存在的临时fixture目录，失败保留，改为保存的实际C++fixture后54项全过且无跳过。
- 根因：旧雷达传感器位于机体中央上方0.10m，距机身顶面只有0.045m，向下射线大多打到自身；升高后余下浅角射线先打到墙而看不到地面，造成高度约束退化。原始点云真值投影复现地面点降到零，详见diagnostic_01/raw_geometry_observation.jsonl与原始传感器bag。
- 修复：仿真雷达/独立IMU/生成器/桥接/静态TF改为[0.27,0,0.10]m，前倾30°不变；实际硬件平移外参待测量。解析20160条射线中向下4638条，机身遮挡2752→0；22项模型/坐标检查通过，在线多高度尚待结果。
- 扩展失败：第二个抬高目标到达后，Gazebo真值高度从约1.78m持续升到4.72m，而LIO/FCU高度长期约1.44m。20s悬停高度范围3.004m、LIO位移误差2.619m，不通过。清理降落最终上锁，但水平漂移0.627m超限，未掩盖失败；完整bag/ULog已保存。需要查定位高度失真根因，不能宣称实机就绪。
- 同时发现Qt AT-SPI同一进程有空/非空两份登记；gui_snapshot已按唯一PID选择非空树，8项离线检查通过，待实际GUI检查。扩展工具加录原始传感器话题，工具版本已变化。
- 核心最后两项：真实OFFBOARD目标流丢失原生反应0.596s，反升0.002923m、漂移0.034458m；低电量3.308s，反升0.006063m、漂移0.033287m。最终均上锁，ULog通过；低电量完整60s前置悬停已完成。核心报告status=core_passed_remaining_acceptance_pending，扩展/实机准备尚未通过。
- 当前故障：雷达反应0.616s、反升0.004934m、漂移0.010080m；IMU反应0.688s、反升0.000308m、漂移0.011526m；授权撤销反应0.008s、反升0、漂移0.024485m。三项均最终上锁且ULog审计通过。
- 当前版本三轮导航均完成起飞、前后60s悬停、绕障到达、降落上锁和ULog审计；三轮场景包络离线审计完成。前两轮悬停Z峰峰值分别0.084284/0.122528m、0.107808/0.125404m；最大跟踪偏差0.175781/0.178481m。优化图表已保存于live_13/optimization_comparison。整个核心现已通过，扩展尚待实际结果。
- live_12核心最后低电量项失败并结束；前三轮导航和四种其它故障均通过，不能判完整核心通过。会话52646与监督器80726均已退出。
- 低电量失败已定位：SIM_BAT_MIN_PCT=0、SIM_BAT_DRAIN=1实际写入成功；电压跌至14.4V，但无电流/负载模型的模拟器叠加油门压降补偿，使报告剩余电量停在0.154778，仍高于BAT_LOW_THR=0.15，更未到临界0.07。battery_injection_diagnosis.json保存实际ULog参数/范围/告警及源码SHA，未回写旧失败。
- 仿真airframe新增BAT1_V_LOAD_DROP=0：模拟器给的是开路电压，不应再补偿压降。仅仿真参数，真实电池需标定补偿；BAT_LOW/CRIT/EMERGEN阈值和COM_LOW_BAT_ACT=2不变。故障工具增加参数写入返回值核验。
- `20261003_low_battery_corrected`短复测通过（飞前20s悬停，不是完整60s回归）：2.560s触发降落，反升0.010865m，漂移0.031947m，落地上锁；ULog融合审计通过，真实电量最小0.055820，battery_status和failsafe_flags告警码0/1/2。新配置完整回归仍待完成。
- 新变化只涉及仿真电池补偿与注入回读，定位/ENU坐标/外部速度禁止融合/高度协方差/规划保护不变。离线模型/LIO21项通过。

- **当前主会话52646 / 监督器80726不变，版本9711d411f8cd5c458b57fb39457b7a43b87df26b3c42cc548c9323359e15d096。live_12三轮完整导航和雷达/IMU中断、撤销授权、真实OFFBOARD目标流丢失均通过，正在low_battery。** 扩展监督器尚在等待，不存在第二组ROS/Gazebo。
- 三轮前/后60s悬停Z峰峰值：0.093664/0.127644、0.106864/0.107769、0.104988/0.123991m；三轮ULog和全场景包络几何审计通过。跟踪最大偏差0.178372/0.178184/0.196660m。
- 当前故障实测：雷达反应0.548s，无反升，漂移0.020431m；IMU反应0.702s、反升0.013190m、漂移0.006043m；撤销授权反应0.026s、无反升、漂移0.021658m。均落地上锁，ULog禁止融合源保持0。
- 真正目标流丢失：停止管理器后PX4最后记录输入122.672s，123.264s进入failsafe AUTO_LAND，原生0.592s（≤1.5s）通过；ROS模式观察0.660s，无反升、漂移0.035173m，落地上锁。offboard_native_audit.json是必需验收项。
- 优化图表与数值来源保存于live_12/optimization_comparison。旧位置单独控制最大偏差0.685187/0.713614m；当前位置+期望速度前馈三轮0.178372/0.178184/0.196660m。不是单变量因果实验；不据图表宣称其它故障/实机通过。
- **检查点已更新：start/checkpoints/drone_sim_checkpoint_2026-10-03_0526.tar.gz（3.1GiB），latest为指向它的符号链接。** 包含源码/工具依赖、历史实验和live_12三轮闭合bag/ULog，不包含当时正在运行的基础故障轮次。gzip CRC通过，1170个选定源码及闭合数据文件SHA256一致；tar仅因活动实验父目录mtime变化返回1，内容另行严格核对。verification_final.json与archive.sha256已保存。
- 历史20261002_aligned_pose/telemetry.bag.active仍以失败/未闭合证据保留在归档，未计为完成飞行。不能把它和当前正在录制的active bag混淆。归档快照早于后续工具小改动，当前工作区与最新进度文件仍是续作依据。
- 位姿/时间输入故障工具新增桥接ROS错误日志匹配，必须证明触发的是对应的跳变/时间倒退保护，不能以任意invalid=true替代该项覆盖。扩展bag新增/rosout。生产版本未变化。

- `live_11` round_02导航成功后EKF状态年龄2.014s触发保护落地。原始bag状态以约1Hz持续发送，位置有效标志正常，ULog全部空中外部位姿/高度/航向融合且禁止源0；不是Faster-LIO大幅定位错误。旧未来样本直接丢弃使低频状态流出现假超时。
- 已修复管理器：小幅超前的EKF状态只暂存（最多100ms），在ROS时间真正达到采集时刻后才提交；不能用暂存样本刷新健康，超过100ms未来不缓存。未来20ms原接收容忍和状态2s超时均不增加。54项管理器检查通过，包含暂存/不提前使用/超大未来拒绝；21项模型与LIO检查通过。
- **当前主进程exec会话52646：`20261003_core_live_12`，配置指纹9711d411f8cd5c458b57fb39457b7a43b87df26b3c42cc548c9323359e15d096。round_01、round_02完整导航+前后60s悬停+降落上锁+ULog通过，round_03起飞过程中。不要启动其它ROS/PX4/Gazebo或修改生产源码。** 首轮高度峰峰值0.093664/0.127644m，LIO位移最大误差0.027179/0.032838m。
- **当前扩展监督器exec会话80726**等待live_12同配置核心全部通过，然后运行`20261003_extended_live_01`共17种测试。旧监督器6018已因live_11失败退出。工具仍未进入扩展飞行；其源码SHA会在实际开始时固定，开始后禁止修改工具。
- 两轮几何审计exec会话92558正在离线处理已闭合bag；这是读取数据，不是第二组仿真。
- Qt可访问性验证：本机默认IsEnabled/ScreenReaderEnabled均false，当前默认启动的GUI树为空。临时状态开启无法激活旧Qt树，已恢复所有桌面状态；独立PyQt窗口设置QT_LINUX_ACCESSIBILITY_ALWAYS_ON=1后，确实能跨进程读到真实按钮，测试窗口已自动关闭。扩展GUI启动已设置该环境变量，仍需实际故障按钮检查。
- 工具离线结果：扩展场景8项、UDP转发/丢弃/不重放2项、GUI严格断言5项、原生OFFBOARD审计5项均通过，证据在`20261003_extended_tools`。离线结果不得等同于在线故障通过。

## 此前阶段记录

- `20261003_core_live_09` 已结束：三次完整导航冷启动、雷达短暂中断、雷达IMU短暂中断、空中撤销授权均通过并有ULog融合审计。全部GPS/气压/外部速度禁止融合源保持关闭。
- 三轮飞前/到达后60s真值Z峰峰值：0.093479/0.124127、0.094435/0.121715、0.113947/0.128559m，均小于0.15m；三轮场景包络几何审计均通过。第三轮51945个真值样本，最小分离轴间隙0.239967m。
- 雷达/IMU短暂中断反应仿真时间0.476/0.524s，降落无反升，水平最大漂移0.020114/0.035862m，最终自动上锁。这些实验恢复传感器后继续降落，不代表永久定位丢失能力。
- 原offboard_loss测试失败原因：AUTO.LOITER需要全局位置；第一次改POSCTL也被拒绝，因为其需要遥控输入而本项目已关闭遥控。两个失败实验均安全清理降落，原始记录保留，未判通过。ULog command176的result=1证明请求被拒绝，MAVROS mode_sent不能当作飞控接受。
- 当前offboard_loss测试已改为停止唯一目标发布节点`/drone_flight_manager`，保留LIO和MAVROS，核验实际PX4 AUTO.LAND/AUTO.DESCEND及落地上锁，测试真正的目标流丢失，而不是模拟模式请求。
- `20261003_offboard_stream_loss`已结束，原在线门槛判失败：ROS模式观察延迟1.508s；实际无反升、漂移0.031713m、落地上锁。新增ULog独立审计确认PX4最后记录目标99.002s，99.610s进入failsafe AUTO_LAND，原生反应0.608s（≤1.5s）。保留原失败，不回写为整轮通过。
- 当前测试把ROS模式观察等待与原生反应分开：在线目标流丢失观察≤3s，run_cold_start_tests另外强制ULog原生反应≤1.5s；未放宽飞控反应标准。故障真值轨迹现在从注入瞬间记录，而不是保护阶段后才开始。
- 修复明显ROS时钟倒退和LIO采集时间倒退：桥接锁定无效，管理器fresh_pose拒绝，两个Python定时器reset=True保证不因时钟异常死亡；fallback下降dt限[0,0.1]防止负dt反升。52项管理器检查通过（带C++真实地图fixture、无跳过），21项模型/坐标/LIO检查通过。
- 系统MAVROS默认required=true会在节点退出时关闭整个ROS链路；stack.launch现使用标准PX4配置文件的独立required=false节点，禁止该连带退出。启动器DRONE_FCU_URL可选覆盖，默认URL不变；用于实验UDP代理，生产配置指纹新增GUI源码/二进制与包配置。
- `live_10`首轮导航与ULog通过，但在落地上锁后人为停止，以应用MAVROS启动修复；不是三轮或完整核心通过。
- **当前运行：`20261003_core_live_11`，exec会话51181，配置指纹a585d043791a191f6ca07159bba2cea73dab25bb06211c14d4907f5a2f212c80。round_01完整通过并有ULog与几何审计，round_02在悬停。不能并行启动其它ROS/PX4/Gazebo，不能修改生产文件。** 首轮几何51995样本、最小分离轴间隙0.225370m，最大跟踪偏差0.193042m。
- **后续自动监督器已运行：exec会话6018，`run_sim_campaign.py`等待live_11同一配置核心全部通过，才启动`20261003_extended_live_01`的17种扩展验收；核心失败则停止，不跳过失败。** 无通过结果前不得标记实机可用。
- `start/tools/extended_flight_probe.py`和`run_extended_scenarios.py`新增多高度连续目标、corridor、three_d、blocked、boundaries；planner/traj/LIO/bridge/manager/MAVROS永久退出；双向MAVLink丢包；位姿跳变/采集时间倒退/ROS时钟重置；GUI相机断流；10次往返加600s悬停压力验收。通过生产服务发ENU目标；每轮冷启动、bag、ULog、全场景包络检查、清理降落，失败即停止。工具8项、UDP代理2项、GUI断言5项离线检查通过，扩展飞行尚未执行。
- 独立NativeMonitor已实测UDP14550读取PX4 custom_mode=393216、base_mode=145、OFFBOARD/armed=true；旧pymavlink高层模式解释会对该base_mode返回UNKNOWN，实验工具按本机PX4v1.15源码枚举直接解码并保留raw字段。不是ENU/NED坐标转换。pymavlink2.4.49安装于start/tools/python_deps，安装日志已保存；未改系统包。
- 新GUI工具在真实故障发生时截图、读取Qt AT-SPI按钮enabled标志并严格检查；无可访问树不能判GUI通过。扩展启动设置QT_LINUX_ACCESSIBILITY_ALWAYS_ON=1用于读取，尚待实测可访问性。扩展工具源码快照、SHA、依赖版本另行保存；执行期间也禁止更改工具源码。
- 已实际截图`live_09/fault_imu_gui.png`：LANDING时解锁/起飞/悬停/取消禁用，一键降落保留，两相机与体素可见，底部健康故障原因可见。截图时传感器已恢复，因此不能据此勾选LIO-invalid显示；雷达截图观察器错过故障并超时，未记录为成功。
- 下一步：目标流丢失复测→低电量→新版本完整核心回归→多高度/复杂场景→节点和通信丢失、位姿/时间异常、GUI故障与长期压力。实机准备仍为false。

## 不可改变的约束

MID360 前倾 30°，独立雷达 IMU，前视/下视相机；ROS ENU/FLU，MAVROS 负责协议坐标转换。
Faster-LIO 位姿变换到机体中心后发送 `/mavros/odometry/out`。PX4 `EKF2_EV_CTRL=11`、`EKF2_HGT_REF=3`；外部速度、GPS、气压高度融合关闭。PX4 IMU 参与状态传播和速度估计，不能字面上从位置传播中移除。
真实检测模型、真机标定与飞行另行验证；全局任务/扫描仍按原约定待需求明确，不把局部规划称为全局寻路。

## 新增的实验和验收工具

- `src/drone_stack/SIM_ACCEPTANCE.md`：指标和待办。
- `validate_simulation.py`：起飞、悬停、轨迹检查、执行、全过程降落、双相机 TF/处理图像、地面/空中非法操作、雷达/IMU 中断、授权撤销、退出 OFFBOARD、低电量测试。
- `run_sim_regression.py`：执行单轮测试，保存输出/返回码，失败时降落；`run_cold_start_tests.py`：管理完整冷启动与原始 ROS bag。
- `test_flight_safety.py`：42 项独立状态/指令、轨迹和跨语言消息检查已通过（2026-10-03），不能等同于真实异常飞行验收。
- `analyse_sim_experiments.py`：汇总实验数据并生成对比、悬停和降落图表。
- 新实验位于 `start/experiments/`；旧基线仍在 `start/validation/`。不得覆盖失败实验或以旧基线代替当前验收。
- 统一启动器会将当次 config/launch/scripts/patches、EGO map 源码和头文件保存到 source_snapshot，并记录 EGO 二进制 SHA256；之后改文件不影响已有快照。

## 已修复/改进

1. Faster-LIO 初始化阈值支持 ROS 参数 `mapping/imu_init_samples`，仿真设 400，算法默认仍 20。
2. MAVROS 仿真用 PASSTHROUGH 并关闭主动 TIMESYNC：实测原 near-zero 负偏移样本溢出到约 2^63，估计达到数亿秒。真实硬件不能照搬透传；需要正常时钟同步。
3. PX4 在 500 次 timesync 交换前会把外部位姿采集时刻替换为到达时刻；仿真管理器等待到 65 秒仿真时间才允许 READY。
4. 修复未解锁心跳把 LOCALIZING 错改 READY；允许 20 ms 有界未来时间偏差；LIO 健康心跳超时 0.5 秒不能继续沿用旧的 valid=true。
5. LIO 仿真位姿协方差为位置 0.0025（5 cm 标准差）。最新将 PX4 `EKF2_EVP_NOISE` 下限从 0.10 改为 0.03 m，以使消息协方差实际生效；v3 首轮 60 秒悬停通过，但整轮因地图过期拒绝导航而未通过。
6. Gazebo FCU IMU 改为完整 250 Hz HIL 窗口的物理速度差比力，保留原噪声和随机偏置。修正源码完整保存在包的 patches/gazebo_imu，prepare_px4_sitl.py 幂等复制到固定 PX4 版本。
7. 落脚碰撞增加 kp=1000、kd=40、max_vel=0.1、min_depth=0.001、零回弹。改善地面接触冲击；修正后雷达 IMU 初始化模长 9.81009，原值可达 10.1983。
8. 添加地面上锁接口；撤销地面授权时请求上锁；非法起飞高度和目标飞行范围拒绝；到达目标并低速保持 1 秒后自动 HOLD。
9. 增加目标占据体素和地图新鲜度检查。仿真适配器提供 lidar/imu 故障注入服务，仅用于仿真。
10. 两相机增加光学 TF，消息头为 front_camera_optical/down_camera_optical；仿真红色目标处理节点发布 image_processed 和像素框。Qt 已改看处理画面，并按状态启用按钮、增加地面上锁和授权状态文字。
11. 生成 corridor/3d/blocked 三个测试场景，尚未完成飞行验收；DRONE_SIM_WORLD 可切换场景。
12. PX4 `COM_LOW_BAT_ACT=2`，临界电量降落，低电量故障测试尚待执行。

## 实验结果（不可混为全部通过）

| 实验 | 结果 |
| --- | --- |
| init400 | 早期 1 m 起飞真值升高 1.80 m；30 秒悬停 Z 峰峰值 0.5075 m，不合格 |
| passthrough | 时间大偏移消失；早期起飞未完成 HOLD，不合格 |
| aligned_pose | 起飞真值 0.9808 m；随后旧时间判据误触发降落，整轮不合格 |
| regression_01 | 90 秒悬停 Z 0.1981 m，LIO 位移误差 0.0470 m；降落曾反升并水平漂移，不合格；双相机处理和 TF 通过 |
| imu_interval | 90 秒悬停 Z 0.1584 m，LIO误差0.0327 m；降落反升2.03 m，不合格 |
| compliant_feet | 短回归通过：30 秒悬停 Z 0.0610 m，LIO误差0.0150 m；降落无反升，水平漂移0.0226 m，自动上锁 |
| cold_regression | 自动工具等待启动过短，未起飞，已修正初始等待到120秒 |
| cold_regression_v2/round_01 | 起飞/相机/非法指令通过；60 秒悬停 Z 0.1661 m 不合格；降落无反升，漂移0.0056 m，通过 |
| cold_regression_v3/round_01 | 60 秒悬停通过：真值 Z 峰峰值0.108343 m、LIO位移误差0.032543 m；轨迹体素检查1500采样零碰撞；flight 被 Need fresh obstacle map before navigation 拒绝，整轮失败，未执行后续两轮；降落通过 |

降落验收新增全过程指标：反升不超过0.15 m、水平漂移不超过0.30 m，最终落地上锁。不能只看最后位置。

## 2026-10-03 已完成的阶段

- 源码明确复现根因：`pcl::toROSMsg` 接收到没有 stamp 的临时体素云，生成的 `/grid_map/occupancy_inflate` 时间为0。管理器拒绝无时间戳地图的保护没有被删除。
- EGO MappingData 新增实际观测时间；成功处理 lidar cloud 后更新，点云的 ENU frame、非零时间、时间递增、有效点检查通过后才使用。深度路径在地图更新完成后提交图像采集时间。三个地图消息发布函数统一使用 makeMapMessage，保留纳秒时间。
- 重复显示旧地图不会刷新采集时间。缺少新数据/时间倒退后不能伪装成新地图；重新启动并重建地图才恢复。
- 管理器复用 fresh_map，导航中地图超时取消导航、转 HOLD，并在同一周期继续发布悬停目标，维持 OFFBOARD 数据流。
- 冷启动记录增加 `/clock`、`/drone/cloud_fcu_world`、`/grid_map/occupancy_inflate`、bspline、规划开关、目标、estimator_status、timesync_status、外部odometry。上一轮 bag 缺少地图和时钟，不能直接测量其历史地图时间戳。
- 完整 catkin_make 编译通过；C++ 离线消息目标编译通过，7 项检查通过；Python 管理器24项检查通过、无跳过。其中 C++ 生成实际 ROS 序列化点云，Python 反序列化交给 on_map：旧零时间被拒绝，新时间保留、占据目标拒绝、空闲目标接受。
- 旧 v3 bag 离线审计：解锁且 OFFBOARD 时记录的目标消息最大间隔约0.074秒；这不证明每条消息都被 PX4 接收，也不是新的飞行实验。
- 更新实验汇总图表 `start/experiments/EXPERIMENT_COMPARISON.md` 和 `height_comparison.png`。
- 报告、fixture、构建/检查日志、源码差异：`start/experiments/20261003_map_timestamp_fix/`。离线通过不得当作导航执行/三轮冷启动通过。

## 2026-10-03 执行保护阶段（当前源码）

- 已补齐执行命令检查：采集时间、ENU frame、有限数值、飞行范围、0.75 m跟踪跳变上限、当前机体到目标点的体素穿越。DDA同时检查角点和网格面相邻体素；500条随机线段与独立闭合包围盒算法对照通过。
- 轨迹有过期数据时0.5秒转HOLD，不再等待初始5秒规划超时；到达/超时/保护切HOLD的同周期继续发布目标。
- 新增未裁剪高度的 `/grid_map/occupancy_inflate_safety` 用于执行保护和dry检查。原 `/grid_map/occupancy_inflate` 保持显示裁剪，Qt仍显示该体素话题。避免使用显示云代替完整安全地图。
- 虚拟天花板2.6 m已加入点云建图路径，规划查询阻止在天花板上方寻路；ceil索引校验防止越界写入。执行目标仍限制z≤2.5 m。重复地图时间不变时跳过重复Python解析，C++订阅状态只查询一次，降低新增数据开销。
- LIO时间检查允许未来偏差仅20 ms；健康判据不再把未来位姿判为有效。位置跳变上限0.5+3*dt m、角度上限0.3+4*dt rad；跳变拒绝转发并锁定无效，需落地后重启桥接和检查原因，不能自动接受新跳变原点。
- Faster-LIO、桥接、管理器不再设required=true；其中一个退出时其它节点保持运行，使管理器保护或PX4 OFFBOARD-loss降落有机会执行。实际故障飞行效果尚待验证。
- 管理器增加 `/drone/manager_heartbeat`。Qt用墙钟检查PX4/LIO/管理器/相机消息，新鲜度失效禁用普通操作；管理器不可用时，一键降落可直接通过MAVROS请求PX4 AUTO.LAND。仅编译通过，真实GUI/故障显示尚待联调。
- 回归工具捕获启动/超时异常并保留suite_result，降落清理允许连接120秒+下降55秒；land验证不依赖LIO/管理器状态话题，管理器缺失时可请求MAVROS AUTO.LAND。
- 每轮冷启动保存flight.ulg并执行audit_px4_ulog.py，未获取ULog不能判通过；仅雷达/IMU故障实验允许外部位姿融合在故障后退出，其它融合源全程必须禁用。bag采用无损LZ4。
- 新run_core_acceptance.py管理第一阶段三次导航冷启动+五种基础故障，保留配置/二进制指纹，代码变化后不能复用旧通过结果。它明确不代表全部场景/GUI/压力验收。

证据位于 `start/experiments/20261003_execution_guards/`：最终编译通过；管理器42、模型/坐标/LIO17、工具6、C++13，共78项离线检查通过，无跳过。
旧v3的1223条LIO里程计重构回放无误拒绝（不是重跑原始点云/Faster-LIO）；旧ULog 57个空中样本外部位置/高度/航向均融合，外部速度/GPS/气压等禁用源计数为0。原ULog副本reference_v3.ulg已保存。两项历史审计均不是本轮新飞行。

## 当前运行状态与继续步骤

### 2026-10-03 网络恢复后的在线实验

- `20261003_core_live_01` 首轮实际起飞：地面/空中非法操作、起飞、双相机、60秒悬停、轨迹体素预检、降落通过。起飞真值相对高度1.038076m，LIO位移误差0.012278m；悬停真值Z峰峰值0.107089m，LIO位移最大误差0.016010m；降落无反升，水平漂移0.038722m，落地自动上锁。
- 安全地图实测时间戳非零。10秒在线采样目标流最大仿真间隔0.062s，地图年龄最大0.388s。
- 导航执行开始后被 `Invalid or stale trajectory acquisition time/frame` 切回HOLD，整轮失败。bag显示指令frame=odom、时间邻近/clock，但不能替代管理器内部时钟诊断；未直接放宽时间保护。
- 首轮ULog审计通过：105个空中样本均融合外部位置/高度/航向，外部速度/GPS/气压等禁止源全程计数0；原始bag、ULog、截图和报告保留。
- 在启动器设OPENBLAS_NUM_THREADS=1，避免Python小矩阵唤醒多线程，LIO桥接进程CPU从约128%降至约5.5%。轨迹时间错误增加age/stamp/frame诊断，验证工具在保护取消目标后快速保留失败并降落。
- live_02/live_03分别在起飞/悬停后触发定位健康保护并落地上锁。live_03内部诊断确认是飞控里程计相对管理器时钟超前22ms，LIO健康和EKF标志正常；已记录clock没有倒退。不能将此误报称为Faster-LIO定位误差。
- live_04导航时间误报为恰好-20ms；复现Duration.to_sec()得到-0.020000000000000018，改为从整数纳秒转换，新增20ms允许、20ms+1ns拒绝的回归检查。管理器现45项检查通过。
- live_05仍捕捉到-22ms，确认在init_node之后设置TCP_NODELAY无法重新协商已经建立的/clock连接。当前源码在init_node之前预注册callback-free /clock订阅，保持rospy内置回调管理时间；控制订阅/目标发布同时设立即发送和queue_size=1，地图接收缓冲4MiB。未来容忍20ms、轨迹最大年龄0.5s均未放宽。
- live_06导航再次遇到26ms超前。独立时钟观察器记录大多数FCU消息年龄-4至12ms，短时clock调度墙钟间隔可达0.1s；只修传输选项不能保证多个TCPROS话题同步到达。
- 当前管理器丢弃尚未到时的FCU/estimator样本，保留上一有效样本，不能用未来数据刷新健康；没有有效FCU数据0.7s仍降落。EGO小幅未来指令被丢弃，上一有效指令仍须通过时间/体素/范围检查；有效指令过期0.5s或时钟偏差持续0.5s转HOLD并保持同周期目标流，超过0.5s的大幅未来指令立即取消。未来20ms容忍未放宽。新增检查使管理器共48项通过，LIO/模型17项通过。
- live_07实现实际绕柱到达并自动HOLD：23.85墙钟秒，最小真值柱中心距离0.961346m，最终位置距目标约0.037m；起飞相对高度1.041772m，LIO位移误差0.015050m，飞前60s高度峰峰值0.114066m。到达后60s高度峰峰值0.151183m，略超0.15m，因此整轮失败；LIO位移误差仍仅0.026294m，不能归为LIO大幅误差。降落/自动上锁和融合审计通过。
- live_07数据估计LIO高度相对真值残差标准差7-8mm，最大约25mm，已在仿真lio.launch将报告的Z标准差50mm降至30mm（方差0.0025→0.0009）；XY/姿态协方差、EKF融合源、0.15m验收门槛保持。实机需要独立标定，不直接照搬仿真协方差。调参依据height_tuning_basis.json位于live_08。
- live_08飞前悬停Z峰峰值0.105920m通过，导航在柱体拐弯处触发体素穿越保护而失败。bag重构机体到规划目标线段确实穿过体素[2.65,0.75,0.95]，不删除保护。旧管理器只发规划位置，实际跟踪偏差最高0.69-0.71m，拐弯连接线会切入膨胀边界。
- 当前NAVIGATING发送规划位置+EGO期望速度前馈，使用PX4支持的position+velocity setpoint；速度有限值与模长≤0.75m/s检查，机体到目标0.75m跳变/体素检查保持。该速度仅是控制目标，`/mavros/odometry/out`速度仍未知，EKF2_EV_CTRL仍11，无外部速度融合。其它阶段保持原位置/地面零速度控制。管理器50项、模型/LIO17项检查通过。
- 正在运行 `20261003_core_live_09`（exec会话66580），进程由run_core_acceptance.py管理。三轮导航中的round_01、round_02已完整通过，并各有ULog融合审计通过。首轮两次60s高度峰峰值0.093479/0.124127m，LIO位移最大误差0.016399/0.041741m；全部空中109条flags外部速度/GPS/气压均0。第三轮在飞前悬停，尚未结束；三轮后自动继续五种基础故障。不要另外启动ROS master。
- 新独立几何验收工具位于start/tools/sim_world_geometry.py、audit_world_bag.py，使用场景全部静态碰撞box和模型/link/collision层级姿态，车辆含传感器保守包络半尺寸[0.35,0.40,0.21]m（含30mm余量），支持真值姿态旋转和批量SAT检查。8项检查通过，含500个随机姿态批量/单点一致性；证据start/experiments/20261003_world_geometry。它明确是有限采样包络检查，不能冒充Gazebo接触传感器或连续碰撞证明。
- live_09首两轮原始bag几何检查已通过，首轮最小分离轴间隙0.235428m；跟踪偏差最高0.177953/0.184794m（按最新接收FCU位置估计，未插值），明显优于位置单独控制。工具/世界SHA256保存在world_geometry_audit.json；第一轮timing_audit.json也保存。
- Qt综合操作台已实际打开，雷达点云/EGO体素/双相机处理画面可见；初始化截图在live_02，悬停截图在live_03，到达目标截图在live_07。live_01至live_08均未整体通过，融合审计均通过，原始数据完整保留。
- 修改后管理器42项离线检查通过，无跳过（使用上一阶段编译的地图消息fixture）。

- 以下网络受限记录属于01:33时的旧会话环境，03:09后已经恢复实际飞行，不能再作为当前阻塞原因。
- 用户已授权执行和测试，不需要再次询问。旧会话禁止本地TCP/UDP套接字，旧preflight返回PermissionError / Operation not permitted。
- 最终启动记录 `start/experiments/20261003_core_acceptance_attempt_v2/core_acceptance_result.json`：status=blocked_environment，cases=[]、passed=false、real_hardware_ready=false。
- 上次实际仿真仍是2026-10-02 v3，落地上锁；本轮所有启动和测试进程已结束，没有运行中的ROS/PX4/Gazebo。
- ROS本地通信可用后，先重启完整仿真，不能热加载旧节点混用新安全话题。使用新目录运行：

```bash
source /opt/ros/noetic/setup.bash
source /home/d/robotproject/project0/devel/setup.bash
unset ROS_IP
export ROS_HOSTNAME=127.0.0.1 ROS_MASTER_URI=http://127.0.0.1:11311 ROS_HOME=/tmp/drone_ros_home
python3 src/drone_stack/scripts/run_core_acceptance.py start/experiments/20261003_core_live_01
```

- 如失败，保留数据，修复后用新目录。只有同一源码/参数/二进制指纹可以--resume；已经创建的case目录会使用下一次attempt，不覆盖失败实验。
- 重点核对：安全体素非零采集时间和完整高度；dry→HOLD→flight实际绕柱、到达自动HOLD；连续目标流；跳变保护无误触发；新增安全地图的CPU负载和时间延迟；天花板限制不会造成误碰撞。
- 第一阶段通过后继续：多高度/连续目标、corridor/3d/blocked/边界、节点/通信失效、定位跳变/时间重置、界面故障状态、长期重复任务。当前flight验证仍使用demo柱体检查，需要扩展各场景几何检查后再自动套用其它世界，不能据同一柱体检查宣称通道/三维场景通过。
- 这些在线项目全部待验收，硬件标定/真机噪声/真实时钟同步/真实检测仍属于实机阶段。全局任务与扫描仍按原约定后续明确。
- 本轮完成的是源码与离线检查阶段；尚未满足进入实机验收条件。

## 启动与测试

```bash
cd /home/d/robotproject/project0
src/drone_stack/scripts/start_simulation.sh
# Ctrl-C 清理整组仿真；启动器拒绝已有 ROS master。
```

测试前 source /opt/ros/noetic/setup.bash、devel/setup.bash，unset ROS_IP，设置 ROS_HOSTNAME=127.0.0.1、ROS_MASTER_URI=http://127.0.0.1:11311。

```bash
python3 src/drone_stack/scripts/run_cold_start_tests.py start/experiments/<新目录> --rounds 3 --navigation
python3 src/drone_stack/scripts/run_cold_start_tests.py start/experiments/<新故障目录> --rounds 1 --hover-seconds 20 --fault-case fault_lidar
python3 src/drone_stack/scripts/analyse_sim_experiments.py start/experiments
```

最新检查点：`start/checkpoints/drone_sim_checkpoint_2026-10-03_0133.tar.gz`，完整drone_stack/EGO源码、Faster-LIO关键修改、Gazebo IMU补丁、全部实验数据及原始bag/参考ULog已保存。最新副本为 `start/checkpoints/drone_sim_checkpoint_latest.tar.gz`；历史检查点保留。完整PX4源码和运行依赖仍在原工作区，根目录.git元数据不完整，未创建commit。
# 2026-10-07：三页Qt、已知区域与DeepSeek文字控制（最新）

- Qt增加“综合页面 / 区域巡检 / AI文本控制”页签。巡检有独立高度及已知区域库；综合/巡检共享一个RViz地图，AI使用完整文字空间，切页保留任务与分栏尺寸，顶部状态/一键降落公共。
- 已知区域按实际加载地图SHA256保存到 `start/inspection_regions/`，支持中文名称、加载、更新、删除；保存原多边形和参数而非执行路线。重启保留，加载后需重新生成。地图切换清除旧选区，地图文件变化拒绝旧区域库操作。
- 新增DeepSeek计划代理和白名单校验，文字输入，无语音。支持状态、解锁/地面锁定、相对起飞、降落、HOLD、地图/相对点到点、已知区域巡检及暂停/继续/取消；实际任务结束才派发下一步。AI解析/执行授权与原飞行授权分开。
- Qt心跳、AI请求代次、导航唯一任务ID及巡检请求来源阻止迟到响应/取消跨任务。飞行管理器独立监视AI解锁/起飞/导航来源，巡检管理器独立监视AI来源；取消不覆盖降落，起飞中不因迟到地面标志DISARM，人工继续可接管巡检。
- 用户新密钥已设置在Git忽略的本机 `start/private/deepseek.local`（文件600，目录700），不写入公开配置/ROS参数/文档/日志。只读API探测因ProxyError未完成，密钥有效性未验证。
- 编译与离线检查覆盖现有巡检31项、区域库12项、AI/取消保护27项，共70项；实际ROS/Gazebo和真实DeepSeek任务仍未联调。本工具环境TCP监听被拒绝；实验继续 `DRONE_SHOW_GAZEBO=0`，不要自动飞行。
- 操作与接口见 `docs/INSPECTION.md`、`docs/AI_CONTROL.md`、README及架构文档。新增服务/来源字段后需完整重启程序；默认AI授权关闭。
- 验证摘要与精简输出已保存到 `docs/simulation/2026-10-07-ui-ai/`；无新增自动飞行或完整录包。

# 2026-10-07：新增单区域下视巡检

- 已接入 Qt 矩形/多边形选区、手动/自动间距、固定高度、方向/候选入口、预览、开始/暂停/继续/取消及异常摘要。第一版仅支持已加载且对齐的预建地图，手动起飞后开始。
- 新增 `inspection_manager.py`、纯几何模块和 `inspection.yaml`。默认手动间距0.5m、速度0.5m/s、自动重叠20%、地面地图Z=0、关键帧1Hz。相机内参外参配置可改，启动前核对CameraInfo。巡检高度沿用Qt点选目标地图Z。
- 统一全局节点接收巡检参考意图，滚动前方最多12m/1800点；EGO仍取5m，连续窗口不启用中间到达停止，最终窗口使用原收敛。增加受阻标志防止安全前段变成完整执行路线，防止接近末点的早期条带误判到达。
- 动态障碍HOLD后重连剩余段、标漏扫；相机失效/地图变化/飞行健康中断暂停或失败；严重保护仍由原飞行管理器执行。独立巡检心跳1秒失效HOLD，旧更新不能重新启动已中断会话。普通目标互斥，正常任务速度不被巡检速度永久覆盖。
- 相机采集位姿投影统计平面几何覆盖，后台有界队列记录轨迹、事件、1Hz图像与结果；写入失败警告，不自行降落。覆盖可视化只有变化时最多1Hz发布，状态2Hz，减少重复负载。
- 已完成EGO与drone_stack编译、Python语法检查及31项离线检查（含三维安全参考、柱体、窗口/到达、状态保护和接口隔离）。参考 `docs/INSPECTION.md`。这不是实际飞行通过证明。
- 本阶段精简验证记录在 `docs/simulation/2026-10-07-inspection/`，保存离线测试与构建摘要，无新增rosbag或自动飞行数据。
- 本机TCP监听探测返回 `PermissionError: [Errno 1] Operation not permitted`，当前工具环境无法启动ROS master，未进行本轮ROS/Gazebo飞行或Qt渲染验收。后续应先完整重启（消息/service定义已变更），再验证入口、扫描/换行、相机同步、动态障碍、暂停/取消和负载。
- 用户最新要求：实验不打开Gazebo图形界面，用 `DRONE_SHOW_GAZEBO=0 bash start/start_gazebo.sh`；另一终端 `bash start/start_navigation.sh inspection`，保留Qt。禁止自动授权/ARM/起飞。

# 2026-10-07：建图／巡检分离启动入口

- 新增 `start/start_gazebo.sh`：独立管理 ROS master、PX4、Gazebo 与现有带传感器机体；等待地图会话提交出生请求，避免预建地图选点与实际模型出生位置不一致。
- 新增 `start/start_navigation.sh mapping|inspection [地图.dmap]`：启动 MAVROS、Faster-LIO、EGO、地图会话、飞行管理器、相机处理与 Qt。建图无地图，巡检加载 `.dmap` 并在 Qt 先选初始位姿。
- 新增被动启动检查器 `check_sim_startup.py`：关键话题数据、时间推进、位姿/点云格式、地图对齐、READY/HEALTHY、EGO 服务和 Qt 节点；连续正常 3 秒记录通过。出生请求前不超时，提交后默认 600 秒墙钟超时；失败只报告，不自动中止定位或飞行。
- Qt 地图栏标题与模式显示明确区分建图和巡检。地图存储格式及运行中更新策略未改变：原始体素存档，实时观测确认/清除，主动保存才更新文件。
- 已完成 `catkin_make -j1 -l1 --pkg drone_stack` 编译及 Bash/Python 语法检查。未启动新的仿真，未进行实际 ROS/Gazebo 联调或飞行验收；后续应按 README 的两个终端入口检查巡检位姿等待和建图自动出生两条链路。
- 旧 `start_simulation.sh` 一体启动器保留，禁止和新入口同时运行。切换模式须落地、未解锁后先停止导航入口，再停止仿真入口，再启动新会话。
