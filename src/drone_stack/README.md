# 巡检无人机仿真

在工作空间根目录运行：

```bash
src/drone_stack/scripts/start_simulation.sh
```

默认带图启动先打开ROS和Qt，确认出生位姿后才启动PX4/Gazebo。无图使用`DRONE_PREBUILT_MAP=`；后台在线使用`DRONE_PREBUILT_MAP= ... --no-gui`。按 Ctrl-C 会停止整组进程，包含后台 gzserver；禁止重复启动已有仿真。日志按运行时间保存在 `start/logs/`，最新一次为 `start/logs/latest/`。

默认使用 `ROS_MASTER_URI=http://127.0.0.1:11311` 和 `ROS_HOSTNAME=127.0.0.1`，避免系统 `.bashrc` 的 `d.local` 与节点地址混用。跨机器使用时，通过 `DRONE_ROS_MASTER_URI` 和 `DRONE_ROS_HOSTNAME` 显式指定地址。

## 数据链路

- Gazebo原生LaserScanStamped测量时间/距离 → C++ `sim_livox_native_adapter` → `/livox/lidar` → Faster-LIO；旧ROS block-laser与Python逐点路径不参与默认启动。
- Gazebo 雷达 IMU → `/drone/sim/lidar/imu_raw` → 启动稳定等待 → `/livox/imu` → Faster-LIO。
- Faster-LIO `/Odometry` → 雷达到机体中心外参和初始世界方向变换 → `/drone/lio/odom`、`/mavros/odometry/out`。
- PX4 `/mavros/local_position/odom` → EGO 与飞行管理器；桥接点云 `/drone/cloud_fcu_world` 使用同一数值坐标。
- `/drone/fcu/odom` 是用于 RViz 的 FCU 本地位姿，重标记为与上述点云一致的 `odom` 帧，避免 MAVROS `map` 标签造成显示层 TF 错误。

ROS 端数据为 ENU/FLU；MAVROS 负责 MAVLink 所需的坐标转换。PX4 参数 `EKF2_EV_CTRL=11`、`EKF2_HGT_REF=3`，只融合外部位置、高度与航向，外部速度不参与融合。IMU 仍参与 EKF 状态传播和速度估计。

解锁阶段持续发送零速度目标；收到 PX4 状态中的实际 ARM 后才进入 ARMED。显式起飞指令使用 0.3 m/s 的位置目标爬升斜坡。悬停和导航阶段持续发送 30 Hz 目标。

## 仿真验证

验证工具只针对 `inspection_demo` 场景，会要求 Gazebo `inspection_quad` 真值话题存在。打开另一终端：

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
unset ROS_IP
export ROS_HOSTNAME=127.0.0.1 ROS_MASTER_URI=http://127.0.0.1:11311
python3 src/drone_stack/scripts/validate_simulation.py takeoff
python3 src/drone_stack/scripts/validate_simulation.py hold_check
python3 src/drone_stack/scripts/validate_simulation.py dry
python3 src/drone_stack/scripts/validate_simulation.py flight
python3 src/drone_stack/scripts/validate_simulation.py land
```

先验证起飞与悬停，再检查 dry 报告中的地图地板和占据体素碰撞计数，检查通过后执行 flight。报告写入 `/tmp/drone_validation_*.json`；已保存的验证结果位于 `start/validation/`。

两个相机现在显示 `/drone/front/image_processed`、`/drone/down/image_processed`，包含仿真红色目标像素框。实际巡检缺陷检测模型尚未接入；此颜色检测仅验证处理画面链路。模拟雷达为规则射线与瞬时扫描，视场和安装倾角按 MID360 配置，不模拟其真实非重复扫描图案与全部硬件误差。

## 最新仿真验收

见 [SIM_ACCEPTANCE.md](SIM_ACCEPTANCE.md) 和工作空间 DRONE_SIM_PROGRESS.md。实验原始 bag、配置快照、逐项报告位于 start/experiments；有限场景通过不代表不存在所有缺陷。

仿真 MAVROS 使用 PASSTHROUGH，共享 Gazebo/PX4 lockstep 时钟。真实硬件必须使用正常 MAVLink 时间同步，不能照搬仿真配置。SITL 等待 65 秒仿真时间后才允许解锁，避免 PX4 同步收敛前使用消息到达时刻融合。

场景切换：`DRONE_SIM_WORLD=/绝对路径/inspection_corridor.world src/drone_stack/scripts/start_simulation.sh`；另有 inspection_3d.world 和 inspection_blocked.world。新场景尚待逐项验收。

自动冷启动工具 run_cold_start_tests.py 管理启动、原始遥测和退出；--fault-case 可选择雷达/IMU中断、撤销授权、退出OFFBOARD或低电量。故障项目必须实际执行通过后才算验收完成。


## 地图时间戳离线回归（2026-10-03）

EGO 地图消息保留实际生成地图的输入点云/深度图采集时间；重发不会改变时间。
导航目标检查和运行中的地图年龄保护均为2秒，过期停止规划、持续发送悬停目标。
以下检查不需要 ROS master，不能替代仿真飞行：

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
catkin_make map_message_offline_test -j2 -l2
mkdir -p /tmp/drone_map_fixtures
devel/lib/plan_env/map_message_offline_test /tmp/drone_map_fixtures
DRONE_MAP_FIXTURE_DIR=/tmp/drone_map_fixtures python3 src/drone_stack/scripts/test_flight_safety.py
python3 src/drone_stack/scripts/analyse_bag_timing.py <telemetry.bag> <audit.json>
```

ROS/PX4/Gazebo 运行需要本地网络套接字；受限环境的启动失败必须与飞行失败区分。
具体进度和下一次冷启动命令见根目录 DRONE_SIM_PROGRESS.md。


## 执行保护与第一阶段自动验收

执行保护使用 `/grid_map/occupancy_inflate_safety` 完整高度体素；显示仍用原 occupancy_inflate。
命令必须在飞行范围内、采集时间有效、距当前机体不超过0.75 m，且整条跟踪线段不能穿过膨胀体素。
轨迹过期切悬停仍继续目标流；当前可调虚拟天花板默认2.5 m（ENU Z），目标与轨迹保留0.20 m高度裕量。
LIO桥位置/角度跳变上限分别为0.5+3*dt m和0.3+4*dt rad，拒绝异常位姿并锁定无效，落地后检查并重启桥接。
这些保护已离线验证，实际飞行效果仍待回归。

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
python3 src/drone_stack/scripts/test_sim_contracts.py
python3 src/drone_stack/scripts/test_regression_runner.py
python3 src/drone_stack/scripts/run_core_acceptance.py start/experiments/<新目录>
```

core入口执行三次导航冷启动和五种基础故障实验；每轮保存ULog并审计融合标志。
相同源码/参数/二进制可--resume，修改后必须新目录。该第一阶段通过不代表复杂场景、GUI和压力验收通过。
本地TCP/UDP套接字不可用时记录blocked_environment，未飞行，不能判定实机就绪。

## 仿真雷达安装位置

MID360 仿真传感器与独立 IMU 共用 `[0.27, 0, 0.10] m` 机体系平移，绕机体 Y 轴前倾30°。此位置用于把传感器放在机身前方，避免向下射线被机身盒体遮挡；真实硬件平移外参仍需要测量。模型生成器、雷达 IMU、静态 TF 和 LIO 位姿桥外参必须一致。

## 静态障碍记忆

无人机点云地图启用 `grid_map/retain_cloud_obstacles=true`。某帧缺少一个点并不能证明原位置为空，因此保留已观测静态占据，供盲区返程规划和执行保护使用；安全地图发布完整已知占据；原始证据由实测命中和自由射线更新，膨胀引用随原始占据增加/删除，不重复叠加旧膨胀。没有重新观测到的盲区继续保留。原点云输入新鲜度保护保持不变。

静态点云记忆在管理器首次确认 READY 时清除初始化阶段的地图后启用，避免 PX4 本地原点收敛期间的占据残留。后续 HOLD/READY 不再清空已扫描障碍；没有自由空间射线证据时仍保留静态占据。

## 完整曲线执行门控

管理器先验证 `/planning/bspline` 的整条三次曲线，再接受该曲线 ID 的 `/planning/pos_cmd`。逐段转换为 Bezier 曲线并递归收缩控制点包络，检查整个连续曲线是否越过飞行体积或触碰膨胀占据体素；原有当前目标、跟踪距离、速度和地图时间保护仍逐次执行。非法输入、计算预算耗尽或几何不安全都会取消导航并持续悬停目标流。

曲线与 `/clock` 可能经不同连接先后到达。稍早到达的曲线只完成几何检查并等待激活，时钟到达曲线起始时刻且地图仍新鲜才开放执行；不能用未来数据提前控制。新目标代次会清除待激活曲线。

完整去畸变点云与桥接配套机体位姿使用相同消息时间，由 EGO ExactTime 配对；桥接位姿来自采集时刻附近的 FCU 样本。地图按对应雷达原点到返回点之间的射线记录自由体素，返回后空间保持未知。当前机身0.47×0.47×0.11m所占体素标记为自身空间，已知膨胀占据始终优先；首次READY同时清除初始化占据和自由记录。

严格自由空间模式可通过 `require_observed_free=true` 启用：目标可以先排队，EGO可以生成未来候选路线；整曲线仍检查已知碰撞与体积边界，接下来3秒的执行前瞻及当前目标线段必须处于已知自由/自身空间。前瞻每0.1秒随指令重检，验证在控制锁外，未知尾段不提前执行。无顶场景的向上无返回不能证明自由空间，这个严格模式目前会阻止部分抬高目标；仿真默认关闭，仍记录自由地图。默认新增点面法向位置观测约束检查，弱方向通过高不确定度协方差使桥接停止EV并锁定，随后保护降落。阈值和飞行尚在验证，当前完整回归未通过。实机外参、采集时序和现场能力另见 `HARDWARE_ACCEPTANCE.md`。

### 连续曲线碰撞检查（2026-10-03）

EGO优化、时间细化与最终发布前采用逐个三次Bezier控制凸包递归检查完整曲线，闭合体素面也判相交；另加5cm规划储备。占据或未能在有界预算内证明安全时重新优化/规划，不发布候选曲线。管理器仍独立检查完整占据碰撞和执行体积。修复原上游前2/3稀疏采样漏检；核心第3轮实际失败曲线已保存为离线回归fixture。当前新版本完整飞行回归尚在进行。


## 连续朝向、虚拟天花板与多点队列（2026-10-05）

- **朝向**：起步先悬停对准。移动中以实际获准执行的 EGO B 样条的有向 XY 切线为目标，按最多30°/s连续调整机头；不使用位置跟踪误差或终点直线决定朝向。切线与实际机头或当前指令朝向超出±60°时仍先悬停对准，低水平速度/纯垂直路段保持上一朝向。
- **最高高度**：Qt右侧“最高Z（ENU）设置”，范围0.8～2.5 m。它限制机体中心在当前 ROS ENU 坐标中的绝对 Z，和“点选目标绝对 Z”使用同一基准，不是距地高度或相对起飞高度。目标、起飞终点、全局参考和获准执行的整段 EGO 曲线不得高于设置值减0.20 m。EGO地图中的虚拟天花板同步调整，超高0.05 m触发降落保护；这不能保证有惯性/估计误差的实际飞机绝不瞬间越界。
- **修改条件**：未解锁且已确认地面，或健康 OFFBOARD HOLD 且没有排队目标。空中不能把上限降到当前高度加0.20 m以下；导航中先按“悬停”取消队列，再修改高度。启动参数为 `roslaunch drone_stack stack.launch max_flight_height:=1.5`，根参数 `/drone/max_flight_height_m`；Qt修改在本次运行有效，重新启动仍采用启动参数。
- **连续点选**：起飞进入HOLD后点击“点选XY目标”，可连续左键点击，滚轮缩放；再次点“结束连续点选”仅结束鼠标点选，已有任务继续。第一个点立即启动，其余点以红色实心圆和剩余顺序编号排队，最多100点（含当前）。相对目标按钮也加入同一队列。只显示并控制当前目标的黄色全局路线和紫色 EGO 曲线，绿色轨迹在整次队列任务中累积，最后一项完成后清空。
- **顺序执行**：上一点进入15 cm到达容差、速度不高于0.15 m/s并稳定1 s后完成；下一点重新检查最新地图并开始规划，没有后台预先搜索后续点。已知膨胀障碍内的点拒绝加入，未知区域仍可加入。若排队期间下一点变成已知障碍，取消剩余任务并在HOLD报告原因。
- **任务取消**：悬停、取消目标、降落、撤销授权触发的保护、健康异常取消导航均清空剩余队列；不会在恢复后自行继续原任务。短暂健康警告沿用原保护时序。

接口：`/drone/queue_goal`（`drone_stack/QueueGoal`），`/drone/goal_queue`（latched `nav_msgs/Path`，仅保存目标点，不是规划路线），`/drone/set_max_flight_height`（`drone_stack/SetMaxFlightHeight`），`/drone/max_flight_height`（latched `std_msgs/Float64`）。原 `/drone/local_goal` 保留单目标接口，调用会替换/清空待执行队列。

## 终点接近与收敛（2026-10-05）

- 沿当前全局曲线的剩余弧长进入接近阶段，默认0.8 m；高速时按制动距离扩大。实际EGO控制曲线重新规划减速，末段目标速度0.2 m/s，保留当前起始速度/加速度边界。末端精确到目标，期望速度与加速度为零；时间调整后再次检查物理可行性和完整碰撞。
- 终点短轨迹最小长度降至0.02 m，消除原20 cm规划下限与15 cm到达容差之间的空档。不满足可行性或安全条件的短轨迹仍拒绝。
- 到达条件统一覆盖规划、等待、转向与运动：XYZ三维球半径15 cm、速度不高于0.15 m/s、连续稳定1 s。确认期间取消旧规划回调影响，保持同一批准末段的安全零速度终点与原朝向；没有合格终点则保持测得位置。轻微离开15cm仅重置到达计时，超过20cm持续0.5s或收敛超过4s才重新申请短轨迹。最终到达仍要求15cm内低速稳定1s。
- 合格的零速度终点允许最多3 s位置收敛。每个当前机体到终点的小跟踪段仍检查障碍、飞行范围及启用时的已观测自由空间；超过范围或视野不能直接修正。收敛超时再申请新规划，安全保护仍生效。
- `/drone/navigation_terminal_stage`为独立状态话题：IDLE、CRUISE、APPROACH、CONVERGING、ARRIVAL_CONFIRM、COMPLETED。共享参数为`/drone/terminal_approach_distance_m`、`terminal_speed_mps`、`terminal_deceleration_mps2`、`terminal_settle_time_s`（后三项同一`/drone/`命名空间）。紫线仍来自真实批准控制样条。

本轮完成源码与启动检查后仍需手动飞行验证；绕障急弯、视野限制、规划检查超预算等仍可能要求悬停或取消，不能据此宣称所有终点问题已经通过飞行验收。


## 地图保存、加载与导航模式

Qt右侧新增“建图与预建地图导航”：

1. **开始新建图**：地面未解锁、READY且健康时清除上一张障碍/自由地图。随后按原流程起飞，手动扫描或依次导航采集；实时EGO地图始终更新。
2. **保存地图**：扫描后降落上锁，在READY状态保存`.dmap`文件，默认目录`start/maps/`。保存全部记忆范围内的原始占据证据与已观测自由体素，不存膨胀层或虚拟天花板。文件写入临时文件再替换目标文件。
3. **加载地图导航**：地面READY且健康、没有目标队列时加载已有地图；校验版本、ENU坐标系、分辨率、网格范围与全部记录，失败保持现有地图。加载后按当前机体膨胀参数重建安全层，同时供黄色全局规划、EGO实际控制和管理器碰撞/自由空间检查使用；雷达继续更新、确认新障碍并沿自由射线清除旧占据。
4. **返回在线建图导航**：清除预建地图，从当前扫描重新建立地图；不会保留预建地图的未知区域占据。

地图坐标变换为`p_current = Rz(yaw) * p_saved + [dx,dy,dz]`，输入在Qt中显示为“地图→当前ENU”。相同仿真场景、相同起飞位置及朝向使用零偏移。换起点或换坐标原点时必须正确对齐；本功能不做自动重定位，也不把预建地图送给Faster-LIO匹配。占据体素在旋转/非网格平移后按变换后的体素包络保守重栅格化；历史自由空间仅在零yaw、整格平移时导入，其他变换需由实时雷达重新证明自由。

存图和加载只开放落地未解锁状态。文件包括原始地图、历史自由信息与网格元数据，加载不绕过实时定位健康门、OFFBOARD目标流或避障保护。预建地图不会改善雷达当下的位置观测约束不足问题。

接口：`/drone/map_archive`（`plan_env/MapArchive`，action为`new/save/load/online`，path绝对路径及ENU变换），latched `/drone/map_mode`（MAPPING/PRIOR_NAV）、`/drone/map_status`。地图模式显示在Qt右侧，体素仍来自参与真实规划的同一EGO地图。

全局A*上限15000节点，25ms CPU分批执行；同一目标的PLANNING/SPACE_WAIT转换保留搜索状态，目标变化、飞行范围变化或起点显著移动仍使旧搜索失效。管理器完整全局路径等待45秒墙钟、ACK总窗口50秒墙钟，连续30Hz悬停指令保持；没有安全路线时不发布显示专用路径。

### 管理器曲线校验墙钟预算（2026-10-05）

完整EGO曲线与执行前瞻的默认墙钟上限调整为2.0 s；线程CPU预算仍为0.05 s、递归节点上限仍为10000。检查在控制锁外进行，目标流继续；轨迹时间、地图新鲜度、碰撞检查和任务代次仍按原保护判定。全局参考平滑调用显式设置的0.15/0.5 s预算保持。Python已运行进程需重新启动才能使用新的默认值。

## 终点滞回、实测自由射线与地图初始位姿（本次修改）

修改前基线已推送 GitHub main：`11c9129`。以下改动在该提交之后，飞行效果待手动回归。

### 终点

最终到达门槛保持15cm三维球、速度≤0.15m/s、稳定1仿真秒。确认阶段独立退出半径默认20cm（`~arrival_exit_tolerance_m`），超过该半径持续0.5仿真秒或确认收敛超过`terminal_settle_time_s+1`秒才重规划。瞬时位置/速度/时间抖动重置稳定计时，保持朝向而不立即转头。小修正只能跟踪同一批准零速度终点，每周期继续检查碰撞、已观测自由空间与飞行范围；不安全修正停止执行。

### 地图更新

仿真适配器额外发布实际Gazebo射线 `/drone/sim/lidar/mapping_rays`，XYZ为雷达坐标，intensity=1为有效返回、0为仿真明确的最大量程无返回；近场、自身及无效测量不作为自由证据。它不进入Faster-LIO或EKF。桥接按扫描时刻配套FCU位姿和传感器外参转换至 `/drone/mapping/rays`，EGO地图与此时刻机体位姿ExactTime配对。

地图先登记命中，再投射实测自由射线；只保护命中体素，不保护历史15cm回波邻域。无返回射线不登记障碍，只清除其实际穿过的体素；射线不延伸到真实返回之后。原始占据在2～3独立自由扫描后清除，对应膨胀引用同步扣减，未观测盲区不按时间清空。硬件无明确无返回射线时只处理有效返回，不将缺失Livox点视为无障碍。自由证据只保存到配置的导航范围（读取管理器goal_min_xyz/goal_max_xyz）；范围外射线仍清除原始占据，不产生无用自由体素传输。Qt体素显示Decay Time=0，更新替换当前帧。

### 地图坐标与初始位姿

`map_session.py`维护地面设置的`map → odom`刚体变换，不修改LIO输出、飞控EKF原点或ENU/NED转换。在线定位首次READY后自动将起始机体位置设为地图原点、初始朝向设为地图+X；返回在线/开始新建图时重新建立本次起点。

预建地图使用流程：

1. 地面上锁、READY、健康定位且目标队列为空。
2. 选择`.dmap`，点击“打开预建地图预览”。地图仅预览，解锁和导航锁定。
3. 点击“点选初始位置并拖动朝向”，鼠标按下选位置，拖动并松开确定地图中的机头方向。XYZ/yaw也可数值输入；roll/pitch来自重力估计。
4. 点击“确认初始位姿并加载导航”。后端验证并变换预建原始占据，重新生成当前膨胀，确认成功后允许解锁和选点。
5. 后续实时雷达持续更新地图。手动位姿对齐不是Faster-LIO自动地图重定位，初始位置/方向需与实际情况吻合。

Qt固定显示帧为map，黄色/紫色真实曲线、机头、目标和点云使用同一变换。点选目标Z是地图Z；控制仍是当前ENU，允许高度范围由ENU导航下限和天花板换算。最高Z设置仍明确限制控制ENU高度。

地图会话接口 `/drone/map_session` (`drone_stack/MapSession`)，操作prepare/load/new/online/save。`/drone/map_alignment_ready`门控管理器解锁及目标接收；`/drone/map_transform`是map中odom原点位姿，`/drone/map_session_status`提供就绪说明。`.dmap`新增DRONE_GRID_V2头保存当时map_from_odom平移/yaw；V1仍兼容，V1的保存坐标直接作为地图坐标。直接使用旧`/drone/map_archive`接口时需理解use_map_frame的变换含义，Qt统一通过地图会话操作。

## 当前仿真场景预建地图启动

已提供场景几何生成器，运行中的 Gazebo 可执行：

```bash
source devel/setup.bash
rosrun drone_stack build_simulation_scene_map.py \
  --world /home/d/robotproject/project0/src/drone_stack/worlds/inspection_demo.world \
  --output /home/d/robotproject/project0/start/maps/inspection_demo_scene.dmap
```

随后停止上一套仿真，以该地图启动：

```bash
DRONE_PREBUILT_MAP=/home/d/robotproject/project0/start/maps/inspection_demo_scene.dmap \
  bash src/drone_stack/scripts/start_simulation.sh
```

地图采用场景静态碰撞几何，包含四周墙体、两根立柱和黄色设备，分辨率10cm；这是完整障碍几何先验，不是飞行扫描地图。原始占据写入V2文件，加载后重新生成膨胀层，实际雷达继续更新。未伪造自由空间，执行前的观测自由检查仍生效。当前生成器支持仅绕Z旋转的静态方盒碰撞体，其他碰撞形状会报错。

`DRONE_PREBUILT_MAP`用于选择仿真预建地图；最新版本改为先由Qt确认出生位姿再启动Gazebo（见下文最新流程）。定位达到READY/HEALTHY、落地未解锁后，核对实际出生位姿并加载地图，Qt默认地图文件同步。Gazebo真值仅用于这个初始地图坐标设置，不进入Faster-LIO、EKF或持续运动控制。实机仍使用Qt手动初始位置/朝向流程；启动不会自动解锁或起飞。

## 仿真初始位姿与在线队列跳点（最新流程）

默认运行 `bash src/drone_stack/scripts/start_simulation.sh` 为带图模式：先启动ROS及Qt，显示默认场景地图；Gazebo/PX4保持未启动。Qt中点选出生位置并拖动机头朝向（亦可输入XYZ/yaw），点击“确认初始位姿并启动仿真”后，才启动Gazebo并按所填世界ENU位姿生成机体。平地机体初始Z为0.15～0.20m，推荐0.17m；XY限制±8m，出生点距原始地图障碍至少0.55m。当前默认场景地图采用Gazebo世界ENU坐标。

定位就绪后核对实际出生位姿，再对齐并加载地图，允许解锁。Faster-LIO仍在线定位，不做预建点云地图重定位；ENU/MAVROS和外部仅位姿融合保持现有约定。已启动的带图仿真不能仅通过修改地图TF变更出生位姿，需重启重新设置。自动加载只裁剪当前局部网格之外的地图部分；有效飞行范围内障碍仍保留。

需要无图模式时，在启动前的Qt点击“返回在线建图导航”或“开始新建图”；也可用 `DRONE_PREBUILT_MAP= bash src/drone_stack/scripts/start_simulation.sh` 直接在线启动。没有GUI时可通过 `/drone/map_session` 服务提交出生位姿，启动器用墙钟等待，不依赖尚未存在的 `/clock`。

在线模式中：已知障碍内的点在入队时拒绝，其余队列保留；前一点结束时，跳过被最新地图判为占据的后续点；正在前往的点被新扫描确认为占据时，停止旧规划和轨迹、进入HOLD，约0.3仿真秒后派发下一个点。后续点仍只在轮到它时规划；全部点被跳过则保持HOLD。地图/定位过期、离开OFFBOARD或其他安全异常继续执行原有取消/降落保护，带图模式的障碍点处理保留原有规则。

## CPU优化与1秒EGO心跳门槛

- EGO通过 `/grid_map/voxel_delta` (`plan_env/VoxelUpdate`)发布占据/自由体素的新增和删除ID；每次有效扫描有一个递增revision，即使几何没变也发布小型空增量，采集时间仍来自扫描。epoch在地图清空/重建后更新。`/grid_map/get_voxel_snapshot` (`plan_env/GetVoxelSnapshot`)为晚订阅或断序恢复提供完整同版本快照。
- 管理器与全局规划器统一使用 `voxel_map_client.py`：验证坐标/网格/版本；断序立即使本地地图时间无效，再请求快照。占据与自由使用同一版本、不可变布尔位图；只复制发生变化的层，控制锁内仅交换引用。保留盲区障碍、原始占据确认、射线清除和保守膨胀。
- `trajectory_guard.curve_geometry`按曲线几何缓存24组样条/分段多项式；位置/朝向和安全检查复用几何，地图/时间/碰撞结论不缓存，执行前仍检查最新地图和观测自由。
- Qt体素显示4Hz，RViz渲染15FPS；规划地图每个扫描更新。旧安全/自由PointCloud2接口仅有订阅时生成兼容快照，常规控制不再订阅。默认仿真只打开Qt，Gazebo物理/雷达/相机后台运行；需要Gazebo窗口可使用 `DRONE_SHOW_GAZEBO=1` 启动。
- 管理器 `~max_planner_age_s` 默认及launch值均为1.0sim秒，配置上限1.0；外部仅位姿融合、ENU/NED约定、连续OFFBOARD、碰撞保护不变。
- 无图启动使用 `DRONE_PREBUILT_MAP= bash src/drone_stack/scripts/start_simulation.sh`。新增明确online_mode参数避免roslaunch对空地图路径回退默认值；默认带图仍先确认出生位姿。

## 定位链路分段计时

`DRONE_RECORD_LIO_TIMING=true`启动时启用计时；默认false，不改变采集时间、输入数据、滤波器、队列大小或飞行保护。诊断话题为 `/faster_lio/latency_timing`、`/drone/latency/lidar_adapter`、`/drone/latency/lio_bridge`、`/drone/latency/lidar_transport`。CPP记录各计算阶段的墙钟/进程CPU/主线程CPU，桥接记录锁等待；仿真前端另记录接收线程CPU。跨进程单调时钟仅适用于同一主机，仿真秒与墙钟秒分开分析。

详细字段、帧BEGIN/END配对及本轮结果见DRONE_SIM_PROGRESS.md指向的timing_schema.json和lio_segment_timing_analysis.md。地面计时已发现Python前端成本及原Gazebo block-laser时间标签滞后；此次只部署探针，源时间戳修复和C++转换替换尚未实施。

## 原生仿真雷达前端与最新健康时序

当前 `lio.launch` 仅启动C++ `sim_livox_native_adapter`，直接订阅Gazebo原生LaserScanStamped，以引擎提供的测量时间生成Livox CustomMsg和实际建图射线。Gazebo block-laser旧ROS插件已从generate_model.py及生成模板移除，Python旧适配器仅保留历史源码，不参与默认启动。点云仍在lidar局部坐标，不读取world_pose或真值定位。

MID360模型的10Hz、360×56射线、40m量程、前倾30°及安装位移保持。有效回波进入Livox，明确无返回射线进入建图流；近场与异常样本不产生自由证据，瞬时扫描的点统一位于包末，header/timebase为包起点、offset_time=0.1s。IMU、故障模拟服务及启动静置过滤均在C++前端保留。

用户已明确选择健康时序：短暂异常警告，持续1sim秒取消任务进入HOLD，持续2sim秒降落，严重异常立即降落。管理器和LIO几何门槛使用一致配置；estimator_age上限仍2s，未修改；EGO心跳1s保持。数据过期门槛和这些持续异常计时属于不同设置。

地面相近场景299帧：前端构造中位0.53ms，CPU约4.1%单核；位姿到桥接发布年龄中位22ms仿真时间、95分位30ms、最大46ms。仅为地面测量，飞行安全效果由本轮手动实验继续验证。

所有显示器拔除时，Ogre1.9会因RandR空视频模式列表崩溃。启动器仅对自己的子进程启用libdrone_headless_xrandr_compat.so，使Ogre使用虚拟X屏幕尺寸，GPU渲染保留；桌面配置和系统库未修改。有已连接输出时不启用该兼容层。
