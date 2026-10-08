# 四旋翼巡检仿真与 Qt 操作台

ROS Noetic 工作区，集成 PX4 SITL、Gazebo Classic、MID360 仿真、Faster-LIO 定位、全局 A* 平滑路线、EGO 局部规划和 Qt/RViz 操作台。

详细说明见 [当前系统架构](docs/ARCHITECTURE.md)：包含坐标与定位、地图来源/存档/更新、A* 与 EGO 衔接、机头朝向、终点收敛、安全保护和两种启动模式，并整理相关问题与实现限制。

新增功能见 [单区域下视巡检](docs/INSPECTION.md)：预建地图下矩形/多边形选区、条带预览、真实路线执行、暂停恢复、Qt异常显示、相机覆盖与关键帧记录。代码已接入，尚未完成ROS/Gazebo飞行验收。

Qt现在分为“综合页面 / 区域巡检 / AI文本控制”。巡检页支持命名保存、加载、更新和删除当前地图的已知区域；[DeepSeek文字控制](docs/AI_CONTROL.md) 支持按名称巡检、点到点、解锁、起飞、悬停和降落，解析与执行授权独立。

**当前版本：2026-10-07。处于仿真验证阶段，尚未完成实机验收。** 修改前基线为 `11c9129`；本版包含后续地图、终点、初始位姿、定位前端修复及新增单区域巡检。新增巡检尚未完成飞行验收。

## 环境与编译

使用 Ubuntu 20.04、ROS Noetic、Gazebo Classic 11、Qt5/RViz；PX4 仿真配置基于 v1.15.4。需要 Gazebo 开发库及 Livox、PCL、Eigen、OpenCV 等工作区依赖。PX4 源码与仿真依赖位于 `external/`，首次 PX4 构建由启动脚本调用。

```bash
source /opt/ros/noetic/setup.bash
catkin_make -j1 -l1
source devel/setup.bash
```

不要同时启动两套程序。分离启动器的 Ctrl-C 停止各自管理的进程；旧一体启动器停止整组。统一停止全部项目仿真进程使用下面的 `stop_all.sh`。

## 启动与地图模式

### 推荐：仿真世界与导航操作台分开启动

终端 A（Gazebo、PX4、机体及传感器）：

```bash
cd /home/d/robotproject/project0
bash start/start_gazebo.sh
```

终端 B（二选一，均包含 MAVROS、Faster-LIO、EGO、飞行管理器、相机处理和 Qt）：

```bash
# 建图模式：无预建地图，自动使用默认地面出生位姿
bash start/start_navigation.sh mapping

# 巡检模式：加载体素地图，在 Qt 选择出生位置、拖动朝向并确认
bash start/start_navigation.sh inspection
# 或指定地图文件
bash start/start_navigation.sh inspection /绝对路径/场景.dmap
```

终端 A 首先启动 ROS master 并等待出生请求。巡检模式必须先在 Qt 确认初始位姿，之后才启动 Gazebo/PX4，保证模型出生位置与选定位置一致。建图模式自动提交默认位姿。**Gazebo 仿真启动时默认同时打开 Gazebo 界面**，巡检等待选位姿期间尚不生成世界。默认世界为 `inspection_demo.world`，含墙体、柱体、设备和检测目标；四旋翼模型含前倾 30° MID360、雷达 IMU、前视与下视相机，不使用 GPS/气压计定位。可用 `DRONE_SHOW_GAZEBO=0` 隐藏 Gazebo 窗口，`DRONE_SIM_WORLD=/绝对路径/场景.world` 指定其他世界（预建地图须与场景对应）。界面日志为终端 A 日志目录下的 `gazebo_client.log`。

启动检查观察时钟推进、雷达/IMU、LIO/外部里程计/FCU 位姿、建图射线、体素增量、两路处理图像、管理器心跳、地图对齐和健康状态，并检查 EGO 启动服务。连续正常 3 秒写入通过结果；计时从出生请求提交开始，默认 600 秒墙钟超时。待操作员选择位姿不计超时。尚无任务时 EGO 不生成运动轨迹，检查器不会为了验收心跳而开启规划或自动飞行。

日志分别位于 `start/logs/world_*` 与 `start/logs/mapping_*` 或 `inspection_*`。终端 B 的 `startup_check.log` 和 `startup_check.json` 记录未就绪原因及通过结果。超时只报告，不自动杀死定位/控制节点；检查结果是启动快照，飞行中仍以实时健康保护为准。

两个脚本分别管理自己的进程组。落地且未解锁后，先停止终端 B，再停止终端 A；切换启动模式或重新选择出生位姿，需要停止两个入口再启动。禁止与下面的一体启动器同时使用。Qt 地图栏明确显示“建图模式 / 巡检模式”，支持新建图、保存 `.dmap`、地图预览和初始位姿选择；巡检地图在内存中仍接受实时观测更新，文件只有主动保存时才更新。

### 一键停止全部项目仿真进程

```bash
cd /home/d/robotproject/project0
bash start/stop_all.sh
```

清理本工作区的启动器、Qt、定位、规划、相机处理、MAVROS、PX4、Gazebo server/client、所属 ROS master 和后台子进程，兼容新的分离入口与旧一体入口。先发送 SIGINT 等待退出，仍未退出时依次发送 SIGTERM、SIGKILL；没有进程时也可执行。通过启动器身份记录、项目环境标记和子进程关系识别，不使用全系统 `killall`，不删除地图或实验日志。无需 ROS master 在线。

这是停止进程命令，不是降落指令；正常结束实验应先降落。旧一体启动器现在也默认打开 Gazebo 界面，使用 `--no-gui` 或 `DRONE_SHOW_GAZEBO=0` 可隐藏。

### 巡检实验：暂时关闭 Gazebo 图形界面

```bash
# 终端 A，保留后台仿真
DRONE_SHOW_GAZEBO=0 bash start/start_gazebo.sh
# 终端 B，打开 Qt 与预建地图导航
bash start/start_navigation.sh inspection
```

Qt确认初始位姿后，切换“区域巡检”页框选矩形或绘制多边形，设置独立的巡检地图Z。可命名保存为已知区域，重启后加载；加载后重新生成路线。默认手动间距0.5m、速度0.5m/s、自动重叠20%。手动起飞后健康HOLD时开始，末点悬停。无地图模式禁止巡检。

AI页先允许云端解析，需要实际动作时再授权AI执行。原飞行授权仍在综合页，安全门由现有管理器负责。可输入“起飞1米，然后巡检设备区，最后降落”。密钥已设置于Git忽略的本机私有文件，不在公开配置中；当前尚未完成真实DeepSeek/ROS联调。

下视相机内参/外参、地面Z和记录参数位于 `src/drone_stack/config/inspection.yaml`；必须与实际CameraInfo一致。默认1Hz保存关键帧及轨迹/覆盖/异常到 `start/inspection_records/`。巡检异常在Qt单独显示原因、动作和恢复方法，任务失败不会自行覆盖飞控的安全动作。

本次扩展了 EGO 路线消息和规划服务，使用前需一起编译并重新启动两套程序：

```bash
source /opt/ros/noetic/setup.bash
catkin_make -j1 -l1 --pkg ego_planner drone_stack
source devel/setup.bash
python3 src/drone_stack/scripts/test_inspection.py
python3 src/drone_stack/scripts/test_inspection_regions.py
python3 src/drone_stack/scripts/test_drone_ai.py
```

### 无图在线模式

首次检出没有本地地图时，先使用此模式：

```bash
DRONE_PREBUILT_MAP= bash src/drone_stack/scripts/start_simulation.sh
```

在线定位就绪后，将本次起始位置设为地图原点、初始机头设为地图 +X。等待 Qt 显示 READY、定位健康，再手动授权、解锁和起飞。启动不会自动飞行。

### 带图模式：先选择出生位姿

```bash
bash src/drone_stack/scripts/start_simulation.sh
```

默认读取 `start/maps/inspection_demo_scene.dmap`。先打开 ROS 和 Qt 并预览地图，**此时 Gazebo/PX4 尚未启动**。

1. 在 Qt 地图栏点选初始位置并拖动朝向，或输入 XYZ/yaw。
2. 当前平地场景机体初始 Z 推荐 `0.17 m`，允许 `0.15～0.20 m`；出生点需要避开地图障碍。
3. 点击“确认初始位姿并启动仿真”。Gazebo 按所填位姿生成机体。
4. 定位就绪后，核对实际出生位姿、对齐并加载地图，成功后允许解锁与导航。

已有仿真不能仅修改地图 TF 来改变实际出生位置；需要重启重新设置。地图对齐是初始位姿设置，**不是 Faster-LIO 自动预建地图重定位**。实机需要手动提供可靠的初始位置和朝向。

本地地图不随源码提交。缺少默认地图时，可以在启动前选择其他 `.dmap`，或点击“返回在线建图导航”。当前场景的几何先验可在无图仿真落地、未解锁时生成：

```bash
source devel/setup.bash
rosrun drone_stack build_simulation_scene_map.py \
  --world "$PWD/src/drone_stack/worlds/inspection_demo.world" \
  --output "$PWD/start/maps/inspection_demo_scene.dmap"
```

随后停止旧仿真，再按默认带图模式启动。这张图由场景碰撞几何生成，包含四面墙、两根立柱和黄色设备；不是飞行扫描建图结果。导航期间真实模拟雷达仍会更新占据与自由证据。

### 显示和诊断选项

```bash
# 额外打开 Gazebo 三维窗口，默认只开 Qt
DRONE_SHOW_GAZEBO=1 DRONE_PREBUILT_MAP= \
  bash src/drone_stack/scripts/start_simulation.sh

# 记录心跳判定与定位分段计时
DRONE_RECORD_HEARTBEAT=true DRONE_RECORD_LIO_TIMING=true \
  DRONE_PREBUILT_MAP= bash src/drone_stack/scripts/start_simulation.sh

# 在线模式，仅后台运行
DRONE_PREBUILT_MAP= bash src/drone_stack/scripts/start_simulation.sh --no-gui
```

默认使用本机 ROS 地址 `127.0.0.1:11311`。跨机器时用 `DRONE_ROS_MASTER_URI`、`DRONE_ROS_HOSTNAME` 显式配置。

## 定位和传感器链路

- MID360 仿真：10 Hz、360×56 射线、最大量程 40 m；雷达位于机体 `[0.27, 0, 0.10] m`，绕 Y 轴前倾 30°。另有雷达 IMU、前视相机和下视相机。
- **Gazebo 原生 LaserScanStamped → C++ `sim_livox_native_adapter` → `/livox/lidar` → Faster-LIO。** 使用原生测量时间，旧 block-laser 插件已从模型生成器移除，Python 逐点适配器不参与默认启动。
- Gazebo 雷达 IMU经启动静置过滤后发布 `/livox/imu`。有效回波进入 Livox；明确无返回射线用于建图清除；近场和无效数据不生成自由证据。
- Faster-LIO `/Odometry` 经传感器外参换算为机体中心位姿，发布 `/drone/lio/odom` 和 `/mavros/odometry/out`。
- ROS 使用 **ENU/FLU**，MAVROS 自动转换为飞控使用的 NED/FRD；ROS 端不额外转换。
- `EKF2_EV_CTRL=11`、`EKF2_HGT_REF=3`：外部位置、高度和航向融合，**外部速度不融合**。GPS、气压计不作为定位源；飞控 IMU继续进行状态传播和速度估计。
- 原生扫描中的 `world_pose` 不用于定位或持续控制。仿真真值仅用于一次地面地图对齐/核对及只读实验分析。

## 导航和 Qt 功能

- 全局 A* 最多扩展 15000 节点，生成参与规划控制的平滑路线；同一路线传给 EGO，管理器核验实际执行 B 样条。
- 黄色为最新全局路线，紫色为实际获准执行的 EGO 曲线，绿色为实际轨迹；目标和机头箭头显示在对应图层。
- 支持连续目标队列，前一点到达后才规划下一点。在线模式下，已知障碍内的点拒绝入队；后续发现目标占据时仅跳过该点，保留后续队列。
- 起步悬停转向，前进时按实际规划路线的有向切线调整朝向；超出前方 ±60° 时重新对齐。
- 支持授权、ARM、相对起飞高度、HOLD、取消目标、一键降落、地面上锁、移动速度 `0.1～1.0 m/s` 和虚拟天花板。
- 终点接近减速、零末速度曲线和到达确认。最终到达条件为三维球半径 15 cm、速度 ≤0.15 m/s、稳定 1 秒；确认阶段采用 20 cm 退出滞回。
- Qt 状态卡按健康程度变色，显示两路处理后相机画面。地图显示 4 Hz、RViz 渲染 15 FPS；控制目标持续发送 30 Hz。

## 地图更新与性能优化

- 实测回波确认原始占据，射线清除重新观测到的空闲区域；膨胀引用随占据删除，盲区障碍保留。
- `/grid_map/voxel_delta` 传输占据/自由体素增删编号，带 epoch、revision 和真实观测时间。断序先使本地地图无效，再通过 `/grid_map/get_voxel_snapshot` 恢复完整快照。
- 管理器和全局规划器使用同一版本的占据/自由位图，控制锁外解码，锁内交换引用。保留旧 PointCloud2接口供兼容订阅者使用。
- 样条和分段多项式几何缓存复用；时间、地图新鲜度和碰撞结论不缓存。SciPy样条使用独立可写缓冲，已修复只读数组求值异常。
- 无物理显示器时，项目子进程自动使用 Ogre/RandR兼容层，避免空视频模式列表崩溃，保留 GPU 渲染。

## 当前保护参数

| 项目 | 当前设置 |
|---|---|
| 一般健康异常 | 先警告；持续 **1 秒仿真时间**取消任务并 HOLD |
| 持续异常降落 | **2 秒仿真时间**；严重异常立即保护 |
| EGO心跳超时 | **1 秒仿真时间** |
| `estimator_age` 上限 | 保持原有 **2 秒** |
| LIO最弱平移约束门槛 | 30 |
| 曲线校验预算 | 墙钟2秒、线程CPU0.05秒、最多10000节点 |

地图/定位过期、离开OFFBOARD、严重位姿异常等仍有独立检查。严重异常锁定后需检查并重启，已取消队列不会自动恢复。

## 已验证与待验收

本版已完成编译、地面启动、消息格式、增量地图一致性、健康边界和录制曲线数值回归。

相近地面场景299帧测量中，前端构造中位约0.53 ms实际时间；位姿发布时的数据年龄中位22 ms仿真时间、95分位30 ms。旧前端对应地面测量约90 ms转换、230 ms报告年龄。这些是地面结果，不是所有飞行条件的保证。

仍需继续验证：多点绕障与终点收敛、动态地图清除、未知区域执行限制、转向衔接、健康保护恢复、带图出生位姿交互，以及全部实机外参、时序和场地能力。当前版本不能标记为仿真全通过或实机验收完成。

## 文档与数据

- [无人机栈详细说明](src/drone_stack/README.md)
- [实机验收说明](src/drone_stack/HARDWARE_ACCEPTANCE.md)
- [项目进度](DRONE_SIM_PROGRESS.md)
- [本版验证摘要](docs/simulation/2026-10-06/README.md)

`src/` 和必要的 `external/` 源码/补丁纳入版本管理。`build/`、`devel/`、日志、录包、地图与检查点是本地生成数据，不纳入本次提交。
