# 原生雷达前端部署验证

- 用户最终要求：源时间戳修复+C++转换；estimator_age不改，异常1s HOLD/2s降落，严重立即保护。
- 直接Gazebo原生测量时间生成lidar局部Livox/rays，不取world_pose，不额外ENU→NED转换；EV_CTRL11、HGT_REF3、GPS_CTRL0/BARO_CTRL0运行读取核对通过。旧插件由模型生成器永久移除，当前生成SDF核对没有mid360_ros_cloud。
- 完整catkin_make以及CPP前端目标最终编译退出0。启动曾在Ogre RenderSystem_GL初始化崩溃；原始core/SDK源码/当前xrandr确认所有物理输出断开且Ogre空视频模式解引用。兼容库只在无已连接输出的项目进程隐藏RandR查询，保留GPU，当前Qt/Gazebo启动成功。没有改系统桌面或系统库。
- native_input_health_validation.json：9061有效Livox点、lidar帧、timebase/header一致、100ms统一offset/tag0x10/line<4、距离0.4506～12.7507m；10071建图射线（4531真实回波、5540明确无回波）；健康表达式在0/.999警告，1/1.999 HOLD，2 SEVERE，严重异常0即SEVERE；运行ROS参数1/2一致，estimator_age仍2。
- native_ground_profile.json：sim65～95共299帧，前端构造中位0.525ms/p95 .924ms，前端CPU中位.663ms；桥接发布年龄中位.022sim秒/p95 .030/max .046，适配器进程平均4.1%单核。相近旧地面基线构造90ms、报告年龄230ms；图形模式/进程与采集时域同时改变，不将这些结果当成所有飞行情形保证。
- 用户已开始手动飞行：20:46:56 TAKEOFF，20:47:18 NAVIGATING；20:47:27发现执行前瞻与膨胀障碍交叉，保留目标等待；20:47:50目标被实时地图判占据而只跳过当前点，0.308sim秒后派发下一点。健康保持HEALTHY，近期抽样发布年龄中位28ms/p95 46/max60。该记录未证明队列最终全部完成。
- 新源码、说明和检查点保存，当前实验程序未停止/重启；后台监测和诊断录包持续。不自动飞行，未提交/推送GitHub。
