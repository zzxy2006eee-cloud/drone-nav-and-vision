# AI仿真启动尝试

2026-10-07，按用户要求实际尝试运行，保持Gazebo图形界面关闭。

1. 本机TCP监听探测被当前工具执行环境拒绝。
2. 实际调用独立端口11421的roscore，默认ROS_HOME只读；改用可写 `/tmp` 后仍报XML-RPC监听 `Operation not permitted`，退出码1。见 [roscore.log](roscore.log)。
3. 私有密钥的真实DeepSeek纯解析请求返回ProxyError，没有获得计划，见 [deepseek.log](deepseek.log)。

ROS master未启动，未进入Gazebo或飞行控制阶段，没有发送飞行指令。工具可见进程为空不证明宿主机没有运行任务。未停止已有仿真。

在允许本地网络监听的终端中使用：

```bash
cd /home/d/robotproject/project0
# 终端A
DRONE_SHOW_GAZEBO=0 bash start/start_gazebo.sh
# 终端B
bash start/start_navigation.sh inspection
```

Qt先确认初始位姿并等待READY/HEALTHY，AI页先仅解析，再授权执行。完整飞行验收和真实API有效性仍未完成。
