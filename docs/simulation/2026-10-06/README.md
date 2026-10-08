# 2026-10-06 当前版本验证摘要

这些文件来自原生C++雷达前端部署的只读检查，作为本版功能和参数的证据；不包含大型录包或点云。

- native_ground_profile.json：地面sim65～95、299帧的分段计时和进程CPU。
- native_input_health_validation.json：Livox格式、实际建图hit/miss及1s HOLD/2s降落边界。
- fusion_parameters.json：运行PX4外部仅位姿融合、无GPS/气压计融合参数。
- native_frontend_deployment_report.md：部署、无显示器启动兼容、地面核对和部分手动飞行观察。

位置/导数的27条录制曲线回归已在项目进度记录。当前飞行回归仍未全部完成，地面时延降低不能直接证明所有导航/安全场景通过。详细运行进度见根目录DRONE_SIM_PROGRESS.md；运行日志留在本地start目录。
