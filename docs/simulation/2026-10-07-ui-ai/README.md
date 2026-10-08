# 三页界面、已知区域与AI：离线验证

日期：2026-10-07。

- EGO与drone_stack编译退出码0，Qt新页面编译成功：[build_summary.log](build_summary.log)。
- 原巡检31项：[inspection_tests.log](inspection_tests.log)。
- 已知区域12项：[region_tests.log](region_tests.log)。
- AI计划/取消/权限27项：[ai_tests.log](ai_tests.log)。
- 共70项通过，均为离线检查；没有初始化ROS节点或发送实际飞行指令。
- Python语法、差异空白、文档本地链接检查通过。
- 私有密钥文件600、Git忽略，公开源码/文档检查未发现用户密钥。未将密钥写入本目录。

已知区域检查包括中文名称、重启持久化、不同地图隔离、重复名称不覆盖、显式更新/删除、损坏库拒绝和原子写入。AI检查包括白名单、未知区域/非法坐标、仅解析、迟到模型回复、请求代次、导航取消身份、人工接管、起飞禁止锁定和降落禁止改HOLD。

本环境本机TCP监听被拒绝，ROS/Gazebo/Qt实际渲染未验收。DeepSeek `/models` 只读校验遇到ProxyError，未验证真实API或密钥有效性。实际实验继续关闭Gazebo图形界面，保留Qt，并完整重启新增服务版本。

下一阶段：区域保存/加载交互、三页切换尺寸、真实API只读解析、解锁/起飞撤权、已知区域连续任务、点到点到达、取消与网络失效。具体使用见 [AI_CONTROL.md](../../AI_CONTROL.md) 和 [INSPECTION.md](../../INSPECTION.md)。
