# AGH运行参赛版本

[English integration specification](INTEGRATION.md) · [中文完整集成规范](INTEGRATION.zh-CN.md) · [中文首页](../README.zh-CN.md)

## 1. 安装与账号

先完成根README的Python安装。AGH从[官方仓库](https://github.com/AgnesAI-Labs/agnes-harness)安装并按其说明登录；使用你自己的账号配置模型。参赛原生会话使用`agnes-3.0-flash`，服务实际可用模型以账号为准。不要把Key放进本仓库。

## 2. 填写stdio服务

在AGH的MCP设置中添加stdio服务，运行`python tools/print_mcp_config.py`取得本机Python绝对路径、脚本绝对路径和参数。服务名建议`embodiedguard-competition-v14`。

参数的公开地图形式：

```text
<repo>/EmbodiedGuard-v14/run_checked_final_mcp.py
--mode public
--map maze512-16-0
--line 142
--scale 0.5
--fault freeze_once
--output <repo>/runtime_data/agh-public
```

其中每一项均为独立参数。不要把整条shell命令塞进executable。输出需保持在仓库根目录内；启动入口会检查冻结源文件与地图指纹。stdio进程等待输入时没有网页界面，这是工具服务的正常行为。

可允许的focused工具：`observe,navigate,reobserve,wait,stop,verify`。连接和权限通过AGH实际界面确认；不用全访问代替工具配置。AGH管理探测可能创建独立后端记录；正式任务须新建会话和新Mission进程，首次observe应为frame=0、静止、无已完成目标。合成巡检从原点开始，公开地图的起点由所选地图场景决定。工具目录读取成功与任务执行成功应分别确认。

旧三目标巡检请用`--mode inspection --scenario blocked --response-mode compact --interface-profile focused`替代公开地图参数。原温度数据是合成fixture。

## 3. 发送参赛提示

使用根目录`prompt-plus.frozen.txt`原r3提示词。与`EmbodiedGuard-v14/prompts/`旧提示区分，它包含终态核验补充。它没有预先填写全部路线与恢复答案。

典型恢复结果：先观测，导航遭遇过期后停稳，Agent调用重观测，看到有效反馈后选择resume，最后显式verify。新运行的选择和结果可能不同，按真实日志记录。

## 4. 保存结果

机器人运动证据写入本机`runtime_data/agh-public/run-*/`。它与AGH原生对话/工具轨迹作用不同；如需参赛格式证据，通过AGH官方界面手动导出原始诊断ZIP。分享前另外检查账号、路径和凭据，不将运行目录直接提交Git。

## 5. 本次公开整理的边界

这份说明基于参赛入口和本地核对的官方MCP文档。AGH版本变化时，UI布局或管理命令可能不同。公开整理验收会检查本地MCP协议和目录；不消耗你的模型账号重新跑一组正式比赛实验。
