# EmbodiedGuard — 黑客松参赛版本 V14

**无人巡检自主决策与安全恢复智能体**

2026江苏省AI+科学与工程创新实践黑客松（高校组）｜本科生组｜U308  
停下吃花生 · 南京航空航天大学  
邹鸿生（队长，AGH接入、全链路联调与优化）／王雯婷（代码实现与完善、测试与展示）

> **本仓库是参赛提交版本的公开整理副本。** 代码固定为V14，来源为最终r3交付包；`r3`是交付材料修订号。原提交包SHA-256为 `e20e22a2266a29f97d331fcc7955cbacfc5c299432cd7525ff72f867520b7eb7`。参赛应用目录的78个Python文件与原r3提示词逐字节保留，文档脱敏与辅助文件改动单列记录。版本身份见 [COMPETITION_VERSION.json](COMPETITION_VERSION.json)，建议发布标签 `competition-v14`。

[演示视频](https://www.bilibili.com/video/BV1WYpm6vErh/) · [执行链路](docs/ARCHITECTURE.md) · [AGH接入](docs/AGH-MCP.md) · [参赛版本说明](docs/COMPETITION-RELEASE.md) · [来源与授权](THIRD_PARTY_NOTICES.md)

## 作品解决什么问题

无人巡检遇到观测过期、通道阻塞或执行失败后，系统需要先停车，再根据反馈决定怎样继续。EmbodiedGuard把Agnes的任务级决策与本地规划控制连接起来：模型选择目标、路线和恢复工具，底层逐步检查状态与动作，最终通过本地`verify`核验任务结果。

运行基座为Agnes Harness（AGH），参赛真实会话使用`agnes-3.0-flash`。环境采用轻量二维前向Ackermann运动学，支持三目标合成巡检与已知公开地图到达任务。

## 核心能力

- **有反馈的恢复：** 观测过期后停稳，Agent可选择重观测、续行、重新规划或明确退出。
- **确定性执行：** 有界全局搜索、自动驾驶PID控制底座、路线句柄与幂等执行请求。
- **独立安全检查：** 状态有效性、动作约束、制动预测、车体占位、速度与预算检查。
- **可追溯核验：** 分别保存工具请求、执行动作、状态、故障反馈和最终验收。
- **紧凑工具接口：** focused配置公开`observe`、`navigate`、`reobserve`、`wait`、`stop`、`verify`六个工具。

## 环境与安装

Python **3.11或更新版本**。基本三目标仿真使用标准库，公开地图需要NumPy，测试需要pytest。无需ROS、CARLA、GPU或模型权重。以下命令均在仓库根目录执行。

```sh
python -m venv .venv
# Windows PowerShell：.venv\Scripts\Activate.ps1
# Linux/macOS：source .venv/bin/activate
python -m pip install -e "./EmbodiedGuard-v14[public,test]"
python tools/verify_release.py
```

若PowerShell限制激活脚本，可直接使用`.venv/Scripts/python.exe`执行后续命令。`pyproject.toml`中的包版本`0.1.0`为保留的历史软件元数据；本次参赛身份以V14及根版本文件为准。

## 快速体验：无需模型账号

```sh
python -m embodiedguard.cli demo --scenario stale --policy fixed_recovery --segment-seconds 15 --output runtime_data/local-demo
```

打开生成的`runtime_data/local-demo/replay.html`查看运动回放；`summary.json`记录验收结果。这是明确标注的固定规则工程演示，用来检查安装、执行与日志。Agnes真实决策需要下一节的AGH接入。

## 使用AGH运行参赛任务

按照 [AGH-MCP说明](docs/AGH-MCP.md) 安装并登录官方AGH，在MCP配置中指定本机Python与脚本绝对路径。下面的命令启动stdio工具服务，它会等待客户端请求：

```sh
python EmbodiedGuard-v14/run_checked_final_mcp.py --mode public --map maze512-16-0 --line 142 --scale 0.5 --fault freeze_once --output runtime_data/agh-public
```

生成适合本机的入口参数：

```sh
python tools/print_mcp_config.py
```

该命令仅打印配置，不修改AGH权限，也不发起模型调用。工具连接后，新建独立会话并发送根目录 [原r3冻结提示词](prompt-plus.frozen.txt)。模型账号、权限及调用费用由使用者自己的AGH配置管理。

## 回归测试

```sh
python -m pytest -q EmbodiedGuard-v14/tests
python tools/verify_release.py
```

本次公开整理验证：268项测试、87项子测试通过，详见 [验证记录](docs/VALIDATION.md)。测试在本地仿真与临时目录运行。历史实验和本次公开整理的安装检查分别记录；公开整理不增加参赛AGH样本数。

## 参赛结果与验证范围

历史正式验证登记了8个AGH会话：6个可行任务完成，2个任务有据安全退出，均保存最终核验，碰撞、边界和超速违规为0。恢复成功会话中，模型实际选择重观测和原路线续行。公开仓库提供实现、任务提示和运行方法；带账号上下文的原始AGH诊断包留存在参赛原件中。

当前结果适用于合成二维运动学、已知地图与受控故障；真实设备接入、视觉感知精度与真实底盘动力学仍需进一步验证。温度示例来自合成fixture。项目没有加载AutoVLA权重，也没有训练新的VLA模型。

## 目录

```text
EmbodiedGuard-v14/  参赛代码、原有测试、历史来源记录
sources/           参赛入口需要的Moving AI地图与署名
provenance/        提交来源哈希、原始指纹和公开整理差异
docs/              架构、AGH接入、版本与结果说明
tools/             公开副本校验、本机参数生成
reports/           仅补入原有测试依赖的固定策略fixture
```

新用户请使用`run_checked_final_mcp.py`。旧`run_checked_mcp.py`、`scripts/verify_sources.py`、`scripts/freeze_candidate.py`及历史README记录旧工作区，不作为本仓库的安装或启动入口。

## 来源与许可

本项目团队自有代码及公开整理文档采用 [MIT License](LICENSE)。Moving AI地图保留其ODC-By v1.0许可与署名；AGH作为外部依赖单独安装，遵循其自身许可。既有控制与状态处理来源、文件边界和引用见 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。

未来改进请使用其他分支或新标签，并在结果中注明版本。`competition-v14`用于标识本次参赛代码，公开整理差异可通过`python tools/verify_release.py`检查。
