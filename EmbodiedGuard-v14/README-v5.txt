EmbodiedGuard v5 — 动作契约与条件性时序轨迹（独立候选版）

来源：按 20261011-efficiency-r2/source-candidate.json 的 64 个 SHA-256 文件，从冻结 EmbodiedGuard-v4 逐项复制。
原 v4、原自动驾驶仓库、旧实验及比赛预审压缩包保留。规划、控制、状态、安全、运动学、执行、时耗估计、恢复账本八个模块未修改。

本版新增：
1. run_mcp.py 默认采用 ShadowMission，仍通过原 MCP 的八个工具执行。--disable-shadow 可切回原执行器。
2. execute_route 的实际同帧观测作为锚点；深复制当前跟踪器，生成最长 3 秒、0.05 秒固定步长的条件性预测。
3. 预测同时包含世界坐标 (t,x,y,yaw,v) 与测量锚点下的局部位移 / 转角。借鉴 AutoVLA 的局部运动表达，没有使用其权重、码本或动作 token。
4. 只读动作契约绑定实际工具请求和结果；恢复记录绑定真实失败与后续 plan_route/reobserve/wait/stop 请求。没有短理由就记 null，不补造 Agent 意图、隐藏推理或根因。
5. 新 HTML 回放叠加橙色预测，区分空间参考、条件性预测和实际执行，显示对应锚点及截至游标的误差。预测不可用、历史窗口过期均明确标注。

强制边界：
预测不授权运动，不提前保证安全。原执行器依然拥有同帧观测、预算、逐步安全、制动、巡检与 verify 权限。
模型工具目录、调用参数与返回保持原样；完整预测数组和动作契约只存本地 sidecar。无新增模型往返或传感器读取。
有效历史计划的旧观测不能授权新执行。每次新执行使用实际传感器观测；过期预测只作历史显示。
重复 operation_id 依照原实现返回历史结果，不产生新的预测、运动或巡检。
预测不包含未来障碍、安全中断、传感器故障和执行段末制动。安全中断后的预测与实际分离是预期边界，不能据预测继续运动。
本验证仅为合成二维 Ackermann 运动学；同一模型产生预测和实际仿真，吻合不能证明实机准确率。

在 PowerShell 启动本版（示例，AGH 启动配置仍应使用绝对路径）：
& '<user-home>\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe' '<competition-workspace>\EmbodiedGuard-v5\run_mcp.py' --scenario blocked --response-mode compact --output '<competition-workspace>\EmbodiedGuard-v5\outputs\mcp'
stdio 通道只输出 MCP JSON-RPC；不要把它当普通交互命令行输入自然语言。
本轮没有替换运行中的 AGH 配置或进行新模型实验。旧决策重放会在报告中单独标注。

本地 sidecar：shadow.json（绑定原 run.json 和各 sidecar 的 SHA-256）、trajectory_previews.jsonl、preview_checks.jsonl、action_contracts.jsonl、recovery_decisions.jsonl、shadow_errors.jsonl。
原 summary.json/run.json/events.jsonl/trace.jsonl/plans.jsonl 保持原格式。evidence.load_run 原样验证原执行证据；shadow.attach_shadow 检查绑定哈希后再附加新证据。
外部证据加载器增加 compact 记录支持：重新计算精简投影并与模型实际返回逐项核对，同时将完整 verify 与本地 summary 核对。模型来源标记和费用不被改写；原 v4 加载器及旧数据未修改。
传统 CLI demo/experiment 入口仍使用原 Mission；需要预测证据时使用本版 MCP 入口或 validate_shadow.py，不将旧 CLI 输出宣传成新能力已运行。

重现本轮工程配对与已有动作重放：
& '<user-home>\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe' '<competition-workspace>\EmbodiedGuard-v5\tools\validate_shadow.py' --output '<competition-workspace>\EmbodiedGuard-v5\outputs\shadow-recheck-new-directory' --native-root '<competition-workspace>\reports\agent-validation\20261011-recovery-comparison-r2'
输出目录必须不存在；脚本拒绝覆盖旧结果。0/30 秒延迟是单独的受控时钟诊断，部署 MCP 仍计真实模型等待。新场景配对、旧动作重放均不算新的 Agnes 调用，更不宣称 Agent 已优于固定规则。

后续优先：冻结本版后进行新的 Agnes 故障暴露实验，保证运动阶段确实遭遇故障；同时保留同信息、同恢复选项的完整固定基线。若持平，如实保留。无需引入训练、硬件或复杂机器人平台。
