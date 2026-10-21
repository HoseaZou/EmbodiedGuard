EmbodiedGuard v7：任务约束与恢复执行

新增 embodiedguard/task_contract.py，在冻结执行器外围管理巡检目标子集、条件分支、通道权限与任务版本。
四个开发诊断夹具：selective_targets、conditional_skip、restricted_detour、restricted_unreachable。
更新只在实际检查 panel_a 并停稳后触发；温度/异常仍来自原合成巡检夹具，不是视觉检测或真实设备。
模型与完整任务感知固定策略读取相同结构化 task；本轮不声称未知自然语言理解、泛化或模型优于规则。
控制、规划、状态、安全、运动学、执行、计时、恢复八个核心文件保留 v6 原始字节。
新旧代码及实验分别保存。旧 v4/v5/v6、比赛 ZIP 不覆盖。CLI 的原场景与运动故障继续可用。
示例：python run_mcp.py --task-fixture restricted_detour --response-mode compact --output outputs/task-demo
