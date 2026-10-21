EmbodiedGuard v14｜已知公开地图全局规划候选

冻结 v13 位于同级 EmbodiedGuard-v13，本候选不覆盖 v13、原自动驾驶源码、历史报告或提交材料。
适用范围：已知人工静态地图、合成传感器故障、前向 Ackermann 二维运动学。没有真实温度/视觉感知、差速机器人、车辆动力学或硬件安全验证。

新增能力：
1. global：有限时间/节点预算的连续 x,y,yaw 曲率受限搜索；普通巡检保持 direct/upper/lower 原接口。
2. 多次转弯按弧长维护剩余路线，保持原 RouteManager、PID 的执行器限幅和变化率。
3. 仅全局曲线使用有符号曲率参考和转向过渡减速；没有提高转向、速度或安全距离阈值。
4. 细格自由空间上包络不连通时提供占位不可达证据；普通网格失败/搜索预算耗尽仍为未知，不宣称无路。
5. 公开地图 navigate 请求至多两个原始 15 秒窗口。每窗口重新观测、检查预算/每个动作并制动。
   失败或到达立即返回；工具不选择替代路线、不恢复、不改目标、不代替 Agent 调用最终 verify。
6. 当前任务 envelope 与完整工具反馈一致落盘；微小制动位移按保守误差膨胀处理，不能消失掉安全检查。

安全预算：300 秒（首个规划后包括真实工具间决策等待）、80 个原始执行操作、120 工具请求、3 次恢复。
半径 0.45m，安全余量 0.12m；全局搜索是近似有界搜索，没有最短路线/全局最优保证。

运行公开任务（PowerShell，官方 Agnes Harness 配置的 stdio executable 指向 Python）：
python run_public_mcp.py --map maze512-16-0 --line 143 --scale 0.5 --fault none --output ../runtime_data/my-v14-run
python run_public_mcp.py --map maze512-16-0 --line 142 --scale 0.5 --fault freeze_once --output ../runtime_data/my-v14-stale
公开地图文件保持在 ../sources/movingai-public-maps-20261013。公开地图功能需要 numpy>=1.26,<3；测试需要 pytest。
旧演示仍用 run_checked_mcp.py/run_mcp.py，其三目标巡检测量仍是 synthetic_fixture。

Agent 建议：先 observe；依据当前 permitted_routes 和能力选路线。已知地图可选择 global；失败时读 search_reason、
global_infeasibility_proven、观测有效性和剩余预算，再选择 reobserve/新规划/安全终止。部分完成时只续当前明确路线。
全部完成或终止立即 verify。所有模型必须遵守赛事规则，作品运行时使用官方 Agnes；开发工具不是运行时模型证据。

当前候选代码来源与改动、完整对照与真实模型留证，以 ../reports/experiments/20261013-v14-optimization-r1 为准。
任何几何/控制/费用提升都以实际报告为准，不从此 README 推断测试已经执行。

公开数据：Moving AI Lab 2D Grid Benchmarks，ODC-By v1.0，原始 .map/.scen 文件及署名位于 sources 目录。
算法设计参考：Dolgov et al. (2008), Practical Search Techniques in Path Planning for Autonomous Driving。
https://ai.stanford.edu/~ddolgov/papers/dolgov_gpp_stair08.pdf
全局规划与不可达诊断由本候选实现；不是复现原论文完整算法，也未引入新的训练模型。
