# 集成标准 — 参赛版本V14

[English](INTEGRATION.md) | **简体中文** · [项目首页](../README.zh-CN.md) · [技术架构](ARCHITECTURE.md)

本规范描述已经实现的参赛接口，不另行定义设备协议。可执行依据是 [`mcp_server.py`](../EmbodiedGuard-v14/embodiedguard/mcp_server.py)，支持入口为 [`run_checked_final_mcp.py`](../EmbodiedGuard-v14/run_checked_final_mcp.py)。

## 1. 环境与传输

按根README安装Python 3.11+与依赖。AGH从[官方仓库](https://github.com/AgnesAI-Labs/agnes-harness)单独安装，大模型由AGH模型供应方提供；Python工具服务自身不调用模型。

**在已安装依赖的Python环境内**运行 `python tools/print_mcp_config.py`，获得 `mcpServers → embodiedguard-competition-v14 → command / args`，填入AGH对应的stdio MCP设置。`command`是Python可执行文件，`args`各项是独立参数，不把整条shell命令塞进command。生成的本机绝对路径配置留在本地，不随个人路径提交Git。

协议为stdin/stdout上的UTF-8逐行JSON-RPC 2.0：stdout仅输出协议响应，启动及异常诊断写stderr。连接顺序：

```text
initialize → notifications/initialized → tools/list → tools/call
```

已实现的MCP版本协商支持 `2025-11-25`、`2025-06-18`、`2025-03-26`、`2024-11-05`。这是实现列表，不代表第三方认证。以focused六工具目录为准，旧baseline工具不在此配置中单独暴露。

## 2. 任务配置

| 模式 | 配置参数 | 任务及路线 |
|---|---|---|
| 公开地图 | `--mode public --map maze512-16-0 --line 142 --scale 0.5 --fault freeze_once` | 单目标到达停稳，目标标识为 `exit_c`；提供 `direct/upper/lower/global`，仍受当前任务权限约束 |
| 合成巡检 | `--mode inspection --scenario blocked --response-mode compact --interface-profile focused` | 按要求顺序访问 `panel_a/pump_b/exit_c`；路线 `direct/upper/lower` |

两种模式都必须指定仓库内的 `--output`。公开地图可选 `16room_000`、`32room_000`、`maze512-16-0`，每格米数为 `0.5/1.0`，故障为 `none/freeze_once/persistent_freeze`。`--line`选择场景文件行，不是性能指标或随机种子。入口检查冻结代码和地图哈希、输出路径范围与长度。

独立试验使用**新任务进程和新AGH会话**，只新建聊天不会重置Python任务。AGH管理探测可能生成独立证据目录。确认首次frame=0、静止、无已完成目标、任务正确且episode为新建。合成巡检从原点开始；公开地图初始位姿来自所选地图场景。最终公开地图运行时使用不透露故障名称的episode标识。

## 3. 六个focused工具

输入均为对象且 `additionalProperties=false`；类型、必填项、枚举与范围以当前 `tools/list` 为准。数值必须有限，不把布尔值当数值。

| 工具 | 必填参数 | 可选参数 | 语义 |
|---|---|---|---|
| `observe` | 无，传 `{}` | 无 | 读取可见状态、观测有效性、任务与预算 |
| `navigate` | `target/route/route_mode/operation_id` | `rationale` | 显式规划或原有效句柄续行，经检查执行并制动，仅到达停稳后巡检 |
| `reobserve` | 无，传 `{}` | `rationale` | 停车重采样；持续冻结可能仍无效 |
| `wait` | 无，传 `{}` | `seconds/rationale` | 停车推进环境，`0 < seconds <= 5`，之后观测；省略seconds时默认1秒 |
| `stop` | 无，传 `{}` | `terminate/reason` | 制动；`terminate=true`不可逆地终止任务 |
| `verify` | 无，传 `{}` | 无 | 返回当前本地判定、证据、安全指标与预算 |

`route_mode`为 `plan/resume`。`target`须在Schema枚举内，此外运行时还检查任务顺序；路线枚举随上表模式变化。`operation_id`长度1–128；`rationale/reason`最多1000字符；`terminate`为布尔值。选择等待时建议显式传入seconds，实现中的默认值为1秒。

### 导航与幂等性

- `plan`明确请求当前所选目标和路线的新几何规划。
- `resume`必须匹配当前目标、当前路线的活动句柄；缺失或失效会被拒绝，不暗中改成新规划。句柄由服务端管理，此focused API通过target/route选择，不要求调用方提交 `route_id`。
- 公开地图每次navigate最多**两个原始窗口**，合成focused巡检为**一个**。每窗跟踪最多15秒仿真时间，之后另行制动；不是工具总耗时15秒保证。
- 仅正常部分完成的公开地图窗口可内部续行，失败或到达立即返回。替代路线、恢复、新目标与最终verify仍由Agent决定。
- **新执行和新续行必须用新ID**。相同ID与签名重试返回历史收据，无新运动；同ID换请求返回 `OPERATION_CONFLICT`。签名包含 `rationale`，精确重试时也须保持不变。

## 4. 请求与反馈示例

以下仅为**接口示例，不是实验日志或保证执行顺序**。先读取当前工具Schema与任务；`exit_c/global`用于上述公开地图配置：

```json
{"jsonrpc":"2.0","id":1,"method":"initialize","params":{"protocolVersion":"2025-11-25","capabilities":{},"clientInfo":{"name":"example-client","version":"1"}}}
{"jsonrpc":"2.0","method":"notifications/initialized"}
{"jsonrpc":"2.0","id":2,"method":"tools/list"}
{"jsonrpc":"2.0","id":3,"method":"tools/call","params":{"name":"observe","arguments":{}}}
{"jsonrpc":"2.0","id":4,"method":"tools/call","params":{"name":"navigate","arguments":{"target":"exit_c","route":"global","route_mode":"plan","operation_id":"example-nav-1"}}}
```

仅在**实际反馈指出可恢复的观测过期**时，Agent可选择重观测。后续resume仍需有效新观测、匹配的有效路线和足够预算，示例：

```json
{"jsonrpc":"2.0","id":5,"method":"tools/call","params":{"name":"reobserve","arguments":{"rationale":"Reacquire after the reported stale observation"}}}
{"jsonrpc":"2.0","id":6,"method":"tools/call","params":{"name":"navigate","arguments":{"target":"exit_c","route":"global","route_mode":"resume","operation_id":"example-nav-2"}}}
```

实际最终到达/巡检后调用verify；没有有据可行的续行时，先 `stop(terminate=true)` 再verify。`ARRIVED`是请求最终核验的依据，不代表可自行填写核验结果：

```json
{"jsonrpc":"2.0","id":7,"method":"tools/call","params":{"name":"verify","arguments":{}}}
```

返回包括 `result.content`（JSON文本）、`result.structuredContent` 与 `result.isError`。格式错误或未知调用返回JSON-RPC错误；已分派工具若结果 `ok=false`，则 `isError=true`。verify的 `task_success=false` 仍是有效判定，应读取该字段，不能将协议受理当成任务完成。

| 反馈 | 判读方法 |
|---|---|
| `status/reason/navigation/inspection` | 区分规划拒绝、部分完成、失败停车、到达与实际巡检，具体字段随工具变化 |
| `observation`、状态与时效字段 | 检查episode/frame/time、有效标志与来源，缺失内容不猜测补齐 |
| `task` | 遵守当前revision、顺序、next_target与允许路线；重复详情可能以 `details_revision`省略 |
| `budget/tool_budget/recovery` | 读取当前时间、执行/请求余量、恢复次数和条件式resume提示 |
| 最终verify | 查看 `task_success/safe_termination/terminated`、已完成/缺失目标、停稳、安全指标及模式专属证据 |

## 5. 坐标、安全与核验

世界坐标使用**米**，yaw使用**弧度**且逆时针为正，时间为**秒**。运动模型是前向Ackermann，控制适配层处理既有驾驶模块的约定，不能直接解释成差速轮速指令。规划结果为空间路线，不是保证可执行的定时参考轨迹。

本地状态、动作、制动、障碍、车体占位、边界、速度与预算检查始终生效。条件式预测与 `recovery.resume`不是运动许可，也不刷新传感器；公开地图还检查原始占位。有界全局搜索失败不等于不可达证明。

部署默认计时包含固定步运动，以及首次规划后工具之间的实测墙钟等待，包括模型或审批等待。任务限时300秒，执行上限80，工具阈值120，失败后恢复上限3次。stop/verify超过请求阈值仍可用，但不延长预算。恢复额度耗尽会拒绝恢复并制动，不保证任务已终止；适当时须显式终止。时间等硬预算耗尽可自动终止。

verify在本地运行，经MCP/AGH返回Agent，应在最终巡检/到达或终止后及时调用。执行中的逐步安全检查独立运行。核验器检查当前完成、停稳、安全和时限，入口还处理预算；历史 `completion_receipt`不能覆盖后续失败。**安全终止与任务成功分别统计。**

## 6. 留证与接入验收

输出写入 `<output>/run-*/`：`summary.json/events.jsonl/trace.jsonl/plans.jsonl/run.json`及模式专属证据。实际运动与仿真判定不同于AGH原生模型/工具记录，仿真字段 `model_call_verified=false/model_usage=null/model_cost=null`不能认证真实模型会话。AGH原生证据另行保留，必要时由官方界面导出，分享前检查凭据和账号上下文。

```sh
python tools/verify_release.py
python tools/smoke_mcp.py
python -m pytest -q EmbodiedGuard-v14/tests
```

第一条检查来源一致性；第二条检查校验入口stdio握手、六工具目录、初始观测、stop/verify，**零运动、零模型调用**；回归覆盖本地工程行为。Agent端到端验收还需要AGH、模型账号、工具权限、新任务状态、实际调用及最终核验留存。

不公开Key、个人配置、含账号诊断包和运行目录。服务不暴露shell或任意文件工具；AGH审批策略仍在AGH配置，接入服务不关闭它。本版本是MCP仿真接入，**不是MHS硬件适配器或认证机器人驱动**。
