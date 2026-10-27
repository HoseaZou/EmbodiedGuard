# Integration Standards — Competition V14

**English** | [简体中文](INTEGRATION.zh-CN.md) · [Overview](../README.md) · [Architecture](ARCHITECTURE.en.md)

This specification describes the existing competition implementation, not a new device protocol. Its executable source of truth is [`mcp_server.py`](../EmbodiedGuard-v14/embodiedguard/mcp_server.py). The supported entry is [`run_checked_final_mcp.py`](../EmbodiedGuard-v14/run_checked_final_mcp.py).

## 1. Runtime and Transport

Install Python 3.11+ and the dependencies from the root README. AGH is a separate application installed from its [official repository](https://github.com/AgnesAI-Labs/agnes-harness). Its model provider supplies the LLM; this Python server does not call a model.

Run `python tools/print_mcp_config.py` **inside the installed Python environment**. It prints `mcpServers → embodiedguard-competition-v14 → command / args` with machine-local absolute paths. Add the parameters to the corresponding AGH stdio MCP settings. `command` is the Python executable; each member of `args` is a separate argument. The generated JSON is local configuration, not a file to commit with personal paths.

The server speaks newline-delimited UTF-8 JSON-RPC 2.0 over stdin/stdout. Only protocol responses go to stdout; startup and error diagnostics go to stderr. Handshake:

```text
initialize → notifications/initialized → tools/list → tools/call
```

The server negotiates these implemented MCP protocol versions: `2025-11-25`, `2025-06-18`, `2025-03-26`, `2024-11-05`. This lists implemented versions, not an external conformance certification. The six-tool focused catalog is authoritative; historical baseline tools are not separately exposed in this profile.

## 2. Mission Configuration

| Mode | Configuration | Task and routes |
|---|---|---|
| Public map | `--mode public --map maze512-16-0 --line 142 --scale 0.5 --fault freeze_once` | One arrival-and-stop goal, identified as `exit_c`; `direct`, `upper`, `lower`, `global` are available subject to current task permissions |
| Synthetic inspection | `--mode inspection --scenario blocked --response-mode compact --interface-profile focused` | Visit `panel_a`, `pump_b`, `exit_c` in the required order; `direct`, `upper`, `lower` |

Both require `--output` under the repository root. Public maps are restricted to `16room_000`, `32room_000`, `maze512-16-0`; metres per cell is `0.5` or `1.0`; fault is `none`, `freeze_once` or `persistent_freeze`. `--line` is a scenario-file line selector, not a metric or random seed. Entry preflight verifies pinned implementation and map hashes and the output path budget.

Use a **new mission process and new AGH session per independent trial**; a new chat alone does not reset a running Python mission. AGH management probes can create their own evidence directories. Confirm initial frame 0, stationary state, no completed targets, correct task and a fresh episode. Synthetic inspection starts at the origin; public-map pose comes from the selected map scenario. Episode IDs in the final public-map runtime are opaque.

## 3. Six Focused Tools

All inputs are objects with `additionalProperties=false`. Types, required arguments and enumerations come from `tools/list` in the active mode. Numeric inputs must be finite; booleans are not accepted as numbers.

| Tool | Required arguments | Optional arguments | Meaning |
|---|---|---|---|
| `observe` | None (`{}`) | None | Read visible state, freshness, task and budgets |
| `navigate` | `target`, `route`, `route_mode`, `operation_id` | `rationale` | Explicit new plan or exact active-handle resume, checked execution, braking and inspection only on arrival and stop |
| `reobserve` | None (`{}`) | `rationale` | Reacquire a frame while stopped; a persistent freeze may remain invalid |
| `wait` | None (`{}`) | `seconds`, `rationale` | Stay stopped and advance the environment for `0 < seconds <= 5`, then observe; omitted `seconds` defaults to 1 |
| `stop` | None (`{}`) | `terminate`, `reason` | Brake; `terminate=true` ends the mission irreversibly |
| `verify` | None (`{}`) | None | Obtain the current local verdict, evidence, safety metrics and budgets |

`route_mode` is `plan` or `resume`. `target` is a schema-listed identifier, but runtime task order is additionally enforced. Route enums differ by mission as above. `operation_id` has 1–128 characters; `rationale` and `reason` are strings of at most 1000 characters. `terminate` is a boolean. Prefer an explicit `seconds` value when choosing a wait; its implemented default is 1 second.

### Navigation and Idempotency

- `plan` explicitly requests new geometry for the selected target and route.
- `resume` must match the active current-target/current-route handle; missing or invalid handles are rejected without a silent planning fallback. The handle is server-owned; this focused API selects it semantically through target/route rather than accepting a caller-provided `route_id`.
- Public-map `navigate` runs at most **two original windows** on the selected handle. Synthetic focused inspection runs **one**. Each window tracks for at most 15 simulated seconds, then brakes separately; this is not a 15-second tool wall-time guarantee.
- Only a successful partial public-map window may continue internally. Failure or arrival returns immediately. Alternative routes, recovery, next targets and final `verify` remain Agent choices.
- Use a **new ID for every new execution or continuation**. Repeating the same ID and signature returns a historical receipt without new motion. A different signature with an old ID returns `OPERATION_CONFLICT`. Even `rationale` is part of the signature, so exact retries must preserve it.

## 4. Illustrative Calls and Feedback

The following is an **interface example**, not a recorded experiment or guaranteed sequence. Read current `tools/list` and task feedback first. `exit_c/global` applies to the configured public-map mission:

```json
{"jsonrpc":"2.0","id":1,"method":"initialize","params":{"protocolVersion":"2025-11-25","capabilities":{},"clientInfo":{"name":"example-client","version":"1"}}}
{"jsonrpc":"2.0","method":"notifications/initialized"}
{"jsonrpc":"2.0","id":2,"method":"tools/list"}
{"jsonrpc":"2.0","id":3,"method":"tools/call","params":{"name":"observe","arguments":{}}}
{"jsonrpc":"2.0","id":4,"method":"tools/call","params":{"name":"navigate","arguments":{"target":"exit_c","route":"global","route_mode":"plan","operation_id":"example-nav-1"}}}
```

Only **if actual feedback reports a recoverable stale observation**, the Agent may choose `reobserve`. A later resume requires fresh state, a matching valid route and sufficient budgets. The illustrative calls are:

```json
{"jsonrpc":"2.0","id":5,"method":"tools/call","params":{"name":"reobserve","arguments":{"rationale":"Reacquire after the reported stale observation"}}}
{"jsonrpc":"2.0","id":6,"method":"tools/call","params":{"name":"navigate","arguments":{"target":"exit_c","route":"global","route_mode":"resume","operation_id":"example-nav-2"}}}
```

After actual final arrival/inspection, call `verify`. When there is no justified continuation, call `stop` with `terminate=true`, then `verify`. An `ARRIVED` status is a reason to request final verification, not permission to invent its result.

```json
{"jsonrpc":"2.0","id":7,"method":"tools/call","params":{"name":"verify","arguments":{}}}
```

Responses include `result.content` (JSON text), `result.structuredContent` and `result.isError`. Malformed or unknown calls return JSON-RPC errors. A dispatched tool failure uses `isError=true` when its result has `ok=false`. A `verify` verdict of `task_success=false` is still a meaningful evaluator result; inspect that field rather than treating transport success as task completion.

| Feedback | How to use it |
|---|---|
| `status`, `reason`, `navigation`, `inspection` | Distinguish planning rejection, partial progress, stopped failure, arrival and actual inspection; fields depend on the tool |
| `observation`, state and freshness metadata | Check episode/frame/time, validity and source; never fill in missing state by guesswork |
| `task` | Respect current revision, order, next target and permitted routes; repeated detail may be omitted using `details_revision` |
| `budget`, `tool_budget`, `recovery` | Respect current remaining time, execution/request counts, recovery quota and conditional resume metadata |
| Final `verify` | Read `task_success`, `safe_termination`, `terminated`, missing/completed targets, stopping, metrics and mode-specific evidence |

## 5. Coordinates, Safety and Verification

World coordinates use **metres**, yaw **radians** with counterclockwise positive direction, and time **seconds**. Motion is forward Ackermann kinematics. The controller adapter handles the retained driving module's conventions. Do not reinterpret its controls as differential-drive wheel commands. Plans are spatial routes, not a guaranteed timed reference trajectory.

Local state, action, braking, obstacle, footprint, boundary, speed and budget checks remain mandatory. Conditional forecasts and `recovery.resume` metadata do not authorize motion or refresh sensing. Public-map execution also checks raw occupancy. Bounded global search failure is not an impossibility certificate.

The default deployment clock charges fixed-step motion plus measured between-tool wall waits after first planning, including model or approval delays. The mission deadline is 300 seconds, execution cap 80, tool threshold 120, and recovery cap 3 after failures. `stop` and `verify` remain callable beyond the request threshold; they do not extend the mission budget. Recovery exhaustion rejects the recovery and brakes; it does not by itself guarantee a terminal mission. Explicitly stop with termination if appropriate. Time or other hard budget exhaustion can terminate automatically.

`verify` runs locally and returns to the Agent through MCP/AGH. Call it promptly after the final inspection/arrival or termination. Per-step safety is separate and continuous inside execution. Its evaluator checks current completion, stopping, safety and deadline; the public entry also applies budget handling. A historical `completion_receipt` cannot override a later failure. **Safe termination and successful completion are separate outcomes.**

## 6. Evidence and Acceptance

Outputs go to `<output>/run-*/`: `summary.json`, `events.jsonl`, `trace.jsonl`, `plans.jsonl`, `run.json`, plus mode-specific evidence. Actual motion traces and simulator evaluations differ from AGH's native model/tool records. Simulator fields `model_call_verified=false`, `model_usage=null`, `model_cost=null` do not authenticate a model session. Preserve AGH evidence separately and export official diagnostics through the AGH UI when required; check credentials and account context before sharing.

```sh
python tools/verify_release.py
python tools/smoke_mcp.py
python -m pytest -q EmbodiedGuard-v14/tests
```

The first command checks provenance; the second checks the checked-entry stdio handshake, six-tool catalog, initial observation, stop and verify with **zero motion / zero model calls**. The test suite covers local engineering behavior. An end-to-end Agent trial additionally requires AGH, a model account, tool permissions, fresh mission state, actual calls and saved final verification.

Do not publish API keys, personal configuration, raw account-bearing diagnostics or runtime folders. The server exposes neither shell nor general file tools. AGH approval policy remains configured in AGH; connecting the server does not disable it. This release implements an MCP simulator integration, **not an MHS hardware adapter or certified robot driver**.
