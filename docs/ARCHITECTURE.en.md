# Architecture and Execution Chain

**English** | [简体中文](ARCHITECTURE.md) · [Overview](../README.md) · [Integration](INTEGRATION.md)

## Responsibilities

**AGH** hosts sessions, model calls, tool permissions/dispatch and native records. **Agnes 3.0 Flash** reads tool schemas and feedback and selects explicit calls. **MCP** transports and validates those calls; it does not decide which recovery is appropriate. **EmbodiedGuard's local executor** owns planning, tracking, safety and simulation. **Local verify** establishes the current verdict.

```mermaid
sequenceDiagram
    actor User
    participant H as AGH
    participant A as Agnes 3.0 Flash
    participant M as MCP server
    participant X as Checked executor and 2D world
    participant V as Local verifier
    User->>H: Task and supervisor-owned constraints
    H->>A: Task, tool schemas and current context
    A->>H: observe with arguments
    H->>M: tools/call
    M->>X: Read actual observation
    X-->>M: State, freshness and budgets
    M-->>H: Structured result
    H->>A: Tool feedback
    A->>H: navigate: target, route, plan, operation_id
    H->>M: tools/call
    M->>X: Check plan, track, check actions, brake
    X-->>M: Outcome or failure with actual state
    M-->>H: Structured result and budgets
    H->>A: Feedback for next decision
    opt Agent chooses recovery based on failure
        A->>H: reobserve, wait, new plan or resume
        H->>M: Explicit selected call
        M->>X: Original checks and bounded operation
        X-->>M: Measured result
        M-->>H: Result
        H->>A: Feedback; repeat decision loop if needed
    end
    opt No justified continuation
        A->>H: stop with terminate=true
        H->>M: tools/call
        M->>X: Brake and terminate
        X-->>M: Stopped/termination state
        M-->>H: Result
        H->>A: Feedback
    end
    Note over A,V: After actual final arrival/inspection or termination
    A->>H: verify
    H->>M: tools/call
    M->>V: Current local evaluation and budget handling
    V-->>M: Completion, safe exit or incomplete; evidence
    M-->>H: Structured verdict
    H->>A: Final verification result
    A->>H: Report supported by the verdict
    H-->>User: Result
```

## Local Implementation

- [`mcp_server.py`](../EmbodiedGuard-v14/embodiedguard/mcp_server.py) validates schemas and dispatches the six focused tools.
- [`efficiency.py`](../EmbodiedGuard-v14/embodiedguard/efficiency.py) implements explicit plan/resume, idempotency and the bounded chosen-route transaction. A valid resume reuses geometry and tracker state rather than replanning.
- [`planning.py`](../EmbodiedGuard-v14/embodiedguard/planning.py) and [`global_planning.py`](../EmbodiedGuard-v14/embodiedguard/global_planning.py) generate candidate spatial routes. Global search is bounded and forward-only; it is not a complete reachability oracle.
- [`control.py`](../EmbodiedGuard-v14/embodiedguard/control.py) and [`legacy/`](../EmbodiedGuard-v14/embodiedguard/legacy/) adapt the retained driving route/PID modules. The simulated platform is Ackermann, not differential drive.
- [`state.py`](../EmbodiedGuard-v14/embodiedguard/state.py), [`safety.py`](../EmbodiedGuard-v14/embodiedguard/safety.py) and [`action_contract.py`](../EmbodiedGuard-v14/embodiedguard/action_contract.py) enforce state and motion checks. The model exposes no tool to turn them off.
- [`execution.py`](../EmbodiedGuard-v14/embodiedguard/execution.py) owns execution, stopped inspection, budgets, logs and the base verdict. [`public_maps.py`](../EmbodiedGuard-v14/embodiedguard/public_maps.py) adds known-map occupancy checks and arrival evidence; [`runtime_final.py`](../EmbodiedGuard-v14/embodiedguard/runtime_final.py) provides final integration bookkeeping.

## Navigation, Failure and Verification

The focused synthetic-inspection mode has one tracking window per navigation call. The public-map mode has at most two on the same chosen handle. Each tracks up to 15 simulated seconds, then brakes separately. A successful partial public-map window may continue to the second; failure returns immediately. There is no hidden alternative-route selection, sensor refresh, next-target switch or final verify inside navigate.

Unknown or stale state is not free space. The executor checks observation identity/freshness and candidate actions, including braking, footprint, speed, obstacles, bounds and available budgets. The Agent decides what to attempt after feedback. Recovery exhaustion rejects the attempted recovery and brakes; explicit termination may still be needed. Hard budget exhaustion can terminate automatically.

The Agent calls `verify` promptly after actual final inspection/arrival or termination. It is **not called as a model operation after every control step**. The local result returns to AGH and the Agent through MCP; the Agent bases the user-facing answer on it. The verdict is independent of model wording. The interface also handles exhausted budgets, so the whole entry should not be described as side-effect-free. A historical completion receipt is not a substitute for the current verdict.

## Recorded Recovery Example

The recorded business-tool sequence was:

```text
observe → navigate(plan, global) → STALE_OBSERVATION / stopped
        → reobserve → navigate(resume, global) → verify
```

The freeze was a controlled fault fixture. The Agent read the failure, chose reacquisition and then the existing route. The public source includes the implementation and original r3 prompt; account-bearing native diagnostic ZIPs remain in the retained competition evidence rather than this repository. The [demo video](https://www.bilibili.com/video/BV1WYpm6vErh/) presents the recorded workflow.
