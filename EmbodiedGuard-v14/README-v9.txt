EmbodiedGuard-v9 — exact selected-route reuse candidate

Source: a hash-verified copy of all 81 frozen EmbodiedGuard-v8 source files.
Inherited README-v5/v6/v7/v8 files are historical. This file describes v9.
Original v8 sources, evidence, v7 video/archive and runtime configuration are
preserved. No hardware, MHS, dynamics, vision accuracy or learned VLA claim.

Select the new semantics explicitly via compact + lean (or bundle):
  run_checked_mcp.py --task-fixture restricted_detour --seed 39
    --response-mode compact --interface-profile lean
    --clock-policy measured_idle_wall_deployment --output <new local folder>
The checked entry verifies reports/implementation/20261012-route-reuse-v9/
source-candidate.json. Default baseline retains the original eight-tool API.

navigate(target, route, NEW operation_id) still selects exactly one route for
one target. When its target AND route match the active handle AND current
required target, it invokes the original guarded ExecutionBundle on that handle
without rebuilding geometry or resetting the tracker. No historical/superseded
handle is searched. The caller's route selection is not changed.

If no exact current handle matches, navigate performs its original checked plan
and 15-second bounded execution/stop/arrival-inspection. If a matching handle
fails version, permission, sensing, budget or actuator safety checks, the failure
returns to the Agent without silently replanning, refreshing sensors or changing
routes. plan_route remains the explicit tool for requesting a new geometry.

navigation.route_reused/planning_dispatched/source_plan_id and parent-linked
navigation_route_reused events distinguish real reused plans from new plans.
Historical operation retries remain historical. A new operation_id is required
for any actual continuation. stop and independent final verify remain Agent calls.

Only actual new planning is charged to the existing recovery ledger. Reusing
geometry does not itself authorize motion or consume a fictitious planning
recovery. The same 300-second deadline, real between-tool idle accounting,
three recovery actions, 80 execution operations and 120 native requests apply.

Eleven modules are unchanged: control, planning, state, safety, sim, execution,
timing, task_contract, shadow, recovery and execution_bundle. Speed limits,
tracking window, per-step safety and independent verification are unchanged.

CPU regression: 246 tests and 87 parameter subcases passed before source freeze.
28 deterministic comparisons distinguish a strong complete client from a
deliberate semantic handoff/continuation stress client. CPU runs have zero model
calls and do not establish new Agnes latency or advantage over a strong policy.
New native evidence must be separately preregistered, fresh, single-submission,
exactly scoped and independently bound; no old run is resubmitted or replaced.

Implementation/reproduction: ../reports/implementation/20261012-route-reuse-v9/
Native validation: ../reports/agent-validation/20261012-route-reuse-v9-r1/
