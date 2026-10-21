EmbodiedGuard v6 — motion-triggered diagnostic fixtures

Independent candidate based on frozen EmbodiedGuard-v5. Original v4/v5 and
competition ZIP remain unchanged. The eight planning, control, state, safety,
simulation, execution, timing and recovery core files are copied unchanged.

New fixture adapter: embodiedguard/motion_faults.py.
All eight MCP tools and the compact arrival-inspection-15s-v1 profile remain.
The evaluator injects faults only after cumulative actual plant displacement:
  motion_sensor_freeze: 2 m; sensor packet freezes until actual reobserve.
  motion_blocked_aisle: 6 m; a stationary circle blocks the direct aisle.
  motion_sensor_unavailable: 2 m; observation stays unknown after reobserve.
Stationary wall-clock waiting cannot trigger or cure these faults. Journals
are evaluator-only and are absent from Agent tool feedback. All three frame-0
observations and tool catalogs are identical, including episode_id=motion-0.

Run example (substitute your Python executable):
  python run_mcp.py --fixture motion_sensor_freeze --clock-policy execution_only_diagnostic --response-mode compact --output outputs/example

Two explicitly separated clock conditions:
  execution_only_diagnostic: only simulator steps advance task time. Used to
    isolate the recovery mechanism; this is not a deployment timing result.
  measured_idle_wall_deployment: measured model/tool approval idle after the
    first plan advances simulator time under the original 300 s deadline.
Legacy --scenario calls keep their original measured-idle clock by default.
For --fixture, the nominal base world is normal; --scenario is not consulted.

Deterministic control baselines reuse the frozen feedback-only policies. The
first new target route is direct for every method. Recovery cap remains 3.
No extra route, safety bypass, future-state hint, model or broad permission.

Fault and shadow evidence are bound to the immutable final-verify run snapshot
by SHA-256. Shadow previews remain advisory and never authorize execution.
Validation is synthetic 2D Ackermann kinematics only. No physical safety,
learned VLA equivalence, real detection accuracy or Agent superiority claimed.
