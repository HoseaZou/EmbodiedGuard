"""Public control contract, independently integrated kinematic examples only."""
import math
import pytest

from embodiedguard.control import LegacyTracker


def run_route(points, *, start=(0.0, 0.0, 0.0), seconds=60):
    tracker = LegacyTracker(points)
    x, y, yaw = start
    speed = 0.0
    trace = []
    for frame in range(int(seconds / .05)):
        command = tracker.command(x=x, y=y, yaw_rad=yaw, speed_mps=speed,
                                  frame_id=frame, sim_time_sec=frame * .05)
        assert 0 <= command['throttle'] <= 1
        assert 0 <= command['brake'] <= 1
        assert -1 <= command['steer_left_normalized'] <= 1
        # Independent bicycle reference with midpoint position integration.
        acceleration = 2.5 * command['throttle'] - 4.0 * command['brake'] - .15 * speed
        next_speed = max(0.0, speed + acceleration * .05)
        mean_speed = (speed + next_speed) / 2
        turn = mean_speed * math.tan(command['steer_left_normalized'] * .6) * .05
        x += mean_speed * math.cos(yaw + turn / 2) * .05
        y += mean_speed * math.sin(yaw + turn / 2) * .05
        yaw += turn
        speed = next_speed
        trace.append((x, y, yaw, speed, command))
    return trace


def test_straight_route_reaches_goal_and_remains_stopped():
    trace = run_route([(float(i), 0.0) for i in range(11)])
    x, y, _, speed, command = trace[-1]
    assert abs(x - 10.0) <= .6
    assert abs(y) < .05
    assert speed <= .05
    assert all(point[3] <= .05 for point in trace[-100:])
    assert command['target_speed_mps'] == 0


@pytest.mark.parametrize('side', [-1, 1])
def test_smooth_left_and_right_curve_steering_and_geometric_error(side):
    points = [(i / 4, 0.0) for i in range(13)]
    points += [(3 + 6 * math.sin(i * math.pi / 80),
                side * 6 * (1 - math.cos(i * math.pi / 80))) for i in range(1, 41)]
    points += [(9.0, side * (6 + i / 4)) for i in range(1, 17)]
    trace = run_route(points, seconds=65)
    x, y, _, speed, _ = trace[-1]
    assert math.dist((x, y), points[-1]) <= .6
    assert speed <= .05
    assert any(side * item[4]['steer_left_normalized'] > .1 for item in trace)
    # A separate point-to-segment calculation audits physical tracking; do not
    # use the legacy controller's own CTE estimate as the oracle.
    errors = []
    for x, y, _, _, _ in trace:
        candidates = []
        for a, b in zip(points, points[1:]):
            dx, dy = b[0] - a[0], b[1] - a[1]
            fraction = max(0., min(1., ((x - a[0]) * dx + (y - a[1]) * dy) / (dx * dx + dy * dy)))
            candidates.append(math.hypot(x - a[0] - fraction * dx, y - a[1] - fraction * dy))
        errors.append(min(candidates))
    assert max(errors) <= .75
    assert math.sqrt(sum(value * value for value in errors) / len(errors)) <= .35


@pytest.mark.parametrize('route', ['direct', 'upper', 'lower'])
def test_actual_mvp_aisles_reach_goal_using_public_plant(route):
    from embodiedguard.planning import aisle_path
    from embodiedguard.sim import AckermannPlant, Action, PlantConfig, PoseState
    config = PlantConfig()
    points = aisle_path(PoseState(), (12., 0.), route, config)
    tracker = LegacyTracker(points)
    plant = AckermannPlant(config)
    max_cte = 0.
    for frame in range(1200):
        state = plant.state
        command = tracker.command(x=state.x, y=state.y, yaw_rad=state.yaw_rad,
            speed_mps=state.speed_mps, frame_id=frame, sim_time_sec=frame * .05)
        max_cte = max(max_cte, command['cte_m'])
        plant.step(Action(**{key: command[key] for key in
            ('throttle', 'brake', 'steer_left_normalized')}), .05)
    assert math.dist((plant.state.x, plant.state.y), (12., 0.)) <= .6
    assert plant.state.speed_mps <= .05
    assert max_cte <= .75


def test_invalid_observation_is_rejected_and_reset_starts_new_timeline():
    tracker = LegacyTracker([(0., 0.), (5., 0.)])
    packet = dict(x=0., y=0., yaw_rad=0., speed_mps=0., frame_id=0, sim_time_sec=0.)
    initial = tracker.command(**packet)
    with pytest.raises(ValueError, match='strictly increasing'):
        tracker.command(**packet)
    for field, invalid in [('x', float('nan')), ('y', True), ('speed_mps', -1.),
                           ('yaw_rad', float('inf')), ('dt_sec', 0.), ('frame_id', True)]:
        bad = {**packet, 'frame_id': 1, 'sim_time_sec': .05, field: invalid}
        with pytest.raises(ValueError):
            tracker.command(**bad)
    # Rejected packets cannot consume the next valid frame.
    tracker.command(**{**packet, 'frame_id': 1, 'sim_time_sec': .05})
    tracker.reset()
    assert tracker.command(**packet) == initial


def test_controller_reset_after_bounded_stop_preserves_route_progress():
    from embodiedguard.sim import AckermannPlant, Action, STOP
    tracker = LegacyTracker([(i / 4, 0.) for i in range(81)])
    plant = AckermannPlant()
    frame = 0
    while plant.state.x < 11.:
        state = plant.state
        command = tracker.command(x=state.x, y=state.y, yaw_rad=state.yaw_rad,
            speed_mps=state.speed_mps, frame_id=frame, sim_time_sec=frame * .05)
        plant.step(Action(**{k: command[k] for k in
            ('throttle', 'brake', 'steer_left_normalized')}), .05)
        frame += 1
        assert frame < 1000
    while plant.state.speed_mps > 0:
        plant.step(STOP, .05)
        frame += 1
    tracker.reset_controller()
    for _ in range(800):
        state = plant.state
        command = tracker.command(x=state.x, y=state.y, yaw_rad=state.yaw_rad,
            speed_mps=state.speed_mps, frame_id=frame, sim_time_sec=frame * .05)
        plant.step(Action(**{k: command[k] for k in
            ('throttle', 'brake', 'steer_left_normalized')}), .05)
        frame += 1
    assert math.dist((plant.state.x, plant.state.y), (20., 0.)) <= .6
    assert plant.state.speed_mps <= .05
