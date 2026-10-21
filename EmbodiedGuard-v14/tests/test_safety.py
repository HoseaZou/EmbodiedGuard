from embodiedguard.safety import assess_action, swept_clearance
from embodiedguard.sim import Action, PlantConfig, PoseState


def observation(objects=(), **updates):
    return dict(observed_at=0., obstacles_valid=True, obstacles=list(objects), **updates)


def test_guard_accounts_for_braking_distance_not_only_the_next_step():
    state = PoseState(speed_mps=1.2)
    obj = dict(id='wall', x=.9, y=0., radius=.2, vx=0., vy=0.)
    verdict = assess_action(state, Action(throttle=.5), observation([obj]), now=0.,
                            config=PlantConfig(), bounds=(-2,-6,40,6))
    assert not verdict.allowed
    assert verdict.reason == 'PATH_BLOCKED'


def test_relative_sweep_catches_obstacle_crossing_between_endpoints():
    # Obstacle traverses the stopped robot in one interval; endpoints are both clear.
    obj = dict(id='fast', x=0., y=-1., radius=.1, vx=0., vy=40.)
    clearance = swept_clearance(PoseState(), PoseState(), [obj], 0., .05, .45)
    assert clearance < 0


def test_guard_rejects_unknown_state_speed_and_boundary():
    cfg = PlantConfig()
    unknown = {'observed_at':0.,'obstacles_valid':False,'obstacles':[]}
    assert assess_action(PoseState(),Action(),unknown,now=0.,config=cfg,bounds=(-2,-6,40,6)).reason == 'UNKNOWN_OBSERVATION'
    assert assess_action(PoseState(speed_mps=1.5),Action(throttle=1.),observation(),now=0.,config=cfg,bounds=(-2,-6,40,6)).reason == 'SPEED_LIMIT'
    assert assess_action(PoseState(x=39.8),Action(),observation(),now=0.,config=cfg,bounds=(-2,-6,40,6)).reason == 'BOUNDARY_RISK'


def test_braking_endpoint_must_not_be_in_future_crossing_traffic_path():
    # Safe at this instant, but stopping here would let the cart hit the robot later.
    obj = dict(id='cart', x=18., y=-1., radius=.65, vx=0., vy=.4)
    verdict = assess_action(PoseState(x=16.9), Action(brake=1.), observation([obj]), now=0.,
                            config=PlantConfig(), bounds=(-2,-6,40,6))
    assert not verdict.allowed and verdict.reason == 'PATH_BLOCKED'
