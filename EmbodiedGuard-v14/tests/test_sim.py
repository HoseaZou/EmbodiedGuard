"""Public plant behavior with known geometry; no vehicle dynamics claim."""
import math

import pytest

from embodiedguard.sim import Action, AckermannPlant, PlantConfig


def test_straight_motion_and_full_brake_reach_rest_without_reversing():
    plant = AckermannPlant()
    for _ in range(60):
        plant.step(Action(throttle=0.4), 0.05)
    assert plant.state.x > 2.0
    assert plant.state.y == 0.0
    for _ in range(40):
        plant.step(Action(brake=1.0), 0.05)
    assert plant.state.speed_mps == 0.0
    assert plant.state.x > 2.0


def test_left_and_right_turns_are_mirror_images():
    left, right = AckermannPlant(), AckermannPlant()
    for _ in range(60):
        left.step(Action(throttle=0.3, steer_left_normalized=0.3), 0.05)
        right.step(Action(throttle=0.3, steer_left_normalized=-0.3), 0.05)
    assert left.state.y > 0.15
    assert right.state.y < -0.15
    assert left.state.x == pytest.approx(right.state.x)
    assert left.state.y == pytest.approx(-right.state.y)
    assert left.state.yaw_rad == pytest.approx(-right.state.yaw_rad)


@pytest.mark.parametrize('action', [dict(throttle=True), dict(brake=float('nan')),
                                    dict(steer_left_normalized=1.1), dict(throttle=.2, brake=.2)])
def test_invalid_actuator_requests_cannot_enter_the_plant(action):
    with pytest.raises(ValueError):
        AckermannPlant().step(Action(**action), 0.05)


def test_stopped_vehicle_cannot_rotate_in_place():
    plant = AckermannPlant()
    for _ in range(30):
        plant.step(Action(steer_left_normalized=1.0), .05)
    assert (plant.state.x, plant.state.y, plant.state.yaw_rad) == (0., 0., 0.)
