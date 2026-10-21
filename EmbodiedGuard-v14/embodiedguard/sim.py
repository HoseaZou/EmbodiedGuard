"""Fixed-step Ackermann kinematics with explicit, uncalibrated actuator mapping."""
from __future__ import annotations

from dataclasses import asdict, dataclass, replace
import math


def finite_number(value: object, name: str) -> float:
    if type(value) not in (int, float):
        raise ValueError(f'{name} must be a finite number, not bool/string')
    try:
        result = float(value)
    except OverflowError as exc:
        raise ValueError(f'{name} exceeds the finite numeric range') from exc
    if not math.isfinite(result):
        raise ValueError(f'{name} must be finite')
    return result


@dataclass(frozen=True)
class PlantConfig:
    wheelbase_m: float = 1.0
    max_steer_rad: float = .6
    acceleration_mps2: float = 2.5
    braking_mps2: float = 4.0
    drag_per_sec: float = .15
    max_speed_mps: float = 1.5
    radius_m: float = .45  # Conservative circular footprint, centered on tracked pose.
    safety_margin_m: float = .12
    dt_sec: float = .05

    def __post_init__(self):
        for key, value in asdict(self).items():
            if finite_number(value, key) <= 0:
                raise ValueError(f'{key} must be positive')
        if self.max_steer_rad >= math.pi / 2 or self.dt_sec > .1:
            raise ValueError('invalid steering or integration step')


@dataclass(frozen=True)
class PoseState:
    x: float = 0.
    y: float = 0.
    yaw_rad: float = 0.
    speed_mps: float = 0.


@dataclass(frozen=True)
class Action:
    throttle: float = 0.
    brake: float = 0.
    steer_left_normalized: float = 0.

    def __post_init__(self):
        values = {k: finite_number(v, k) for k, v in asdict(self).items()}
        if not 0 <= values['throttle'] <= 1 or not 0 <= values['brake'] <= 1:
            raise ValueError('throttle/brake must be in [0,1]')
        if not -1 <= values['steer_left_normalized'] <= 1:
            raise ValueError('steer must be in [-1,1]')
        if self.throttle > 0 and self.brake > 0:
            raise ValueError('simultaneous throttle and brake rejected')


STOP = Action(brake=1.)


def integrate(state: PoseState, action: Action, dt: float, config: PlantConfig) -> PoseState:
    dt = finite_number(dt, 'dt')
    if not 0 < dt <= config.dt_sec + 1e-9:
        raise ValueError('integration step out of bounds')
    accel = (config.acceleration_mps2 * action.throttle - config.braking_mps2 * action.brake
             - config.drag_per_sec * state.speed_mps)
    speed = max(0., state.speed_mps + accel * dt)
    # Integrate only until rest if braking would cross zero; no phantom reverse motion.
    moving_dt = min(dt, state.speed_mps / -accel) if accel < 0 and speed == 0 else dt
    mean_speed = (state.speed_mps + speed) * .5
    yaw_delta = mean_speed / config.wheelbase_m * math.tan(
        action.steer_left_normalized * config.max_steer_rad) * moving_dt
    yaw_mid = state.yaw_rad + yaw_delta * .5
    return PoseState(state.x + mean_speed * math.cos(yaw_mid) * moving_dt,
                     state.y + mean_speed * math.sin(yaw_mid) * moving_dt,
                     math.atan2(math.sin(state.yaw_rad + yaw_delta), math.cos(state.yaw_rad + yaw_delta)),
                     speed)


class AckermannPlant:
    def __init__(self, config: PlantConfig | None = None, state: PoseState | None = None):
        self.config = config or PlantConfig()
        self._state = state or PoseState()
        for key, value in asdict(self._state).items():
            finite_number(value, key)
        if self._state.speed_mps < 0:
            raise ValueError('reverse speed not supported')

    @property
    def state(self) -> PoseState:
        return self._state

    def step(self, action: Action, dt_sec: float) -> PoseState:
        self._state = integrate(self._state, action, dt_sec, self.config)
        return self._state


@dataclass(frozen=True)
class Obstacle:
    id: str
    x: float
    y: float
    radius: float = .65
    vx: float = 0.
    vy: float = 0.

    def at(self, time_sec: float) -> dict:
        return dict(id=self.id, x=self.x + self.vx * time_sec,
                    y=self.y + self.vy * time_sec, radius=self.radius, vx=self.vx, vy=self.vy)


SCENARIOS = ('normal', 'blocked', 'stale', 'unreachable', 'dynamic', 'execution_fault')


class InspectionWorld:
    """Ground truth stays in the executor; packets emulate an ideal range sensor.

    No perception accuracy claim: circle observations are synthetic, with explicit
    freeze faults. Independent truth collision counting is never an Agent tool.
    """
    bounds = (-2., -6., 40., 6.)

    def __init__(self, scenario: str = 'normal', seed: int = 0, episode_id: str = 'episode-0'):
        if scenario not in SCENARIOS:
            raise ValueError('unknown scenario')
        if type(seed) is not int or seed < 0:
            raise ValueError('seed must be a nonnegative integer')
        self.scenario, self.seed, self.episode_id = scenario, seed, episode_id
        self.checkpoints = {'panel_a': (12., 0.), 'pump_b': (24., 0.), 'exit_c': (36., 0.)}
        self.plant = AckermannPlant()
        self.time_sec = 0.
        self.frame_id = 0
        self._frozen_packet = None
        self._refresh_done = False
        self._obstacles = []
        if scenario == 'blocked':
            self._obstacles = [Obstacle('blocked_aisle', 18., 0.)]
        elif scenario == 'unreachable':
            self._obstacles = [Obstacle(f'closed_{i}', 18., y, radius=.85)
                               for i, y in enumerate((-4., -2., 0., 2., 4.))]
        elif scenario == 'dynamic':
            # Clock-driven crossing; seed changes timing, never Agent choice.
            self._obstacles = [Obstacle('crossing_cart', 18., -10. - (seed % 3) * .3, vy=.4)]

    def obstacles_at(self, time_sec: float | None = None) -> list[dict]:
        return [obj.at(self.time_sec if time_sec is None else time_sec) for obj in self._obstacles]

    def packet(self) -> dict:
        from copy import deepcopy
        pose = self.plant.state
        packet = dict(episode_id=self.episode_id, frame_id=self.frame_id,
                      observed_at=self.time_sec, pose=dict(x=pose.x, y=pose.y, yaw_rad=pose.yaw_rad),
                      speed_mps=pose.speed_mps, obstacles_valid=True,
                      obstacles=self.obstacles_at(), source='simulated_sensor')
        if self.scenario == 'stale' and self.time_sec >= 4. and not self._refresh_done:
            if self._frozen_packet is None:
                self._frozen_packet = packet
            return deepcopy(self._frozen_packet)
        return packet

    def refresh_sensor(self) -> None:
        self._refresh_done = True  # Simulated recovery of one injected sensor freeze.

    def step(self, action: Action, dt_sec: float | None = None) -> tuple[PoseState, PoseState]:
        previous = self.plant.state
        dt = self.plant.config.dt_sec if dt_sec is None else finite_number(dt_sec, 'dt_sec')
        self.plant.step(action, dt)
        self.time_sec += dt
        self.frame_id += 1
        return previous, self.plant.state
