"""Strict, episode-scoped observations for the deterministic execution layer.

History sampling adapts RAVAL's exact/short causal interpolation policy; it does
not fabricate missing poses or convert unknown obstacles into free space.
All coordinates use metres, radians and a world frame with y left / CCW yaw.
"""

from __future__ import annotations

from copy import deepcopy
from collections import deque
import math

MAX_INTERPOLATION_GAP_SEC = 0.11
TIME_TOLERANCE_SEC = 1e-9


class ObservationError(ValueError):
    """A caller-visible observation failure with a stable machine-readable code."""

    def __init__(self, code: str, message: str):
        self.code = code
        super().__init__(message)


def _number(value, field: str, *, minimum=None, positive=False) -> float:
    if type(value) not in (int, float):
        raise ObservationError("INVALID_OBSERVATION", f"{field} must be a number, excluding bool")
    try:
        result = float(value)
    except OverflowError as exc:
        raise ObservationError("INVALID_OBSERVATION", f"{field} is too large") from exc
    if not math.isfinite(result) or (minimum is not None and result < minimum) or (positive and result <= 0):
        raise ObservationError("INVALID_OBSERVATION", f"{field} is outside its finite range")
    return result


def _identifier(value, field: str) -> str:
    if type(value) is not str or not value.strip():
        raise ObservationError("INVALID_OBSERVATION", f"{field} must be a nonempty string")
    return value


def _packet(packet: dict) -> dict:
    if not isinstance(packet, dict):
        raise ObservationError("INVALID_OBSERVATION", "packet must be an object")
    try:
        episode = _identifier(packet["episode_id"], "episode_id")
        frame = packet["frame_id"]
        if type(frame) is not int or frame < 0:
            raise ObservationError("INVALID_OBSERVATION", "frame_id must be a nonnegative integer")
        stamp = _number(packet["observed_at"], "observed_at", minimum=0)
        pose_input = packet["pose"]
        if not isinstance(pose_input, dict):
            raise ObservationError("INVALID_OBSERVATION", "pose must be an object")
        pose = {key: _number(pose_input[key], f"pose.{key}") for key in ("x", "y", "yaw_rad")}
        speed = _number(packet["speed_mps"], "speed_mps", minimum=0)
        valid = packet["obstacles_valid"]
        if type(valid) is not bool or type(packet["obstacles"]) is not list:
            raise ObservationError("INVALID_OBSERVATION", "obstacles_valid must be bool and obstacles must be a list")
        obstacles = []
        ids = set()
        for item in packet["obstacles"]:
            if not isinstance(item, dict):
                raise ObservationError("INVALID_OBSERVATION", "each obstacle must be an object")
            oid = _identifier(item["id"], "obstacle.id")
            if oid in ids:
                raise ObservationError("INVALID_OBSERVATION", "obstacle IDs must be unique")
            ids.add(oid)
            obstacles.append({
                "id": oid,
                **{key: _number(item[key], f"obstacle.{key}") for key in ("x", "y", "vx", "vy")},
                "radius": _number(item["radius"], "obstacle.radius", positive=True),
            })
        if packet["source"] != "simulated_sensor":
            raise ObservationError("INVALID_OBSERVATION", "this MVP accepts simulated_sensor observations only")
    except KeyError as exc:
        raise ObservationError("INVALID_OBSERVATION", f"missing required field {exc.args[0]}") from exc
    return {
        "episode_id": episode, "frame_id": frame, "observed_at": stamp,
        "pose": pose, "speed_mps": speed, "obstacles_valid": valid,
        "obstacles": obstacles, "source": "simulated_sensor",
    }


class ObservationStore:
    """Own sensor packets; sensor age always uses observed_at, never arrival time."""

    def __init__(self, episode_id: str, max_age_sec: float = 0.3):
        self.episode_id = _identifier(episode_id, "episode_id")
        self.max_age_sec = _number(max_age_sec, "max_age_sec", positive=True)
        self._records = deque(maxlen=256)
        self._clock = 0.0

    def reset(self, *, episode_id: str) -> None:
        """Explicitly start a different episode; automatic timeline resets are forbidden."""
        new_episode = _identifier(episode_id, "episode_id")
        if new_episode == self.episode_id:
            raise ObservationError("EPISODE_MISMATCH", "reset requires a new episode identifier")
        self.episode_id = new_episode
        self._records.clear()
        self._clock = 0.0

    def ingest(self, packet: dict, *, received_at: float) -> dict:
        stored = _packet(packet)
        stored["received_at"] = _number(received_at, "received_at", minimum=0)
        if stored["observed_at"] > stored["received_at"]:
            raise ObservationError("FUTURE_OBSERVATION", "observation is later than its arrival time")
        if stored["received_at"] < self._clock:
            raise ObservationError("OUT_OF_ORDER", "arrival time precedes the last accepted clock")
        if stored["episode_id"] != self.episode_id:
            raise ObservationError("EPISODE_MISMATCH", "observation belongs to a different episode")
        if self._records:
            previous = self._records[-1]
            if stored["frame_id"] == previous["frame_id"]:
                same = {k: v for k, v in stored.items() if k != "received_at"} == {
                    k: v for k, v in previous.items() if k != "received_at"
                }
                if not same:
                    raise ObservationError("DUPLICATE_CONFLICT", "duplicate frame changed its contents")
                self._clock = stored["received_at"]
                return deepcopy(previous)
            if stored["frame_id"] < previous["frame_id"] or stored["observed_at"] <= previous["observed_at"]:
                raise ObservationError("OUT_OF_ORDER", "frame and observation time must advance together")
        self._records.append(stored)
        self._clock = stored["received_at"]
        return deepcopy(stored)

    def current(self, *, now: float) -> dict:
        now = _number(now, "now", minimum=0)
        if now < self._clock:
            raise ObservationError("OUT_OF_ORDER", "clock may not move backwards inside an episode")
        self._clock = now
        if not self._records:
            raise ObservationError("UNKNOWN_OBSERVATION", "no observation received")
        result = deepcopy(self._records[-1])
        result["age_sec"] = now - result["observed_at"]
        if not result["obstacles_valid"]:
            raise ObservationError("UNKNOWN_OBSERVATION", "obstacle sensing is unknown, even if its list is empty")
        if result["age_sec"] > self.max_age_sec + TIME_TOLERANCE_SEC:
            raise ObservationError("STALE_OBSERVATION", "latest observation exceeds its allowed sensor age")
        return result

    def history(self, *, ages: tuple, now: float) -> list:
        """Sample motion state at ``now - age`` without extrapolation.

        Only observations already received in this episode may bracket a
        target. Gaps over 0.11 seconds remain missing. Yaw interpolation takes
        the shortest angular path. Obstacles are deliberately not interpolated.
        The returned samples identify both their kind and supporting frames.
        """
        if type(ages) is not tuple:
            raise ObservationError("INVALID_OBSERVATION", "ages must be a tuple")
        validated_ages = [_number(age, "history age", minimum=0) for age in ages]
        current = self.current(now=now)
        now = float(now)
        records = list(self._records)
        samples = []
        for age in validated_ages:
            target = now - age
            if target < 0:
                raise ObservationError("MISSING_HISTORY", f"no recorded state for age {age}")
            exact = next((r for r in records if r["observed_at"] == target), None)
            if exact is None:
                nearby = [r for r in records if abs(r["observed_at"] - target) <= TIME_TOLERANCE_SEC]
                if len(nearby) == 1:
                    exact = nearby[0]
            if exact is not None:
                samples.append({
                    "episode_id": self.episode_id, "observed_at": exact["observed_at"],
                    "pose": deepcopy(exact["pose"]), "speed_mps": exact["speed_mps"],
                    "sample_kind": "observed", "source_frame_ids": [exact["frame_id"]],
                })
                continue
            before = [r for r in records if r["observed_at"] < target]
            after = [r for r in records if target < r["observed_at"] <= current["observed_at"]]
            if not before or not after:
                raise ObservationError("MISSING_HISTORY", f"no observed bracket for age {age}")
            left, right = before[-1], after[0]
            gap = right["observed_at"] - left["observed_at"]
            if gap > MAX_INTERPOLATION_GAP_SEC + TIME_TOLERANCE_SEC:
                raise ObservationError("MISSING_HISTORY", f"observed gap is too large for age {age}")
            weight = (target - left["observed_at"]) / gap
            left_yaw = math.atan2(math.sin(left["pose"]["yaw_rad"]), math.cos(left["pose"]["yaw_rad"]))
            right_yaw = math.atan2(math.sin(right["pose"]["yaw_rad"]), math.cos(right["pose"]["yaw_rad"]))
            yaw_delta = (right_yaw - left_yaw + math.pi) % math.tau - math.pi
            yaw = left_yaw + weight * yaw_delta
            pose = {
                key: (1 - weight) * left["pose"][key] + weight * right["pose"][key]
                for key in ("x", "y")
            }
            pose["yaw_rad"] = math.atan2(math.sin(yaw), math.cos(yaw))
            samples.append({
                "episode_id": self.episode_id, "observed_at": target, "pose": pose,
                "speed_mps": (1 - weight) * left["speed_mps"] + weight * right["speed_mps"],
                "sample_kind": "interpolated", "source_frame_ids": [left["frame_id"], right["frame_id"]],
            })
        return samples
