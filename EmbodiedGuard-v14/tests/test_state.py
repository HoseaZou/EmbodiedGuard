import copy
import math
import unittest

from embodiedguard.state import ObservationError, ObservationStore


def packet(frame=0, stamp=0.0, **changes):
    value = {
        "episode_id": "inspection-1", "frame_id": frame, "observed_at": stamp,
        "pose": {"x": 0.0, "y": 0.0, "yaw_rad": 0.0}, "speed_mps": 0.0,
        "obstacles_valid": True, "obstacles": [], "source": "simulated_sensor",
    }
    value.update(changes)
    return value


class ObservationStoreTests(unittest.TestCase):
    def assert_error(self, code, callback):
        with self.assertRaises(ObservationError) as caught:
            callback()
        self.assertEqual(caught.exception.code, code)

    def test_current_observation_preserves_sensor_time_and_owns_its_data(self):
        store = ObservationStore("inspection-1")
        incoming = packet(1, 0.1, obstacles=[
            {"id": "crate", "x": 2.0, "y": 0.0, "radius": 0.4, "vx": 0.0, "vy": 0.0}
        ])
        accepted = store.ingest(incoming, received_at=0.2)
        incoming["pose"]["x"] = 800
        accepted["obstacles"].clear()
        state = store.current(now=0.25)
        self.assertEqual(state["observed_at"], 0.1)
        self.assertAlmostEqual(state["age_sec"], 0.15)
        self.assertEqual(state["pose"]["x"], 0.0)
        self.assertEqual(state["obstacles"][0]["id"], "crate")
        state["pose"]["x"] = 900
        self.assertEqual(store.current(now=0.25)["pose"]["x"], 0.0)

    def test_invalid_sensor_values_never_replace_the_last_accepted_observation(self):
        variants = []
        for value in (True, "0", float("nan"), float("inf"), -float("inf")):
            for field in ("observed_at", "speed_mps"):
                variants.append(packet(1, 0.1, **{field: value}))
            for field in ("x", "y", "yaw_rad"):
                candidate = packet(1, 0.1)
                candidate["pose"][field] = value
                variants.append(candidate)
            for field in ("x", "y", "radius", "vx", "vy"):
                obstacle = {"id": "obstacle", "x": 0, "y": 0, "radius": 1, "vx": 0, "vy": 0}
                obstacle[field] = value
                variants.append(packet(1, 0.1, obstacles=[obstacle]))
        for field in ("pose", "obstacles", "obstacles_valid", "source", "frame_id"):
            candidate = packet(1, 0.1)
            candidate.pop(field)
            variants.append(candidate)
        variants.extend([
            packet(True, 0.1), packet(-1, 0.1), packet(1, -0.1),
            packet(1, 0.1, speed_mps=-1), packet(1, 0.1, obstacles=None),
            packet(1, 0.1, obstacles_valid="true"), packet(1, 0.1, source="guessed"),
            packet(1, 0.1, obstacles=[{"id": "x", "x": 0, "y": 0, "radius": 0, "vx": 0, "vy": 0}]),
        ])
        for incoming in variants:
            with self.subTest(incoming=incoming):
                store = ObservationStore("inspection-1")
                store.ingest(packet(), received_at=0)
                self.assert_error("INVALID_OBSERVATION", lambda: store.ingest(incoming, received_at=0.1))
                self.assertEqual(store.current(now=0.1)["frame_id"], 0)

    def test_current_requires_known_fresh_causal_state_and_monotonic_clock(self):
        store = ObservationStore("inspection-1")
        self.assert_error("UNKNOWN_OBSERVATION", lambda: store.current(now=0))
        self.assert_error("FUTURE_OBSERVATION", lambda: store.ingest(packet(1, 0.2), received_at=0.1))
        store.ingest(packet(1, 0.1, obstacles_valid=False), received_at=0.1)
        self.assert_error("UNKNOWN_OBSERVATION", lambda: store.current(now=0.1))
        store.ingest(packet(2, 0.2), received_at=0.2)
        self.assert_error("OUT_OF_ORDER", lambda: store.current(now=0.19))
        self.assertEqual(store.current(now=0.5)["frame_id"], 2)
        self.assert_error("STALE_OBSERVATION", lambda: store.current(now=0.50001))
        self.assert_error("OUT_OF_ORDER", lambda: store.ingest(packet(3, 0.4), received_at=0.4))
        for value in (True, "1", float("nan"), float("inf"), -1):
            with self.subTest(clock=value):
                self.assert_error("INVALID_OBSERVATION", lambda: store.current(now=value))
                self.assert_error("INVALID_OBSERVATION", lambda: store.ingest(packet(4, 0.6), received_at=value))

    def test_episode_reset_is_explicit_and_duplicate_frames_cannot_refresh_sensor_age(self):
        store = ObservationStore("inspection-1")
        incoming = packet(10, 1.0)
        first = store.ingest(incoming, received_at=1.0)
        self.assertEqual(store.ingest(incoming, received_at=1.2), first)
        self.assert_error("STALE_OBSERVATION", lambda: store.current(now=1.31))
        changed = copy.deepcopy(incoming)
        changed["pose"]["x"] = 1.0
        self.assert_error("DUPLICATE_CONFLICT", lambda: store.ingest(changed, received_at=1.4))
        self.assert_error("OUT_OF_ORDER", lambda: store.ingest(packet(9, 0.9), received_at=1.4))
        self.assert_error("OUT_OF_ORDER", lambda: store.ingest(packet(11, 1.0), received_at=1.4))
        self.assert_error("EPISODE_MISMATCH", lambda: store.ingest(packet(11, 1.4, episode_id="inspection-2"), received_at=1.4))
        store.reset(episode_id="inspection-2")
        self.assert_error("UNKNOWN_OBSERVATION", lambda: store.current(now=0))
        state = store.ingest(packet(0, 0, episode_id="inspection-2"), received_at=0)
        self.assertEqual(state["frame_id"], 0)
        self.assertEqual(store.current(now=0)["episode_id"], "inspection-2")

    def test_history_prefers_actual_samples_and_interpolates_only_observed_short_gaps(self):
        store = ObservationStore("inspection-1")
        store.ingest(packet(0, 0, pose={"x": 0, "y": 0, "yaw_rad": math.radians(179)}, speed_mps=1), received_at=0)
        store.ingest(packet(1, 0.1, pose={"x": 2, "y": 4, "yaw_rad": math.radians(-179)}, speed_mps=3), received_at=0.1)
        samples = store.history(ages=(0.1, 0.05, 0), now=0.1)
        self.assertEqual(samples[0]["sample_kind"], "observed")
        self.assertEqual(samples[0]["source_frame_ids"], [0])
        self.assertEqual(samples[1]["sample_kind"], "interpolated")
        self.assertEqual(samples[1]["source_frame_ids"], [0, 1])
        self.assertEqual(samples[1]["pose"]["x"], 1.0)
        self.assertEqual(samples[1]["pose"]["y"], 2.0)
        self.assertAlmostEqual(abs(samples[1]["pose"]["yaw_rad"]), math.pi)
        self.assertEqual(samples[1]["speed_mps"], 2.0)
        self.assertNotIn("obstacles", samples[1])
        samples[0]["pose"]["x"] = 100
        self.assertEqual(store.history(ages=(0.1,), now=0.1)[0]["pose"]["x"], 0)
        self.assert_error("MISSING_HISTORY", lambda: store.history(ages=(0,), now=0.15))
        self.assert_error("MISSING_HISTORY", lambda: store.history(ages=(1,), now=0.15))
        store.ingest(packet(2, 0.3), received_at=0.3)
        self.assert_error("MISSING_HISTORY", lambda: store.history(ages=(0.1,), now=0.3))
        self.assertEqual(store.current(now=0.3)["frame_id"], 2)
        store.reset(episode_id="inspection-2")
        store.ingest(packet(0, 0, episode_id="inspection-2"), received_at=0)
        self.assert_error("MISSING_HISTORY", lambda: store.history(ages=(0.1,), now=0))

    def test_time_thresholds_do_not_lose_exact_samples_to_floating_point_rounding(self):
        store = ObservationStore("inspection-1")
        store.ingest(packet(0, 0.1), received_at=0.1)
        self.assertAlmostEqual(store.current(now=0.4)["age_sec"], 0.3)
        store.reset(episode_id="inspection-2")
        store.ingest(packet(0, 0, episode_id="inspection-2"), received_at=0)
        store.ingest(packet(1, 1e-10, episode_id="inspection-2", pose={"x": 1, "y": 0, "yaw_rad": 0}), received_at=1e-10)
        sample = store.history(ages=(0,), now=1e-10)[0]
        self.assertEqual(sample["source_frame_ids"], [1])
        self.assertEqual(sample["pose"]["x"], 1)

    def test_history_gap_limit_and_invalid_configuration_are_explicit(self):
        for gap, allowed in ((0.11, True), (0.11001, False)):
            with self.subTest(gap=gap):
                store = ObservationStore("inspection-1")
                store.ingest(packet(), received_at=0)
                store.ingest(packet(1, gap, pose={"x": 2, "y": 0, "yaw_rad": 0}), received_at=gap)
                if allowed:
                    self.assertEqual(store.history(ages=(gap / 2,), now=gap)[0]["pose"]["x"], 1)
                else:
                    self.assert_error("MISSING_HISTORY", lambda: store.history(ages=(gap / 2,), now=gap))
        for value in (True, "0.3", 0, -0.1, float("nan"), float("inf")):
            with self.subTest(ttl=value):
                self.assert_error("INVALID_OBSERVATION", lambda: ObservationStore("inspection-1", value))
        store = ObservationStore("inspection-1")
        store.ingest(packet(), received_at=0)
        for ages in ([0], (True,), ("0",), (-1,), (float("nan"),)):
            with self.subTest(ages=ages):
                self.assert_error("INVALID_OBSERVATION", lambda: store.history(ages=ages, now=0))
        self.assert_error("EPISODE_MISMATCH", lambda: store.reset(episode_id="inspection-1"))
        self.assertEqual(store.current(now=0)["frame_id"], 0)


if __name__ == "__main__":
    unittest.main()
