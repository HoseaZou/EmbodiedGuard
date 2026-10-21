"""Motion-triggered diagnostic fixtures behind the unchanged executor interface."""
from __future__ import annotations

from copy import deepcopy
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path

from .shadow import ShadowMission
from .sim import InspectionWorld, Obstacle
from .state import ObservationStore


FIXTURES = ('motion_sensor_freeze','motion_blocked_aisle','motion_sensor_unavailable')


class MotionFaultWorld(InspectionWorld):
    """The fault trigger sees only actual movement, never model waiting time.

    Fault journals are evaluator-only. Controllers receive the same original
    measured packet and failures. No plan, route option, speed, steering, safety,
    perception freshness or task budget is relaxed for any controller.
    """
    def __init__(self, fixture, seed=0):
        if fixture not in FIXTURES:
            raise ValueError('Unknown motion fixture')
        super().__init__('normal',seed,f'motion-{seed}')
        self.fixture=fixture
        self.trigger_distance_m=6. if fixture=='motion_blocked_aisle' else 2.
        self.actual_distance_m=0.
        self.triggered=False
        self.recovered=False
        self._fault_snapshot=None
        self.fault_events=[]

    def _fault_event(self,event):
        self.fault_events.append(dict(event_id=f'fault-{len(self.fault_events)+1}',
            event=event,fixture=self.fixture,sim_time_sec=self.time_sec,frame_id=self.frame_id,
            actual_distance_m=self.actual_distance_m,trigger_distance_m=self.trigger_distance_m,
            evaluation_state=asdict(self.plant.state),source='simulator_fault_injector_evaluator_only'))

    def step(self,action,dt_sec=None):
        previous,current=super().step(action,dt_sec)
        displacement=math.dist((previous.x,previous.y),(current.x,current.y))
        self.actual_distance_m+=displacement
        if not self.triggered and self.actual_distance_m>=self.trigger_distance_m:
            self.triggered=True
            if self.fixture=='motion_blocked_aisle':
                self._obstacles=[Obstacle('blocked_aisle',18.,0.,radius=.65)]
            self._fault_event('fault_triggered_by_motion')
        return previous,current

    def packet(self):
        measured=super().packet()
        if not self.triggered or self.recovered or self.fixture=='motion_blocked_aisle':
            return measured
        if self.fixture=='motion_sensor_unavailable':
            measured['obstacles_valid']=False
            return measured
        if self._fault_snapshot is None:
            self._fault_snapshot=deepcopy(measured)
            self._fault_event('sensor_packet_frozen')
        return deepcopy(self._fault_snapshot)

    def refresh_sensor(self):
        self._fault_event('sensor_refresh_requested')
        if self.triggered and self.fixture=='motion_sensor_freeze':
            self.recovered=True
            self._fault_event('sensor_refresh_effective')
        # Permanent unavailability and physical corridor blockage do not heal
        # through this tool. Recovery cannot be claimed from an API receipt.

    def fault_evidence(self):
        return deepcopy(dict(schema_version=1,fixture=self.fixture,
            trigger_kind='cumulative_actual_plant_displacement',
            trigger_distance_m=self.trigger_distance_m,actual_distance_m=self.actual_distance_m,
            triggered=self.triggered,recovered=self.recovered,events=self.fault_events,
            clock_invariant='stationary model idle and explicit stopped wait do not advance the fault trigger',
            evaluator_only=True,physical_scope='synthetic_2d_kinematics_only'))


class MotionFaultMission(ShadowMission):
    """Own fixture configuration; all task/control/safety tools are inherited."""
    def __init__(self,fixture,*,seed=0,**kwargs):
        super().__init__('normal',seed=seed,**kwargs)
        self._world=MotionFaultWorld(fixture,seed)
        self._store=ObservationStore(self._world.episode_id)
        self._log('motion_fixture_configured',fixture=fixture,
            trigger_distance_m=self._world.trigger_distance_m,
            trigger_source='actual motion, not idle clock',evaluator_only=True)

    def evidence(self):
        evidence=super().evidence()
        evidence.update(scenario=self._world.fixture,base_scenario='normal',
                        fault_fixture=self._world.fault_evidence())
        evidence['evaluation_only_fields'].append('fault_fixture')
        return evidence

    def save(self,directory,*,tool_context=None):
        super().save(directory,tool_context=tool_context)
        path=Path(directory)
        fixture=self._world.fault_evidence()
        (path/'fault-events.jsonl').write_text(''.join(json.dumps(row,ensure_ascii=False,allow_nan=False)+'\n'
            for row in fixture['events']),encoding='utf-8')
        (path/'fault-fixture.json').write_text(json.dumps(fixture,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
        manifest=dict(base_run_sha256=hashlib.sha256((path/'run.json').read_bytes()).hexdigest(),
            evaluator_only=True,files={name:hashlib.sha256((path/name).read_bytes()).hexdigest()
            for name in ('fault-events.jsonl','fault-fixture.json')})
        (path/'fault-manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2),encoding='utf-8')
