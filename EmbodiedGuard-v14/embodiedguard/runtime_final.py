"""Final integration fixes around the immutable v14 navigation core.

No planning, control, geometry or safety thresholds change in this module.
"""
from copy import deepcopy
import uuid
from .public_maps import PublicMapMission
from .state import ObservationStore

class PublicMapMissionFinal(PublicMapMission):
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        # Fault names in episode IDs otherwise reveal the injected diagnosis
        # before any failure. Assign an opaque identity before the first frame.
        self._world.episode_id = 'public-'+uuid.uuid4().hex
        self._store = ObservationStore(self._world.episode_id)

    def recovery_context(self):
        context=super().recovery_context()
        target=context['next_target']
        known={row['plan_id'] for row in context['assessed_routes']}
        for plan in self._plans:
            if (plan['target']==target and plan['reason'] in ('GLOBAL_SEARCH_FAILED','GLOBAL_PATH_AUDIT_REJECTED')
                    and plan['plan_id'] not in known):
                context['assessed_routes'].append({key:deepcopy(plan[key]) for key in
                    ('route','plan_id','status','reason','observation_frame','sim_time_sec')})
        context['assessed_routes'].sort(key=lambda item:int(item['plan_id'].split('-')[1]))
        tried={row['route'] for row in context['assessed_routes']}
        context['untried_routes']=[route for route in self._permitted_routes if target is not None and route not in tried]
        return context
