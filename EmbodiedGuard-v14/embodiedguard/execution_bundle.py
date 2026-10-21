"""Optional execution/inspection transaction, without route or recovery policy.

Only an explicit Agent request enables the second step. The existing executor
and inspector retain every freshness, pose, stopping and budget check.
"""
from copy import deepcopy
import json


class ExecutionBundle:
    def __init__(self, mission):
        self.mission = mission
        self._modes = {}
        self._receipts = {}

    def execute(self, arguments):
        args = dict(arguments)
        requested = args.pop('inspect_on_arrival', False)
        operation = args['operation_id']
        signature = json.dumps([args['route_id'], float(args['max_sim_seconds']),
            str(args.get('rationale', ''))], ensure_ascii=False)
        if operation in self._modes and self._modes[operation] != requested:
            return self.mission._failure('OPERATION_CONFLICT')
        if requested and operation in self._receipts:
            old_signature, receipt = self._receipts[operation]
            return deepcopy(receipt) if old_signature == signature else self.mission._failure('OPERATION_CONFLICT')

        result = self.mission.execute_route(**args)
        if result.get('operation_id') == operation:
            self._modes[operation] = requested
        if not requested:
            return result

        result = deepcopy(result)
        result['inspection'] = dict(requested=True, performed=False, reason='EXECUTION_NOT_ARRIVED')
        if result.get('ok') and result.get('status') == 'ARRIVED' and result.get('stopping_verified') is True:
            measured = self.mission.inspect(target=result['target'])
            result['inspection'] = dict(requested=True, performed=True, **deepcopy(measured))
            # Progress is observed after the real inspector, not inferred from arrival.
            for key in ('completed_targets', 'remaining_targets', 'next_target', 'completion_receipt', 'task'):
                if key in measured:
                    result[key] = deepcopy(measured[key])
            if not measured.get('ok'):
                result.update(ok=False, execution_status=result['status'], execution_reason=result['reason'],
                    status='INSPECTION_FAILED', reason=measured.get('reason', 'INSPECTION_FAILED'))
        if result.get('operation_id') == operation:
            self._receipts[operation] = (signature, deepcopy(result))
        return result
