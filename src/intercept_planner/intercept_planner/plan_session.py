"""Node-lifetime plan identifiers and active-plan updates (SPEC-400 §4.5)."""
from dataclasses import replace
import math

from .intercept import Plan, PlanRequest, PlannerParams, Result, Status, _compute_plan, compute_plan
from .prediction import TrackState


class PlanSession:
    """Own one instance per node lifetime; methods return publishable responses.

    ``compute`` replaces the active plan and consumes an ID on every call.
    ``update`` returns None after closure or for an unrelated track. The adapter
    can also call it on a timer, passing None, to expire a plan without new data.
    """

    def __init__(self, p: PlannerParams):
        self.params = p
        self.active = None
        self._request = None
        self._next_id = 1

    def compute(self, track: TrackState | None, now: float, req: PlanRequest) -> Plan:
        plan_id = self._next_id
        self._next_id += 1
        reply = compute_plan(track, now, req, self.params)
        self.active = None
        self._request = req
        if reply.plan is not None:
            payload = replace(reply.plan, plan_id=plan_id, revision=1)
            reply = replace(reply, plan=payload)
            if payload.status != Status.EXPIRED:
                self.active = payload
        return reply

    def expire(self, reason: str) -> Plan | None:
        """Close once for adapter events such as a ROS clock reset."""
        previous = self.active
        if previous is None:
            return None
        self.active = None
        return Plan(Result.FAILED, reason, replace(previous, status=Status.EXPIRED,
                    reason=reason, revision=previous.revision + 1))

    def update(self, track: TrackState | None, now: float, *,
               current_epoch: int | None = None) -> Plan | None:
        previous = self.active
        if previous is None:
            return None
        epoch = (current_epoch if current_epoch is not None else
                 track.epoch if track is not None else self.params.current_epoch)
        reason = ''
        if now > previous.valid_until:
            reason = 'PLAN_EXPIRED'
        elif epoch != previous.target_epoch:
            reason = 'EPOCH_MISMATCH'
        elif track is not None and track.track_id == previous.target_track_id and (
                track.status == TrackState.LOST or now > track.valid_until):
            reason = 'TRACK_LOST'
        if reason:
            self.active = None
            return Plan(Result.FAILED, reason, replace(previous, status=Status.EXPIRED,
                        reason=reason, revision=previous.revision + 1))
        if track is None or track.track_id != previous.target_track_id:
            return None
        # Service deadline governs the initial computation only. Subsequent
        # publications live until the release window ends, with a fixed release.
        req = replace(self._request, deadline=math.inf)
        reply = _compute_plan(track, now, req, replace(self.params, current_epoch=epoch),
                              fixed_release=previous.release_time)
        if reply.plan is None:          # request-level rejection (e.g. track epoch mismatch)
            self.active = None
            return Plan(Result.FAILED, reply.reason_code or 'PLAN_INVALID',
                        replace(previous, status=Status.EXPIRED, reason=reply.reason_code,
                                revision=previous.revision + 1))
        payload = replace(reply.plan, plan_id=previous.plan_id,
                          revision=previous.revision + 1,
                          valid_until=previous.valid_until)
        self.active = payload
        return replace(reply, plan=payload)
