"""drop_interfaces 공통 규약 시험: 요청 공통 필드 순서, 결과 코드."""
from __future__ import annotations

from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
COMMON_REQUEST_FIELDS = ("cycle_id", "step_id", "deadline")
COMMON_RESULT_CODES = {
    "OK": 0,
    "FAILED": 1,
    "TIMEOUT": 2,
    "PRECONDITION": 3,
    "CANCELED": 4,
    "FAULT": 5,
}


def test_common_request_fields_order():
    srv_mod = pytest.importorskip("drop_interfaces.srv")
    act_mod = pytest.importorskip("drop_interfaces.action")

    srv_names = sorted(p.stem for p in (REPO_ROOT / "src" / "drop_interfaces" / "srv").glob("*.srv"))
    act_names = sorted(p.stem for p in (REPO_ROOT / "src" / "drop_interfaces" / "action").glob("*.action"))
    assert len(srv_names) == 6
    assert len(act_names) == 6 and "DropTask" in act_names

    for name in srv_names:
        req_cls = getattr(srv_mod, name).Request
        fields = req_cls.get_fields_and_field_types()
        keys = tuple(fields.keys())
        assert keys[:3] == COMMON_REQUEST_FIELDS, f"srv/{name}.Request: {keys[:3]}"
        assert fields["cycle_id"] == "string"
        assert fields["step_id"] == "uint32"
        assert fields["deadline"] == "builtin_interfaces/Time"

    for name in act_names:
        goal_cls = getattr(act_mod, name).Goal
        fields = goal_cls.get_fields_and_field_types()
        keys = tuple(fields.keys())
        if name == "DropTask":
            assert keys[:3] != COMMON_REQUEST_FIELDS
            continue
        assert keys[:3] == COMMON_REQUEST_FIELDS, f"action/{name}.Goal: {keys[:3]}"
        assert fields["cycle_id"] == "string"
        assert fields["step_id"] == "uint32"
        assert fields["deadline"] == "builtin_interfaces/Time"


def test_interface_constants_match_spec():
    msg_mod = pytest.importorskip("drop_interfaces.msg")
    srv_mod = pytest.importorskip("drop_interfaces.srv")
    act_mod = pytest.importorskip("drop_interfaces.action")

    det = msg_mod.ObjectDetection
    assert det.REGION_UNKNOWN == 0
    assert det.REGION_MAT == 1
    assert det.REGION_BELT == 2
    assert det.CLASS_UNKNOWN == 0
    assert det.CLASS_STONE == 1
    assert det.CLASS_CAP == 2
    assert det.CLASS_BELT_MARKER == 3

    trk = msg_mod.TrackedObject
    assert trk.OBSERVED == 0
    assert trk.COASTING == 1
    assert trk.LOST == 2

    belt = msg_mod.BeltMotion
    assert belt.SOURCE_CAP == 0
    assert belt.SOURCE_BELT_MARKER == 1

    plan = msg_mod.InterceptPlan
    assert plan.OK == 0
    assert plan.TIME_INSUFFICIENT == 1
    assert plan.UNREACHABLE == 2
    assert plan.UNCERTAIN == 3
    assert plan.EXPIRED == 4

    drop_goal = act_mod.DropTask.Goal
    assert drop_goal.MODE_STATIC == 0
    assert drop_goal.MODE_PREDICTED == 1
    assert drop_goal.MODE_SYNCHRONIZED == 2

    drop_res = act_mod.DropTask.Result
    assert drop_res.SUCCESS == 0
    assert drop_res.FAILURE == 1
    assert drop_res.UNKNOWN == 2
    assert drop_res.CANCELED == 3
    assert drop_res.SAFE_ABORT == 4
    assert drop_res.PROTECTIVE_STOP == 5
    assert drop_res.NOT_ATTEMPTED == 6

    v_res = srv_mod.VerifyDrop.Response
    assert v_res.SUCCESS == 0
    assert v_res.FAILURE == 1
    assert v_res.UNKNOWN == 2

    stop_req = srv_mod.Stop.Request
    assert stop_req.ORCHESTRATOR == 0
    assert stop_req.SUPERVISOR == 1

    # 모든 srv Response와 스킬 action Result의 공통 결과 코드 상수 확인
    srv_responses = [
        srv_mod.DetectObjects.Response,
        srv_mod.VerifyDrop.Response,
        srv_mod.ComputeIntercept.Response,
        srv_mod.ResetTracking.Response,
        srv_mod.SelectTarget.Response,
        srv_mod.Stop.Response,
    ]
    skill_results = [
        act_mod.PickStone.Result,
        act_mod.MoveToPose.Result,
        act_mod.PlaceStone.Result,
        act_mod.DynamicPlace.Result,
        act_mod.SafeRetreat.Result,
    ]
    for cls in srv_responses + skill_results:
        for const_name, expected_val in COMMON_RESULT_CODES.items():
            assert getattr(cls, const_name) == expected_val, f"{cls.__qualname__}.{const_name}"
