"""시도 기록 스키마 시험(TEST_SPEC.md TC-REC-001-01)."""
import copy
import json
import os

import pytest
from jsonschema import Draft202012Validator

SCHEMA_PATH = os.path.join(os.path.dirname(__file__), "trial_schema.json")

with open(SCHEMA_PATH, encoding="utf-8") as f:
    SCHEMA = json.load(f)

VALIDATOR = Draft202012Validator(SCHEMA)

BASE_RECORD = {
    "trial_id": "2026-11-20-G6-0042",
    "env": "real",
    "interface_version": "SPEC-010 0.2",
    "scenario": {"name": "", "seed": 0},
    "assumed_params": [],
    "gate": "G6",
    "git_sha": "...",
    "spec_versions": {"SPEC-100": "0.3"},
    "calibration_id": "...",
    "doosan_sha": "...",
    "controller_ver": "...",
    "gripper_fw": "...",
    "mode": "MODE_SYNCHRONIZED",
    "belt_speed_ref_mps": 0.020,
    "belt_speed_est_mps": 0.0197,
    "lighting": "lab-A",
    "operator": "...",
    "result": "SUCCESS",
    "abort_level": 0,
    "reason_code": "",
    "observer_verdict": "SUCCESS",
    "auto_verdict": "SUCCESS",
    "release_gate": {
        "e_xy_m": 0.0012,
        "v_rel_mps": 0.0021,
        "track_status": "COASTING",
        "t_since_obs_s": 0.18,
        "pos_std_m": 0.0004,
    },
    "release_height_m": 0.010,
    "release_error_m": 0.0013,
    "stone_offset_m": 0.0041,
    "gripper": {"open_cmd_latency_s": 0.12, "grip_current_ma": 150},
    "passes_skipped": 0,
    "time_margin_s": 9.1,
    "cycle_time_s": 24.3,
    "step_durations_s": {
        "PICK_STONE": 6.2,
        "WAIT_CAP": 8.4,
        "APPROACH": 1.1,
    },
    "rosbag": {"path": "...", "sha256": "..."},
    "notes": "",
}

REQUIRED_FIELDS = [
    "trial_id",
    "env",
    "interface_version",
    "scenario",
    "assumed_params",
    "gate",
    "git_sha",
    "mode",
    "result",
    "abort_level",
    "reason_code",
    "passes_skipped",
    "cycle_time_s",
]


def is_valid(record):
    return VALIDATOR.is_valid(record)


@pytest.mark.req("REQ-REC-001")
@pytest.mark.tc("TC-REC-001-01")
@pytest.mark.level("U")
def test_schema_is_valid():
    Draft202012Validator.check_schema(SCHEMA)
    assert SCHEMA["$id"] == "trial_schema.json"
    assert SCHEMA["title"] == "trial record (PLAN.md 10.2)"
    assert SCHEMA["additionalProperties"] is True


@pytest.mark.req("REQ-REC-001")
@pytest.mark.tc("TC-REC-001-01")
@pytest.mark.level("U")
def test_plan_example_passes():
    assert is_valid(BASE_RECORD)


@pytest.mark.req("REQ-REC-001")
@pytest.mark.tc("TC-REC-001-01")
@pytest.mark.level("U")
@pytest.mark.parametrize("field", REQUIRED_FIELDS)
def test_missing_required_field_fails(field):
    record = copy.deepcopy(BASE_RECORD)
    del record[field]
    assert not is_valid(record)


@pytest.mark.req("REQ-REC-001")
@pytest.mark.tc("TC-REC-001-01")
@pytest.mark.level("U")
@pytest.mark.parametrize(
    "path, value",
    [
        (("env",), "fake"),
        (("result",), "MAYBE"),
        (("mode",), "MODE_UNKNOWN"),
        (("observer_verdict",), "MAYBE"),
        (("auto_verdict",), "MAYBE"),
        (("abort_level",), 5),
        (("abort_level",), -1),
        (("passes_skipped",), -1),
    ],
)
def test_out_of_range_value_fails(path, value):
    record = copy.deepcopy(BASE_RECORD)
    target = record
    for key in path[:-1]:
        target = target[key]
    target[path[-1]] = value
    assert not is_valid(record)


@pytest.mark.req("REQ-REC-001")
@pytest.mark.tc("TC-REC-001-01")
@pytest.mark.level("U")
def test_scenario_seed_required():
    record = copy.deepcopy(BASE_RECORD)
    del record["scenario"]["seed"]
    assert not is_valid(record)


@pytest.mark.req("REQ-REC-001")
@pytest.mark.tc("TC-REC-001-01")
@pytest.mark.level("U")
def test_mock_trial_passes():
    record = copy.deepcopy(BASE_RECORD)
    record["env"] = "mock"
    record["scenario"] = {"name": "nominal_20mms", "seed": 0}
    record["assumed_params"] = ["robot", "world.belt"]
    assert is_valid(record)
