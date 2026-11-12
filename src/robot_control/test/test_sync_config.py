"""추종 코어 설정 파일과 다른 패키지 기준값의 일치(SPEC-500 REQ-ROB-013, TEST_SPEC.md TC-ROB-013-06)."""
import json
from dataclasses import asdict
from pathlib import Path

import pytest
import yaml

from robot_control.belt_sync import SyncConfig

pytestmark = [pytest.mark.level('U'), pytest.mark.req('REQ-ROB-013'), pytest.mark.tc('TC-ROB-013-06')]
SRC = Path(__file__).resolve().parents[2]


def test_belt_sync_json_matches_defaults():
    data = json.loads((SRC / 'robot_control/config/belt_sync.json').read_text())
    loaded, default = asdict(SyncConfig(**data)), asdict(SyncConfig())
    assert set(data) == set(default)
    assert {k: tuple(v) if isinstance(v, (list, tuple)) else v for k, v in loaded.items()} == default


def test_position_std_limits_agree_across_packages():
    cfg = SyncConfig()
    judge = yaml.safe_load((SRC / 'orchestrator/config/orchestrator.yaml').read_text())['judge']
    planner = json.loads((SRC / 'intercept_planner/config/planner.yaml').read_text())
    assert judge['plan']['position_std_max_m'] == cfg.max_position_std_m
    assert planner['uncertainty']['position_std_max_m'] == cfg.max_position_std_m
    assert judge['sync']['release_position_std_max_m'] == cfg.release_position_std_max_m
