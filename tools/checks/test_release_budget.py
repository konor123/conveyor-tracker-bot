"""orchestrator RELEASE 단계 제한과 controller 해제 창의 관계.

정상·게이트 대기 경우의 RELEASE 길이(진입 → 끝 feedback)가 orchestrator RELEASE 단계 제한 안에 들어야 controller가
먼저 끝난다. 값은 fake transport 기준 잠정값이다.
- SYNCHRONIZED: 게이트 대기 상한 + commit→개방 + 개방 확인 유지
- PREDICTED: 하강 끝에서 commit까지 최대 `entry_lead_s − commit_lead_s`, 이어서 commit→개방 + 확인 유지
개방을 확인하지 못한 경우(확인 유지 대신 `release_timeout_s`)는 범위 밖이다.
"""
from __future__ import annotations

import inspect
import sys
from dataclasses import replace
from pathlib import Path

import pytest
import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / 'src' / 'robot_control'))

from robot_control.dynamic_place import DynamicPlaceCfg  # noqa: E402
from robot_control.fake_place_transport import SimulatedPlaceTransport  # noqa: E402


def release_lengths(cfg: DynamicPlaceCfg, commit_lead_s: float) -> dict[str, float]:
    return {'synchronized': cfg.release_wait_max_s + commit_lead_s + cfg.release_hold_s,
            'predicted': (cfg.entry_lead_s - commit_lead_s) + commit_lead_s + cfg.release_hold_s}


def _release_limit() -> float:
    with open(REPO_ROOT / 'src/orchestrator/config/orchestrator.yaml', encoding='utf-8') as fh:
        return float(yaml.safe_load(fh)['step_timeout']['release_s'])


def test_release_step_limit_covers_controller_release_window():
    limit = _release_limit()
    lead = float(inspect.signature(SimulatedPlaceTransport).parameters['open_delay_s'].default)
    lengths = release_lengths(DynamicPlaceCfg(), lead)
    assert lengths == pytest.approx({'synchronized': 1.4, 'predicted': 1.6})
    assert limit == pytest.approx(2.0)
    assert all(v <= limit for v in lengths.values()), (lengths, limit)
    # 11-13까지의 게이트 대기 상한 2.0 s였으면 SYNCHRONIZED가 제한을 넘는다(이 검사가 그 관계를 잡는다)
    assert release_lengths(replace(DynamicPlaceCfg(), release_wait_max_s=2.0), lead)['synchronized'] > limit
