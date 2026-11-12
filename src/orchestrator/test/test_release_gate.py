"""해제 게이트 기준 분리(SPEC-100 REQ-ORC-014, TEST_SPEC.md TC-ORC-014-01)."""
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

pytest.importorskip('rclpy')

from orchestrator.common import Mode  # noqa: E402
from orchestrator.cycle_runner import OrchestratorCfg  # noqa: E402
from orchestrator.orchestrator_node import RosSkillPort  # noqa: E402

CONFIG = Path(__file__).resolve().parents[1] / 'config' / 'orchestrator.yaml'


class _Clock:
    def ros_now(self):
        return 0.0


@pytest.mark.req('REQ-ORC-014')
@pytest.mark.tc('TC-ORC-014-01')
@pytest.mark.level('U')
def test_dynamic_place_uses_release_gate_threshold():
    cfg_dict = yaml.safe_load(CONFIG.read_text())
    cfg = OrchestratorCfg.from_dict(cfg_dict)
    node = SimpleNamespace(cfg=cfg, cfg_dict=cfg_dict, clock=_Clock(), get_logger=lambda: None, _watch=lambda: None)
    port = RosSkillPort(node)
    port.target, port.plan_id = (7, 0), 3
    sent = []
    port._send = lambda name, goal, deadline, goal_deadline=None: sent.append((name, goal, goal_deadline)) and False
    assert port.place_start(Mode.SYNCHRONIZED, deadline=1.0, goal_deadline=9.5) is None
    name, goal, goal_deadline = sent[0]
    assert goal_deadline == 9.5                      # goal 전체 상한을 _send에 그대로 넘긴다(REQ-ORC-015)
    assert name == 'robot/dynamic_place'
    assert goal.position_std_max == pytest.approx(0.0005)
    assert cfg.sync.release_position_std_max_m == pytest.approx(0.0005)
    assert cfg.plan.position_std_max_m == pytest.approx(0.00325)
