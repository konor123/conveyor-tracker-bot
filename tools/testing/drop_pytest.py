"""conveyor-tracker-bot 공통 pytest 규칙(TEST_SPEC.md 0.4절).

각 패키지 test/conftest.py에서 `from drop_pytest import *`로 가져온다.

마커:
  @pytest.mark.req("REQ-ABT-003")      검증하는 요구사항
  @pytest.mark.tc("TC-ABT-003-01")     시험 케이스
  @pytest.mark.level("U")              U / C / I0 / I1 / I2
  @pytest.mark.needs_data("off1_bags") [ON*] 자료. 자료가 없으면 skip(통과로 세지 않음)

자료 위치: 환경 변수 DROP_DATASETS(YAML 경로) 또는 <저장소>/records/datasets.yaml
  형식:  off1_bags: /mnt/ssd/drop_bags/2026-11-07
"""
import os
import re

import pytest

__all__ = ["pytest_configure", "pytest_collection_modifyitems", "dataset_path", "LEVELS"]

LEVELS = ("U", "C", "I0", "I1", "I2")
_REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir, os.pardir))
_ID = {"req": re.compile(r"^REQ-[A-Z]{2,4}-\d{3}$"), "tc": re.compile(r"^TC-[A-Z]{2,4}-\d{3}-\d{2}$")}


def _datasets():
    path = os.environ.get("DROP_DATASETS", os.path.join(_REPO, "records", "datasets.yaml"))
    if not os.path.exists(path):
        return {}
    try:
        import yaml
    except ImportError:          # pragma: no cover - ROS 환경에는 PyYAML이 있다
        return {}
    with open(path) as fh:
        data = yaml.safe_load(fh) or {}
    return {k: v for k, v in data.items() if isinstance(v, str)}


def dataset_path(name):
    """자료 세트 경로. 등록되지 않았거나 경로가 없으면 None."""
    p = _datasets().get(name)
    return p if p and os.path.exists(os.path.expanduser(p)) else None


def pytest_configure(config):
    config.addinivalue_line("markers", "req(id): 검증하는 요구사항 ID (REQ-<영역>-<번호>)")
    config.addinivalue_line("markers", "tc(id): 시험 케이스 ID (TC-<영역>-<REQ 번호>-<순번>)")
    config.addinivalue_line("markers", "level(name): 시험 수준 U/C/I0/I1/I2")
    config.addinivalue_line("markers", "needs_data(name): [ON*] 자료 세트. 없으면 skip")
    config.addinivalue_line("markers", "realtime_pc: 실시간 스케줄에 의존하는 시험. 공유 CI 러너에서는 실행·기록만 하고 "
                                       "판정은 PC(LEGION 또는 officepc)에서 한다(SPEC-700 REQ-MCK-005)")


def pytest_collection_modifyitems(config, items):
    for item in items:
        for kind in ("req", "tc"):
            for m in item.iter_markers(kind):
                for v in m.args:
                    if not _ID[kind].match(str(v)):
                        raise pytest.UsageError(f"{item.nodeid}: 잘못된 {kind} ID {v!r}")
        for m in item.iter_markers("level"):
            if not m.args or m.args[0] not in LEVELS:
                raise pytest.UsageError(f"{item.nodeid}: level은 {LEVELS} 중 하나")
        level = item.get_closest_marker("level")
        if level is not None:
            item.user_properties.append(("level", level.args[0]))
        for m in item.iter_markers("needs_data"):
            for name in m.args:
                if dataset_path(name) is None:
                    item.add_marker(pytest.mark.skip(reason=f"needs_data: {name} 없음(records/datasets.yaml)"))
