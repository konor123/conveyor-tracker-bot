"""conveyor-tracker-bot 공통 pytest 규칙.

각 패키지 test/conftest.py에서 `from drop_pytest import *`로 가져온다.

마커:
  @pytest.mark.needs_data("off1_bags")  저장소 밖 자료가 필요한 시험. 자료가 없으면 skip(통과로 세지 않음)
  @pytest.mark.realtime_pc              실시간 스케줄에 의존하는 시험. 공유 CI 러너에서는 결과를 참고만 한다

자료 위치: 환경 변수 DROP_DATASETS가 가리키는 YAML(`이름: 절대 경로`).
"""
import os

import pytest

__all__ = ["pytest_configure", "pytest_collection_modifyitems", "dataset_path"]


def _datasets():
    path = os.environ.get("DROP_DATASETS")
    if not path or not os.path.exists(path):
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
    config.addinivalue_line("markers", "needs_data(name): 저장소 밖 자료 세트. 없으면 skip")
    config.addinivalue_line("markers", "realtime_pc: 실시간 스케줄에 의존하는 시험. 공유 CI 러너에서는 참고만 하고 "
                                       "판정은 현장 PC에서 한다")


def pytest_collection_modifyitems(config, items):
    for item in items:
        for m in item.iter_markers("needs_data"):
            for name in m.args:
                if dataset_path(name) is None:
                    item.add_marker(pytest.mark.skip(reason=f"needs_data: {name} 없음(DROP_DATASETS)"))
