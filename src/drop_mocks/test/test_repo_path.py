"""저장소 루트 기준 경로 규칙의 drop_mocks 사본이 원본(calibration.repo_paths)과 같게 동작한다."""
import os
from pathlib import Path

import pytest

pytest.importorskip('rclpy')
from drop_mocks.ros_common import repo_path  # noqa: E402

REPO = Path(__file__).resolve().parents[3]


def test_repo_path_rule(tmp_path, monkeypatch):
    monkeypatch.delenv('DROP_REPO_ROOT', raising=False)
    monkeypatch.chdir(tmp_path)
    assert repo_path('records/trials') == str(REPO / 'records' / 'trials')
    assert repo_path('/abs/x') == '/abs/x'
    assert repo_path('~/x') == os.path.expanduser('~/x')
    assert repo_path('') == ''
    monkeypatch.setenv('DROP_REPO_ROOT', str(tmp_path))
    assert repo_path('a/b.yaml') == str(tmp_path / 'a' / 'b.yaml')


def test_scenario_names_still_use_installed_share(monkeypatch):
    from drop_mocks.ros_common import scenario_file
    monkeypatch.delenv('DROP_REPO_ROOT', raising=False)
    assert scenario_file('src/drop_mocks/scenarios/nominal_20mms.yaml') == str(
        REPO / 'src' / 'drop_mocks' / 'scenarios' / 'nominal_20mms.yaml')
