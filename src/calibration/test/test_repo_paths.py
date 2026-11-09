"""설정 파일 경로 규칙(SPEC-000 REQ-SYS-005): 상대 경로는 저장소 루트 기준."""
import os
from pathlib import Path

import pytest

from calibration.repo_paths import ENV_REPO_ROOT, repo_root, resolve_path

REPO = Path(__file__).resolve().parents[3]


@pytest.mark.level('U')
@pytest.mark.req('REQ-SYS-005')
@pytest.mark.tc('TC-SYS-005-01')
def test_relative_paths_resolve_against_repo_root_regardless_of_cwd(tmp_path, monkeypatch):
    monkeypatch.delenv(ENV_REPO_ROOT, raising=False)
    monkeypatch.chdir(tmp_path)                                   # 실행 위치와 무관
    assert repo_root() == REPO                                    # 소스 트리에서 src·records를 가진 폴더
    assert resolve_path('records/calib/x.yaml') == str(REPO / 'records' / 'calib' / 'x.yaml')
    assert resolve_path('/abs/x.yaml') == '/abs/x.yaml'
    assert resolve_path('~/x.yaml') == os.path.expanduser('~/x.yaml')
    assert resolve_path('') == '' and resolve_path(None) == ''


@pytest.mark.level('U')
@pytest.mark.req('REQ-SYS-005')
@pytest.mark.tc('TC-SYS-005-01')
def test_repo_root_from_install_tree_and_env_override(tmp_path, monkeypatch):
    monkeypatch.delenv(ENV_REPO_ROOT, raising=False)
    (tmp_path / 'src').mkdir()
    (tmp_path / 'records').mkdir()
    mod = tmp_path / 'install' / 'calibration' / 'lib' / 'python3.12' / 'site-packages' / 'calibration'
    mod.mkdir(parents=True)
    assert repo_root(mod / 'repo_paths.py') == tmp_path          # colcon install은 워크스페이스 아래
    other = tmp_path / 'elsewhere'
    other.mkdir()
    monkeypatch.setenv(ENV_REPO_ROOT, str(other))
    assert repo_root(mod / 'repo_paths.py') == other
    assert resolve_path('a/b.yaml') == str(other / 'a' / 'b.yaml')
