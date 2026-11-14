"""pick_place 설정 읽기: 설정 안 파일 경로의 상대 경로는 저장소 루트 기준."""
import os
from pathlib import Path

import pytest
import yaml

from static_pick_place.pick_place import load_config, load_geometry

REPO = Path(__file__).resolve().parents[3]
SITE = REPO / 'config' / 'site.yaml'


def test_relative_paths_resolve_against_repo_root(tmp_path, monkeypatch):
    monkeypatch.delenv('DROP_REPO_ROOT', raising=False)
    (tmp_path / 'a.yaml').write_text(yaml.safe_dump({
        'mat_plane_file': 'planes/mat.yaml', 'belt_plane_file': '/abs/belt.yaml', 'calibration_file': ''}))
    monkeypatch.chdir('/')                                   # 실행 위치와 무관해야 한다
    cfg = load_config([str(tmp_path / 'a.yaml')])
    assert cfg['mat_plane_file'] == str(REPO / 'planes' / 'mat.yaml')
    assert cfg['belt_plane_file'] == '/abs/belt.yaml'
    assert cfg['calibration_file'] == ''


def test_repo_site_config_has_no_absolute_paths_and_loads(monkeypatch):
    doc = yaml.safe_load(SITE.read_text())
    assert not any(os.path.isabs(doc[k]) for k in ('calibration_file', 'mat_plane_file', 'belt_plane_file'))
    monkeypatch.chdir('/')
    geom = load_geometry(load_config([str(SITE)]))
    assert geom.calibration_id == 'cal-2026-11-07-handeye-01'
    assert geom.belt.offset - geom.mat.offset == pytest.approx(0.093 * geom.mat.normal[2], abs=1e-9)
