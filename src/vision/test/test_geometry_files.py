"""geometry_files.load_geometry: 저장소 상대 경로, base_link 별칭, 누락·불일치 거부."""
from pathlib import Path

import pytest
import yaml

from vision.geometry_files import load_geometry
from vision.localizer import HeightParams

REPO = Path(__file__).resolve().parents[3]
SITE = REPO / 'records' / 'calib' / '2026-11-09-planes' / 'vision_node.yaml'
H = HeightParams(0.015, 0.022, 0.0015)


def _site():
    return yaml.safe_load(SITE.read_text())['vision_node']['ros__parameters']


@pytest.mark.level('U')
def test_site_files_load_from_any_cwd_with_base_link_parent(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    p = _site()
    assert not any(Path(p[k]).is_absolute() for k in ('calibration_file', 'mat_plane_file', 'belt_plane_file'))
    g = load_geometry(p['calibration_file'], p['mat_plane_file'], p['belt_plane_file'], H)
    assert g.calibration_id == 'cal-2026-11-07-handeye-01'           # 외부 변환 parent_frame은 base_link
    assert g.belt.offset - g.mat.offset == pytest.approx(0.093 * g.mat.normal[2], abs=1e-9)
    assert (p['h_stone_offset_m'], p['h_rim_m']) == (0.015, 0.022)  # 2026-11-09 실측


@pytest.mark.level('U')
def test_empty_partial_missing_and_mismatched(tmp_path):
    p = _site()
    assert load_geometry('', '', '', H) is None
    assert load_geometry('', p['mat_plane_file'], p['belt_plane_file'], H) is None   # 보정만 비우면 픽셀 검출만
    with pytest.raises(ValueError, match='required'):
        load_geometry(p['calibration_file'], '', '', H)
    with pytest.raises(ValueError, match='not found'):
        load_geometry(p['calibration_file'], 'records/calib/no-such.yaml', p['belt_plane_file'], H)
    bad = yaml.safe_load((REPO / p['mat_plane_file']).read_text())
    bad['calibration_id'] = 'other'
    f = tmp_path / 'mat.yaml'
    f.write_text(yaml.safe_dump(bad))
    with pytest.raises(ValueError, match='mismatch'):
        load_geometry(p['calibration_file'], str(f), p['belt_plane_file'], H)
