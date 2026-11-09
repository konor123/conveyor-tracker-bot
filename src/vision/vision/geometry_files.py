"""보정 파일(외부 변환·매트·벨트 평면)을 읽어 pipeline.Geometry를 만든다. vision_node와 static_pick_place가 함께 쓴다.

- 경로가 상대 경로면 저장소 루트 기준이다(calibration.repo_paths, SPEC-000 REQ-SYS-005).
- 외부 변환의 parent frame은 `robot_base` 또는 Doosan 드라이버 이름 `base_link`(SPEC-000 4.2절, 같은 프레임)여야 한다.
- 평면 파일은 `frame: robot_base`와 외부 변환과 같은 `calibration_id`를 가져야 한다(SPEC-600 2절).
"""
from __future__ import annotations

from pathlib import Path

import numpy as np
import yaml

from .localizer import CameraModel, HeightParams, Plane
from .pipeline import Geometry

ROBOT_BASE_FRAMES = ('robot_base', 'base_link')


def load_geometry(calibration_file: str, mat_plane_file: str, belt_plane_file: str,
                  heights: HeightParams) -> Geometry | None:
    """calibration_file이 비면 None(보정 없음, 픽셀 검출만). 외부 변환을 지정했는데 평면이 비었거나,
    지정한 파일이 없거나, 프레임·ID가 어긋나면 ValueError(경로 오타를 조용히 넘기지 않는다)."""
    from calibration.calibration_file import load_extrinsics
    from calibration.repo_paths import resolve_path

    raw = (calibration_file, mat_plane_file, belt_plane_file)
    if not calibration_file:
        return None
    if not all(raw):
        raise ValueError('mat_plane_file and belt_plane_file are required with calibration_file')
    paths = [resolve_path(p) for p in raw]
    for given, path in zip(raw, paths):
        if not Path(path).is_file():
            raise ValueError(f'calibration file not found: {given} -> {path}')
    extr = load_extrinsics(paths[0])
    if extr.parent_frame not in ROBOT_BASE_FRAMES:
        raise ValueError(f'extrinsics parent frame must be one of {ROBOT_BASE_FRAMES}: {extr.parent_frame}')
    planes = []
    for path in paths[1:]:
        data = yaml.safe_load(Path(path).read_text())
        if data.get('frame') != 'robot_base' or data.get('calibration_id') != extr.calibration_id:
            raise ValueError(f'plane frame/calibration_id mismatch: {path}')
        planes.append(Plane(np.asarray(data['normal'], float), float(data['offset_m'])))
    return Geometry(extr.calibration_id, CameraModel(extr.K, extr.dist), extr.T, *planes, heights)
