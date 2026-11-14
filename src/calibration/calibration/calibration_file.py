"""보정 결과 파일 camera_extrinsics.yaml 읽기·쓰기."""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import yaml

from .geometry import R_from_quat, make_T, quat_from_R

REQUIRED_KEYS = ('calibration_id', 'date', 'residuals', 'camera_profile', 'transform')
PROFILE_KEYS = ('serial', 'width', 'height', 'fx', 'fy', 'cx', 'cy', 'distortion_model', 'distortion')
TRANSFORM_KEYS = ('parent_frame', 'child_frame', 'translation_m', 'rotation_xyzw')


class CalibrationFileError(ValueError):
    pass


@dataclass(frozen=True)
class Extrinsics:
    calibration_id: str
    date: str
    T: np.ndarray                 # parent_frame ← child_frame
    parent_frame: str
    child_frame: str
    K: np.ndarray
    dist: np.ndarray
    camera_profile: dict
    residuals: dict


def plain(v):
    """numpy 값 → YAML로 쓸 수 있는 기본 타입."""
    if isinstance(v, dict):
        return {str(k): plain(x) for k, x in v.items()}
    if isinstance(v, (list, tuple, np.ndarray)):
        return [plain(x) for x in v]
    if isinstance(v, np.generic):
        return v.item()
    return v


def make_extrinsics(calibration_id: str, date: str, T: np.ndarray, camera_profile: dict, residuals: dict,
                    parent_frame: str, child_frame: str) -> dict:
    return {
        'calibration_id': calibration_id,
        'date': str(date),
        'transform': {
            'parent_frame': parent_frame,
            'child_frame': child_frame,
            'translation_m': [float(v) for v in T[:3, 3]],
            'rotation_xyzw': [float(v) for v in quat_from_R(T[:3, :3])],
        },
        'camera_profile': plain(camera_profile),
        'residuals': plain(residuals),
    }


def save_extrinsics(doc: dict, path) -> None:
    _validate(doc)
    with open(path, 'w') as fh:
        yaml.safe_dump(doc, fh, sort_keys=False, allow_unicode=True)


def _validate(doc) -> None:
    if not isinstance(doc, dict):
        raise CalibrationFileError('보정 파일이 매핑이 아니다')
    missing = [k for k in REQUIRED_KEYS if k not in doc or doc[k] in (None, '', {}, [])]
    if missing:
        raise CalibrationFileError(f'보정 파일 필수 항목 없음: {", ".join(missing)}')
    for name, keys in (('camera_profile', PROFILE_KEYS), ('transform', TRANSFORM_KEYS)):
        sub = doc[name]
        if not isinstance(sub, dict):
            raise CalibrationFileError(f'{name}이 매핑이 아니다')
        miss = [k for k in keys if k not in sub or sub[k] in (None, '')]
        if miss:
            raise CalibrationFileError(f'{name} 필수 항목 없음: {", ".join(miss)}')
    q = np.asarray(doc['transform']['rotation_xyzw'], float)
    if q.shape != (4,) or not np.isclose(np.linalg.norm(q), 1.0, atol=1e-6):
        raise CalibrationFileError(f'rotation_xyzw가 단위 quaternion이 아니다: {q.tolist()}')
    if len(doc['transform']['translation_m']) != 3:
        raise CalibrationFileError('translation_m은 3개 값이어야 한다')


def load_extrinsics(path) -> Extrinsics:
    with open(path) as fh:
        doc = yaml.safe_load(fh)
    _validate(doc)
    tf = doc['transform']
    prof = doc['camera_profile']
    K = np.array([[prof['fx'], 0.0, prof['cx']], [0.0, prof['fy'], prof['cy']], [0.0, 0.0, 1.0]], float)
    return Extrinsics(calibration_id=str(doc['calibration_id']), date=str(doc['date']),
                      T=make_T(R_from_quat(tf['rotation_xyzw']), np.asarray(tf['translation_m'], float)),
                      parent_frame=tf['parent_frame'], child_frame=tf['child_frame'], K=K,
                      dist=np.asarray(prof['distortion'], float), camera_profile=prof, residuals=doc['residuals'])
