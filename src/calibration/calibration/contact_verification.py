"""독립 접촉점 검증.

카메라 외부 보정(eye-to-hand) 후보에 대해 독립적인 로봇 TCP 포인터 접촉점을
기준으로 3D 검증 오차(XY 평면 잔차, p50, p95, 최대)를 평가하고 거부 기준을 검사한다.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import os
from typing import Any
import numpy as np
import yaml

from calibration.calibration_file import Extrinsics, load_extrinsics
from calibration.errors import CalibrationError


@dataclass
class PointerTcpConfig:
    name: str
    offset_m: np.ndarray
    mode: str = "flange_relative"
    verified: bool = True

    def to_dict(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "offset_m": [float(x) for x in np.asarray(self.offset_m, dtype=float).flatten()],
            "mode": self.mode,
            "verified": bool(self.verified),
        }

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> PointerTcpConfig:
        return cls(
            name=str(data.get("name", "")),
            offset_m=np.array(data.get("offset_m", [0.0, 0.0, 0.0]), dtype=float),
            mode=str(data.get("mode", "flange_relative")),
            verified=bool(data.get("verified", True)),
        )


@dataclass
class ContactVerificationPoint:
    point_id: str
    robot_pos: np.ndarray
    robot_rot: np.ndarray
    cam_optical: np.ndarray
    surface: str = "belt"
    contact_height_m: float = 0.0

    def to_dict(self) -> dict[str, Any]:
        return {
            "point_id": self.point_id,
            "robot_pos": [float(x) for x in np.asarray(self.robot_pos, dtype=float).flatten()],
            "robot_rot": [[float(x) for x in row] for row in np.asarray(self.robot_rot, dtype=float)],
            "cam_optical": [float(x) for x in np.asarray(self.cam_optical, dtype=float).flatten()],
            "surface": self.surface,
            "contact_height_m": float(self.contact_height_m),
        }

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> ContactVerificationPoint:
        return cls(
            point_id=str(data["point_id"]),
            robot_pos=np.array(data["robot_pos"], dtype=float),
            robot_rot=np.array(data["robot_rot"], dtype=float),
            cam_optical=np.array(data["cam_optical"], dtype=float),
            surface=str(data.get("surface", "belt")),
            contact_height_m=float(data.get("contact_height_m", 0.0)),
        )


@dataclass
class VerificationDataset:
    provenance: dict[str, Any]
    pointer_tcp: PointerTcpConfig
    points: list[ContactVerificationPoint]
    camera_profile: dict[str, Any]

    def to_dict(self) -> dict[str, Any]:
        return {
            "provenance": dict(self.provenance),
            "pointer_tcp": self.pointer_tcp.to_dict(),
            "camera_profile": dict(self.camera_profile),
            "points": [p.to_dict() for p in self.points],
        }

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> VerificationDataset:
        pointer_tcp = PointerTcpConfig.from_dict(data["pointer_tcp"])
        points = [ContactVerificationPoint.from_dict(p) for p in data.get("points", [])]
        return cls(
            provenance=dict(data.get("provenance", {})),
            pointer_tcp=pointer_tcp,
            points=points,
            camera_profile=dict(data.get("camera_profile", {})),
        )


@dataclass
class VerificationResult:
    n_points: int
    residuals_xy_m: list[float]
    p50_m: float
    p95_m: float
    max_m: float
    pass_: bool
    is_physical: bool
    refusal_reasons: list[str] = field(default_factory=list)


def verify_contact_points(
    dataset: VerificationDataset,
    extrinsics: Extrinsics,
    min_points: int = 20,
    p95_max_m: float = 0.001,
    max_m: float = 0.002,
    require_physical: bool = False,
    fit_point_set: set[str] | None = None,
) -> VerificationResult:
    """독립 접촉점 검증을 수행하고 통계 및 합격 여부를 판정한다."""
    # 1. 점 개수 확인
    if len(dataset.points) < min_points:
        raise CalibrationError("INSUFFICIENT_POINTS", "INSUFFICIENT_POINTS")

    # 2. 물리 데이터 확인
    data_source = dataset.provenance.get("data_source", "")
    is_physical = (data_source == "physical_touch")
    if require_physical and not is_physical:
        raise CalibrationError("SYNTHETIC_DATA_REJECTED", "SYNTHETIC_DATA_REJECTED")

    # 3. 출처(provenance) 확인
    if "operator" not in dataset.provenance or not dataset.provenance["operator"]:
        raise CalibrationError("MISSING_PROVENANCE", "MISSING_PROVENANCE")

    # 4. calibration_id 및 프로파일 일치 확인
    target_cal_id = dataset.provenance.get("target_calibration_id")
    if target_cal_id != extrinsics.calibration_id:
        raise CalibrationError("CALIBRATION_MISMATCH", "CALIBRATION_MISMATCH")

    ds_cam_serial = dataset.camera_profile.get("serial")
    ext_cam_serial = extrinsics.camera_profile.get("serial")
    if ds_cam_serial != ext_cam_serial:
        raise CalibrationError("PROFILE_MISMATCH", "PROFILE_MISMATCH")

    # 5. 포인터 TCP 검증 여부 확인
    if (
        not dataset.pointer_tcp
        or not getattr(dataset.pointer_tcp, "verified", False)
        or dataset.pointer_tcp.offset_m is None
        or not np.all(np.isfinite(dataset.pointer_tcp.offset_m))
    ):
        raise CalibrationError("UNKNOWN_POINTER_TCP", "UNKNOWN_POINTER_TCP")

    # 6. 후보 변환 행렬 정상 여부 확인
    R_ext = extrinsics.T[:3, :3]
    det = np.linalg.det(R_ext)
    if not np.isfinite(det) or abs(det - 1.0) > 1e-4 or not np.allclose(R_ext.T @ R_ext, np.eye(3), atol=1e-3):
        raise CalibrationError("MALFORMED_TRANSFORM", "MALFORMED_TRANSFORM")

    # 7. 중복 점, fit 점 재사용, 비유한값, 단위 오류 확인
    seen_ids: set[str] = set()
    seen_positions: list[np.ndarray] = []

    for pt in dataset.points:
        # ID 중복
        if pt.point_id in seen_ids:
            raise CalibrationError("DUPLICATE_POINTS", "DUPLICATE_POINTS")
        seen_ids.add(pt.point_id)

        # fit 점 재사용
        if fit_point_set and pt.point_id in fit_point_set:
            raise CalibrationError("FIT_POINT_REUSE", "FIT_POINT_REUSE")

        # 비유한값 검사
        if (
            not np.all(np.isfinite(pt.robot_pos))
            or not np.all(np.isfinite(pt.robot_rot))
            or not np.all(np.isfinite(pt.cam_optical))
        ):
            raise CalibrationError("NONFINITE_INPUT", "NONFINITE_INPUT")

        # 단위 검사 (m 단위여야 하므로 절대값 > 10m 거부)
        if np.any(np.abs(pt.robot_pos) > 10.0) or np.any(np.abs(pt.cam_optical) > 10.0):
            raise CalibrationError("INVALID_UNITS", "INVALID_UNITS")

        # 좌표 중복 검사
        pos = np.asarray(pt.robot_pos, dtype=float)
        for prev_pos in seen_positions:
            if np.allclose(pos, prev_pos, atol=1e-5):
                raise CalibrationError("DUPLICATE_POINTS", "DUPLICATE_POINTS")
        seen_positions.append(pos)

    # 잔차 계산
    R_cam_base = extrinsics.T[:3, :3]
    t_cam_base = extrinsics.T[:3, 3]
    pointer_offset = np.asarray(dataset.pointer_tcp.offset_m, dtype=float)

    residuals_xy: list[float] = []
    for pt in dataset.points:
        # 로봇 베이스 기준 접촉점 실제 위치: p_base = robot_pos + robot_rot @ pointer_offset
        p_touch_base = np.asarray(pt.robot_pos, dtype=float) + np.asarray(pt.robot_rot, dtype=float) @ pointer_offset
        # 카메라 관측점을 베이스 좌표계로 투영: p_cam_base = T_base_cam @ p_cam_opt
        p_cam_base = R_cam_base @ np.asarray(pt.cam_optical, dtype=float) + t_cam_base

        err_xy = float(np.linalg.norm(p_touch_base[:2] - p_cam_base[:2]))
        residuals_xy.append(err_xy)

    residuals_arr = np.array(residuals_xy, dtype=float)
    p50 = float(np.percentile(residuals_arr, 50))
    p95 = float(np.percentile(residuals_arr, 95))
    max_err = float(np.max(residuals_arr))
    pass_flag = bool((p95 <= p95_max_m) and (max_err <= max_m))

    return VerificationResult(
        n_points=len(dataset.points),
        residuals_xy_m=residuals_xy,
        p50_m=p50,
        p95_m=p95,
        max_m=max_err,
        pass_=pass_flag,
        is_physical=is_physical,
        refusal_reasons=[],
    )


def generate_verification_template(template_path: str, candidate_path: str) -> dict[str, Any]:
    """오프라인 검증 템플릿 YAML 파일을 생성한다."""
    cand = load_extrinsics(candidate_path)
    points_template = []
    for i in range(20):
        points_template.append({
            "point_id": f"pt_{i:02d}",
            "robot_pos": [0.50, float(i * 0.01 - 0.10), 0.02],
            "robot_rot": [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            "cam_optical": [0.0, float(i * 0.01 - 0.10), 0.50],
            "surface": "belt",
            "contact_height_m": 0.02,
        })

    data: dict[str, Any] = {
        "provenance": {
            "operator": "operator_name",
            "date": "2026-11-07",
            "target_calibration_id": cand.calibration_id,
            "data_source": "physical_touch",
            "layout_height_mm": 600,
        },
        "camera_profile": dict(cand.camera_profile),
        "pointer_tcp": {
            "name": "needle_pointer",
            "offset_m": [0.0, 0.0, 0.05],
            "mode": "flange_relative",
            "verified": True,
        },
        "points": points_template,
    }

    os.makedirs(os.path.dirname(os.path.abspath(template_path)), exist_ok=True)
    with open(template_path, "w", encoding="utf-8") as fh:
        yaml.safe_dump(data, fh, sort_keys=False)

    return data


def save_verification_dataset(dataset: VerificationDataset, path: str) -> None:
    """검증 데이터셋을 YAML 파일로 저장한다."""
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w", encoding="utf-8") as fh:
        yaml.safe_dump(dataset.to_dict(), fh, sort_keys=False)


def load_verification_dataset(path: str) -> VerificationDataset:
    """YAML 파일로부터 검증 데이터셋을 로드한다."""
    with open(path, "r", encoding="utf-8") as fh:
        data = yaml.safe_load(fh)
    return VerificationDataset.from_dict(data)
