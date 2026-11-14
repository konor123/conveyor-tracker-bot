"""독립 접촉점 검증.

- 독립 접촉점 검증 통계·기준 판정 (n>=20, p95<=1mm, max<=2mm)
- 독립 접촉점 거부 규칙 (점 부족, 중복, fit 점 재사용, 비유한값, 포인터 미정의 등)
- 오프라인 캡처/가져오기 도구 및 템플릿 검증 (로봇 이동 0건)
"""
from __future__ import annotations

import os
import numpy as np
import pytest
import yaml

from calibration.calibration_file import Extrinsics, make_extrinsics
from calibration.contact_verification import (
    ContactVerificationPoint,
    PointerTcpConfig,
    VerificationDataset,
    generate_verification_template,
    load_verification_dataset,
    save_verification_dataset,
    verify_contact_points,
)
from calibration.errors import CalibrationError
from calibration.geometry import make_T


def _sample_candidate(cal_id: str = "cal-2026-11-07-handeye-01") -> Extrinsics:
    """테스트용 정상 카메라 외부 보정 후보."""
    # 광축이 아래(+z 방향 하향)를 향하는 카메라 변환 예시
    R = np.array([
        [0.0, 1.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 0.0, -1.0],
    ], dtype=float)
    t = np.array([0.60, 0.0, 0.50], dtype=float)
    T = make_T(R, t)
    prof = {
        "serial": "313522301601",
        "width": 1280,
        "height": 800,
        "fx": 646.3336,
        "fy": 645.4055,
        "cx": 640.6196,
        "cy": 405.6260,
        "distortion_model": "plumb_bob",
        "distortion": [0.0, 0.0, 0.0, 0.0, 0.0],
    }
    K = np.array([
        [prof["fx"], 0.0, prof["cx"]],
        [0.0, prof["fy"], prof["cy"]],
        [0.0, 0.0, 1.0],
    ], dtype=float)
    dist = np.array(prof["distortion"], dtype=float)
    return Extrinsics(
        calibration_id=cal_id,
        date="2026-11-07",
        T=T,
        parent_frame="base_link",
        child_frame="camera_color_optical_frame",
        K=K,
        dist=dist,
        camera_profile=prof,
        residuals={"status": "CANDIDATE_UNVERIFIED_PHYSICAL_ACCURACY", "verify": {"pass": False}},
    )


def _generate_synthetic_points(
    n: int = 25,
    error_scale_m: float = 0.0004,
    pointer_offset_m: list[float] | None = None,
    is_physical: bool = True,
    cal_id: str = "cal-2026-11-07-handeye-01",
) -> VerificationDataset:
    """테스트용 검증 데이터셋 생성 helper."""
    cand = _sample_candidate(cal_id)
    pointer_offset = np.array(pointer_offset_m if pointer_offset_m is not None else [0.0, 0.0, 0.05], dtype=float)
    pointer_cfg = PointerTcpConfig(
        name="needle_pointer_p1",
        offset_m=pointer_offset,
        mode="flange_relative",
        verified=True,
    )

    # 작업면 위 5x5 격자 점 생성 (X: 0.40~0.60 m, Y: -0.15~0.15 m, Z: 0.02 m)
    xs = np.linspace(0.40, 0.60, 5)
    ys = np.linspace(-0.15, 0.15, 5)
    grid = np.array([[x, y, 0.02] for x in xs for y in ys])[:n]

    points = []
    rng = np.random.default_rng(42)

    R_flange = np.eye(3)  # 플랜지 회전 (단위행렬)

    for i, p_true in enumerate(grid):
        # 로봇 플랜지 위치: p_flange = p_true - R_flange @ pointer_offset
        p_flange = p_true - R_flange @ pointer_offset
        noise_xy = rng.normal(0.0, error_scale_m, size=2)
        p_noisy = np.array([p_true[0] + noise_xy[0], p_true[1] + noise_xy[1], p_true[2]])

        # 카메라 3D 광학 좌표계에서의 점 계산: T_cam_base = inv(T_base_cam)
        T_inv = np.linalg.inv(cand.T)
        p_opt = (T_inv[:3, :3] @ p_noisy) + T_inv[:3, 3]

        points.append(ContactVerificationPoint(
            point_id=f"pt_{i:02d}",
            robot_pos=p_flange,
            robot_rot=R_flange,
            cam_optical=p_opt,
            surface="belt",
            contact_height_m=float(p_true[2]),
        ))

    return VerificationDataset(
        provenance={
            "operator": "test_operator",
            "date": "2026-11-07",
            "target_calibration_id": cal_id,
            "data_source": "physical_touch" if is_physical else "synthetic",
            "camera_serial": cand.camera_profile["serial"],
            "layout_height_mm": 600,
        },
        pointer_tcp=pointer_cfg,
        points=points,
        camera_profile=dict(cand.camera_profile),
    )


# ==============================================================================
# 독립 접촉점 검증 통계·기준 판정
# ==============================================================================

def test_contact_verification_statistics_passing():
    """25개 독립 접촉점에 대해 p95 <= 1mm, max <= 2mm 기준을 만족하면 통과."""
    cand = _sample_candidate()
    ds = _generate_synthetic_points(n=25, error_scale_m=0.0003, is_physical=True)

    result = verify_contact_points(
        dataset=ds,
        extrinsics=cand,
        min_points=20,
        p95_max_m=0.001,
        max_m=0.002,
        require_physical=True,
    )

    assert result.n_points == 25
    assert len(result.residuals_xy_m) == 25
    assert result.p95_m <= 0.001
    assert result.max_m <= 0.002
    assert result.pass_ is True
    assert result.is_physical is True
    assert len(result.refusal_reasons) == 0


def test_contact_verification_threshold_exceeded_fails():
    """오차가 기준(max 2mm 또는 p95 1mm)을 초과하면 pass=False 반환."""
    cand = _sample_candidate()
    # 노이즈를 1.5mm 수준으로 주어 p95 및 max 초과 유도
    ds = _generate_synthetic_points(n=25, error_scale_m=0.0015, is_physical=True)

    result = verify_contact_points(
        dataset=ds,
        extrinsics=cand,
        min_points=20,
        p95_max_m=0.001,
        max_m=0.002,
        require_physical=True,
    )

    assert result.n_points == 25
    assert result.pass_ is False
    assert (result.p95_m > 0.001 or result.max_m > 0.002)


def test_contact_verification_pointer_tcp_correction():
    """포인터 오프셋(flange_relative)이 물리 접촉점에 정확히 보정되는지 검증."""
    cand = _sample_candidate()
    offset = [0.01, -0.02, 0.08]
    ds = _generate_synthetic_points(n=20, error_scale_m=0.0, pointer_offset_m=offset, is_physical=True)

    result = verify_contact_points(dataset=ds, extrinsics=cand, require_physical=True)

    # 노이즈가 0이므로 오차가 수치 오차 이내(< 1e-6 m)여야 함
    assert result.pass_ is True
    assert result.max_m < 1e-6


# ==============================================================================
# 독립 접촉점 거부 규칙
# ==============================================================================

def test_refuse_insufficient_points():
    """점 개수가 20점 미만이면 거부 (INSUFFICIENT_POINTS)."""
    cand = _sample_candidate()
    ds = _generate_synthetic_points(n=19, is_physical=True)

    with pytest.raises(CalibrationError) as exc:
        verify_contact_points(dataset=ds, extrinsics=cand, min_points=20)
    assert exc.value.code == "INSUFFICIENT_POINTS"


def test_refuse_duplicate_points():
    """검증 세트 내 중복 ID 또는 중복 좌표가 있으면 거부 (DUPLICATE_POINTS)."""
    cand = _sample_candidate()
    ds = _generate_synthetic_points(n=20, is_physical=True)

    # 1. 중복 ID 주입
    ds.points[1].point_id = ds.points[0].point_id
    with pytest.raises(CalibrationError) as exc:
        verify_contact_points(dataset=ds, extrinsics=cand)
    assert exc.value.code == "DUPLICATE_POINTS"

    # 2. 다른 ID이지만 동일 위치 좌표 주입
    ds2 = _generate_synthetic_points(n=20, is_physical=True)
    ds2.points[1].robot_pos = np.copy(ds2.points[0].robot_pos)
    with pytest.raises(CalibrationError) as exc2:
        verify_contact_points(dataset=ds2, extrinsics=cand)
    assert exc2.value.code == "DUPLICATE_POINTS"


def test_refuse_fit_points_reuse():
    """외부 보정 fit에 사용된 점 또는 ID가 검증 세트에 재사용되면 거부 (FIT_POINT_REUSE)."""
    cand = _sample_candidate()
    ds = _generate_synthetic_points(n=20, is_physical=True)

    # 후보 extrinsics의 fit 점 집합에 검증 세트의 한 점이 포함되어 있는 경우
    fit_point_ids = {"fit_pt_01", ds.points[0].point_id}
    with pytest.raises(CalibrationError) as exc:
        verify_contact_points(dataset=ds, extrinsics=cand, fit_point_set=fit_point_ids)
    assert exc.value.code == "FIT_POINT_REUSE"


def test_refuse_nonfinite_inputs():
    """NaN, Inf 등 비유한 입력값이 포함되면 거부 (NONFINITE_INPUT)."""
    cand = _sample_candidate()
    ds = _generate_synthetic_points(n=20, is_physical=True)
    ds.points[5].robot_pos[0] = float("nan")

    with pytest.raises(CalibrationError) as exc:
        verify_contact_points(dataset=ds, extrinsics=cand)
    assert exc.value.code == "NONFINITE_INPUT"


def test_refuse_malformed_transform():
    """불량 변환 행렬(행렬식 != 1 또는 비직교 회전) 거부 (MALFORMED_TRANSFORM)."""
    cand = _sample_candidate()
    # 회전 행렬을 불량 행렬로 변조 (det = 2.0)
    cand.T[:3, :3] = np.eye(3) * 2.0

    ds = _generate_synthetic_points(n=20, is_physical=True)
    with pytest.raises(CalibrationError) as exc:
        verify_contact_points(dataset=ds, extrinsics=cand)
    assert exc.value.code == "MALFORMED_TRANSFORM"


def test_refuse_invalid_units():
    """좌표 단위가 m가 아닌 mm로 잘못 기재된 경우(좌표 > 10m 등) 거부 (INVALID_UNITS)."""
    cand = _sample_candidate()
    ds = _generate_synthetic_points(n=20, is_physical=True)
    # 500 mm를 500.0으로 입력한 경우
    ds.points[0].robot_pos[0] = 500.0

    with pytest.raises(CalibrationError) as exc:
        verify_contact_points(dataset=ds, extrinsics=cand)
    assert exc.value.code == "INVALID_UNITS"


def test_refuse_unknown_pointer_tcp():
    """포인터 TCP 정의가 누락되거나 미검증 상태면 거부 (UNKNOWN_POINTER_TCP)."""
    cand = _sample_candidate()
    ds = _generate_synthetic_points(n=20, is_physical=True)
    # 포인터 TCP를 미검증 또는 비유한값으로 변조
    ds.pointer_tcp.verified = False

    with pytest.raises(CalibrationError) as exc:
        verify_contact_points(dataset=ds, extrinsics=cand)
    assert exc.value.code == "UNKNOWN_POINTER_TCP"


def test_refuse_missing_provenance_and_mismatches():
    """출처 누락 및 calibration_id / camera profile 불일치 거부."""
    cand = _sample_candidate(cal_id="cal-2026-11-07-handeye-01")
    ds = _generate_synthetic_points(n=20, is_physical=True, cal_id="cal-OTHER-999")

    # 1. calibration_id 불일치
    with pytest.raises(CalibrationError) as exc:
        verify_contact_points(dataset=ds, extrinsics=cand)
    assert exc.value.code == "CALIBRATION_MISMATCH"

    # 2. 출처 누락 (operator 누락)
    ds2 = _generate_synthetic_points(n=20, is_physical=True)
    del ds2.provenance["operator"]
    with pytest.raises(CalibrationError) as exc2:
        verify_contact_points(dataset=ds2, extrinsics=cand)
    assert exc2.value.code == "MISSING_PROVENANCE"

    # 3. 카메라 프로파일 불일치 (시리얼 번호 다름)
    ds3 = _generate_synthetic_points(n=20, is_physical=True)
    ds3.camera_profile["serial"] = "DIFFERENT_SERIAL_123"
    with pytest.raises(CalibrationError) as exc3:
        verify_contact_points(dataset=ds3, extrinsics=cand)
    assert exc3.value.code == "PROFILE_MISMATCH"


def test_refuse_synthetic_data_for_physical_verification():
    """합성 데이터(synthetic)를 물리 게이트 인증에 제출 시 거부 (SYNTHETIC_DATA_REJECTED)."""
    cand = _sample_candidate()
    ds = _generate_synthetic_points(n=25, is_physical=False)  # data_source: "synthetic"

    with pytest.raises(CalibrationError) as exc:
        verify_contact_points(dataset=ds, extrinsics=cand, require_physical=True)
    assert exc.value.code == "SYNTHETIC_DATA_REJECTED"


# ==============================================================================
# 오프라인 캡처/가져오기 도구 및 템플릿 검증
# ==============================================================================

def test_offline_template_generation_and_roundtrip(tmp_path):
    """로봇 제어 연결 없이 오프라인 템플릿 생성 및 YAML 저장/로드 왕복 일관성 확인."""
    cand = _sample_candidate()
    cand_path = str(tmp_path / "camera_extrinsics_candidate.yaml")
    with open(cand_path, "w") as fh:
        yaml.safe_dump(make_extrinsics(
            cand.calibration_id, cand.date, cand.T, cand.camera_profile, cand.residuals,
            cand.parent_frame, cand.child_frame
        ), fh)

    template_path = str(tmp_path / "verification_template.yaml")
    template_data = generate_verification_template(template_path, candidate_path=cand_path)

    assert os.path.exists(template_path)
    assert template_data["provenance"]["target_calibration_id"] == cand.calibration_id
    assert template_data["camera_profile"]["serial"] == cand.camera_profile["serial"]
    assert len(template_data["points"]) >= 20

    # 저장 및 재로드 왕복 일치 확인
    ds = _generate_synthetic_points(n=20, is_physical=True)
    saved_path = str(tmp_path / "saved_ds.yaml")
    save_verification_dataset(ds, saved_path)
    loaded_ds = load_verification_dataset(saved_path)

    assert loaded_ds.provenance["target_calibration_id"] == ds.provenance["target_calibration_id"]
    assert len(loaded_ds.points) == len(ds.points)
    assert loaded_ds.pointer_tcp.name == ds.pointer_tcp.name
    np.testing.assert_allclose(loaded_ds.points[0].robot_pos, ds.points[0].robot_pos)
