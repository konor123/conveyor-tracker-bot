"""투입 자동 확인 verifier.py 합성 시험(SPEC-200 REQ-VIS-009, TEST_SPEC.md TC-VIS-009-01).

후퇴 완료 뒤 30 Hz 프레임마다 선택한 병뚜껑(track 7)의 `stone_inside`·`stone_inside_valid`와
뚜껑 중심 → 공깃돌 중심 XY를 만든다. 뚜껑은 벨트를 따라 +x로 20 mm/s 움직인다.
"""
import math

import pytest

from vision.verifier import FAILURE, SUCCESS, UNKNOWN, CapObs, FrameObs, VerifyParams, verify

FPS = 30.0
V_BELT = 0.02                      # [m/s]
R_IN = 0.5 * 0.032 * 0.80          # 뚜껑 바깥 지름 32 mm × inner_diameter_ratio [m]
CAP = 7                            # 선택한 뚜껑 track id
OTHER = 8                          # 옆 뚜껑
P = VerifyParams(hold_time_s=1.0)


def _cap(track_id, offset, valid=True):
    """offset: 뚜껑 중심 → 공깃돌 중심 [m]. None이면 이 뚜껑 근처에 공깃돌이 없다."""
    if not valid:
        return CapObs(track_id, stone_inside_valid=False)
    inside = offset is not None and math.hypot(*offset) < R_IN
    return CapObs(track_id, stone_inside_valid=True, stone_inside=inside,
                  stone_offset_xy_m=offset if inside else None)


def _frames(offset_at, n=46, other=None, valid_at=lambda t: True):
    """offset_at(t) → 선택한 뚜껑 기준 공깃돌 위치. n 프레임(기본 1.5 s)."""
    out = []
    for k in range(n):
        t = 100.0 + k / FPS
        caps = [_cap(CAP, offset_at(k / FPS), valid_at(k / FPS))]
        if other is not None:
            caps.append(_cap(OTHER, other))
        out.append(FrameObs(stamp_s=t, caps=tuple(caps)))
    return out


SETTLED = (0.003, -0.002)          # 뚜껑 안에서 멈춘 공깃돌


CASES = {
    # 뚜껑 안에 들어가 함께 이동(1.5 s 관측)
    'normal_hold': (_frames(lambda t: SETTLED), SUCCESS, ''),
    # 그리퍼가 뚜껑을 가려 판정 불가
    'occluded': (_frames(lambda t: SETTLED, valid_at=lambda t: False), UNKNOWN, 'NOT_OBSERVED'),
    # 0.9 s 동안 안에 있다가 튀어 나감
    'inside_0p9s_then_out': (_frames(lambda t: SETTLED if t <= 0.9 + 1e-9 else (0.03, 0.0)), UNKNOWN, 'HOLD_SHORT'),
    # 튕겨 나가 벨트 위(뚜껑과 30 mm 떨어져 같이 이동), 공깃돌은 옆 뚜껑 안에 들어감
    'bounced_onto_belt': (_frames(lambda t: (0.03, 0.0), other=SETTLED), UNKNOWN, 'NOT_INSIDE'),
    # 그리퍼에 붙어 공중에 멈춤: 뚜껑만 20 mm/s로 멀어진다(0.64 s 뒤 원 밖)
    'stuck_to_gripper': (_frames(lambda t: (-V_BELT * t, 0.0)), UNKNOWN, 'HOLD_SHORT'),
}


@pytest.mark.level('U')
@pytest.mark.req('REQ-VIS-009')
@pytest.mark.tc('TC-VIS-009-01')
@pytest.mark.parametrize('name', list(CASES))
def test_verify_cases(name):
    frames, expected, reason = CASES[name]
    v = verify(frames, CAP, P)
    assert v.verdict == expected, v
    assert v.verdict != FAILURE
    assert v.reason == reason
    if expected == SUCCESS:
        assert v.stone_inside
        assert v.stone_offset_valid
        assert v.stone_offset_m == pytest.approx(math.hypot(*SETTLED), abs=1e-9)
        assert v.hold_s == pytest.approx(45 / FPS, abs=1e-9)
    else:
        assert not v.stone_offset_valid


@pytest.mark.level('U')
@pytest.mark.req('REQ-VIS-009')
@pytest.mark.tc('TC-VIS-009-01')
@pytest.mark.parametrize('inside_frames,expected', [(30, UNKNOWN), (31, SUCCESS), (32, SUCCESS)])
def test_hold_time_boundary(inside_frames, expected):
    """안에 있는 구간 (inside_frames − 1)/30 s: 직전 0.967 s / 경계 1.000 s / 직후 1.033 s."""
    frames = _frames(lambda t: SETTLED if t < (inside_frames - 0.5) / FPS else (0.03, 0.0))
    assert verify(frames, CAP, P).verdict == expected


@pytest.mark.level('U')
@pytest.mark.req('REQ-VIS-009')
@pytest.mark.tc('TC-VIS-009-01')
@pytest.mark.parametrize('missing,expected', [(2, SUCCESS), (8, UNKNOWN)])
def test_missing_cap_frames_bound_continuity(missing, expected):
    """선택한 뚜껑이 안 보인 프레임(검출 누락)은 정보 없음이다. 관측 사이 공백이 max_gap_s를 넘으면 연속 유지가 아니다.

    1.3 s 관측 중 0.5 s 뒤 `missing` 프레임 누락: 관측 공백 0.1 s(≤ 0.2 s) / 0.3 s(> 0.2 s, 앞뒤 구간 각 0.5 s).
    """
    frames = _frames(lambda t: SETTLED, n=40)
    frames = [FrameObs(f.stamp_s, ()) if 16 <= k < 16 + missing else f for k, f in enumerate(frames)]
    v = verify(frames, CAP, VerifyParams(hold_time_s=1.0, max_gap_s=0.2))
    assert v.verdict == expected
    assert v.verdict != FAILURE


@pytest.mark.level('U')
@pytest.mark.req('REQ-VIS-009')
@pytest.mark.tc('TC-VIS-009-01')
def test_no_frames_and_constants():
    assert (SUCCESS, FAILURE, UNKNOWN) == (0, 1, 2)          # VerifyDrop.Response 상수(SPEC-010 3.3절)
    v = verify([], CAP, P)
    assert (v.verdict, v.reason, v.stone_inside, v.stone_offset_valid) == (UNKNOWN, 'NOT_OBSERVED', False, False)
    with pytest.raises(ValueError):
        verify([], CAP, VerifyParams(hold_time_s=0.0))
