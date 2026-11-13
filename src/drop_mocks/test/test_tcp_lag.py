"""mock_robot_controller TCP 1차 지연 모델(SPEC-700 4.5절, REQ-MCK-009, TEST_SPEC.md TC-MCK-009-01)."""
import math

import pytest

from drop_mocks.tcp_lag import DEFAULT_TCP_LAG_S, PlanRef, TcpLag, make_lag

V = (0.02, 0.0, 0.0)


def ref(release_t=10.0):
    return PlanRef(release_t, (0.50, -0.25, 0.1756), V)


@pytest.mark.req('REQ-MCK-009')
@pytest.mark.tc('TC-MCK-009-01')
@pytest.mark.level('U')
def test_ramp_reference_lags_by_speed_times_time_constant():
    """등속 기준(계획 목표 궤적)을 따르는 TCP는 정상 상태에서 v·τ만큼 뒤처지고 상대 속도는 0으로 수렴한다."""
    r = ref()
    lag = TcpLag(DEFAULT_TCP_LAG_S, r.at(7.0), 7.0)
    for k in range(1, 101):                              # 10 ms 간격 1 s(≈ 34τ)
        lag.advance(7.0 + 0.01 * k, r)
    e_xy, v_rel = lag.errors(r)
    assert e_xy == pytest.approx(0.02 * DEFAULT_TCP_LAG_S, abs=1e-9)     # 0.58 mm
    assert v_rel == pytest.approx(0.0, abs=1e-9)
    assert lag.pos[0] < r.at(lag.t)[0]                   # 뒤처짐은 진행 방향 반대쪽


@pytest.mark.req('REQ-MCK-009')
@pytest.mark.tc('TC-MCK-009-01')
@pytest.mark.level('U')
def test_step_response_and_partition_independence():
    """10 mm 떨어진 정지 기준: τ 뒤 오차 10·e⁻¹ mm. 같은 구간을 1번에 진행하든 100번에 나눠 진행하든 같다(정확 이산화)."""
    still = PlanRef(0.0, (0.5, 0.0, 0.2), (0.0, 0.0, 0.0))
    one = TcpLag(DEFAULT_TCP_LAG_S, (0.49, 0.0, 0.2), 0.0)
    one.advance(DEFAULT_TCP_LAG_S, still)
    many = TcpLag(DEFAULT_TCP_LAG_S, (0.49, 0.0, 0.2), 0.0)
    for k in range(1, 101):
        many.advance(DEFAULT_TCP_LAG_S * k / 100, still)
    assert one.errors(still)[0] == pytest.approx(0.01 * math.exp(-1.0), abs=1e-12)
    assert many.pos == pytest.approx(one.pos, abs=1e-12)
    assert one.velocity(still)[0] == pytest.approx(0.01 * math.exp(-1.0) / DEFAULT_TCP_LAG_S, rel=1e-9)
    one.advance(0.0, still)                              # 과거 시각은 무시
    assert one.t == pytest.approx(DEFAULT_TCP_LAG_S)


@pytest.mark.req('REQ-MCK-009')
@pytest.mark.tc('TC-MCK-009-01')
@pytest.mark.level('U')
def test_lag_value_and_disable():
    """기본 시정수 29 ms(2026-11-07 OFF-2 speedl 100 Hz 참조 부하 지령→위치 지연), 0이면 모델을 쓰지 않는다."""
    assert DEFAULT_TCP_LAG_S == 0.029
    assert make_lag(0.0, (0, 0, 0), 0.0) is None
    assert isinstance(make_lag(0.029, (0, 0, 0), 0.0), TcpLag)
    for bad in (-0.01, math.nan, math.inf):
        with pytest.raises(ValueError):
            make_lag(bad, (0, 0, 0), 0.0)
