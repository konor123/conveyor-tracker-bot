"""단계별 정상 측정값(판정 OK). 단계 판정·실행기 시험이 함께 쓴다."""
from orchestrator import step_judges as sj

MM = 1e-3
E_XY_MAX = 2.75 * MM

OK_PRE = sj.PrecheckMeas(nodes_active=True, heartbeat_age_max_s=0.1, calibration_valid=True,
                         robot_ready=True, protective_stop=False, gripper_alive=True,
                         gripper_response_s=0.1, gripper_error_bits=0, gripper_open=True,
                         belt_steady=True, belt_speed_mps=0.02)
OK_DET = sj.DetectMeas(stone_count=1, confidence=0.95, position_valid=True, window_std_m=0.2 * MM,
                       clearance_m=30 * MM)
OK_PICK = sj.PickMeas(grip_width_m=17.5 * MM, grip_current_ma=150, stone_on_mat=False, robot_error=False,
                      gripper_error=False)
OK_STB = sj.StandbyMeas(pose_error_m=0.5 * MM, holding=True)
OK_WAIT = sj.WaitCapMeas(track_observed=True, observation_count=40, speed_std_mps=0.5 * MM,
                         belt_steady=True, stone_inside=False, stone_inside_valid=True,
                         t_available_s=10.0, t_required_s=4.0,
                         budget_components={'transit': 1.5, 'sync': 0.5, 'descend': 1.0,
                                            'release': 0.5, 'clear': 0.5})
OK_PLAN = sj.PlanMeas(plan_ok=True, epoch_match=True, time_margin_s=5.0, position_std_m=0.2 * MM,
                      valid_until=105.0, now=100.0)
OK_APP = sj.ApproachMeas(plan_valid=True, pose_error_m=0.5 * MM)
OK_ENTRY = sj.EntryMeas(entry_time_error_s=0.01, v_rel_mps=1 * MM)
OK_DESC = sj.DescendMeas(sync_ok=True, rim_clearance_m=2.5 * MM, height_error_m=0.1 * MM)
OK_STATIC = sj.StaticPlaceMeas(target_error_m=0.5 * MM, rim_clearance_m=2.5 * MM, height_error_m=0.1 * MM)
OK_REL = sj.ReleaseMeas(open_width_m=22 * MM, sync_held=True)
OK_RET = sj.RetreatMeas(tcp_z_m=0.15, outside_workspace=True, stone_in_gripper=False)
OK_VER = sj.VerifyMeas(responded=True, verdict=0)



def sync_samples(t0=200.0, t_end=0.3, rate_hz=100, bad_at=()):
    """SYNCHRONIZE feedback 표본. bad_at(시작 기준 s)에서는 e_xy가 기준을 넘는다."""
    n = int(round(t_end * rate_hz))
    out = []
    for i in range(n + 1):
        bad = any(abs(i / rate_hz - b) < 1e-6 for b in bad_at)
        out.append(sj.SyncSample(t=t0 + i / rate_hz, e_xy_m=E_XY_MAX * (1.5 if bad else 0.5),
                                 v_rel_mps=1 * MM))
    return out
