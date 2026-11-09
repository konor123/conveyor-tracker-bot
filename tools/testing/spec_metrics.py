"""Independent expected keys transcribed from SPEC-100 §4.4; no production imports."""

EXPECTED_METRIC_KEYS = {
    'PRECHECK': frozenset('nodes_active heartbeat_age_max_s calibration_valid robot_ready protective_stop gripper_alive gripper_response_s gripper_error_bits gripper_open belt_steady belt_speed_mps'.split()),
    'DETECT_STONE': frozenset('stone_count confidence position_valid window_std_m clearance_m'.split()),
    'PICK_STONE': frozenset('grip_width_m grip_current_ma stone_on_mat robot_error gripper_error'.split()),
    'MOVE_STANDBY': frozenset('pose_error_m holding'.split()),
    'WAIT_CAP': frozenset('track_observed observation_count speed_std_mps belt_steady stone_inside stone_inside_valid t_available_s t_required_s time_margin_s'.split()),
    'PLAN_INTERCEPT': frozenset('plan_ok epoch_match time_margin_s position_std_m valid_for_s'.split()),
    'APPROACH': frozenset('plan_valid pose_error_m'.split()),
    'SYNCHRONIZE': frozenset('elapsed_s hold_s e_xy_m v_rel_mps'.split()),
    'SYNCHRONIZE_PREDICTED': frozenset('entry_time_error_s v_rel_mps'.split()),
    'DESCEND': frozenset('sync_ok rim_clearance_m height_error_m'.split()),
    'DESCEND_STATIC': frozenset('target_error_m rim_clearance_m height_error_m'.split()),
    'RELEASE': frozenset('open_width_m sync_held'.split()),
    'RETREAT': frozenset('tcp_z_m outside_workspace stone_in_gripper'.split()),
    'VERIFY': frozenset('responded verdict'.split()),
}
