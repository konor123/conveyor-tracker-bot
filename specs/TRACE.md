# TRACE — REQ ↔ TC ↔ 테스트

`tools/spec/trace.py`가 생성한다. 손으로 고치지 않는다.

- REQ 122개 중 TC 연결 122개
- pytest 대상 TC 205개 중 테스트 연결 192개

| TC | 수준 | REQ | 테스트 |
|---|---|---|---|
| TC-ABT-001-01 | U | REQ-ABT-001 | `src/orchestrator/test/test_abort_policy.py::test_decide_matches_table`<br>`src/orchestrator/test/test_abort_policy.py::test_every_hard_fault_is_l4_even_after_release`<br>`src/orchestrator/test/test_abort_policy.py::test_detect_and_pick_failures_are_l2_without_retry`<br>`src/orchestrator/test/test_safety_supervisor.py::test_supervisor_monitor_detects_heartbeat_error_code` |
| TC-ABT-001-02 | U | REQ-ABT-001 | `src/orchestrator/test/test_fault_transitions.py::test_all_state_event_transitions` |
| TC-ABT-001-03 | U | REQ-ABT-001 | `src/orchestrator/test/test_cycle_runner.py::test_released_fault_executes_cleanup`<br>`src/orchestrator/test/test_cycle_runner.py::test_retreat_failure_overrides_latched_soft_fault` |
| TC-ABT-002-01 | U | REQ-ABT-002 | `src/orchestrator/test/test_abort_policy.py::test_escalate_keeps_higher`<br>`src/orchestrator/test/test_cycle_runner.py::test_released_soft_fault_escalates_to_hard_fault` |
| TC-ABT-003-01 | U | REQ-ABT-003 | `src/orchestrator/test/test_abort_policy.py::test_l2_path_timeout` |
| TC-ABT-003-02 | U | REQ-ABT-003 | `src/orchestrator/test/test_abort_policy.py::test_l3_path_timeout`<br>`src/orchestrator/test/test_abort_policy.py::test_path_done_stops_path_timer`<br>`src/orchestrator/test/test_cycle_runner.py::test_abort_path_hang_escalates_to_stop` |
| TC-ABT-003-03 | U | REQ-ABT-003 | `src/orchestrator/test/test_node_failure_regressions.py::test_wait_cancel_and_steady_guard`<br>`src/orchestrator/test/test_node_failure_regressions.py::test_wait_frozen_clock_shutdown_and_cleanup`<br>`src/orchestrator/test/test_node_failure_regressions.py::test_wait_backward_ros_clock_uses_scaled_steady_budget` |
| TC-ABT-004-01 | U | REQ-ABT-004 | `src/orchestrator/test/test_abort_policy.py::test_cancel_without_response_escalates_one_level`<br>`src/orchestrator/test/test_abort_policy.py::test_cancel_ack_prevents_escalation`<br>`src/orchestrator/test/test_cycle_runner.py::test_pick_hang_cancel_escalation`<br>`src/orchestrator/test/test_cycle_runner.py::test_cancel_escalation_timeline` |
| TC-ABT-004-02 | U | REQ-ABT-004 | `src/orchestrator/test/test_node_failure_regressions.py::test_pending_acceptance_is_not_cancel_ack_and_late_handle_is_cancelled`<br>`src/orchestrator/test/test_node_failure_regressions.py::test_late_acceptance_callback_cancels_without_another_poll`<br>`src/orchestrator/test/test_node_failure_regressions.py::test_failed_terminal_future_is_not_confirmation` |
| TC-ABT-005-01 | U | REQ-ABT-005 | `src/orchestrator/test/test_abort_policy.py::test_user_cancel_while_holding_is_l3` |
| TC-ABT-006-01 | U | REQ-ABT-006 | `src/orchestrator/test/test_deadlines.py::test_effective_deadline_is_minimum` |
| TC-ABT-007-01 | U | REQ-ABT-007 | `src/orchestrator/test/test_deadlines.py::test_derive_step_timeout`<br>`src/orchestrator/test/test_deadlines.py::test_derive_step_timeout_floor_is_p95_plus_half` |
| TC-ABT-008-01 | U | REQ-ABT-008 | `src/orchestrator/test/test_watchdogs.py::test_cycle_ends_when_ros_clock_frozen`<br>`src/orchestrator/test/test_watchdogs.py::test_cycle_timer_uses_ros_deadline_when_clock_runs` |
| TC-ABT-009-01 | U | REQ-ABT-009 | `src/orchestrator/test/test_watchdogs.py::test_heartbeat_ignores_ros_jump`<br>`src/orchestrator/test/test_watchdogs.py::test_heartbeat_detected_with_frozen_ros` |
| TC-ABT-009-02 | C | REQ-ABT-009 | `src/drop_mocks/test/test_mock_nodes.py::test_heartbeat_and_status_keep_publishing_when_ros_time_frozen`<br>`src/orchestrator/test/test_watch_timers_ros.py::test_orchestrator_watch_timer_uses_steady_clock`<br>`src/orchestrator/test/test_watch_timers_ros.py::test_supervisor_detects_heartbeat_loss_when_ros_time_frozen`<br>`src/robot_control/test/test_robot_controller_heartbeat.py::test_heartbeat_and_status_keep_publishing_when_ros_time_frozen` |
| TC-ABT-009-03 | U | REQ-ABT-009 | `src/orchestrator/test/test_node_failure_regressions.py::test_supervisor_retries_until_confirmed_with_one_inflight`<br>`src/orchestrator/test/test_node_failure_regressions.py::test_supervisor_retries_failed_response` |
| TC-ABT-010-01 | U | REQ-ABT-010 | `src/orchestrator/test/test_safety_supervisor.py::test_supervisor_monitor_detects_protective_stop_and_heartbeat_loss`<br>`src/orchestrator/test/test_watchdogs.py::test_heartbeat_boundary`<br>`src/orchestrator/test/test_watchdogs.py::test_heartbeat_unknown_node_is_lost`<br>`src/orchestrator/test/test_watchdogs.py::test_heartbeat_loss_levels` |
| TC-ABT-011-01 | U | REQ-ABT-011 | `src/orchestrator/test/test_watchdogs.py::test_observation_freshness` |
| TC-ABT-011-02 | C | REQ-ABT-011 | — |
| TC-ABT-012-01 | U | REQ-ABT-012 | `src/orchestrator/test/test_watchdogs.py::test_clock_regression` |
| TC-ABT-012-02 | C | REQ-ABT-012 | — |
| TC-ABT-013-01 | U | REQ-ABT-013 | `src/orchestrator/test/test_abort_policy.py::test_belt_unsteady` |
| TC-ABT-013-02 | C | REQ-ABT-013 | — |
| TC-ABT-014-01 | U | REQ-ABT-014 | `src/orchestrator/test/test_watchdogs.py::test_gripper_monitor`<br>`src/orchestrator/test/test_watchdogs.py::test_gripper_alive_counter_stall`<br>`src/orchestrator/test/test_watchdogs.py::test_gripper_fault_levels` |
| TC-ABT-014-02 | C | REQ-ABT-014 | — |
| TC-ABT-015-01 | U | REQ-ABT-015 | `src/orchestrator/test/test_abort_policy.py::test_fault_event_sequences`<br>`src/orchestrator/test/test_abort_policy.py::test_fault_list_covers_plan_7_3` |
| TC-ABT-016-01 | U | REQ-ABT-016 | `src/orchestrator/test/test_abort_policy.py::test_wait_operator_has_no_timeout` |
| TC-BEN-001-01 | B | REQ-BEN-001(SPEC 없음) | 절차 시험 |
| TC-BEN-002-01 | B | REQ-BEN-002(SPEC 없음) | 절차 시험 |
| TC-BEN-003-01 | B | REQ-BEN-003(SPEC 없음) | 절차 시험 |
| TC-BEN-004-01 | B | REQ-BEN-004(SPEC 없음) | 절차 시험 |
| TC-BEN-005-01 | B | REQ-BEN-005(SPEC 없음) | 절차 시험 |
| TC-BEN-006-01 | B | REQ-BEN-006(SPEC 없음) | 절차 시험 |
| TC-BEN-007-01 | B | REQ-BEN-007(SPEC 없음) | 절차 시험 |
| TC-BEN-008-01 | B | REQ-BEN-008(SPEC 없음) | 절차 시험 |
| TC-BEN-010-01 | B | REQ-BEN-010(SPEC 없음) | 절차 시험 |
| TC-BEN-011-01 | B | REQ-BEN-011(SPEC 없음) | 절차 시험 |
| TC-BEN-012-01 | B | REQ-BEN-012(SPEC 없음) | 절차 시험 |
| TC-CAL-001-01 | U | REQ-CAL-001 | `src/calibration/test/test_plane_calibration.py::test_fit_plane_with_outliers`<br>`src/calibration/test/test_plane_calibration.py::test_fit_plane_rejects_collinear_points` |
| TC-CAL-001-91 | U | REQ-CAL-001 | `src/calibration/test/test_plane_calibration.py::test_fit_plane_and_direction_use_thin_svd` |
| TC-CAL-002-01 | U | REQ-CAL-002 | `src/calibration/test/test_plane_calibration.py::test_fit_direction_straight_track`<br>`src/calibration/test/test_plane_calibration.py::test_fit_direction_projects_onto_plane` |
| TC-CAL-002-02 | U | REQ-CAL-002 | `src/calibration/test/test_board_session.py::test_belt_direction_from_board_track` |
| TC-CAL-003-01 | U | REQ-CAL-003 | `src/calibration/test/test_eye_to_hand_calibration.py::test_rigid_known_transform` |
| TC-CAL-003-02 | U | REQ-CAL-003 | `src/calibration/test/test_eye_to_hand_calibration.py::test_rigid_rejects_collinear`<br>`src/calibration/test/test_eye_to_hand_calibration.py::test_pnp_rejects_collinear_and_too_few` |
| TC-CAL-003-03 | U | REQ-CAL-003 | `src/calibration/test/test_eye_to_hand_calibration.py::test_pnp_touch_points` |
| TC-CAL-003-04 | U | REQ-CAL-003 | `src/calibration/test/test_board_session.py::test_board_session_meets_verify_gate`<br>`src/calibration/test/test_board_session.py::test_bad_touch_is_flagged`<br>`src/calibration/test/test_cli.py::test_calibrate_camera_end_to_end` |
| TC-CAL-004-01 | U | REQ-CAL-004 | `src/calibration/test/test_tcp_offset.py::test_tcp_offset_full_3d`<br>`src/calibration/test/test_tcp_offset.py::test_tcp_offset_xy_only_vertical_approach` |
| TC-CAL-005-01 | U | REQ-CAL-005 | `src/calibration/test/test_calibration_file.py::test_roundtrip`<br>`src/calibration/test/test_calibration_file.py::test_missing_required_key_fails`<br>`src/calibration/test/test_calibration_file.py::test_invalid_values_fail` |
| TC-CAL-005-02 | C | REQ-CAL-005 | — |
| TC-CAL-006-01 | U | REQ-CAL-006 | `src/calibration/test/test_eye_to_hand_calibration.py::test_pixel_to_plane_roundtrip`<br>`src/calibration/test/test_eye_to_hand_calibration.py::test_verify_xy_stats_and_gate` |
| TC-CAL-007-01 | U | REQ-CAL-007 | `src/calibration/test/test_charuco.py::test_corners_and_pose_on_rendered_board`<br>`src/calibration/test/test_charuco.py::test_board_points_match_print_layout` |
| TC-CAL-008-01 | U | REQ-CAL-008 | `src/calibration/test/test_camera_frames.py::test_base_to_camera_link_composes_back` |
| TC-CAL-009-01 | U | REQ-CAL-009 | `src/calibration/test/test_camera_frames.py::test_camera_info_matches_profile`<br>`src/calibration/test/test_camera_frames.py::test_camera_info_mismatch`<br>`src/calibration/test/test_camera_frames.py::test_camera_info_focal_drift_can_be_ignored` |
| TC-CAL-010-01 | U | REQ-CAL-010 | `src/calibration/test/test_contact_verification.py::test_contact_verification_statistics_passing`<br>`src/calibration/test/test_contact_verification.py::test_contact_verification_threshold_exceeded_fails`<br>`src/calibration/test/test_contact_verification.py::test_contact_verification_pointer_tcp_correction` |
| TC-CAL-010-02 | U | REQ-CAL-010 | `src/calibration/test/test_contact_verification.py::test_refuse_insufficient_points`<br>`src/calibration/test/test_contact_verification.py::test_refuse_duplicate_points`<br>`src/calibration/test/test_contact_verification.py::test_refuse_fit_points_reuse`<br>`src/calibration/test/test_contact_verification.py::test_refuse_nonfinite_inputs`<br>`src/calibration/test/test_contact_verification.py::test_refuse_malformed_transform`<br>`src/calibration/test/test_contact_verification.py::test_refuse_invalid_units`<br>`src/calibration/test/test_contact_verification.py::test_refuse_unknown_pointer_tcp`<br>`src/calibration/test/test_contact_verification.py::test_refuse_missing_provenance_and_mismatches`<br>`src/calibration/test/test_contact_verification.py::test_refuse_synthetic_data_for_physical_verification` |
| TC-CAL-010-03 | U | REQ-CAL-010 | `src/calibration/test/test_contact_verification.py::test_offline_template_generation_and_roundtrip` |
| TC-EXP-CMP-01 | E | — | 절차 시험 |
| TC-EXP-G2-01 | E | — | 절차 시험 |
| TC-EXP-G3-01 | E | — | 절차 시험 |
| TC-EXP-G4-01 | E | — | 절차 시험 |
| TC-EXP-G5-01 | E | — | 절차 시험 |
| TC-EXP-G6-01 | E | — | 절차 시험 |
| TC-EXP-G7-01 | E | — | 절차 시험 |
| TC-GRP-001-01 | U | REQ-GRP-001 | `src/robot_control/test/test_gripper_adapter.py::test_pulse_width_roundtrip_within_tolerance` |
| TC-GRP-001-02 | U | REQ-GRP-001 | `src/robot_control/test/test_gripper_adapter.py::test_table_rejects_nonmonotonic_and_out_of_range` |
| TC-GRP-002-01 | U | REQ-GRP-002 | `src/robot_control/test/test_gripper_adapter.py::test_startup_checks_model_firmware_and_error` |
| TC-GRP-003-01 | U | REQ-GRP-003 | `src/robot_control/test/test_gripper_adapter.py::test_protocol_separated_address_tables` |
| TC-GRP-003-02 | U | REQ-GRP-003 | `src/robot_control/test/test_gripper_adapter.py::test_eeprom_write_requires_torque_off` |
| TC-GRP-004-01 | U | REQ-GRP-004 | `src/robot_control/test/test_gripper_adapter.py::test_close_order_and_current_limit`<br>`src/robot_control/test/test_gripper_adapter.py::test_no_inverse_convention_and_hardware_range_beyond_cli_envelope`<br>`src/robot_control/test/test_gripper_adapter.py::test_transport_write_failure_closes_as_fault`<br>`tools/bench/test_gripper_once.py::test_reject_outside_approved_envelope`<br>`tools/bench/test_gripper_once.py::test_site_endpoint_and_close_default`<br>`tools/bench/test_gripper_once.py::test_all_commands_default_100_and_reject_151`<br>`tools/bench/test_gripper_once.py::test_legacy_transports_reject_101_before_io` |
| TC-GRP-005-01 | U | REQ-GRP-005 | `src/robot_control/test/test_gripper_adapter.py::test_partial_open_uses_measured_table` |
| TC-GRP-006-01 | U | REQ-GRP-006 | `src/robot_control/test/test_gripper_adapter.py::test_holding_boundaries_three_points` |
| TC-GRP-007-01 | U | REQ-GRP-007 | `src/robot_control/test/test_gripper_adapter.py::test_response_timeout_boundary_and_no_response` |
| TC-GRP-008-01 | U | REQ-GRP-008 | `src/robot_control/test/test_gripper_adapter.py::test_error_bits_reason_contains_name` |
| TC-GRP-009-01 | U | REQ-GRP-009 | `src/robot_control/test/test_gripper_adapter.py::test_timing_chain_is_monotonic` |
| TC-GRP-010-01 | U | REQ-GRP-010 | `src/robot_control/test/test_gripper_adapter.py::test_transport_import_only_in_adapter_and_tests` |
| TC-GRP-011-01 | U | REQ-GRP-011 | `src/robot_control/test/test_gripper_adapter.py::test_modbus_frames_match_standard_vectors`<br>`src/robot_control/test/test_gripper_adapter.py::test_modbus_response_rejects_bad_crc_and_length`<br>`tools/bench/test_gripper_once.py::test_validated_wire_frames` |
| TC-GRP-012-01 | U | REQ-GRP-012 | `src/robot_control/test/test_gripper_adapter.py::test_register_codec_roundtrip_and_range` |
| TC-GRP-012-02 | U | REQ-GRP-012 | `src/robot_control/test/test_gripper_adapter.py::test_prewrite_then_release_triggers_once` |
| TC-GRP-013-01 | U | REQ-GRP-013 | `src/robot_control/test/test_gripper_adapter.py::test_loop_counter_stall_boundary` |
| TC-GRP-013-02 | U | REQ-GRP-013 | `src/robot_control/test/test_gripper_adapter.py::test_loop_start_requires_standby` |
| TC-GRP-014-01 | U | REQ-GRP-014 | `src/robot_control/test/test_gripper_session.py::test_session_starts_once_and_rejects_use_before_start` |
| TC-GRP-014-02 | U | REQ-GRP-014 | `src/robot_control/test/test_gripper_session.py::test_ensure_open_skips_when_already_at_width` |
| TC-GRP-014-03 | U | REQ-GRP-014 | `src/robot_control/test/test_gripper_session.py::test_arm_then_fire_sends_only_the_trigger`<br>`src/robot_control/test/test_gripper_session.py::test_grasp_invalidates_an_earlier_arm` |
| TC-GRP-015-01 | U | REQ-GRP-015 | `src/robot_control/test/test_resident_transport.py::test_grasp_is_judged_by_stall_not_by_reaching_the_end` |
| TC-GRP-016-01 | U | REQ-GRP-016 | `src/robot_control/test/test_resident_transport.py::test_transport_maps_registers_and_starts_loop_once`<br>`src/robot_control/test/test_resident_transport.py::test_loop_init_failure_is_a_fault_and_drl_text_matches_registers`<br>`src/robot_control/test/test_resident_transport.py::test_loop_write_failure_is_a_fault_for_that_command_only`<br>`src/robot_control/test/test_resident_transport.py::test_success_waits_for_the_loop_to_execute_the_command` |
| TC-GRP-016-02 | U | REQ-GRP-016 | `src/robot_control/test/test_resident_transport.py::test_armed_trigger_delay_is_spin_plus_one_write` |
| TC-GRP-016-03 | U | REQ-GRP-016 | `src/robot_control/test/test_resident_transport.py::test_fake_static_cycle_spends_far_less_time_on_the_gripper` |
| TC-GRP-016-04 | U | REQ-GRP-016 | `src/robot_control/test/test_resident_transport.py::test_timeouts_shorter_than_the_loop_sample_period_are_rejected` |
| TC-IF-001-01 | U | REQ-IF-001 | `tools/spec/test_interfaces.py::test_interface_snapshots_match` |
| TC-IF-002-01 | U | REQ-IF-002 | `tools/spec/test_interfaces.py::test_common_request_fields_order` |
| TC-IF-003-01 | U | REQ-IF-003 | `tools/spec/test_interfaces.py::test_interface_constants_match_spec` |
| TC-IF-004-01 | C | REQ-IF-004 | `src/orchestrator/test/test_watch_timers_ros.py::test_actual_orchestrator_publishers_match_qos`<br>`tools/spec/test_qos_contract.py::test_qos_checker_accepts_fixture_publishers` |
| TC-IF-004-02 | C | REQ-IF-004 | `tools/spec/test_qos_contract.py::test_qos_mismatch_best_effort_pub_reliable_sub_detected` |
| TC-IF-005-01 | C | REQ-IF-005 | `src/drop_mocks/test/contract/test_mock_robot_controller_contract.py::<module>` |
| TC-IF-006-01 | C | REQ-IF-006 | `src/drop_mocks/test/test_mock_nodes.py::test_mock_nodes_have_no_qos_event_handlers`<br>`src/orchestrator/test/test_watch_timers_ros.py::test_orchestrator_nodes_have_no_qos_event_handlers`<br>`src/robot_control/test/test_robot_controller_heartbeat.py::test_robot_controller_has_no_qos_event_handlers` |
| TC-INT-001-01 | I0 | REQ-INT-001(SPEC 없음) | `src/drop_bringup/test/test_i0_integration.py::test_i0_nominal_20_cycles_and_records` |
| TC-INT-002-01 | I0 | REQ-INT-002(SPEC 없음) | `src/drop_bringup/test/test_i0_integration.py::test_i0_robot_hang_escalates_and_stops` |
| TC-INT-003-01 | I0 | REQ-INT-003(SPEC 없음) | `src/drop_bringup/test/test_i0_integration.py::test_i0_protective_stop_waits_for_operator_resume`<br>`src/orchestrator/test/test_safety_supervisor.py::test_safety_supervisor_node_calls_stop_only` |
| TC-INT-004-01 | I0 | REQ-INT-004(SPEC 없음) | `src/drop_bringup/test/test_i0_integration.py::test_i0_heartbeat_loss_all_roles` |
| TC-INT-005-01 | I0 | REQ-INT-005(SPEC 없음) | `src/drop_bringup/test/test_i0_integration.py::test_i0_time_insufficient_pass_skip_and_not_attempted_record` |
| TC-INT-006-01 | I0 | REQ-INT-006(SPEC 없음) | `src/drop_bringup/test/test_i0_integration.py::test_i0_inactive_node_rejected_at_precheck` |
| TC-INT-007-01 | I1 | REQ-INT-007(SPEC 없음) | — |
| TC-INT-020-01 | I3 | REQ-INT-020(SPEC 없음) | 절차 시험 |
| TC-INT-021-01 | I4 | REQ-INT-021(SPEC 없음) | 절차 시험 |
| TC-MCK-001-01 | U | REQ-MCK-001 | `src/drop_mocks/test/test_scenario_engine.py::test_same_scenario_and_seed_give_same_output` |
| TC-MCK-002-01 | C | REQ-MCK-002 | `src/drop_mocks/test/test_mock_nodes.py::test_role_split_processes_see_same_world` |
| TC-MCK-003-01 | C | REQ-MCK-003 | `src/drop_mocks/test/contract/test_mock_world_tracker_contract.py::<module>`<br>`src/drop_mocks/test/contract/test_mock_world_vision_contract.py::<module>` |
| TC-MCK-004-01 | C | REQ-MCK-004 | `src/drop_mocks/test/contract/test_mock_robot_controller_contract.py::<module>` |
| TC-MCK-005-01 | C | REQ-MCK-005 | `src/drop_mocks/test/test_frame_timing.py::test_latest_frame_after_delayed_wake_preserves_event_phase`<br>`src/drop_mocks/test/test_frame_timing.py::test_phase_oracle_rejects_early_late_and_stuck_effects`<br>`src/drop_mocks/test/test_mock_nodes.py::test_frame_callback_delayed_wake_skips_to_latest`<br>`src/drop_mocks/test/test_mock_nodes.py::test_controlled_clock_publishes_first_event_frame`<br>`src/drop_mocks/test/test_mock_nodes.py::test_injected_events_appear_at_scheduled_time` |
| TC-MCK-005-02 | U | REQ-MCK-005 | `src/drop_mocks/test/test_frame_timing.py::test_first_scheduled_frame_straddles_event_exactly`<br>`src/drop_mocks/test/test_scenario_engine.py::test_events_take_effect_at_scheduled_frame` |
| TC-MCK-006-01 | U | REQ-MCK-006 | `src/drop_mocks/test/test_scenario_engine.py::test_every_value_is_assumed_or_measured`<br>`src/drop_mocks/test/test_scenario_engine.py::test_unclassified_or_double_classified_value_is_rejected` |
| TC-MCK-006-02 | I0 | REQ-MCK-006 | `src/drop_bringup/test/test_i0_integration.py::test_i0_nominal_20_cycles_and_records` |
| TC-MCK-007-01 | I0 | REQ-MCK-007 | `src/drop_bringup/test/test_mock_bringup.py::test_mock_refuses_when_real_provider_is_running` |
| TC-MCK-007-02 | C | REQ-MCK-007 | `src/drop_mocks/test/test_mock_nodes.py::test_mock_exits_when_real_provider_appears_after_start` |
| TC-ORC-001-01 | U | REQ-ORC-001 | `src/orchestrator/test/test_cycle_runner.py::test_synchronized_nominal_sequence` |
| TC-ORC-001-02 | U | REQ-ORC-001 | `src/orchestrator/test/test_cycle_runner.py::test_static_skips_synchronize_and_detects_cap`<br>`src/orchestrator/test/test_step_judges.py::test_static_place` |
| TC-ORC-001-03 | U | REQ-ORC-001 | `src/orchestrator/test/test_cycle_runner.py::test_predicted_uses_entry_judgement`<br>`src/orchestrator/test/test_step_judges.py::test_predicted_entry` |
| TC-ORC-002-01 | U | REQ-ORC-002 | `src/orchestrator/test/test_cycle_runner.py::test_no_next_skill_after_failure`<br>`src/orchestrator/test/test_cycle_runner.py::test_retreat_failure_is_protective_stop` |
| TC-ORC-002-02 | U | REQ-ORC-002 | `src/orchestrator/test/test_cycle_runner.py::test_release_failure_continues_to_retreat_and_verify` |
| TC-ORC-003-01 | U | REQ-ORC-003 | `src/orchestrator/test/test_step_judges.py::test_precheck_heartbeat_age`<br>`src/orchestrator/test/test_step_judges.py::test_precheck_other_reasons`<br>`src/orchestrator/test/test_step_judges.py::test_params_load_from_config` |
| TC-ORC-003-02 | U | REQ-ORC-003 | `src/orchestrator/test/test_step_judges.py::test_precheck_gripper` |
| TC-ORC-003-03 | U | REQ-ORC-003 | `src/orchestrator/test/test_step_judges.py::test_detect_stone` |
| TC-ORC-003-04 | U | REQ-ORC-003 | `src/orchestrator/test/test_step_judges.py::test_pick_width` |
| TC-ORC-003-05 | U | REQ-ORC-003 | `src/orchestrator/test/test_step_judges.py::test_pick_current_and_mat`<br>`src/orchestrator/test/test_step_judges.py::test_pick_current_at_minimum_is_ok` |
| TC-ORC-003-06 | U | REQ-ORC-003 | `src/orchestrator/test/test_step_judges.py::test_move_standby`<br>`src/orchestrator/test/test_step_judges.py::test_approach` |
| TC-ORC-003-07 | U | REQ-ORC-003 | `src/orchestrator/test/test_step_judges.py::test_wait_cap_track_ready` |
| TC-ORC-003-08 | U | REQ-ORC-003 | `src/orchestrator/test/test_step_judges.py::test_wait_cap_empty_and_budget` |
| TC-ORC-003-09 | U | REQ-ORC-003 | `src/orchestrator/test/test_step_judges.py::test_plan_intercept` |
| TC-ORC-003-10 | U | REQ-ORC-003 | `src/orchestrator/test/test_step_judges.py::test_synchronize_restarts_count_after_violation`<br>`src/orchestrator/test/test_step_judges.py::test_synchronize_ok_without_violation`<br>`src/orchestrator/test/test_step_judges.py::test_synchronize_timeout`<br>`src/orchestrator/test/test_step_judges.py::test_synchronize_velocity_gate` |
| TC-ORC-003-11 | U | REQ-ORC-003 | `src/orchestrator/test/test_step_judges.py::test_descend` |
| TC-ORC-003-12 | U | REQ-ORC-003 | `src/orchestrator/test/test_step_judges.py::test_release_open_width`<br>`src/orchestrator/test/test_step_judges.py::test_release_sync_lost` |
| TC-ORC-003-13 | U | REQ-ORC-003 | `src/orchestrator/test/test_step_judges.py::test_retreat_failure_is_l4` |
| TC-ORC-003-14 | U | REQ-ORC-003 | `src/orchestrator/test/test_cycle_runner.py::test_verify_deadline_and_final_outcome`<br>`src/orchestrator/test/test_step_judges.py::test_verify_no_response_is_unknown` |
| TC-ORC-004-01 | U | REQ-ORC-004 | `src/orchestrator/test/test_step_judges.py::test_metric_keys_match_spec`<br>`src/orchestrator/test/test_step_judges.py::test_metric_keys_present_on_failure_too` |
| TC-ORC-005-01 | U | REQ-ORC-005 | `src/orchestrator/test/test_cycle_runner.py::test_stale_detection_blocks_pick`<br>`src/orchestrator/test/test_cycle_runner.py::test_detection_at_age_limit_allows_pick` |
| TC-ORC-006-01 | U | REQ-ORC-006 | `src/orchestrator/test/test_cycle_runner.py::test_detect_failure_is_l2_without_retry` |
| TC-ORC-006-02 | U | REQ-ORC-006 | `src/orchestrator/test/test_cycle_runner.py::test_pick_failure_is_l2_without_regrasp_or_redetect` |
| TC-ORC-007-01 | U | REQ-ORC-007 | `src/orchestrator/test/test_cycle_runner.py::test_pass_skip_then_proceed`<br>`src/orchestrator/test/test_cycle_runner.py::test_wait_then_ok_within_pass` |
| TC-ORC-007-02 | U | REQ-ORC-007 | `src/orchestrator/test/test_cycle_runner.py::test_passes_exhausted_is_not_attempted` |
| TC-ORC-008-01 | U | REQ-ORC-008 | `src/orchestrator/test/test_cycle_runner.py::test_plan_failure_goes_to_next_pass` |
| TC-ORC-009-01 | U | REQ-ORC-009 | `src/orchestrator/test/test_cycle_runner.py::test_resume_requires_reason`<br>`src/orchestrator/test/test_cycle_runner.py::test_precheck_failure_returns_to_idle_without_wait_operator` |
| TC-ORC-010-01 | U | REQ-ORC-010 | `src/orchestrator/test/test_ros_skill_port.py::test_soft_fault_ends_wait_before_release`<br>`src/orchestrator/test/test_ros_skill_port.py::test_soft_fault_does_not_end_wait_after_release`<br>`src/orchestrator/test/test_ros_skill_port.py::test_hard_fault_ends_wait_after_release_and_overrides_soft`<br>`src/orchestrator/test/test_ros_skill_port.py::test_soft_fault_does_not_override_hard`<br>`src/orchestrator/test/test_ros_skill_port.py::test_reset_clears_release`<br>`src/orchestrator/test/test_ros_skill_port.py::test_concurrent_soft_and_hard_raise_keeps_hard`<br>`src/orchestrator/test/test_ros_skill_port.py::test_concurrent_consume_and_hard_raise_loses_nothing` |
| TC-ORC-010-02 | U | REQ-ORC-010 | `src/orchestrator/test/test_cycle_runner.py::test_cleanup_after_real_cycle_deadline`<br>`src/orchestrator/test/test_node_failure_regressions.py::test_ready_cleanup_result_after_original_deadline`<br>`src/orchestrator/test/test_node_failure_regressions.py::test_runner_cleanup_with_production_ros_port`<br>`src/orchestrator/test/test_node_failure_regressions.py::test_released_cleanup_frozen_clock_is_still_bounded`<br>`src/orchestrator/test/test_ros_skill_port.py::test_ready_response_after_original_cycle_deadline` |
| TC-ORC-011-01 | U | REQ-ORC-011 | `src/orchestrator/test/test_node_failure_regressions.py::test_stopped_skill_result_latches_hard_fault`<br>`src/orchestrator/test/test_node_failure_regressions.py::test_stopped_result_reported_by_phase_wait`<br>`src/orchestrator/test/test_node_failure_regressions.py::test_specific_cause_wins_over_generic_stop` |
| TC-PLN-001-01 | U | REQ-PLN-001 | `src/intercept_planner/test/test_planner.py::test_budget_formula` |
| TC-PLN-001-02 | U | REQ-PLN-001 | `src/intercept_planner/test/test_planner.py::test_budget_boundary` |
| TC-PLN-001-03 | U | REQ-PLN-001 | `src/intercept_planner/test/test_planner.py::test_belt_boundary` |
| TC-PLN-002-01 | U | REQ-PLN-002 | `src/intercept_planner/test/test_planner.py::test_prediction_state_time` |
| TC-PLN-003-01 | U | REQ-PLN-003 | `src/intercept_planner/test/test_planner.py::test_covariance` |
| TC-PLN-004-01 | U | REQ-PLN-004 | `src/intercept_planner/test/test_planner.py::test_target_offset_transform` |
| TC-PLN-005-01 | U | REQ-PLN-005 | `src/intercept_planner/test/test_planner.py::test_workspace_boundary` |
| TC-PLN-006-01 | C | REQ-PLN-006 | `src/intercept_planner/test/contract/test_planner_node_contract.py::test_revision_and_expiry` |
| TC-PLN-007-01 | C | REQ-PLN-007 | `src/intercept_planner/test/contract/test_planner_node_contract.py::test_epoch` |
| TC-PLN-008-01 | U | REQ-PLN-008 | `src/intercept_planner/test/test_planner.py::test_uncertainty` |
| TC-PLN-009-01 | U | REQ-PLN-009 | `src/intercept_planner/test/test_planner.py::test_earliest_release` |
| TC-PLN-010-01 | U | REQ-PLN-010 | `src/intercept_planner/test/test_planner.py::test_tau` |
| TC-PLN-011-01 | I2 | REQ-PLN-011 | `src/intercept_planner/test/contract/test_planner_node_contract.py::test_i2` |
| TC-REC-001-01 | U | REQ-REC-001 | `src/orchestrator/test/test_record_logger.py::test_schema_validation_from_external_symlink_build`<br>`tools/eval/test_trial_schema.py::test_schema_is_valid`<br>`tools/eval/test_trial_schema.py::test_plan_example_passes`<br>`tools/eval/test_trial_schema.py::test_missing_required_field_fails`<br>`tools/eval/test_trial_schema.py::test_out_of_range_value_fails`<br>`tools/eval/test_trial_schema.py::test_scenario_seed_required`<br>`tools/eval/test_trial_schema.py::test_mock_trial_passes` |
| TC-REC-002-01 | I0 | REQ-REC-002 | `src/drop_bringup/test/test_i0_integration.py::test_i0_time_insufficient_pass_skip_and_not_attempted_record`<br>`src/orchestrator/test/test_record_logger.py::test_build_and_validate_trial_record` |
| TC-REC-002-02 | U | REQ-REC-002 | `src/orchestrator/test/test_node_failure_regressions.py::test_logger_invalid_root_and_io_recovery`<br>`src/orchestrator/test/test_node_failure_regressions.py::test_missing_schema_fails_closed`<br>`src/orchestrator/test/test_node_failure_regressions.py::test_logger_serializes_callbacks_and_surfaces_write_failure`<br>`src/orchestrator/test/test_node_failure_regressions.py::test_actual_step_callbacks_preserve_writer_order`<br>`src/orchestrator/test/test_node_failure_regressions.py::test_trial_open_and_flush_failures_do_not_commit_dedup` |
| TC-REC-002-03 | U | REQ-REC-002 | `src/orchestrator/test/test_node_failure_regressions.py::test_schema_found_from_installed_share` |
| TC-REC-003-01 | I0 | REQ-REC-003 | `src/drop_bringup/test/test_i0_integration.py::test_i0_nominal_20_cycles_and_records`<br>`src/orchestrator/test/test_node_failure_regressions.py::test_step_record_preserves_backward_ros_stamps` |
| TC-REC-004-01 | U | REQ-REC-004 | `tools/eval/test_stats.py::test_wilson_interval`<br>`tools/eval/test_stats.py::test_wilson_interval_empty`<br>`tools/eval/test_stats.py::test_wilson_interval_invalid` |
| TC-REC-004-02 | U | REQ-REC-004 | `tools/eval/test_stats.py::test_success_rate_unknown_in_denominator`<br>`tools/eval/test_stats.py::test_success_rate_invalid` |
| TC-REC-005-01 | U | REQ-REC-005 | `tools/eval/test_stats.py::test_quantiles_random_sample`<br>`tools/eval/test_stats.py::test_quantiles_small_sample`<br>`tools/eval/test_stats.py::test_quantiles_empty` |
| TC-REC-006-01 | U | REQ-REC-006 | `tools/spec/test_trace.py::test_reports_missing_and_nonzero_exit` |
| TC-REC-006-02 | U | REQ-REC-006 | `tools/spec/test_trace.py::test_gate_expression`<br>`tools/spec/test_trace.py::test_gate_rejects_invalid_expression`<br>`tools/spec/test_trace.py::test_gate_scopes_requirements_and_unknown_marks`<br>`tools/spec/test_trace.py::test_repository_m0_gate`<br>`tools/spec/test_trace.py::test_gate_malformed_row_fails_without_writing` |
| TC-REC-007-01 | U | REQ-REC-007 | `tools/eval/test_devlog.py::test_multiple_reports_and_unmeasured` |
| TC-REC-007-02 | U | REQ-REC-007 | `tools/eval/test_devlog.py::test_replace_all_same_day_preserves_other_days` |
| TC-REC-007-03 | U | REQ-REC-007 | `tools/eval/test_devlog.py::test_cli_identity_trace_and_values`<br>`tools/eval/test_devlog.py::test_cli_unmeasured_defaults` |
| TC-REC-007-04 | U | REQ-REC-007 | `tools/eval/test_devlog.py::test_real_pytest_junit_level_including_skip` |
| TC-REC-007-05 | U | REQ-REC-007 | `tools/eval/test_devlog.py::test_invalid_cli_preserves_output`<br>`tools/eval/test_devlog.py::test_bad_xml_and_header_preserve_output` |
| TC-ROB-001-01 | U | REQ-ROB-001 | `src/robot_control/test/test_motion_adapter.py::test_pose_reference_values` |
| TC-ROB-001-02 | U | REQ-ROB-001 | `src/robot_control/test/test_motion_adapter.py::test_pose_roundtrip_random_and_gimbal_singular` |
| TC-ROB-002-01 | U | REQ-ROB-002 | `src/robot_control/test/test_motion_adapter.py::test_speed_unit_conversion` |
| TC-ROB-003-01 | U | REQ-ROB-003 | `src/robot_control/test/test_motion_adapter.py::test_freshness_boundary_and_explicit_rearm` |
| TC-ROB-004-01 | U | REQ-ROB-004 | `src/robot_control/test/test_motion_adapter.py::test_command_authority_rejects_range_and_nonfinite` |
| TC-ROB-005-01 | C | REQ-ROB-005 | `src/robot_control/test/test_motion_adapter.py::test_single_command_authority_owner`<br>`src/robot_control/test/test_skill_logic.py::test_concurrent_goal_rejected` |
| TC-ROB-006-01 | U | REQ-ROB-006 | `src/robot_control/test/test_belt_sync.py::test_speed_and_acceleration_limits`<br>`src/robot_control/test/test_motion_adapter.py::test_tracking_command_law_and_limits`<br>`src/robot_control/test/test_motion_adapter.py::test_tracking_command_rejects_bad_inputs` |
| TC-ROB-007-01 | C | REQ-ROB-007 | `src/robot_control/test/test_skill_logic.py::test_move_to_pose_uses_measured_pose_error`<br>`src/robot_control/test/test_skill_logic.py::test_named_pose_and_missing_config_precondition`<br>`src/robot_control/test/test_skill_logic.py::test_inactive_lifecycle_rejects_skills`<br>`src/robot_control/test/test_skill_logic.py::test_disabled_backend_never_reports_success`<br>`src/robot_control/test/test_skill_logic.py::test_pick_success_requires_measured_holding`<br>`src/robot_control/test/test_skill_logic.py::test_pick_missing_grip_criteria_precondition` |
| TC-ROB-007-02 | C | REQ-ROB-007 | `src/robot_control/test/test_robot_controller_stop.py::test_stop_preempts_goal_with_stopped_reason`<br>`src/robot_control/test/test_robot_controller_stop.py::test_client_cancel_keeps_canceled_reason`<br>`src/robot_control/test/test_skill_logic.py::test_deadline_and_cancel_paths`<br>`src/robot_control/test/test_skill_logic.py::test_stop_requires_confirmed_feedback` |
| TC-ROB-008-01 | U | REQ-ROB-008 | `src/robot_control/test/test_skill_logic.py::test_dynamic_place_fails_closed_until_ready` |
| TC-ROB-009-01 | U | REQ-ROB-009 | `src/robot_control/test/test_skill_logic.py::test_place_reads_measured_open_width` |
| TC-ROB-010-01 | U | REQ-ROB-010 | `src/robot_control/test/test_skill_logic.py::test_safe_retreat_unsupported_modes_fail_closed` |
| TC-ROB-011-01 | C | REQ-ROB-011(SPEC 없음) | — |
| TC-ROB-012-01 | C | REQ-ROB-012 | — |
| TC-ROB-013-01 | U | REQ-ROB-013 | `src/robot_control/test/test_belt_sync.py::test_release_time_reference_not_source_stamp`<br>`src/robot_control/test/test_belt_sync.py::test_new_revision_accepted`<br>`src/robot_control/test/test_belt_sync.py::test_ros_message_mapping` |
| TC-ROB-013-02 | U | REQ-ROB-013 | `src/robot_control/test/test_belt_sync.py::test_invalid_plan_latches`<br>`src/robot_control/test/test_belt_sync.py::test_old_revision_ignored_without_refresh`<br>`src/robot_control/test/test_belt_sync.py::test_source_stale_even_with_recent_receipt` |
| TC-ROB-013-03 | U | REQ-ROB-013 | `src/robot_control/test/test_belt_sync.py::test_watchdogs`<br>`src/robot_control/test/test_belt_sync.py::test_freeze_ros_clock_detected_by_steady_time`<br>`src/robot_control/test/test_belt_sync.py::test_expiration_without_loop_gap`<br>`src/robot_control/test/test_belt_sync.py::test_no_plan_and_nonfinite_tcp` |
| TC-ROB-013-04 | U | REQ-ROB-013 | `src/robot_control/test/test_belt_sync.py::test_speed_and_acceleration_limits`<br>`src/robot_control/test/test_belt_sync.py::test_workspace_failure`<br>`src/robot_control/test/test_belt_sync.py::test_invalid_config` |
| TC-ROB-013-05 | U | REQ-ROB-013 | `src/robot_control/test/test_belt_sync.py::test_stop_and_explicit_rearm` |
| TC-SIM-001-01 | S | REQ-SIM-001(SPEC 없음) | 절차 시험 |
| TC-SIM-002-01 | S | REQ-SIM-002(SPEC 없음) | 절차 시험 |
| TC-SIM-003-01 | S | REQ-SIM-003(SPEC 없음) | 절차 시험 |
| TC-SIM-004-01 | S | REQ-SIM-004(SPEC 없음) | 절차 시험 |
| TC-SYS-001-01 | U | REQ-SYS-001 | `tools/spec/test_sys_conventions.py::test_package_names_do_not_collide_with_underlay` |
| TC-SYS-002-01 | U | REQ-SYS-002 | `tools/spec/test_sys_conventions.py::test_yaml_numeric_keys_have_unit_suffix` |
| TC-SYS-003-01 | C | REQ-SYS-003 | — |
| TC-SYS-003-02 | C | REQ-SYS-003 | — |
| TC-SYS-004-01 | U | REQ-SYS-004 | `tools/spec/test_sys_conventions.py::test_unit_conversions_only_in_adapters` |
| TC-SYS-005-01 | U | REQ-SYS-005 | `src/calibration/test/test_repo_paths.py::test_relative_paths_resolve_against_repo_root_regardless_of_cwd`<br>`src/calibration/test/test_repo_paths.py::test_repo_root_from_install_tree_and_env_override`<br>`src/drop_mocks/test/test_repo_path.py::test_repo_path_rule`<br>`src/orchestrator/test/test_repo_path.py::test_repo_path_rule` |
| TC-TRK-001-01 | U | REQ-TRK-001 | `src/tracking/test/test_tracking.py::test_constant_accuracy` |
| TC-TRK-001-02 | U | REQ-TRK-001 | `src/tracking/test/test_tracking.py::test_nees` |
| TC-TRK-002-01 | U | REQ-TRK-002 | `src/tracking/test/test_tracking.py::test_reverse_stamp` |
| TC-TRK-003-01 | U | REQ-TRK-003 | `src/tracking/test/test_tracking.py::test_coasting` |
| TC-TRK-003-02 | U | REQ-TRK-003 | `src/tracking/test/test_tracking.py::test_reacquire` |
| TC-TRK-004-01 | U | REQ-TRK-004 | `src/tracking/test/test_tracking.py::test_lost_once` |
| TC-TRK-005-01 | U | REQ-TRK-005 | `src/tracking/test/test_tracking.py::test_pass_new_id` |
| TC-TRK-006-01 | U | REQ-TRK-006 | `src/tracking/test/test_tracking.py::test_speed_step` |
| TC-TRK-006-02 | U | REQ-TRK-006 | `src/tracking/test/test_tracking.py::test_no_false_alarm` |
| TC-TRK-007-01 | U | REQ-TRK-007 | `src/tracking/test/test_tracking.py::test_dropped_frames` |
| TC-TRK-008-01 | U | REQ-TRK-008 | `src/tracking/test/test_tracking.py::test_clock_reset` |
| TC-TRK-009-01 | C | REQ-TRK-009 | `src/tracking/test/contract/test_tracker_node_contract.py::test_reset` |
| TC-TRK-010-01 | C | REQ-TRK-010 | `src/tracking/test/contract/test_tracker_node_contract.py::test_select` |
| TC-TRK-011-01 | C | REQ-TRK-011 | `src/tracking/test/contract/test_tracker_node_contract.py::test_belt` |
| TC-TRK-012-01 | C | REQ-TRK-012 | `src/tracking/test/contract/test_tracker_node_contract.py::test_tf_stamp_and_coasting` |
| TC-TRK-013-01 | I1 | REQ-TRK-013 | — |
| TC-VIS-001-01 | U | REQ-VIS-001 | `src/vision/test/test_detector.py::test_stone_center_accuracy_on_mat` |
| TC-VIS-001-02 | U | REQ-VIS-001 | `src/vision/test/test_detector.py::test_cap_center_accuracy_on_belt` |
| TC-VIS-002-01 | U | REQ-VIS-002 | `src/vision/test/test_detector.py::test_mixed_scene_classification` |
| TC-VIS-003-01 | U | REQ-VIS-003 | `src/vision/test/test_detector.py::test_stone_inside` |
| TC-VIS-004-01 | U | REQ-VIS-004 | `src/vision/test/test_localizer.py::test_round_trip`<br>`src/vision/test/test_localizer.py::test_factory_defaults_match_shared_config`<br>`src/vision/test/test_localizer.py::test_parallel_or_nonforward_ray_rejected` |
| TC-VIS-005-01 | U | REQ-VIS-005 | `src/vision/test/test_localizer.py::test_object_planes`<br>`src/vision/test/test_localizer.py::test_unknown_class_and_cap_inside_context_rejected` |
| TC-VIS-006-01 | U | REQ-VIS-006 | `src/vision/test/test_localizer.py::test_covariance_matches_independent_finite_difference`<br>`src/vision/test/test_localizer.py::test_default_covariance`<br>`src/vision/test/test_localizer.py::test_invalid_pixel_covariance_rejected` |
| TC-VIS-007-01 | U | REQ-VIS-007 | `src/vision/test/test_detector.py::test_robot_mask_overlap_marks_occluded_and_is_not_counted` |
| TC-VIS-008-01 | C | REQ-VIS-008 | `src/vision/test/contract/test_vision_node_contract.py::test_stable_window` |
| TC-VIS-008-02 | C | REQ-VIS-008 | `src/vision/test/contract/test_vision_node_contract.py::test_deadlines` |
| TC-VIS-009-01 | U | REQ-VIS-009(SPEC 없음) | — |
| TC-VIS-010-01 | I1 | REQ-VIS-010(SPEC 없음) | — |

## TC가 없는 REQ

- 없음

## 테스트가 없는 TC(pytest 대상)

- TC-ABT-011-02
- TC-ABT-012-02
- TC-ABT-013-02
- TC-ABT-014-02
- TC-CAL-005-02
- TC-INT-007-01
- TC-ROB-011-01
- TC-ROB-012-01
- TC-SYS-003-01
- TC-SYS-003-02
- TC-TRK-013-01
- TC-VIS-009-01
- TC-VIS-010-01
