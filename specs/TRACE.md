# TRACE — REQ ↔ TC ↔ 테스트

`tools/spec/trace.py`가 생성한다. 손으로 고치지 않는다.

- REQ 25개 중 TC 연결 25개
- pytest 대상 TC 43개 중 테스트 연결 43개

| TC | 수준 | REQ | 테스트 |
|---|---|---|---|
| TC-ABT-001-01 | U | REQ-ABT-001 | `src/orchestrator/test/test_abort_policy.py::test_decide_matches_table`<br>`src/orchestrator/test/test_abort_policy.py::test_every_hard_fault_is_l4_even_after_release`<br>`src/orchestrator/test/test_abort_policy.py::test_retry_needs_time_before_deadline`<br>`src/orchestrator/test/test_abort_policy.py::test_non_retryable_failure_is_l2` |
| TC-ABT-002-01 | U | REQ-ABT-002 | `src/orchestrator/test/test_abort_policy.py::test_escalate_keeps_higher` |
| TC-ABT-003-01 | U | REQ-ABT-003 | `src/orchestrator/test/test_abort_policy.py::test_l2_path_timeout` |
| TC-ABT-003-02 | U | REQ-ABT-003 | `src/orchestrator/test/test_abort_policy.py::test_l3_path_timeout`<br>`src/orchestrator/test/test_abort_policy.py::test_path_done_stops_path_timer`<br>`src/orchestrator/test/test_cycle_runner.py::test_abort_path_hang_escalates_to_stop` |
| TC-ABT-004-01 | U | REQ-ABT-004 | `src/orchestrator/test/test_abort_policy.py::test_cancel_without_response_escalates_one_level`<br>`src/orchestrator/test/test_abort_policy.py::test_cancel_ack_prevents_escalation`<br>`src/orchestrator/test/test_cycle_runner.py::test_pick_hang_cancel_escalation` |
| TC-ABT-005-01 | U | REQ-ABT-005 | `src/orchestrator/test/test_abort_policy.py::test_user_cancel_while_holding_is_l3` |
| TC-ABT-006-01 | U | REQ-ABT-006 | `src/orchestrator/test/test_deadlines.py::test_effective_deadline_is_minimum` |
| TC-ABT-007-01 | U | REQ-ABT-007 | `src/orchestrator/test/test_deadlines.py::test_derive_step_timeout`<br>`src/orchestrator/test/test_deadlines.py::test_derive_step_timeout_floor_is_p95_plus_half` |
| TC-ABT-008-01 | U | REQ-ABT-008 | `src/orchestrator/test/test_watchdogs.py::test_cycle_ends_when_ros_clock_frozen`<br>`src/orchestrator/test/test_watchdogs.py::test_cycle_timer_uses_ros_deadline_when_clock_runs` |
| TC-ABT-009-01 | U | REQ-ABT-009 | `src/orchestrator/test/test_watchdogs.py::test_heartbeat_ignores_ros_jump`<br>`src/orchestrator/test/test_watchdogs.py::test_heartbeat_detected_with_frozen_ros` |
| TC-ABT-010-01 | U | REQ-ABT-010 | `src/orchestrator/test/test_watchdogs.py::test_heartbeat_boundary`<br>`src/orchestrator/test/test_watchdogs.py::test_heartbeat_unknown_node_is_lost`<br>`src/orchestrator/test/test_watchdogs.py::test_heartbeat_loss_levels` |
| TC-ABT-011-01 | U | REQ-ABT-011 | `src/orchestrator/test/test_watchdogs.py::test_observation_freshness` |
| TC-ABT-012-01 | U | REQ-ABT-012 | `src/orchestrator/test/test_watchdogs.py::test_clock_regression` |
| TC-ABT-013-01 | U | REQ-ABT-013 | `src/orchestrator/test/test_abort_policy.py::test_belt_unsteady` |
| TC-ABT-014-01 | U | REQ-ABT-014 | `src/orchestrator/test/test_watchdogs.py::test_gripper_monitor`<br>`src/orchestrator/test/test_watchdogs.py::test_gripper_alive_counter_stall`<br>`src/orchestrator/test/test_watchdogs.py::test_gripper_fault_levels` |
| TC-ABT-015-01 | U | REQ-ABT-015 | `src/orchestrator/test/test_abort_policy.py::test_fault_event_sequences`<br>`src/orchestrator/test/test_abort_policy.py::test_fault_list_covers_plan_7_3` |
| TC-ABT-016-01 | U | REQ-ABT-016 | `src/orchestrator/test/test_abort_policy.py::test_wait_operator_has_no_timeout` |
| TC-ORC-001-01 | U | REQ-ORC-001 | `src/orchestrator/test/test_cycle_runner.py::test_synchronized_nominal_sequence` |
| TC-ORC-001-02 | U | REQ-ORC-001 | `src/orchestrator/test/test_cycle_runner.py::test_static_skips_synchronize_and_detects_cap`<br>`src/orchestrator/test/test_step_judges.py::test_static_place` |
| TC-ORC-001-03 | U | REQ-ORC-001 | `src/orchestrator/test/test_cycle_runner.py::test_predicted_uses_entry_judgement`<br>`src/orchestrator/test/test_step_judges.py::test_predicted_entry` |
| TC-ORC-002-01 | U | REQ-ORC-002 | `src/orchestrator/test/test_cycle_runner.py::test_no_next_skill_after_failure`<br>`src/orchestrator/test/test_cycle_runner.py::test_release_failure_continues_to_retreat_and_verify`<br>`src/orchestrator/test/test_cycle_runner.py::test_retreat_failure_is_protective_stop` |
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
| TC-ORC-003-14 | U | REQ-ORC-003 | `src/orchestrator/test/test_step_judges.py::test_verify_no_response_is_unknown` |
| TC-ORC-004-01 | U | REQ-ORC-004 | `src/orchestrator/test/test_step_judges.py::test_metric_keys_match_spec`<br>`src/orchestrator/test/test_step_judges.py::test_metric_keys_present_on_failure_too` |
| TC-ORC-005-01 | U | REQ-ORC-005 | `src/orchestrator/test/test_cycle_runner.py::test_stale_detection_blocks_pick`<br>`src/orchestrator/test/test_cycle_runner.py::test_detection_at_age_limit_allows_pick` |
| TC-ORC-006-01 | U | REQ-ORC-006 | `src/orchestrator/test/test_cycle_runner.py::test_detect_retries_then_succeeds`<br>`src/orchestrator/test/test_cycle_runner.py::test_detect_three_failures_is_l2` |
| TC-ORC-006-02 | U | REQ-ORC-006 | `src/orchestrator/test/test_cycle_runner.py::test_pick_retry_rechecks_detection_age`<br>`src/orchestrator/test/test_cycle_runner.py::test_pick_non_retryable_failure_is_not_retried` |
| TC-ORC-007-01 | U | REQ-ORC-007 | `src/orchestrator/test/test_cycle_runner.py::test_pass_skip_then_proceed`<br>`src/orchestrator/test/test_cycle_runner.py::test_wait_then_ok_within_pass` |
| TC-ORC-007-02 | U | REQ-ORC-007 | `src/orchestrator/test/test_cycle_runner.py::test_passes_exhausted_is_not_attempted` |
| TC-ORC-008-01 | U | REQ-ORC-008 | `src/orchestrator/test/test_cycle_runner.py::test_plan_failure_goes_to_next_pass` |
| TC-ORC-009-01 | U | REQ-ORC-009 | `src/orchestrator/test/test_cycle_runner.py::test_resume_requires_reason`<br>`src/orchestrator/test/test_cycle_runner.py::test_precheck_failure_returns_to_idle_without_wait_operator` |

## TC가 없는 REQ

- 없음

## 테스트가 없는 TC(pytest 대상)

- 없음
