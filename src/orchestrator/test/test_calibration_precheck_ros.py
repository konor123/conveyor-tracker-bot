"""PRECHECK의 calibration_id 일치 확인.

실제 orchestrator_node 하나에 시험 클라이언트가 vision·tracker·robot_controller heartbeat(ACTIVE, 20 Hz),
robot/status, belt/measured_motion, objects/detections(calibration_id)를 낸다. 로봇 action 서버는 없다.
robot/status는 ready=false라 보정 확인을 통과하면 PRECHECK가 ROBOT_NOT_READY에서 멈춘다(대조군).
"""
import json

import pytest

rclpy = pytest.importorskip('rclpy')

from drop_interfaces.action import DropTask  # noqa: E402
from drop_interfaces.msg import BeltMotion, Heartbeat, ObjectDetectionArray, RobotStatus, StepRecord  # noqa: E402
from drop_ros import DOMAIN, Harness, wait_future  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402

from orchestrator.orchestrator_node import OrchestratorNode  # noqa: E402

REQUIRED = ('vision_node', 'tracker_node', 'robot_controller_node')
NOT_ATTEMPTED, FAILED = 6, 1


@pytest.fixture
def ros():
    rclpy.init(domain_id=DOMAIN)
    yield
    rclpy.try_shutdown()


def _precheck(vision_id, orchestrator_id):
    """DropTask 한 번(MODE_STATIC). (결과, PRECHECK StepRecord, last_trial_json)."""
    node = OrchestratorNode(parameter_overrides=[Parameter('calibration_id', value=orchestrator_id)])
    h = Harness(node)
    c = h.client
    try:
        steps = h.record(StepRecord, 'task/step_record')
        hb = c.create_publisher(Heartbeat, 'system/heartbeat', 20)
        status = c.create_publisher(RobotStatus, 'robot/status', 5)
        belt = c.create_publisher(BeltMotion, 'belt/measured_motion', 5)
        dets = c.create_publisher(ObjectDetectionArray, 'objects/detections', 5)
        seq = [0]

        def tick():                      # 시험 스레드가 goal을 기다리는 동안에도 executor 스레드가 낸다
            seq[0] += 1
            for name in REQUIRED:
                hb.publish(Heartbeat(node_name=name, lifecycle_state=3, seq=seq[0], error_code=0))
            status.publish(RobotStatus(connected=True, ready=False, gripper_alive=True, gripper_open=True))
            belt.publish(BeltMotion(steady=True, speed=0.0))
            dets.publish(ObjectDetectionArray(calibration_id=vision_id))

        c.create_timer(0.05, tick)
        h.wait_for(lambda: node.detections is not None and node.robot_status is not None and node.belt is not None
                   and all(node.lifecycle.get(n) == 3 for n in REQUIRED), timeout=10.0, what='orchestrator 입력 수신')
        ac = h.action(DropTask, 'drop_task', timeout=10.0)
        gh = wait_future(ac.send_goal_async(DropTask.Goal(mode=DropTask.Goal.MODE_STATIC, max_belt_speed=0.1,
                                                          timeout=30.0, max_passes=1)), 5.0)
        assert gh.accepted
        res = wait_future(gh.get_result_async(), 15.0).result
        h.wait_for(lambda: any(m.step_name == 'PRECHECK' for _, m in steps), timeout=3.0, what='PRECHECK 기록')
        pre = next(m for _, m in steps if m.step_name == 'PRECHECK')
        trial = json.loads(node.get_parameter('last_trial_json').value)
        return res, pre, trial
    finally:
        h.close()


@pytest.mark.parametrize('vision_id, orchestrator_id, reason', [
    ('2026-11-07-final-layout', '2026-11-09-planes', 'CALIBRATION_MISMATCH'),   # 불일치
    ('2026-11-09-planes', '2026-11-09-planes', 'ROBOT_NOT_READY'),             # 일치: 보정 확인 통과(대조군)
    ('', '2026-11-09-planes', 'CALIBRATION_INVALID'),                           # vision에 보정 없음
])
def test_calibration_id_mismatch_fails_precheck(ros, vision_id, orchestrator_id, reason):
    res, pre, trial = _precheck(vision_id, orchestrator_id)
    assert (res.result, res.reason_code) == (NOT_ATTEMPTED, reason)
    assert (pre.result, pre.reason_code) == (FAILED, reason)
    metrics = dict(zip(pre.metric_keys, pre.metric_values))
    assert metrics['calibration_valid'] == (0.0 if reason.startswith('CALIBRATION') else 1.0)
    assert list(trial['step_durations_s']) == ['PRECHECK']          # 로봇 skill을 부르지 않았다
