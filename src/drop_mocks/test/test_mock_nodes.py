"""mock 노드 C 시험(SPEC-700 4절, TEST_SPEC.md 3.11절). rclpy로 노드를 같은 프로세스 또는 별도 프로세스로 띄운다."""
import os
import subprocess
import sys
import time

import pytest

rclpy = pytest.importorskip('rclpy')

from drop_interfaces.action import MoveToPose, PickStone, SafeRetreat  # noqa: E402
from drop_interfaces.msg import (BeltMotion, Heartbeat, ObjectDetectionArray, RobotStatus,  # noqa: E402
                                 TrackedObjectArray)
from drop_interfaces.srv import DetectObjects, ResetTracking, SelectTarget, Stop  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402

from conftest import scenario_path  # noqa: E402
from drop_ros import DOMAIN, Harness, own_qos_event_handlers, wait_future  # noqa: E402
from drop_mocks.mock_robot_controller_node import MockRobotControllerNode  # noqa: E402
from drop_mocks.mock_world_node import MockWorldNode  # noqa: E402
from drop_mocks.ros_common import ERROR_PROTECTIVE_STOP, from_time, to_time  # noqa: E402
from drop_mocks.scenario_engine import (COASTING, LOST, RESULT, ScenarioEngine)  # noqa: E402

FRAME = 1.0 / 30.0


@pytest.fixture(scope='module')
def ros():
    rclpy.init(domain_id=DOMAIN)
    yield
    rclpy.shutdown()


def _send(ac, goal):
    return wait_future(ac.send_goal_async(goal), 5.0)


def _params(scenario, t0, **extra):
    p = [Parameter('scenario', value=scenario_path(scenario)), Parameter('t0_s', value=t0)]
    return p + [Parameter(k, value=v) for k, v in extra.items()]


# ------------------------------------------------------------------ TC-MCK-005-01
def _first(seq, pred):
    return next((x for x in seq if pred(x)), None)


@pytest.mark.req('REQ-MCK-005')
@pytest.mark.tc('TC-MCK-005-01')
@pytest.mark.level('C')
def test_injected_events_appear_at_scheduled_time(ros):
    t0 = time.time() + 1.0
    eng = ScenarioEngine.from_file(scenario_path('all_events'))
    ev = {e['type']: e for e in eng.events}
    world = MockWorldNode(parameter_overrides=_params('all_events', t0))
    robot = MockRobotControllerNode(parameter_overrides=_params('all_events', t0))
    h = Harness(world, robot)
    try:
        dets = h.record(ObjectDetectionArray, 'objects/detections')
        trks = h.record(TrackedObjectArray, 'objects/tracked')
        belt = h.record(BeltMotion, 'belt/measured_motion')
        hbs = h.record(Heartbeat, 'system/heartbeat')
        pick = h.action(PickStone, 'robot/pick_stone')
        results = {}
        for label, t_start in (('before_delay', 6.0), ('in_delay', 6.5 + 2 * FRAME), ('pstop', 8.3)):
            h.wait_until(t0 + t_start)
            gh = _send(pick, PickStone.Goal())
            assert gh.accepted
            res = wait_future(gh.get_result_async(), 5.0).result
            results[label] = (t_start, h.now() - t0, res)
        h.wait_until(t0 + 10.0)
    finally:
        h.close()

    def scen(stamp):                 # stamp → 시나리오 시각(clock_jump 전)
        return from_time(stamp) - t0

    def near(t_seen, t_event):
        assert t_seen is not None
        assert abs(t_seen - t_event) <= FRAME + 1e-6, (t_seen, t_event)

    trk = [(scen(m.header.stamp), m) for _, m in trks if m.tracks]
    near(_first(trk, lambda x: x[1].tracks[0].status == COASTING)[0], ev['occlusion']['t_s'])
    near(_first(trk, lambda x: x[1].tracks[0].status == LOST)[0], ev['track_lost']['t_s'])
    near(_first(trk, lambda x: x[1].tracks[0].epoch == 1)[0], ev['epoch_increment']['t_s'])
    bm = [(scen(m.header.stamp), m) for _, m in belt]
    near(_first(bm, lambda x: abs(x[1].speed - 0.030) < 1e-9 and not x[1].steady)[0], ev['speed_change']['t_s'])
    caps = [(scen(m.header.stamp), m) for _, m in dets]
    near(_first(caps, lambda x: any(d.stone_inside for d in x[1].detections))[0], ev['stone_in_cap']['t_s'])
    seqs = {m.image_seq for _, m in dets}                                  # frame_drop: 구간 프레임 미발행
    t_drop = ev['frame_drop']
    k0 = eng.frame_index(t_drop['t_s'])
    k1 = eng.frame_index(t_drop['t_s'] + t_drop['duration_s'])
    assert not seqs & set(range(k0, k1)), sorted(seqs & set(range(k0, k1)))
    assert seqs & set(range(k0 - 6, k0)) and seqs & set(range(k1, k1 + 6))   # 구간 앞뒤 0.2 s는 발행
    jumped = _first(dets, lambda x: from_time(x[1].header.stamp) - (t0 + x[1].image_seq * FRAME) < -4.9)
    near(jumped[1].image_seq * FRAME, ev['clock_jump']['t_s'])

    # heartbeat(10 Hz): 구간 안에는 없고, 구간 경계 ± 1 프레임 밖에서는 계속 온다
    hb = ev['heartbeat_stop']
    t_vis = [t - t0 for t, m in hbs if m.node_name == 'vision_node']
    t_end = hb['t_s'] + hb['duration_s']
    assert not [t for t in t_vis if hb['t_s'] + FRAME <= t < t_end - FRAME]
    assert [t for t in t_vis if hb['t_s'] - 0.15 <= t < hb['t_s'] + FRAME]
    assert [t for t in t_vis if t_end - FRAME <= t < t_end + 0.15]

    # gripper_delay: 시작 시각이 이벤트 전이면 0.3 s, 후면 0.3 + 0.6 s
    s0, e0, r0 = results['before_delay']
    s1, e1, r1 = results['in_delay']
    assert r0.result == RESULT['OK'] and e0 - s0 == pytest.approx(0.3, abs=0.1)
    assert r1.result == RESULT['OK'] and e1 - s1 == pytest.approx(0.9, abs=0.1)
    # protective_stop: 8.3 s에 시작한 goal이 8.5 s ± 1 프레임에 FAULT로 끝나고 heartbeat에 표시
    _, e2, r2 = results['pstop']
    assert (r2.result, r2.reason_code) == (RESULT['FAULT'], 'PROTECTIVE_STOP')
    near(e2, ev['protective_stop']['t_s'])
    rb = [(t - t0, m) for t, m in hbs if m.node_name == 'robot_controller_node']
    assert all(m.error_code == ERROR_PROTECTIVE_STOP for t, m in rb if 8.5 + FRAME <= t < 9.0 - FRAME)
    assert all(m.error_code == 0 for t, m in rb if t < 8.5 - FRAME or t >= 9.0 + 0.15)


# ------------------------------------------------------------------ TC-MCK-002-01
@pytest.mark.req('REQ-MCK-002')
@pytest.mark.tc('TC-MCK-002-01')
@pytest.mark.level('C')
def test_role_split_processes_see_same_world(ros):
    t0 = time.time() + 0.5
    path = scenario_path('noisy_20mms')
    env = dict(os.environ, ROS_DOMAIN_ID=str(DOMAIN))
    procs = [subprocess.Popen(
        [sys.executable, '-m', 'drop_mocks.mock_world_node', '--ros-args', '-p', f'role:={role}',
         '-p', f'scenario:={path}', '-p', f't0_s:={t0}', '-p', 'seed:=5'],
        env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL) for role in ('vision', 'tracking')]
    h = Harness()
    try:
        dets = h.record(ObjectDetectionArray, 'objects/detections')
        trks = h.record(TrackedObjectArray, 'objects/tracked')
        h.wait_until(t0 + 5.0)
    finally:
        h.close()
        for p in procs:
            p.terminate()
            p.wait(timeout=5)
    eng = ScenarioEngine.from_file(path, seed=5)
    by_stamp = {from_time(m.header.stamp): m for _, m in trks if m.tracks}
    pairs = 0
    for _, d in dets:
        tr = by_stamp.get(from_time(d.header.stamp))
        caps = [x for x in d.detections if x.object_class == 2]
        if tr is None or not caps:
            continue
        p = eng.base_to_conveyor([caps[0].position.x, caps[0].position.y, caps[0].position.z])
        q = tr.tracks[0].position
        assert max(abs(p[0] - q.x), abs(p[1] - q.y), abs(p[2] - q.z)) <= 1e-9
        pairs += 1
    assert pairs >= 30                # t = 2.5 s(시야 진입) 뒤 1 s 이상


# ------------------------------------------------------------------ 서비스·skill 동작(SPEC-700 4.4~4.6절)
@pytest.mark.level('C')
def test_world_services(ros):
    t0 = time.time()
    world = MockWorldNode(parameter_overrides=_params('nominal_20mms', t0 - 5.0))
    h = Harness(world)
    try:
        r = h.call(DetectObjects, 'vision/detect_objects', region=1, object_class=1, min_count=1)
        assert r.result == RESULT['OK'] and len(r.detections) == 1
        r = h.call(DetectObjects, 'vision/detect_objects', region=2, object_class=2, min_count=1,
                   deadline=to_time(h.now() - 0.1))
        assert (r.result, r.reason_code) == (RESULT['TIMEOUT'], 'DEADLINE')
        r = h.call(SelectTarget, 'tracking/select_target', object_class=2, min_observation_count=30)
        assert r.result == RESULT['OK'] and from_time(r.t_enter_window) == pytest.approx(t0 - 5.0 + 12.5, abs=1e-3)
        assert h.call(ResetTracking, 'tracking/reset').new_epoch == 1
        assert h.call(SelectTarget, 'tracking/select_target', object_class=2).epoch == 1
    finally:
        h.close()


@pytest.mark.level('C')
def test_inactive_node_refuses(ros):
    world = MockWorldNode(parameter_overrides=_params('nominal_20mms', time.time(), autostart=False))
    robot = MockRobotControllerNode(parameter_overrides=_params('nominal_20mms', time.time(), autostart=False))
    h = Harness(world, robot)
    try:
        hbs = h.record(Heartbeat, 'system/heartbeat')
        r = h.call(DetectObjects, 'vision/detect_objects', region=1, object_class=1)
        assert (r.result, r.reason_code) == (RESULT['PRECONDITION'], 'NODE_INACTIVE')
        assert not _send(h.action(MoveToPose, 'robot/move_to_pose'), MoveToPose.Goal()).accepted
        h.wait_until(h.now() + 0.3)
        assert hbs and all(m.lifecycle_state == 1 for _, m in hbs)      # UNCONFIGURED
    finally:
        h.close()


@pytest.mark.level('C')
def test_robot_skill_cancel_hang_stop_deadline(ros):
    robot = MockRobotControllerNode(parameter_overrides=_params('robot_hang', time.time()))
    h = Harness(robot)
    try:
        move = h.action(MoveToPose, 'robot/move_to_pose')
        pick = h.action(PickStone, 'robot/pick_stone')
        retreat = h.action(SafeRetreat, 'robot/safe_retreat')
        # 정상: MoveToPose 2 s, 결과 값 채움
        gh = _send(move, MoveToPose.Goal(named_pose='standby'))
        res = wait_future(gh.get_result_async(), 5.0).result
        assert res.result == RESULT['OK'] and res.pose_error == pytest.approx(0.0005)
        # deadline 자체 중단
        gh = _send(move, MoveToPose.Goal(deadline=to_time(h.now() + 0.5)))
        t = h.now()
        res = wait_future(gh.get_result_async(), 5.0).result
        assert (res.result, res.reason_code) == (RESULT['TIMEOUT'], 'DEADLINE')
        assert h.now() - t == pytest.approx(0.5, abs=0.1)
        # hang + cancel 무시 → 진행 중 새 goal 거부 → Stop으로 끝냄
        gh = _send(pick, PickStone.Goal())
        wait_future(gh.cancel_goal_async(), 2.0)
        h.wait_until(h.now() + 0.6)
        assert not _send(retreat, SafeRetreat.Goal()).accepted
        r = h.call(Stop, 'robot/stop', requester=Stop.Request.ORCHESTRATOR)
        assert r.stopped and r.result == RESULT['OK']
        res = wait_future(gh.get_result_async(), 2.0).result
        assert (res.result, res.reason_code) == (RESULT['CANCELED'], 'STOPPED')
        # cancel을 받아들이는 skill
        gh = _send(retreat, SafeRetreat.Goal())
        wait_future(gh.cancel_goal_async(), 2.0)
        res = wait_future(gh.get_result_async(), 2.0).result
        assert res.result == RESULT['CANCELED']
    finally:
        h.close()


@pytest.mark.level('C')
def test_robot_status_holding_workspace_and_heartbeat_stop(ros, tmp_path):
    import yaml
    from drop_interfaces.action import PlaceStone
    from drop_interfaces.msg import RobotStatus
    s = yaml.safe_load(open(scenario_path('nominal_20mms'), encoding='utf-8'))
    s['robot']['skills'] = {
        'PickStone': [{'duration_s': 0.2}],
        'MoveToPose': [{'duration_s': 0.2}],
        'PlaceStone': [{'duration_s': 0.3, 'phase_metrics': {'DESCEND': {'rim_clearance_m': 0.001}}}],
        'DynamicPlace': [{'duration_s': 0.4}],
        'SafeRetreat': [{'duration_s': 0.2}],
    }
    s['events'] = [{'t_s': 0.0, 'type': 'heartbeat_stop', 'role': 'robot_controller', 'duration_s': 10.0}]
    scen_file = tmp_path / 'status_test.yaml'
    scen_file.write_text(yaml.safe_dump(s), encoding='utf-8')
    robot = MockRobotControllerNode(
        parameter_overrides=[Parameter('scenario', value=str(scen_file)), Parameter('t0_s', value=time.time())])
    h = Harness(robot)
    try:
        statuses = h.record(RobotStatus, 'robot/status')
        hbs = h.record(Heartbeat, 'system/heartbeat')
        pick = h.action(PickStone, 'robot/pick_stone')
        place = h.action(PlaceStone, 'robot/place_stone')
        retreat = h.action(SafeRetreat, 'robot/safe_retreat')
        h.wait_until(h.now() + 0.25)
        assert statuses and not [m for _, m in hbs if m.node_name == 'robot_controller_node']
        st0 = statuses[-1][1]
        assert st0.connected and st0.ready and not st0.holding and st0.gripper_open
        assert st0.gripper_width == pytest.approx(0.022) and st0.gripper_current == pytest.approx(0.0)
        assert st0.tcp_pose.position.x == pytest.approx(0.40) and st0.tcp_pose.position.z == pytest.approx(0.20)
        # PickStone -> holding=True
        gh = _send(pick, PickStone.Goal())
        assert wait_future(gh.get_result_async(), 3.0).result.holding
        h.wait_until(h.now() + 0.15)
        st1 = statuses[-1][1]
        assert st1.holding and not st1.gripper_open
        assert st1.gripper_width == pytest.approx(0.0175) and st1.gripper_current == pytest.approx(150.0)
        # SafeRetreat(place_back=True) -> holding=False
        gh = _send(retreat, SafeRetreat.Goal(place_back=True))
        assert wait_future(gh.get_result_async(), 3.0).result.stone_placed_back
        h.wait_until(h.now() + 0.15)
        assert not statuses[-1][1].holding
        # PickStone again -> PlaceStone -> in_workspace seen during APPROACH~RELEASE, holding=False after RELEASE
        wait_future(_send(pick, PickStone.Goal()).get_result_async(), 3.0)
        fbs = []
        n0 = len(statuses)
        gh = wait_future(place.send_goal_async(PlaceStone.Goal(), feedback_callback=lambda f: fbs.append(f.feedback)), 3.0)
        wait_future(gh.get_result_async(), 3.0)
        h.wait_until(h.now() + 0.15)
        assert any(m.in_workspace for _, m in statuses[n0:])
        assert not statuses[-1][1].holding and not statuses[-1][1].in_workspace
        desc_done = next(fb for fb in fbs if fb.phase == 'DESCEND' and fb.phase_done)
        assert dict(zip(desc_done.metric_keys, desc_done.metric_values))['rim_clearance_m'] == pytest.approx(0.001)
    finally:
        h.close()


# ------------------------------------------------------------------ TC-ABT-009-02
def _count_hb(hbs, name):
    return sum(1 for _, m in hbs if m.node_name == name)


@pytest.mark.req('REQ-ABT-009')
@pytest.mark.tc('TC-ABT-009-02')
@pytest.mark.level('C')
def test_heartbeat_and_status_keep_publishing_when_ros_time_frozen(ros):
    """use_sim_time인데 /clock이 없으면 ROS 시각이 멈춘다. heartbeat·robot/status 타이머는 steady라 10 Hz를 유지한다."""
    sim = [Parameter('use_sim_time', value=True)]
    world = MockWorldNode(parameter_overrides=_params('nominal_20mms', 0.0) + sim)
    robot = MockRobotControllerNode(parameter_overrides=_params('nominal_20mms', 0.0) + sim)
    h = Harness(world, robot)
    try:
        hbs = h.record(Heartbeat, 'system/heartbeat')
        statuses = h.record(RobotStatus, 'robot/status')
        names = ('vision_node', 'tracker_node', 'robot_controller_node')
        h.wait_for(lambda: statuses and all(_count_hb(hbs, n) for n in names), timeout=10.0,
                   what='첫 heartbeat·robot/status 수신')            # discovery
        n0 = {n: _count_hb(hbs, n) for n in names}
        s0 = len(statuses)
        time.sleep(1.0)
        assert world.get_clock().now().nanoseconds == 0           # ROS 시각이 실제로 멈춰 있다
        for name, before in n0.items():
            assert _count_hb(hbs, name) - before >= 7, f'{name} heartbeat 부족'
        assert len(statuses) - s0 >= 7
    finally:
        h.close()


# ------------------------------------------------------------------ TC-MCK-007-02
@pytest.mark.req('REQ-MCK-007')
@pytest.mark.tc('TC-MCK-007-02')
@pytest.mark.level('C')
@pytest.mark.parametrize('service', ['vision/detect_objects', 'camera/detect_objects'])
def test_mock_exits_when_real_provider_appears_after_start(ros, service):
    """기동 확인 창이 지난 뒤 같은 역할의 서버가 나타나도 대역은 REAL_NODE_PRESENT로 종료(코드 1)한다.

    `camera/detect_objects`는 대역의 서비스를 remap해 띄운 경우다(감시가 remap된 이름을 봐야 한다).
    """
    env = dict(os.environ, ROS_DOMAIN_ID=str(DOMAIN))
    remap = [] if service == 'vision/detect_objects' else ['-r', f'vision/detect_objects:={service}']
    proc = subprocess.Popen(
        [sys.executable, '-m', 'drop_mocks.mock_world_node', '--ros-args', '-p', 'role:=vision',
         '-p', f'scenario:={scenario_path("nominal_20mms")}', *remap],
        env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    h = Harness()
    real = None
    try:
        hbs = h.record(Heartbeat, 'system/heartbeat')
        h.wait_for(lambda: _count_hb(hbs, 'vision_node') > 0, timeout=10.0, what='대역 vision heartbeat')
        assert proc.poll() is None
        real = rclpy.create_node('vision_node_real')
        real.create_service(DetectObjects, service, lambda req, res: res)
        h.exec.add_node(real)
        rc = proc.wait(timeout=10.0)
        out = proc.stdout.read()
        assert rc == 1, out[-2000:]
        assert 'REAL_NODE_PRESENT' in out
    finally:
        if proc.poll() is None:
            proc.kill()
            proc.wait(timeout=5)
        h.close()
        if real is not None:
            real.destroy_node()


# ------------------------------------------------------------------ TC-IF-006-01
@pytest.mark.req('REQ-IF-006')
@pytest.mark.tc('TC-IF-006-01')
@pytest.mark.level('C')
def test_mock_nodes_have_no_qos_event_handlers(ros):
    world = MockWorldNode(parameter_overrides=_params('nominal_20mms', 0.0))
    robot = MockRobotControllerNode(parameter_overrides=_params('nominal_20mms', 0.0))
    try:
        for n in (world, robot):
            assert len(list(n.publishers)) >= 3              # heartbeat·상태/데이터 + rclpy 내부
            assert own_qos_event_handlers(n) == [], n.get_name()
    finally:
        world.destroy_node()
        robot.destroy_node()
