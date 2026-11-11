"""정지 벨트 pick-and-place 실행기.

  ros2 run static_pick_place pick_place --config <현장 덮어쓰기.yaml>             # 계획만(로봇 지령 없음)
  ros2 run static_pick_place pick_place --config <...> --execute --confirm       # 단계마다 Enter 확인 후 실행

카메라 1프레임 → vision 검출·평면 교차 → 매트 공깃돌 1개·빈 병뚜껑 1개 선택 → 경유점 계획·역기구학 여유 확인
→ (--execute) 순서대로 실행 → 끝나면 다시 1프레임으로 stone_inside 확인. 실제 로봇 지령은 [OFF]이며 사람 입회가 필요하다.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time

import numpy as np
import yaml

from .doosan_client import DoosanClient, RobotError
from .plan import DOWN_QUAT_XYZW, PickPlaceParams, PlanError, choose_cap, choose_stone, plan_pick_place

_PARAM_KEYS = {f for f in PickPlaceParams.__dataclass_fields__}
_PATH_KEYS = ('calibration_file', 'mat_plane_file', 'belt_plane_file')


def load_config(paths) -> dict:
    """패키지 기본 설정 위에 `--config` 파일들을 차례로 덮어쓴다. `--config`의 상대 경로는 실행 위치 기준이고,
    설정 안 파일 경로 값의 상대 경로는 저장소 루트 기준이다(SPEC-000 REQ-SYS-005)."""
    from calibration.repo_paths import resolve_path
    here = os.path.join(os.path.dirname(__file__), os.pardir, 'config', 'static_pick_place.yaml')
    try:
        from ament_index_python.packages import get_package_share_directory
        here = os.path.join(get_package_share_directory('static_pick_place'), 'config', 'static_pick_place.yaml')
    except Exception:
        pass
    cfg = {}
    for path in [here, *paths]:
        with open(path) as fh:
            cfg.update(yaml.safe_load(fh) or {})
    for key in _PATH_KEYS:
        cfg[key] = resolve_path(cfg.get(key))
    return cfg


def make_params(cfg: dict) -> PickPlaceParams:
    return PickPlaceParams(**{k: v for k, v in cfg.items() if k in _PARAM_KEYS})


def load_geometry(cfg: dict):
    from vision.geometry_files import load_geometry as load_files
    from vision.localizer import HeightParams
    heights = HeightParams(float(cfg['h_stone_offset_m']), float(cfg['h_rim_m']), float(cfg['floor_thickness_m']))
    try:
        geom = load_files(cfg.get('calibration_file', ''), cfg.get('mat_plane_file', ''),
                          cfg.get('belt_plane_file', ''), heights)
    except ValueError as exc:
        raise PlanError('BAD_CALIBRATION', str(exc)) from exc
    if geom is None:
        raise PlanError('NO_CALIBRATION', 'calibration_file·mat_plane_file·belt_plane_file이 비었다')
    return geom


def grab_frame(node, topic: str, n_skip: int = 3, timeout_s: float = 5.0) -> np.ndarray:
    """최신 프레임 하나(BGR). 오래된 retained 샘플을 피하려고 앞 몇 장은 버린다."""
    import rclpy
    from sensor_msgs.msg import Image
    frames = []

    def cb(m):
        a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.step)[:, :m.width * 3].reshape(m.height, m.width, 3)
        frames.append(a[..., ::-1].copy() if m.encoding == 'rgb8' else a.copy())

    sub = node.create_subscription(Image, topic, cb, 10)
    deadline = time.monotonic() + timeout_s
    while len(frames) <= n_skip and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    if len(frames) <= n_skip:
        raise PlanError('NO_IMAGE', topic)
    return frames[-1]


def detect(image, cfg, geom):
    from vision.detector import InsideParams
    from vision.pipeline import SceneConfig, process_frame
    return process_frame(image, SceneConfig(inside=InsideParams(float(cfg['inner_diameter_ratio']))), geom)


def describe(d) -> dict:
    return {'class': d.object_class, 'region': d.region, 'uv_px': [round(d.blob.u, 1), round(d.blob.v, 1)],
            'position_m': [round(x, 4) for x in d.position], 'stone_inside': d.stone_inside,
            'stone_inside_valid': d.stone_inside_valid}


def log(event: str, **kw):
    print(json.dumps({'t': round(time.time(), 3), 'event': event, **kw}, ensure_ascii=False), flush=True)


def main(argv=None):
    ap = argparse.ArgumentParser(description='정지 벨트 공깃돌 pick-and-place(기본은 계획만)')
    ap.add_argument('--config', action='append', default=[], help='덮어쓰기 YAML(여러 번 가능)')
    ap.add_argument('--execute', action='store_true', help='실제 로봇·그리퍼 지령을 보낸다([OFF], 입회 필수)')
    ap.add_argument('--confirm', action='store_true', help='단계마다 Enter를 받아 진행')
    ap.add_argument('--one-drl', action='store_true',
                    help='사이클 전체를 DRL 프로그램 하나로 실행한다(그리퍼 확인 1회, 이미 열려 있으면 열기 생략). '
                         '--confirm과 함께 쓸 수 없다')
    ap.add_argument('--stop-after', help='--one-drl 전용. 이 단계까지만 실행하고 끝낸다(첫 실기 확인용, 투입 확인 생략)')
    ap.add_argument('--save-image', help='검출에 쓴 프레임 저장 경로')
    ap.add_argument('--resume-from', help='중단된 실행을 이 단계부터 잇는다. --stone-xyz·--cap-xyz(앞 실행 로그의 targets)와 함께 쓴다')
    ap.add_argument('--stone-xyz', type=float, nargs=3, help='공깃돌 윗면 [m](--resume-from 전용)')
    ap.add_argument('--cap-xyz', type=float, nargs=3, help='뚜껑 테두리 윗면 [m](--resume-from 전용)')
    ap.add_argument('--cycles', type=int, default=1, help='반복 횟수. 2회째부터는 빈 뚜껑·매트 공깃돌이 보일 때까지 기다린다')
    ap.add_argument('--wait-s', type=float, default=120.0, help='다음 주기 대상을 기다리는 최대 시간 [s]')
    ap.add_argument('--image', help='카메라 대신 저장된 BGR 영상으로 계획만 한다(--execute와 함께 쓸 수 없다)')
    args = ap.parse_args(argv)
    if args.image and args.execute:
        ap.error('--image는 계획 확인용이다. 실행은 현재 카메라 프레임으로만 한다')
    if args.one_drl and (args.confirm or args.resume_from):
        ap.error('--one-drl은 한 프로그램으로 끝까지 실행한다. --confirm·--resume-from과 함께 쓸 수 없다')
    if args.stop_after and not args.one_drl:
        ap.error('--stop-after는 --one-drl 전용이다(단계별 실행은 --confirm을 쓴다)')

    import rclpy
    cfg = load_config(args.config)
    p = make_params(cfg)
    rclpy.init()
    node = rclpy.create_node('static_pick_place')
    try:
        geom = load_geometry(cfg)
        if args.resume_from:
            return _resume(node, cfg, p, geom, args)
        robot = DoosanClient(node, cfg['robot_namespace'])
        for k in range(max(1, args.cycles)):
            log('cycle', index=k + 1, of=args.cycles)
            code = _cycle(node, cfg, p, geom, args, robot, wait_s=args.wait_s if k else 0.0)
            if code:
                return code
        return 0
    except (PlanError, RobotError) as exc:
        log('abort', reason=getattr(exc, 'reason', 'ERROR'), detail=str(exc))
        return 2
    finally:
        node.destroy_node()
        rclpy.shutdown()


def _targets(node, cfg, p, geom, args, wait_s: float):
    """검출해서 (공깃돌, 뚜껑)을 고른다. 없으면 wait_s 동안 1 s 간격으로 다시 본다."""
    deadline = time.monotonic() + wait_s
    while True:
        if args.image:
            import cv2
            image = cv2.imread(args.image)
            if image is None:
                raise PlanError('NO_IMAGE', args.image)
        else:
            image = grab_frame(node, cfg['image_topic'])
        dets = detect(image, cfg, geom)
        stone, cap = choose_stone(dets, p), choose_cap(dets, p)
        if (stone is not None and cap is not None) or time.monotonic() >= deadline:
            break
        time.sleep(1.0)
    if args.save_image:
        import cv2
        cv2.imwrite(args.save_image, image)
    log('detections', calibration_id=geom.calibration_id, items=[describe(d) for d in dets])
    return stone, cap


def _cycle(node, cfg, p, geom, args, robot, wait_s: float) -> int:
    stone, cap = _targets(node, cfg, p, geom, args, wait_s)
    if stone is None or cap is None:
        log('abort', reason='NO_TARGET', stone=stone is not None, cap=cap is not None)
        return 2
    steps = plan_pick_place(stone.position, cap.position, p)
    log('targets', stone=describe(stone), cap=describe(cap))
    for s in steps:
        if s.kind == 'move':
            j = robot.ik_joints(s.pos_m, DOWN_QUAT_XYZW)
            log('plan', step=s.name, pos_m=[round(x, 4) for x in s.pos_m], v_mps=s.v_mps,
                j3_rad=round(float(j[2]), 4))
            if abs(j[2]) < p.j3_min_rad:
                log('abort', reason='NEAR_REACH_LIMIT', step=s.name, j3_rad=float(j[2]))
                return 2
        else:
            log('plan', step=s.name, pulse=s.pulse, current_ma=s.current_ma, grasp=s.grasp)
    if args.stop_after:
        steps = truncate_steps(steps, args.stop_after)
    if not args.execute:
        log('dry_run_done')
        return 0
    t0 = time.monotonic()
    log('ready', status=robot.check_ready())
    code = _run_one_drl(robot, steps, p) if args.one_drl else _run_steps(robot, steps, p, args.confirm)
    if code:
        return code
    if args.stop_after:
        log('partial_done', stop_after=args.stop_after, elapsed_s=round(time.monotonic() - t0, 2))
        return 0
    code = _verify(node, cfg, geom, cap.position)
    log('cycle_done', elapsed_s=round(time.monotonic() - t0, 2), result='SUCCESS' if code == 0 else 'NOT_VERIFIED')
    return code


def _run_steps(robot, steps, p, confirm: bool) -> int:
    for s in steps:
        if confirm and input(f'[{s.name}] Enter=실행, 다른 입력=중단: ').strip():
            log('abort', reason='OPERATOR', step=s.name)
            return 3
        if s.kind == 'move':
            reached = robot.move_line(s.pos_m, DOWN_QUAT_XYZW, s.v_mps, s.a_mps2)
            log('done', step=s.name, reached_m=[round(float(x), 4) for x in reached])
        else:
            g = robot.gripper(s.pulse, s.current_ma, grasp=s.grasp, timeout_s=p.gripper_timeout_s,
                              read_timeout_s=p.gripper_read_timeout_s)
            log('done', step=s.name, position_pulse=g.position_pulse, current_ma=g.current_ma, status=g.status)
            if s.grasp and g.status != 2:
                # 목표(닫힘 끝점)까지 닫혔다 = 손가락 사이에 돌이 없다
                log('abort', reason='GRASP_EMPTY', step=s.name)
                return 4
    return 0


def truncate_steps(steps, stop_after: str):
    """`stop_after` 단계까지만 남긴다. 이름이 없으면 계획 오류다."""
    names = [s.name for s in steps]
    if stop_after not in names:
        raise PlanError('BAD_STOP_AFTER', f'단계 이름: {names}')
    return steps[:names.index(stop_after) + 1]


def cycle_timeout_s(steps, p) -> float:
    """한 프로그램 실행 제한시간: 이동은 속도 기준 여유 2배 + 10 s, 그리퍼는 단계마다 `gripper_timeout_s`."""
    total = 10.0
    for s in steps:
        total += p.gripper_timeout_s if s.kind == 'gripper' else 2.0 * 0.6 / max(s.v_mps, 1e-6)
    return total


def _run_one_drl(robot, steps, p) -> int:
    try:
        out = robot.run_cycle(steps, DOWN_QUAT_XYZW, cycle_timeout_s(steps, p), p.gripper_read_timeout_s,
                              on_step=lambda name, sec: log('done', step=name, seconds=round(sec, 2)))
    except RobotError as exc:
        log('abort', reason=exc.reason, detail=str(exc), program_may_run=exc.program_may_run)
        if exc.program_may_run:
            log('operator_action', action='PENDANT_OR_ESTOP',
                message='PC는 감시를 멈췄지만 제어기 DRL이 아직 돌 수 있다. 펜던트·비상정지로 멈추고 상태를 기록한다(drl_stop 자동 호출 없음)')
        return 4 if exc.reason == 'GRASP_EMPTY' else 2
    result = {}
    if out.grasp_status:                         # 0이면 이번 실행에 파지가 없었다(--stop-after)
        result.update(grasp_pulse=out.grasp_pulse, grasp_current_ma=out.grasp_current_ma,
                      grasp_status=out.grasp_status)
    if out.open_status:
        result.update(open_pulse=out.open_pulse, open_status=out.open_status)
    log('one_drl_done', **result)
    moves = [s for s in steps if s.kind == 'move']
    if moves:                                    # 마지막 이동 목표에 실제로 와 있는지 확인한다
        reached, _ = robot.tcp_pose()
        if float(np.linalg.norm(reached - np.asarray(moves[-1].pos_m))) > 0.001:
            log('abort', reason='MOVE_NOT_REACHED', step=moves[-1].name,
                reached_m=[round(float(x), 4) for x in reached])
            return 2
    return 0


def _verify(node, cfg, geom, cap_pos) -> int:
    from .plan import CLASS_CAP, REGION_BELT
    after = detect(grab_frame(node, cfg['image_topic']), cfg, geom)
    near = [d for d in after if d.object_class == CLASS_CAP and d.region == REGION_BELT
            and np.hypot(d.position[0] - cap_pos[0], d.position[1] - cap_pos[1]) < 0.01]
    log('verify', cap=[describe(d) for d in near])
    return 0 if near and near[0].stone_inside_valid and near[0].stone_inside else 5


def _resume(node, cfg, p, geom, args) -> int:
    if not (args.execute and args.stone_xyz and args.cap_xyz):
        raise PlanError('BAD_RESUME', '--resume-from은 --execute, --stone-xyz, --cap-xyz가 필요하다')
    steps = plan_pick_place(args.stone_xyz, args.cap_xyz, p)
    names = [s.name for s in steps]
    if args.resume_from not in names:
        raise PlanError('BAD_RESUME', f'단계 이름: {names}')
    robot = DoosanClient(node, cfg['robot_namespace'])
    log('ready', status=robot.check_ready(), resume_from=args.resume_from)
    code = _run_steps(robot, steps[names.index(args.resume_from):], p, args.confirm)
    return code or _verify(node, cfg, geom, args.cap_xyz)


if __name__ == '__main__':
    sys.exit(main())
