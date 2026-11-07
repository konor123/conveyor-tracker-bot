#!/usr/bin/env python3
"""E0509 로봇 및 RH-P12-RN 그리퍼 ROS 2 통합 제어 도구 (e0509_ctl).

Doosan E0509 로봇의 상태 조회, 홈 복귀, 관절/직교 모션 및 그리퍼 제어를 제공합니다.
단독 CLI로 실행하거나 파이썬 모듈로 임포트하여 에이전트 및 상위 노드에서 재사용할 수 있습니다.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import json
import math
import os
from pathlib import Path
import socket
import sys
import time
from typing import Any

# 기본 파라미터 및 안전 제한값
DEFAULT_ROBOT_HOST = "110.120.1.38"
DEFAULT_ROBOT_PORT = 12345
DEFAULT_NAMESPACE = "/dsr01"

# 규격 상 홈 및 대기 자세
STANDBY_POSX = [372.96, 0.00, 404.80, 0.71, -179.97, 0.71]  # mm, deg (ZYZ)
MECHANICAL_POSJ = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]            # deg

# 안전 상한값
MAX_TASK_VEL_MMPS = 100.0   # 직교 이동 속도 상한
MAX_TASK_ACC_MMPS2 = 250.0  # 직교 가속도 상한
MAX_JOINT_VEL_DPS = 45.0    # 관절 회전 속도 상한
MAX_JOINT_ACC_DPS2 = 100.0  # 관절 가속도 상한

# 직교 작업 공간 안전 범위 (base_link 기준 mm)
POSX_BOUNDS = {
    "x": (100.0, 850.0),
    "y": (-500.0, 500.0),
    "z": (50.0, 750.0),
}

# 그리퍼 규격 (SPEC-500 REQ-GRP-004)
PULSE_MIN = 0
PULSE_MAX = 1150
PULSE_WORK_MAX = 750        # 현장 운용 안전 상한 (REQ-GRP-004)
CURRENT_MIN_MA = 1
CURRENT_MAX_MA = 100        # 승인 전류 상한 100 mA
CURRENT_DEFAULT_MA = 100


@dataclass
class RobotStatusData:
    connected: bool
    state: int              # 1: STANDBY, 2: MOVING, 3: SAFE_OFF, 4: TEACHING, etc.
    state_str: str
    mode: int               # 0: MANUAL, 1: AUTONOMOUS
    mode_str: str
    drl_state: int
    alarm_level: int
    alarm_group: int
    alarm_index: int
    alarm_msg: str
    posx: list[float]       # [x, y, z, rx, ry, rz] mm, deg
    posj: list[float]       # [j1..j6] deg


def check_network(host: str = DEFAULT_ROBOT_HOST, port: int = DEFAULT_ROBOT_PORT, timeout: float = 1.0) -> bool:
    """제어기 TCP 포트 연결 확인."""
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except (socket.timeout, OSError):
        return False


def validate_posx(posx: list[float]) -> None:
    if len(posx) != 6:
        raise ValueError(f"posx의 길이는 6이어야 합니다: len={len(posx)}")
    x, y, z = posx[:3]
    if not (POSX_BOUNDS["x"][0] <= x <= POSX_BOUNDS["x"][1]):
        raise ValueError(f"posx X={x:.2f}mm가 안전 범위 {POSX_BOUNDS['x']}를 벗어났습니다.")
    if not (POSX_BOUNDS["y"][0] <= y <= POSX_BOUNDS["y"][1]):
        raise ValueError(f"posx Y={y:.2f}mm가 안전 범위 {POSX_BOUNDS['y']}를 벗어났습니다.")
    if not (POSX_BOUNDS["z"][0] <= z <= POSX_BOUNDS["z"][1]):
        raise ValueError(f"posx Z={z:.2f}mm가 안전 범위 {POSX_BOUNDS['z']}를 벗어났습니다.")


def validate_posj(posj: list[float]) -> None:
    if len(posj) != 6:
        raise ValueError(f"posj의 길이는 6이어야 합니다: len={len(posj)}")
    # E0509 각 관절 각도 범위 (-360 ~ 360)
    for i, j in enumerate(posj):
        if not (-360.0 <= j <= 360.0):
            raise ValueError(f"관절 {i+1}의 각도 {j:.2f}deg가 허용 범위(-360..360)를 벗어났습니다.")


def validate_gripper(pulse: int, current_ma: int, allow_hardware_limit: bool = False) -> None:
    max_pulse = PULSE_MAX if allow_hardware_limit else PULSE_WORK_MAX
    if not isinstance(pulse, int) or not (PULSE_MIN <= pulse <= max_pulse):
        raise ValueError(f"그리퍼 pulse({pulse})는 {PULSE_MIN}..{max_pulse} 범위의 정수여야 합니다.")
    if not isinstance(current_ma, int) or not (CURRENT_MIN_MA <= current_ma <= CURRENT_MAX_MA):
        raise ValueError(f"그리퍼 current_ma({current_ma})는 {CURRENT_MIN_MA}..{CURRENT_MAX_MA} mA 범위의 정수여야 합니다.")


class E0509Client:
    """ROS 2 인터페이스를 통한 E0509 로봇 및 그리퍼 제어기."""

    def __init__(self, node: Any, ns: str = DEFAULT_NAMESPACE):
        self.node = node
        self.ns = ns.rstrip("/")

    def call_service(self, srv_type: Any, srv_name: str, req: Any, timeout: float = 5.0) -> Any:
        import rclpy

        full_name = f"{self.ns}/{srv_name}"
        cli = self.node.create_client(srv_type, full_name)
        if not cli.wait_for_service(timeout_sec=timeout):
            raise TimeoutError(f"ROS 2 서비스를 찾을 수 없습니다: {full_name}")
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
        if not fut.done():
            raise TimeoutError(f"ROS 2 서비스 응답 타임아웃: {full_name}")
        return fut.result()

    def get_status(self) -> RobotStatusData:
        from dsr_msgs2.srv import (
            GetCurrentPosj,
            GetCurrentPosx,
            GetDrlState,
            GetLastAlarm,
            GetRobotMode,
            GetRobotState,
        )

        st_res = self.call_service(GetRobotState, "system/get_robot_state", GetRobotState.Request())
        md_res = self.call_service(GetRobotMode, "system/get_robot_mode", GetRobotMode.Request())
        drl_res = self.call_service(GetDrlState, "drl/get_drl_state", GetDrlState.Request())
        alm_res = self.call_service(GetLastAlarm, "system/get_last_alarm", GetLastAlarm.Request())

        px_req = GetCurrentPosx.Request()
        px_req.ref = 0  # DR_BASE
        px_res = self.call_service(GetCurrentPosx, "aux_control/get_current_posx", px_req)

        pj_res = self.call_service(GetCurrentPosj, "aux_control/get_current_posj", GetCurrentPosj.Request())

        posx = [float(v) for v in px_res.task_pos_info[0].data[:6]] if (px_res and px_res.task_pos_info) else []
        posj = [float(v) for v in pj_res.pos[:6]] if pj_res else []

        state_names = {1: "STANDBY", 2: "MOVING", 3: "SAFE_OFF", 4: "TEACHING", 5: "SAFE_STOP"}
        mode_names = {0: "MANUAL", 1: "AUTONOMOUS"}

        state_val = getattr(st_res, "robot_state", -1)
        mode_val = getattr(md_res, "robot_mode", -1)
        drl_val = getattr(drl_res, "drl_state", -1)
        alarm = getattr(alm_res, "log_alarm", None)

        lvl = getattr(alarm, "level", 0) if alarm else 0
        grp = getattr(alarm, "group", 0) if alarm else 0
        idx = getattr(alarm, "index", 0) if alarm else 0
        params = list(getattr(alarm, "param", [])) if alarm else []
        alarm_msg = f"level {lvl}, group {grp}, index {idx}"
        if any(params):
            alarm_msg += f" ({', '.join(p for p in params if p)})"

        return RobotStatusData(
            connected=True,
            state=state_val,
            state_str=state_names.get(state_val, f"UNKNOWN({state_val})"),
            mode=mode_val,
            mode_str=mode_names.get(mode_val, f"UNKNOWN({mode_val})"),
            drl_state=drl_val,
            alarm_level=lvl,
            alarm_group=grp,
            alarm_index=idx,
            alarm_msg=alarm_msg,
            posx=posx,
            posj=posj,
        )

    def set_mode(self, mode: int) -> bool:
        """로봇 모드 변경 (0: MANUAL, 1: AUTONOMOUS)."""
        from dsr_msgs2.srv import SetRobotMode

        req = SetRobotMode.Request()
        req.robot_mode = mode
        res = self.call_service(SetRobotMode, "system/set_robot_mode", req)
        return bool(res and res.success)

    def stop(self, stop_mode: int = 1) -> bool:
        """로봇 모션 정지 (stop_mode: 1=QUICK_STOP)."""
        from dsr_msgs2.srv import MoveStop

        req = MoveStop.Request()
        req.stop_mode = stop_mode
        res = self.call_service(MoveStop, "motion/move_stop", req)
        return bool(res and res.success)

    def move_home(
        self,
        home_type: str = "standby",
        vel: float = 30.0,
        acc: float = 60.0,
        dry_run: bool = False,
    ) -> bool:
        """지정된 홈 위치로 복귀.

        home_type:
          - 'mechanical': 기계 홈 (posj [0,0,0,0,0,0], MoveHome target=0)
          - 'user': 제어기 사용자 정의 홈 (MoveHome target=1)
          - 'standby': 작업 대기 자세 (posx [372.96, 0.0, 404.80, 0.71, -179.97, 0.71])
        """
        home_type = home_type.lower()
        if home_type not in ("mechanical", "user", "standby"):
            raise ValueError(f"지원하지 않는 홈 타입입니다: {home_type}")

        if dry_run:
            print(f"[dry-run] E0509 home_type='{home_type}' 복귀 명령 검증 성공 (vel={vel}, acc={acc})")
            return True

        # 안전 사전 확인
        st = self.get_status()
        if st.state != 1:  # STANDBY
            raise RuntimeError(f"로봇이 STANDBY 상태가 아닙니다: {st.state_str}({st.state})")
        if st.alarm_level > 0:
            raise RuntimeError(f"로봇에 알람이 존재합니다: {st.alarm_msg}")

        # 필요시 AUTONOMOUS 모드 설정
        if st.mode != 1:
            print("  로봇 모드가 AUTONOMOUS가 아니므로 자동 모드(1)로 전환합니다...")
            self.set_mode(1)

        if home_type == "mechanical":
            from dsr_msgs2.srv import MoveHome
            req = MoveHome.Request()
            req.target = 0  # DR_HOME_TARGET_MECHANIC
            res = self.call_service(MoveHome, "motion/move_home", req, timeout=30.0)
            return bool(res and res.success)

        elif home_type == "user":
            from dsr_msgs2.srv import MoveHome
            req = MoveHome.Request()
            req.target = 1  # DR_HOME_TARGET_USER
            res = self.call_service(MoveHome, "motion/move_home", req, timeout=30.0)
            return bool(res and res.success)

        elif home_type == "standby":
            return self.move_line(STANDBY_POSX, vel=vel, acc=acc)

        return False

    def move_line(
        self,
        posx: list[float],
        vel: float = 30.0,
        acc: float = 60.0,
        dry_run: bool = False,
    ) -> bool:
        """직교 좌표(posx)로 직선 이동 (MoveLine)."""
        validate_posx(posx)
        vel = min(vel, MAX_TASK_VEL_MMPS)
        acc = min(acc, MAX_TASK_ACC_MMPS2)

        if dry_run:
            print(f"[dry-run] MoveLine 목표 posx={posx}, vel={vel} mm/s, acc={acc} mm/s² 검증 통과")
            return True

        from dsr_msgs2.srv import MoveLine

        req = MoveLine.Request()
        req.pos = [float(v) for v in posx]
        req.vel = [float(vel), float(vel * 0.67)]  # [mm/s, deg/s]
        req.acc = [float(acc), float(acc * 0.67)]  # [mm/s², deg/s²]
        req.time = 0.0
        req.radius = 0.0
        req.ref = 0        # DR_BASE
        req.mode = 0       # DR_MV_MOD_ABS
        req.blend_type = 0
        req.sync_type = 0  # SYNC (완료 대기)

        res = self.call_service(MoveLine, "motion/move_line", req, timeout=60.0)
        return bool(res and res.success)

    def move_joint(
        self,
        posj: list[float],
        vel: float = 20.0,
        acc: float = 40.0,
        dry_run: bool = False,
    ) -> bool:
        """관절 각도(posj)로 회전 이동 (MoveJoint)."""
        validate_posj(posj)
        vel = min(vel, MAX_JOINT_VEL_DPS)
        acc = min(acc, MAX_JOINT_ACC_DPS2)

        if dry_run:
            print(f"[dry-run] MoveJoint 목표 posj={posj}, vel={vel} deg/s, acc={acc} deg/s² 검증 통과")
            return True

        from dsr_msgs2.srv import MoveJoint

        req = MoveJoint.Request()
        req.pos = [float(v) for v in posj]
        req.vel = float(vel)
        req.acc = float(acc)
        req.time = 0.0
        req.radius = 0.0
        req.mode = 0       # MOVE_MODE_ABSOLUTE
        req.blend_type = 0
        req.sync_type = 0  # SYNC (완료 대기)

        res = self.call_service(MoveJoint, "motion/move_joint", req, timeout=60.0)
        return bool(res and res.success)

    def control_gripper(
        self,
        action: str,
        pulse: int | None = None,
        current_ma: int = CURRENT_DEFAULT_MA,
        dry_run: bool = False,
    ) -> bool:
        """RH-P12-RN 그리퍼 제어.

        action: 'open', 'close', 'set'
        """
        action = action.lower()
        if action == "open":
            target_pulse = PULSE_MIN if pulse is None else pulse
        elif action == "close":
            target_pulse = PULSE_WORK_MAX if pulse is None else pulse
        elif action == "set":
            if pulse is None:
                raise ValueError("set 동작에는 --pulse 값이 필요합니다.")
            target_pulse = pulse
        else:
            raise ValueError(f"지원하지 않는 그리퍼 동작입니다: {action}")

        validate_gripper(target_pulse, current_ma)

        if dry_run:
            print(f"[dry-run] 그리퍼 action='{action}', pulse={target_pulse}, current={current_ma} mA 검증 성공")
            return True

        # 프로젝트 내 검증된 gripper_cmd 로직 연동
        here = Path(__file__).resolve().parent
        bench_script = here.parents[1] / "tools/bench/gripper_cmd.py"
        if not bench_script.exists():
            # 스킬 scripts 폴더 기준 fallback
            bench_script = Path("/home/umm/conveyor-tracker-bot/tools/bench/gripper_cmd.py")

        cmd = [
            sys.executable,
            str(bench_script),
            action,
            "--pulse",
            str(target_pulse),
            "--current",
            str(current_ma),
            "--ns",
            self.ns,
        ]

        import subprocess
        p = subprocess.run(cmd, capture_output=True, text=True)
        if p.returncode != 0:
            print(p.stderr or p.stdout, file=sys.stderr)
            return False
        if p.stdout:
            print(p.stdout.strip())
        return True


def ensure_ros_initialized() -> Any:
    """ROS 2 환경 초기화 및 노드 생성."""
    try:
        import rclpy
    except ImportError as exc:
        raise RuntimeError(
            "ROS 2 환경을 찾을 수 없습니다. 먼저 환경을 소싱하세요:\n"
            "  source /opt/ros/jazzy/setup.bash && source install/setup.bash"
        ) from exc

    if not rclpy.ok():
        rclpy.init()
    return rclpy.create_node("e0509_ctl")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="E0509 로봇 및 RH-P12-RN 그리퍼 ROS 2 통합 제어 도구",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--ns", default=DEFAULT_NAMESPACE, help=f"로봇 네임스페이스 (기본값 {DEFAULT_NAMESPACE})")
    sub = parser.add_subparsers(dest="command", required=True, help="실행할 명령")

    # check
    sub.add_parser("check", help="제어기 네트워크 및 연결 상태 점검")

    # status
    p_status = sub.add_parser("status", help="로봇 상태, 모드, 알람, 현재 좌표 조회")
    p_status.add_argument("--json", action="store_true", help="JSON 형식으로 출력")

    # home
    p_home = sub.add_parser("home", help="홈 위치로 복귀")
    p_home.add_argument(
        "--type",
        choices=["standby", "mechanical", "user"],
        default="standby",
        help="홈 복귀 종류 (standby: 대기 자세 [372.96, 0.0, 404.80], mechanical: 관절 0°, user: 사용자홈)",
    )
    p_home.add_argument("--vel", type=float, default=30.0, help="이동 속도 [mm/s 또는 deg/s] (기본 30)")
    p_home.add_argument("--acc", type=float, default=60.0, help="이동 가속도 [mm/s² 또는 deg/s²] (기본 60)")
    p_home.add_argument("--dry-run", action="store_true", help="실제 모션 없이 명령과 파라미터만 검증")

    # move-posx
    p_movx = sub.add_parser("move-posx", help="직교 좌표 [x y z rx ry rz]로 직선 이동")
    p_movx.add_argument("coords", nargs=6, type=float, help="목표 좌표 (x y z mm, rx ry rz deg)")
    p_movx.add_argument("--vel", type=float, default=30.0, help="속도 mm/s (최대 100)")
    p_movx.add_argument("--acc", type=float, default=60.0, help="가속도 mm/s² (최대 250)")
    p_movx.add_argument("--dry-run", action="store_true", help="검증만 수행")

    # move-posj
    p_movj = sub.add_parser("move-posj", help="관절 각도 [j1..j6]로 관절 이동")
    p_movj.add_argument("angles", nargs=6, type=float, help="목표 관절 각도 (j1..j6 deg)")
    p_movj.add_argument("--vel", type=float, default=20.0, help="속도 deg/s (최대 45)")
    p_movj.add_argument("--acc", type=float, default=40.0, help="가속도 deg/s² (최대 100)")
    p_movj.add_argument("--dry-run", action="store_true", help="검증만 수행")

    # stop
    sub.add_parser("stop", help="로봇 긴급 모션 정지")

    # mode
    p_mode = sub.add_parser("mode", help="로봇 제어 모드 설정")
    p_mode.add_argument("target_mode", choices=["auto", "manual"], help="auto(1, 자율) 또는 manual(0, 수동)")

    # gripper
    p_grp = sub.add_parser("gripper", help="RH-P12-RN 그리퍼 제어")
    p_grp.add_argument("action", choices=["open", "close", "set"], help="그리퍼 동작")
    p_grp.add_argument("--pulse", type=int, default=None, help="목표 pulse (open: 0, close: 750, set: 0..1150)")
    p_grp.add_argument("--current", type=int, default=CURRENT_DEFAULT_MA, help="파지 전류 mA (1..100)")
    p_grp.add_argument("--dry-run", action="store_true", help="검증만 수행")

    args = parser.parse_args(argv)

    if args.command == "check":
        ok = check_network(DEFAULT_ROBOT_HOST, DEFAULT_ROBOT_PORT)
        print(f"제어기 네트워크 점검 ({DEFAULT_ROBOT_HOST}:{DEFAULT_ROBOT_PORT}): {'[정상]' if ok else '[연결 실패]'}")
        return 0 if ok else 1

    # dry-run인 경우 ROS 2 환경 초기화 없이 파라미터 및 안전 검증만 수행
    is_dry_run = getattr(args, "dry_run", False)
    if is_dry_run:
        client = E0509Client(node=None, ns=args.ns)
        if args.command == "home":
            success = client.move_home(home_type=args.type, vel=args.vel, acc=args.acc, dry_run=True)
            return 0 if success else 1
        elif args.command == "move-posx":
            success = client.move_line(args.coords, vel=args.vel, acc=args.acc, dry_run=True)
            return 0 if success else 1
        elif args.command == "move-posj":
            success = client.move_joint(args.angles, vel=args.vel, acc=args.acc, dry_run=True)
            return 0 if success else 1
        elif args.command == "gripper":
            success = client.control_gripper(action=args.action, pulse=args.pulse, current_ma=args.current, dry_run=True)
            return 0 if success else 1

    try:
        node = ensure_ros_initialized()
    except Exception as exc:
        print(f"오류: {exc}", file=sys.stderr)
        return 1

    client = E0509Client(node, ns=args.ns)

    try:
        if args.command == "status":
            st = client.get_status()
            if args.json:
                print(json.dumps(asdict(st), indent=2, ensure_ascii=False))
            else:
                print("================ E0509 로봇 상태 ================")
                print(f"  연결 상태  : {'정상' if st.connected else '단절'}")
                print(f"  로봇 State : {st.state_str} ({st.state})")
                print(f"  로봇 Mode  : {st.mode_str} ({st.mode})")
                print(f"  DRL State  : {st.drl_state}")
                print(f"  최근 알람  : {st.alarm_msg}")
                print(f"  현재 posx  : {[round(v, 2) for v in st.posx]} mm, deg")
                print(f"  현재 posj  : {[round(v, 2) for v in st.posj]} deg")
                print("==================================================")
            return 0

        elif args.command == "home":
            print(f"E0509 홈 복귀 시작 (타입: {args.type}, dry-run={args.dry_run})...")
            success = client.move_home(
                home_type=args.type,
                vel=args.vel,
                acc=args.acc,
                dry_run=args.dry_run,
            )
            print(f"홈 복귀 결과: {'[성공]' if success else '[실패]'}")
            return 0 if success else 1

        elif args.command == "move-posx":
            print(f"E0509 MoveLine 시작: posx={args.coords} (dry-run={args.dry_run})...")
            success = client.move_line(args.coords, vel=args.vel, acc=args.acc, dry_run=args.dry_run)
            print(f"MoveLine 결과: {'[성공]' if success else '[실패]'}")
            return 0 if success else 1

        elif args.command == "move-posj":
            print(f"E0509 MoveJoint 시작: posj={args.angles} (dry-run={args.dry_run})...")
            success = client.move_joint(args.angles, vel=args.vel, acc=args.acc, dry_run=args.dry_run)
            print(f"MoveJoint 결과: {'[성공]' if success else '[실패]'}")
            return 0 if success else 1

        elif args.command == "stop":
            print("E0509 모션 정지(MoveStop) 요청...")
            success = client.stop()
            print(f"정지 결과: {'[성공]' if success else '[실패]'}")
            return 0 if success else 1

        elif args.command == "mode":
            mode_val = 1 if args.target_mode == "auto" else 0
            print(f"E0509 모드 전환: {args.target_mode} ({mode_val})...")
            success = client.set_mode(mode_val)
            print(f"모드 전환 결과: {'[성공]' if success else '[실패]'}")
            return 0 if success else 1

        elif args.command == "gripper":
            print(f"그리퍼 명령 시작: {args.action} (dry-run={args.dry_run})...")
            success = client.control_gripper(
                action=args.action,
                pulse=args.pulse,
                current_ma=args.current,
                dry_run=args.dry_run,
            )
            print(f"그리퍼 제어 결과: {'[성공]' if success else '[실패]'}")
            return 0 if success else 1

    except Exception as exc:
        print(f"[실행 오류] {exc}", file=sys.stderr)
        return 2
    finally:
        try:
            import rclpy
            if rclpy.ok():
                node.destroy_node()
                rclpy.shutdown()
        except Exception:
            pass

    return 0


if __name__ == "__main__":
    sys.exit(main())
