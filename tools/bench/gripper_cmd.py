#!/usr/bin/env python3
"""RH-P12-RN(A) 그리퍼 열기/닫기/위치 제어 CLI 도구.

사용법:
  # 그리퍼 완전 열기 (0 pulse, ~106 mm)
  python3 tools/bench/gripper_cmd.py open

  # 그리퍼 닫기 (1150 pulse, 파지 전류 100 mA)
  python3 tools/bench/gripper_cmd.py close

  # 파지 전류 지정해서 닫기 (예: 100 mA)
  python3 tools/bench/gripper_cmd.py close --current 100

  # 특정 pulse로 이동 (0~1150)
  python3 tools/bench/gripper_cmd.py set --pulse 500 --current 100
"""

from __future__ import annotations

import argparse
import sys

PULSE_MIN = 0
PULSE_MAX = 1150
CURRENT_DEFAULT_MA = 100
CURRENT_MAX_MA = 100

REG_DEFAULT = {"seq": 10, "pos_hi": 11, "pos_lo": 12, "cur_hi": 13, "cur_lo": 14}


def encode7(value: int) -> tuple[int, int]:
    if not isinstance(value, int) or value < 0 or value > 0x3FFF:
        raise ValueError(f"OUT_OF_RANGE: {value!r}")
    return (value >> 7) & 0x7F, value & 0x7F


def next_seq(prev: int) -> int:
    return (prev % 127) + 1


def send_register_command(cli, srv_type, node, rclpy_mod, pulse: int, current_ma: int,
                          regs: dict[str, int], seq_start: int = 1) -> int:
    if type(current_ma) is not int or not 1 <= current_ma <= CURRENT_MAX_MA:
        raise ValueError("OUT_OF_RANGE current: allowed 1..100 mA")
    ph, pl = encode7(pulse)
    ch, cl = encode7(current_ma)
    payload = [
        ("pos_hi", regs["pos_hi"], ph),
        ("pos_lo", regs["pos_lo"], pl),
        ("cur_hi", regs["cur_hi"], ch),
        ("cur_lo", regs["cur_lo"], cl),
    ]

    for name, addr, val in payload:
        req = srv_type.Request()
        req.address = addr
        req.value = val
        fut = cli.call_async(req)
        rclpy_mod.spin_until_future_complete(node, fut, timeout_sec=2.0)
        res = fut.result()
        if res is None or not res.success:
            raise RuntimeError(f"레지스터 쓰기 실패: {name}(주소 {addr}) = {val}")

    # seq (트리거)
    seq = next_seq(seq_start)
    req = srv_type.Request()
    req.address = regs["seq"]
    req.value = seq
    fut = cli.call_async(req)
    rclpy_mod.spin_until_future_complete(node, fut, timeout_sec=2.0)
    res = fut.result()
    if res is None or not res.success:
        raise RuntimeError(f"트리거 레지스터 쓰기 실패: seq(주소 {regs['seq']}) = {seq}")

    return seq


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True, help="수행할 동작")

    # open
    p_open = sub.add_parser("open", help="그리퍼 열기 (0 pulse)")
    p_open.add_argument("--pulse", type=int, default=PULSE_MIN, help=f"열림 목표 pulse (기본 {PULSE_MIN})")
    p_open.add_argument("--current", type=int, default=CURRENT_DEFAULT_MA, help=f"전류 [mA] (기본 {CURRENT_DEFAULT_MA})")
    p_open.add_argument("--ns", default="/dsr01", help="로봇 namespace (기본 /dsr01)")

    # close
    p_close = sub.add_parser("close", help="그리퍼 닫기 (1150 pulse)")
    p_close.add_argument("--pulse", type=int, default=PULSE_MAX, help=f"닫힘 목표 pulse (기본 {PULSE_MAX})")
    p_close.add_argument("--current", type=int, default=CURRENT_DEFAULT_MA, help=f"파지 전류 [mA] (기본 {CURRENT_DEFAULT_MA}, 최대 {CURRENT_MAX_MA})")
    p_close.add_argument("--ns", default="/dsr01", help="로봇 namespace (기본 /dsr01)")

    # set
    p_set = sub.add_parser("set", help="임의 pulse로 이동")
    p_set.add_argument("--pulse", type=int, required=True, help=f"목표 pulse ({PULSE_MIN}~{PULSE_MAX})")
    p_set.add_argument("--current", type=int, default=CURRENT_DEFAULT_MA, help=f"전류 [mA] (기본 {CURRENT_DEFAULT_MA})")
    p_set.add_argument("--ns", default="/dsr01", help="로봇 namespace (기본 /dsr01)")

    args = parser.parse_args(argv)

    if not (PULSE_MIN <= args.pulse <= PULSE_MAX):
        parser.error(f"--pulse 는 {PULSE_MIN}~{PULSE_MAX} 범위여야 합니다.")
    if not (0 < args.current <= CURRENT_MAX_MA):
        parser.error(f"--current 는 1~{CURRENT_MAX_MA} mA 범위여야 합니다.")

    try:
        import rclpy
        from dsr_msgs2.srv import SetOutputRegisterInt
    except ImportError as e:
        print(f"[오류] ROS 2 또는 dsr_msgs2 환경이 로드되지 않았습니다:\n  {e}", file=sys.stderr)
        print("  먼저 아래 명령을 실행하세요:\n    source /opt/ros/jazzy/setup.bash && source install/setup.bash", file=sys.stderr)
        return 1

    rclpy.init()
    node = rclpy.create_node("gripper_cmd")
    srv_name = f"{args.ns}/dsr_controller2/plc/set_output_register_int"
    cli = node.create_client(SetOutputRegisterInt, srv_name)

    print(f"서비스 연결 대기 중: {srv_name} (최대 5초)...")
    if not cli.wait_for_service(timeout_sec=5.0):
        # 대안: /gripper/cmd 확인
        alt_name = f"{args.ns}/gripper/cmd"
        print(f"  {srv_name} 서비스를 찾지 못했습니다. 참고 구현 서비스 {alt_name} 시도...")
        try:
            from dsr_gripper_interfaces.srv import GripperCmd
            alt_cli = node.create_client(GripperCmd, alt_name)
            if alt_cli.wait_for_service(timeout_sec=3.0):
                print(f"  {alt_name} 서비스 연결 성공! 참고 구현 방식으로 명령 전송...")
                req = GripperCmd.Request()
                # 참고 구현은 750 = 완전 열림(하드웨어 0), 0 = 완전 닫힘(하드웨어 750)
                req.position = max(0, min(750, 750 - int(args.pulse * 750 / PULSE_MAX)))
                req.current = args.current
                fut = alt_cli.call_async(req)
                rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
                res = fut.result()
                if res is not None and res.success:
                    print(f"  [성공] 그리퍼 명령 완료 (position={req.position}, current={req.current}mA)")
                    node.destroy_node()
                    rclpy.shutdown()
                    return 0
                else:
                    print(f"  [실패] {alt_name} 응답 실패", file=sys.stderr)
                    node.destroy_node()
                    rclpy.shutdown()
                    return 1
        except ImportError:
            pass

        print(f"[오류] 로봇 드라이버 서비스({srv_name})가 활성화되어 있지 않습니다.", file=sys.stderr)
        print("  로봇 드라이버가 실행 중인지 확인하세요:", file=sys.stderr)
        print("    ros2 launch drop_bringup robot.launch.py mode:=real", file=sys.stderr)
        node.destroy_node()
        rclpy.shutdown()
        return 1

    action_str = f"열기 (pulse={args.pulse})" if args.command == "open" else (
        f"닫기 (pulse={args.pulse}, current={args.current}mA)" if args.command == "close" else f"이동 (pulse={args.pulse})"
    )
    print(f"그리퍼 {action_str} 전송 중...")
    try:
        seq = send_register_command(cli, SetOutputRegisterInt, node, rclpy, args.pulse, args.current, REG_DEFAULT)
        print(f"[성공] 그리퍼 명령 전송 완료 (seq={seq})")
    except Exception as ex:
        print(f"[실패] {ex}", file=sys.stderr)
        node.destroy_node()
        rclpy.shutdown()
        return 1

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
