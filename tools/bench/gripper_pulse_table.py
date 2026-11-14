#!/usr/bin/env python3
"""RH-P12-RN(A) pulse ↔ 핑거 간격 실측표 측정.

상주 루프 DRL(참고 구현 dsr_gripper/gripper_service.py의 루프)이 이미 돌고 있다고 보고,
명령 레지스터(기본 10~14)에 **하드웨어 Goal Position 값**을 직접 쓴다.
참고 구현 서비스 `/dsr01/gripper/cmd`는 0~750으로 제한·반전하므로 이 측정에 쓰지 않는다.

순서:
  0) 탐색 패스: 100 pulse 간격으로 증가 방향 1회 → 공깃돌 지름 ±10 mm 구간의 pulse 범위 계산
  1~3) 증가 방향 3회: 100 pulse 간격 + 위 구간은 25 pulse 간격
  4) 감소 방향 1회: 같은 점(히스테리시스)
매 점마다 정착 대기 후 간격을 입력받아 CSV에 바로 쓴다(중단해도 그때까지 저장).

사용 예:
  python3 tools/bench/gripper_pulse_table.py --stone-mm 18 --start-loop-via-service
  python3 tools/bench/gripper_pulse_table.py --dry-run          # ROS 없이 순서만 확인

입력: 숫자(mm) / Enter = 측정 안 함 / r = 같은 점 재전송 / q = 종료
"""
from __future__ import annotations

import argparse
import csv
import datetime as _dt
import os
import sys
import time
from dataclasses import dataclass
from typing import Callable, Iterable, Sequence

PULSE_MIN = 0
PULSE_MAX = 1150
CURRENT_MAX_MA = 100          # 사용자 지정 현장 전류 상한
REG_DEFAULT = {"seq": 10, "pos_hi": 11, "pos_lo": 12, "cur_hi": 13, "cur_lo": 14}

CSV_FIELDS = ["date", "seq", "direction", "pass", "cmd_pulse", "goal_current",
              "settle_s", "gap_mm", "instrument", "measured_by", "note"]


# ---------------------------------------------------------------- 순수 함수
def encode7(value: int) -> tuple[int, int]:
    """참고 구현 레지스터 코덱: 14비트 값을 7비트 두 칸(hi, lo)으로 나눈다."""
    if not isinstance(value, int) or value < 0 or value > 0x3FFF:
        raise ValueError(f"OUT_OF_RANGE: {value!r}")
    return (value >> 7) & 0x7F, value & 0x7F


def decode7(hi: int, lo: int) -> int:
    return (hi << 7) | lo


def next_seq(prev: int) -> int:
    """루프는 seq가 바뀔 때만 실행한다. 1~127을 순환하고 prev와 같은 값을 내지 않는다."""
    s = (prev % 127) + 1
    return s


def coarse_points(step: int = 100, lo: int = PULSE_MIN, hi: int = PULSE_MAX) -> list[int]:
    pts = list(range(lo, hi + 1, step))
    if pts[-1] != hi:
        pts.append(hi)
    return pts


def interp_pulse_for_gap(samples: Sequence[tuple[int, float]], gap: float) -> float | None:
    """(pulse, gap) 표본에서 gap에 해당하는 pulse를 선형 보간. 범위 밖이면 None."""
    pts = sorted(samples)
    for (p0, g0), (p1, g1) in zip(pts, pts[1:]):
        if g0 == g1:
            continue
        if min(g0, g1) <= gap <= max(g0, g1):
            return p0 + (gap - g0) * (p1 - p0) / (g1 - g0)
    return None


def fine_range(samples: Sequence[tuple[int, float]], stone_mm: float, band_mm: float = 10.0,
               step: int = 25) -> list[int]:
    """공깃돌 지름 ± band 구간에 해당하는 pulse를 step 간격으로 반환(step 배수로 바깥쪽 확장).

    한쪽 끝이 측정 범위 밖이면 그 간격에 가장 가까운 표본의 pulse를 끝으로 쓴다.
    """
    if len(samples) < 2:
        return []
    ends = []
    for g in (stone_mm - band_mm, stone_mm + band_mm):
        p = interp_pulse_for_gap(samples, g)
        if p is None:
            p = min(samples, key=lambda s: abs(s[1] - g))[0]
        ends.append(p)
    lo = max(PULSE_MIN, int(min(ends) // step * step))
    hi = min(PULSE_MAX, int(-(-max(ends) // step) * step))
    if lo == hi:
        return []
    return list(range(lo, hi + 1, step))


def merge_points(*groups: Iterable[int]) -> list[int]:
    return sorted({p for g in groups for p in g})


class Quit(Exception):
    """사용자가 q로 측정을 끝냄."""


@dataclass
class Command:
    pulse: int
    current_ma: int


class RegisterWriter:
    """명령 레지스터 쓰기 순서: payload 4칸 → seq(트리거) 마지막(와 같은 규칙)."""

    def __init__(self, write: Callable[[int, int], bool], regs: dict[str, int] | None = None,
                 seq0: int = 63):
        self._write = write
        self.regs = dict(REG_DEFAULT if regs is None else regs)
        self.seq = seq0

    def send(self, cmd: Command) -> int:
        if not (PULSE_MIN <= cmd.pulse <= PULSE_MAX):
            raise ValueError(f"OUT_OF_RANGE pulse {cmd.pulse}")
        if not (0 < cmd.current_ma <= CURRENT_MAX_MA):
            raise ValueError(f"OUT_OF_RANGE current {cmd.current_ma}")
        ph, pl = encode7(cmd.pulse)
        ch, cl = encode7(cmd.current_ma)
        for name, val in (("pos_hi", ph), ("pos_lo", pl), ("cur_hi", ch), ("cur_lo", cl)):
            if not self._write(self.regs[name], val):
                raise RuntimeError(f"register write failed: {name}={val}")
        self.seq = next_seq(self.seq)
        if not self._write(self.regs["seq"], self.seq):
            raise RuntimeError("register write failed: seq")
        return self.seq


# ---------------------------------------------------------------- ROS 계층
class RosRegisterTransport:
    def __init__(self, ns: str, timeout_s: float = 2.0):
        import rclpy
        from dsr_msgs2.srv import SetOutputRegisterInt
        self._rclpy = rclpy
        self._T = SetOutputRegisterInt
        rclpy.init()
        self.node = rclpy.create_node("gripper_pulse_table")
        self.timeout_s = timeout_s
        name = f"{ns}/dsr_controller2/plc/set_output_register_int"
        self.cli = self.node.create_client(SetOutputRegisterInt, name)
        if not self.cli.wait_for_service(timeout_sec=5.0):
            raise RuntimeError(f"서비스 없음: {name}")

    def write(self, address: int, value: int) -> bool:
        req = self._T.Request()
        req.address = address
        req.value = value
        fut = self.cli.call_async(req)
        self._rclpy.spin_until_future_complete(self.node, fut, timeout_sec=self.timeout_s)
        res = fut.result()
        return bool(res is not None and res.success)

    def start_loop_via_service(self, ns: str, current_ma: int) -> None:
        """참고 구현 서비스로 1회 '완전 열기'를 보내 상주 루프를 띄운다(서비스 규약 750 = 하드웨어 0)."""
        from dsr_gripper_interfaces.srv import GripperCmd
        cli = self.node.create_client(GripperCmd, f"{ns}/gripper/cmd")
        if not cli.wait_for_service(timeout_sec=5.0):
            raise RuntimeError(f"서비스 없음: {ns}/gripper/cmd")
        req = GripperCmd.Request()
        req.position = 750
        req.current = current_ma
        fut = cli.call_async(req)
        self._rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30.0)
        if fut.result() is None or not fut.result().success:
            raise RuntimeError("gripper/cmd 실패")

    def close(self) -> None:
        self.node.destroy_node()
        self._rclpy.shutdown()


class DryTransport:
    def __init__(self):
        self.log: list[tuple[int, int]] = []

    def write(self, address: int, value: int) -> bool:
        self.log.append((address, value))
        return True


# ---------------------------------------------------------------- 측정 진행
def ask_gap(prompt: str, inp: Callable[[str], str] = input) -> str:
    while True:
        s = inp(prompt).strip().lower()
        if s in ("", "r", "q"):
            return s
        try:
            v = float(s)
        except ValueError:
            print("  숫자(mm), Enter, r, q 중 하나")
            continue
        if 0.0 <= v <= 120.0:
            return s
        print("  0~120 mm 범위 밖")


def run_pass(writer: RegisterWriter, pts: Sequence[int], direction: str, pass_no: int,
             args, out: csv.DictWriter, fh, inp=input, sleep=time.sleep) -> list[tuple[int, float]]:
    got: list[tuple[int, float]] = []
    seq_pts = list(pts) if direction == "up" else list(reversed(pts))
    print(f"\n=== pass {pass_no} ({direction}), {len(seq_pts)}점 ===")
    for p in seq_pts:
        while True:
            writer.send(Command(p, args.current))
            sleep(args.settle)
            ans = ask_gap(f"  pulse {p:4d} → 간격 mm (캘리퍼를 뺀 뒤 입력): ", inp)
            if ans != "r":
                break
        if ans == "q":
            raise Quit
        gap = "" if ans == "" else float(ans)
        out.writerow({"date": _dt.date.today().isoformat(), "seq": writer.seq, "direction": direction,
                      "pass": pass_no, "cmd_pulse": p, "goal_current": args.current,
                      "settle_s": args.settle, "gap_mm": gap, "instrument": args.instrument,
                      "measured_by": args.measured_by, "note": ""})
        fh.flush()
        if gap != "":
            got.append((p, gap))
    return got


def main(argv: Sequence[str] | None = None, inp=input, transport=None, sleep=time.sleep) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ns", default="/dsr01")
    ap.add_argument("--stone-mm", type=float, required=True, help="공깃돌 지름(실측 최대값)")
    ap.add_argument("--current", type=int, default=100, help=f"Goal Current [mA], 1~{CURRENT_MAX_MA}")
    ap.add_argument("--settle", type=float, default=1.5, help="정착 대기 [s]")
    ap.add_argument("--passes-up", type=int, default=3)
    ap.add_argument("--passes-down", type=int, default=1)
    ap.add_argument("--instrument", default="caliper")
    ap.add_argument("--measured-by", default="")
    ap.add_argument("--out", default=None, help="기본 records/bench/gripper-table-<날짜>.csv")
    ap.add_argument("--reg-base", type=int, default=10, help="seq 레지스터 주소, payload는 +1~+4")
    ap.add_argument("--start-loop-via-service", action="store_true",
                    help="참고 구현 gripper/cmd로 완전 열기 1회를 보내 상주 루프를 띄운다")
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args(argv)

    if not (0 < args.current <= CURRENT_MAX_MA):
        ap.error(f"--current 는 1~{CURRENT_MAX_MA}")
    repo = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
    out_path = args.out or os.path.join(repo, "records", "bench",
                                        f"gripper-table-{_dt.date.today().isoformat()}.csv")
    os.makedirs(os.path.dirname(out_path), exist_ok=True)

    regs = {"seq": args.reg_base, "pos_hi": args.reg_base + 1, "pos_lo": args.reg_base + 2,
            "cur_hi": args.reg_base + 3, "cur_lo": args.reg_base + 4}
    if transport is None:
        transport = DryTransport() if args.dry_run else RosRegisterTransport(args.ns)
    if args.start_loop_via_service and not args.dry_run:
        transport.start_loop_via_service(args.ns, args.current)
        sleep(1.0)
    writer = RegisterWriter(transport.write, regs)

    print("확인: 핑거 사이에 물체가 없다 / 손이 핑거 사이에 없다 / 로봇은 정지 상태다")
    if inp("계속하려면 yes: ").strip() != "yes":
        return 1

    new = not os.path.exists(out_path)
    with open(out_path, "a", newline="") as fh:
        out = csv.DictWriter(fh, fieldnames=CSV_FIELDS)
        if new:
            out.writeheader()
        coarse = coarse_points()
        try:
            survey = run_pass(writer, coarse, "up", 0, args, out, fh, inp, sleep)
            fine = fine_range(survey, args.stone_mm)
            if not fine:
                print("탐색 패스로 공깃돌 구간을 정하지 못했다(간격 입력이 2개 미만). 100 pulse 간격만 측정한다.")
            pts = merge_points(coarse, fine)
            print(f"\n측정점 {len(pts)}개(정밀 구간 {fine[0] if fine else '-'}~{fine[-1] if fine else '-'} pulse)")
            n = 1
            for _ in range(args.passes_up):
                run_pass(writer, pts, "up", n, args, out, fh, inp, sleep); n += 1
            for _ in range(args.passes_down):
                run_pass(writer, pts, "down", n, args, out, fh, inp, sleep); n += 1
        except Quit:
            print("사용자 종료")
        finally:
            writer.send(Command(PULSE_MIN, args.current))   # 끝나면 0 pulse(개방 쪽)로 복귀
    print(f"\n저장: {out_path}")
    print("주의: 이 스크립트가 seq 레지스터를 바꿨으므로 기존 gripper/cmd 서비스의 다음 명령 1회가 무시될 수 있다. "
          "응답이 없으면 같은 명령을 한 번 더 보낸다.")
    if hasattr(transport, "close"):
        transport.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
