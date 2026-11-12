"""Doosan speedl 스트림 + 상주 루프 그리퍼 해제 transport. **실기 미검증**(SPEC-500 REQ-ROB-015).

`robot_controller_node`의 `backend` 선택지에 연결하지 않는다. OFF-2 측정(상주 루프와 speedl 동시 동작, TCP
피드백 주기·나이, `move_stop` 정지 확인, 개방 지연) 뒤 노드에 연결한다.

- 지령 형식은 OFF-2 측정 도구와 같다(`tools/bench/stream_bench.py`): `<ns>/speedl_stream`에
  `SpeedlStream(vel=[mm/s×3, deg/s×3], acc=[mm/s², deg/s²], time=2주기)`. 메시지 형식(`dsr_msgs2.msg.SpeedlStream`)과
  발행 함수는 주입한다. 단위 변환은 `motion_adapter`에서만 한다.
- 지령마다 `FreshnessWatchdog`(주기 2배)와 속도 한도를 거친다. 넘으면 속도 0을 1회 보내고 `rearm()` 전까지
  거부한다(L0, REQ-ROB-003·004). 지령이 끊긴 것은 노드 타이머가 부르는 `tick()`이 잡는다.
- commit = `GripperSession.fire_open` 1회. fire_open은 그리퍼 도달까지 기다리므로 제어 주기 밖(`runner`, 기본은
  데몬 스레드)에서 부른다. 개방 목표는 goal 시작 때 `prepare()`로 미리 써 둔다(쓰지 않았으면 commit 때 쓴다:
  `late_arm`). 개방 확인은 상주 루프 상태 표본 주기(제안값 0.58 s, REQ-GRP-016)에 묶인다.
- `abort(keep_sync=True)`는 `tick()`이 `abort_sync_hold_s` 동안 계획 XY 속도(z 0)를 계속 보내고 멈춘다.
  goal 사이에도 이어지므로 노드는 goal이 없을 때도 `tick()`을 주기마다 불러야 한다.
- TCP·속도 피드백은 주입한 `read_tcp() -> (ROS 시각, pos_m, vel_mps) | None`이 준다(경로·주기 TBD OFF-2).
"""
from __future__ import annotations

import math
import threading

import numpy as np

from .dynamic_place import MotionRef, ReleasePermit, TransportCaps, TransportSample
from .motion_adapter import CommandAuthority, FreshnessWatchdog, accel_si_to_doosan, speed_si_to_doosan

ZERO = (0.0, 0.0, 0.0)


def _thread_runner(fn) -> None:
    threading.Thread(target=fn, daemon=True).start()


class DoosanSpeedlTransport:
    def __init__(self, publish, speedl_type, clock, read_tcp, gripper, *, period_s: float = .01,
                 commit_lead_s: float = .15, v_max_mps: float = .1, a_max_mps2: float = .3,
                 ang_acc_radps2: float = math.pi / 3, abort_sync_hold_s: float = 1.0, stop_speed_mps: float = 1e-4,
                 move_stop=None, runner=None):
        self._publish = publish
        self._msg = speedl_type
        self.clock = clock
        self._read_tcp = read_tcp
        self.gripper = gripper
        self.period_s = float(period_s)
        self.capabilities = TransportCaps(True, float(commit_lead_s), True)
        self.watchdog = FreshnessWatchdog(self.period_s, clock)
        self.authority = CommandAuthority(v_max_mps=v_max_mps, a_max_mps2=a_max_mps2)
        self._acc = accel_si_to_doosan(a_max_mps2, ang_acc_radps2)
        self.abort_sync_hold_s = float(abort_sync_hold_s)
        self.stop_speed_mps = float(stop_speed_mps)
        self._move_stop = move_stop
        self._runner = runner or _thread_runner
        self.l0 = ''                       # '' / STREAM_STALE / RANGE / NONFINITE
        self.committed = False
        self.released = False
        self.late_arm = False
        self.width_m = math.nan
        self.gripper_fault = ''
        self._stopped = False
        self._zero_sent = False
        self._ff = np.zeros(3)
        self._hold_until = None            # steady: abort(keep_sync) 동안 계획 속도 유지

    # ------------------------------------------------------------ 지령
    def _send_zero(self) -> None:
        if not self._zero_sent:
            self._zero_sent = True
            self._publish(self._msg(vel=[0.0] * 6, acc=list(self._acc), time=2.0 * self.period_s))

    def _latch(self, reason: str) -> bool:
        self.l0 = self.l0 or reason
        self._send_zero()
        return False

    def _send(self, v_mps) -> bool:
        if self._stopped or self.l0:
            return False
        reason = self.authority.validate_speed(v_mps)
        if reason:
            return self._latch(reason)
        if not self.watchdog.on_command():
            return self._latch('STREAM_STALE')
        self._zero_sent = False
        self._publish(self._msg(vel=speed_si_to_doosan(v_mps, ZERO), acc=list(self._acc), time=2.0 * self.period_s))
        return True

    def rearm(self) -> None:
        """작업자 확인 뒤 L0 해제."""
        self.l0 = ''
        self.watchdog.rearm()

    def tick(self) -> None:
        """노드 타이머(주기 `period_s`)가 goal 유무와 관계없이 부른다."""
        if self._hold_until is not None:
            if self.clock.steady_now() <= self._hold_until + 1e-12:
                self._send((self._ff[0], self._ff[1], 0.0))
                return
            self._hold_until = None
            self._send_zero()
            return
        if not self.watchdog.check():
            self._latch('STREAM_STALE')

    # ------------------------------------------------------------ transport 계약
    def prepare(self, open_width_m: float) -> None:
        """goal 시작 때 개방 목표만 미리 쓴다(그리퍼는 움직이지 않음)."""
        self.gripper.arm_open(open_width_m)
        self.committed = self.released = self.late_arm = False
        self.gripper_fault = ''

    def update(self, ref: MotionRef) -> bool:
        self._hold_until = None
        self._ff = np.asarray(ref.velocity_ff_mps, dtype=float)
        return self._send(ref.command_mps)

    def commit_release(self, permit: ReleasePermit) -> bool:
        if self.committed or self._stopped or self.l0:
            return False
        if not getattr(self.gripper, 'armed', False):
            self.late_arm = True
            self.gripper.arm_open(permit.open_width_m)
        self.committed = True

        def fire():
            try:
                result = self.gripper.fire_open()
                self.width_m = float(result.width_m) if result.width_m is not None else math.nan
                self.released = True
            except Exception as exc:       # 그리퍼 오류는 poll()의 fault로 알린다
                self.gripper_fault = f'GRIPPER_FAULT:{exc}'
        self._runner(fire)
        return True

    def poll(self) -> TransportSample:
        tcp = self._read_tcp()
        fault = self.l0 or self.gripper_fault
        if tcp is None:
            return TransportSample(float(self.clock.ros_now()), (math.nan,) * 3, (math.nan,) * 3, 'robot_base',
                                   self.width_m, self.released, self.committed, fault or 'TCP_UNAVAILABLE')
        t, pos, vel = tcp
        return TransportSample(float(t), tuple(map(float, pos)), tuple(map(float, vel)), 'robot_base',
                               self.width_m, self.released, self.committed, fault)

    def abort(self, keep_sync: bool) -> bool:
        if keep_sync and not self._stopped and not self.l0:
            self._hold_until = self.clock.steady_now() + self.abort_sync_hold_s
            return True
        self._hold_until = None
        self._send_zero()
        return True

    def stop(self) -> bool:
        self._stopped = True
        self._hold_until = None
        self._send_zero()
        if self._move_stop is not None:
            self._move_stop()
        return True

    def is_stopped(self) -> bool:
        tcp = self._read_tcp()
        return tcp is not None and float(np.linalg.norm(np.asarray(tcp[2], dtype=float))) <= self.stop_speed_mps

    def reset_stop(self) -> None:
        self._stopped = False
        self.rearm()
