"""DynamicPlace용 시뮬레이션 transport(`backend:=fake`와 U 시험). 실제 로봇·그리퍼에 연결하지 않는다.

값은 하드웨어 측정이 아니다. 가정값:
  - TCP: 속도 지령에 대한 1차 지연, 시정수 `lag_s` 29 ms(OFF-2 참조 부하 speedl 100 Hz에서 잰 지령 → 위치
    지연 29 ms를 시정수로 옮긴 값, Progress.md 2절). 마지막 지령은 `cmd_hold_s` 동안만 유지된다(speedl `time`).
  - 그리퍼: commit 뒤 `commit_lead_s`(기본 = `open_delay_s`)에 `open_width`로 열리고, 그 순간 공깃돌이 떨어진다
    (`stone_drop` = 그 시각과 TCP XY).
  - `mid_course_update=False`이면 한 DRL 프로그램 실행형을 흉내 낸다: commit 때의 위치에서 해제 자세까지
    직선 이동 → 해제 시각에 개방 → `hold_s` 동안 계획 속도 유지 → 계획 속도로 XY를 유지하며 상승.
`robot`을 주면 `SimulatedBackend`와 TCP·정지 상태를 함께 쓴다(`robot/status`와 정적 skill과 같은 TCP).
"""
from __future__ import annotations

import math

import numpy as np

from .dynamic_place import MotionRef, ReleasePermit, TransportCaps, TransportSample


class SimulatedPlaceTransport:
    def __init__(self, clock, pos=(0.0, 0.0, 0.1), robot=None, lag_s: float = .029, open_delay_s: float = .1,
                 commit_lead_s: float | None = None, mid_course_update: bool = True,
                 keep_sync_retreat: bool = True, abort_sync_hold_s: float = 1.0, cmd_hold_s: float = .1,
                 grip_width_m: float = .0175, step_s: float = .002):
        if lag_s <= 0 or open_delay_s < 0 or step_s <= 0:
            raise ValueError('lag_s·step_s는 양수, open_delay_s는 0 이상')
        self.clock = clock
        self.robot = robot
        self._pos = np.asarray(pos, dtype=float).copy()
        self.lag_s = float(lag_s)
        self.open_delay_s = float(open_delay_s)
        self.abort_sync_hold_s = float(abort_sync_hold_s)
        self.cmd_hold_s = float(cmd_hold_s)
        self.step_s = float(step_s)
        self.capabilities = TransportCaps(bool(mid_course_update),
                                          float(open_delay_s if commit_lead_s is None else commit_lead_s),
                                          bool(keep_sync_retreat))
        self.xy_speed_scale = 1.0           # 시험용 외란: XY 실제 속도 = 지령 × 이 값
        self.stamp_offset_s = 0.0           # 시험용: 표본 시각을 tick 시각보다 이만큼 늦게 찍는다
        self.vel = np.zeros(3)
        self.width_m = float(grip_width_m)
        self.released = False
        self.committed = False
        self.stone_drop = None              # (ROS 시각, (x, y))
        self.commit_count = 0
        self.commit_times: list[float] = []
        self.gripper_open_count = 0
        self.aborts: list[tuple[float, bool]] = []
        self.updates = 0
        self.updates_after_commit = 0
        self.log: list[TransportSample] = []
        self.steps = 0                      # 적분 단계 수(긴 정지 구간을 단계별로 적분하지 않는지 시험)
        self._cmd = np.zeros(3)
        self._cmd_until = -math.inf
        self._ff = np.zeros(3)
        self._last = float(clock.steady_now())
        self._open_at = None
        self._permit: ReleasePermit | None = None
        self._program = None
        self._stopped = False

    # ------------------------------------------------------------ 상태 공유
    @property
    def pos(self) -> np.ndarray:
        return self.robot.tcp_pos if self.robot is not None else self._pos

    @pos.setter
    def pos(self, value) -> None:
        value = np.asarray(value, dtype=float)
        if self.robot is not None:
            self.robot.tcp_pos = value        # 새 배열로 바꿔 끼운다(읽는 쪽 스레드는 복사본을 본다)
        else:
            self._pos = value

    def is_stopped(self) -> bool:
        return self.robot.is_stopped() if self.robot is not None else self._stopped

    def stop(self) -> bool:
        if self.robot is not None:
            self.robot.stop()
        self._stopped = True
        self.vel = np.zeros(3)
        self._cmd = np.zeros(3)
        self._program = None
        return True

    def reset_stop(self) -> None:
        reset = getattr(self.robot, 'reset_stop', None)
        if reset is not None:
            reset()
        self._stopped = False
        self._last = float(self.clock.steady_now())

    def reset_cycle(self, grip_width_m: float | None = None) -> None:
        """새 goal 전에 해제 상태를 지운다(같은 공깃돌을 다시 쥔 것으로 본다)."""
        self._advance()
        self.committed = self.released = False
        self._open_at = self._permit = self._program = None
        self.stone_drop = None
        if grip_width_m is not None:
            self.width_m = float(grip_width_m)

    # ------------------------------------------------------------ transport 계약
    def update(self, ref: MotionRef) -> bool:
        self._advance()
        if self.is_stopped() or (self.committed and not self.capabilities.mid_course_update):
            return False
        self._cmd = np.asarray(ref.command_mps, dtype=float).copy()
        self._ff = np.asarray(ref.velocity_ff_mps, dtype=float).copy()
        self._cmd_until = float(self.clock.steady_now()) + self.cmd_hold_s
        self.updates += 1
        if self.committed:
            self.updates_after_commit += 1
        return True

    def commit_release(self, permit: ReleasePermit) -> bool:
        self._advance()
        if self.is_stopped() or self.committed:
            return False
        now = float(self.clock.steady_now())
        lead = permit.t_release_s - permit.t_commit_s
        self.committed = True
        self.commit_count += 1
        self.commit_times.append(float(self.clock.ros_now()))
        self._permit = permit
        self._open_at = now + lead
        if not self.capabilities.mid_course_update:
            self._program = (now, self.pos.copy(), lead)
        return True

    def poll(self) -> TransportSample:
        self._advance()
        s = TransportSample(float(self.clock.ros_now()) + self.stamp_offset_s, tuple(float(v) for v in self.pos),
                            tuple(float(v) for v in self.vel), 'robot_base', self.width_m, self.released,
                            self.committed)
        self.log.append(s)
        return s

    def abort(self, keep_sync: bool) -> bool:
        self._advance()
        self.aborts.append((float(self.clock.ros_now()), bool(keep_sync)))
        self._program = None
        now = float(self.clock.steady_now())
        if keep_sync and self.capabilities.keep_sync_retreat:
            self._cmd = np.array([self._ff[0], self._ff[1], 0.0])     # z는 그대로, XY는 계획 속도
            self._cmd_until = now + self.abort_sync_hold_s
        else:
            # 1차 지연의 정지 거리(v·시정수)를 지금 반영하고 멈춘다: 다음 poll이 늦어도(정적 skill 이동 뒤 등)
            # 남은 속도가 공유 TCP를 밀지 않는다
            self._cmd = np.zeros(3)
            self._cmd_until = now
            self.pos = self.pos + self.vel * self.lag_s
            self.vel = np.zeros(3)
        return True

    def read_tcp_pose(self):
        self._advance()
        return self.pos.copy(), np.array([0.0, 0.0, 0.0, 1.0])

    # ------------------------------------------------------------ 시뮬레이션
    def _advance(self) -> None:
        now = float(self.clock.steady_now())
        if now <= self._last:
            return
        if self.is_stopped():
            self.vel = np.zeros(3)
            self._last = now
            return
        ros_now = float(self.clock.ros_now())
        if self._program is None and self._open_at is None:
            # 지령 유지 시간까지만 단계별로 적분하고, 그 뒤 지령 없는 구간은 닫힌 식으로 한 번에 적분한다
            split = max(self._last, min(now, self._cmd_until))
            if split > self._last:
                self._steps_to(split, now, ros_now)
            if now > self._last:
                total = now - self._last
                decay = math.exp(-total / self.lag_s)
                self.pos = self.pos + self.vel * self.lag_s * (1.0 - decay)
                self.vel = self.vel * decay if float(np.linalg.norm(self.vel)) * decay > 1e-12 else np.zeros(3)
                self.steps += 1
                self._last = now
            return
        self._steps_to(now, now, ros_now)

    def _steps_to(self, t_end, now, ros_now) -> None:
        total = t_end - self._last
        n = max(1, int(math.ceil(total / self.step_s)))
        h = total / n
        self.steps += n
        pos, vel = self.pos.copy(), self.vel.copy()
        scale = np.array([self.xy_speed_scale, self.xy_speed_scale, 1.0])
        alpha = 1.0 - math.exp(-h / self.lag_s)
        for k in range(1, n + 1):
            t = self._last + k * h
            if self._program is not None:
                pos, vel = self._program_state(t)
            else:
                cmd = self._cmd if t <= self._cmd_until + 1e-12 else np.zeros(3)
                vel = vel + (cmd * scale - vel) * alpha
                pos = pos + vel * h
            if self._open_at is not None and not self.released and t >= self._open_at - 1e-12:
                self.released = True
                self._open_at = None
                self.width_m = float(self._permit.open_width_m)
                self.gripper_open_count += 1
                self.stone_drop = (ros_now - (now - t), (float(pos[0]), float(pos[1])))
            if self._program is not None and t >= self._program_end():
                self._program = None             # 프로그램 끝: 정지
                vel = np.zeros(3)
                self._cmd, self._cmd_until = np.zeros(3), t
        self.pos, self.vel = pos, vel
        self._last = t_end

    def _program_end(self) -> float:
        t_c, _, lead = self._program
        pm = self._permit
        return (t_c + lead + pm.hold_s + max(0.0, pm.retreat_z_m - pm.release_xyz_m[2]) / pm.retreat_speed_mps)

    def _program_state(self, t):
        t_c, p0, lead = self._program
        pm = self._permit
        rel = np.asarray(pm.release_xyz_m, dtype=float)
        v = np.asarray(pm.velocity_mps, dtype=float)
        t_r = t_c + lead
        t_h = t_r + pm.hold_s
        rise = max(0.0, pm.retreat_z_m - rel[2])
        t_up = t_h + rise / pm.retreat_speed_mps
        if t <= t_r:
            vel = (rel - p0) / lead if lead > 0 else np.zeros(3)
            return p0 + vel * (t - t_c), vel
        tt = min(t, t_up)
        xy = rel[:2] + v[:2] * (tt - t_r)
        if t <= t_h:
            return np.array([xy[0], xy[1], rel[2]]), np.array([v[0], v[1], 0.0])
        if t <= t_up:
            return (np.array([xy[0], xy[1], rel[2] + pm.retreat_speed_mps * (t - t_h)]),
                    np.array([v[0], v[1], pm.retreat_speed_mps]))
        return np.array([xy[0], xy[1], pm.retreat_z_m]), np.zeros(3)
