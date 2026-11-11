# M0 남은 위험 분석과 수정안 (2026-11-11)

- 수행: Claude Code worker(Fable 5.2), Orca task `task_fa003ca55ada`, 읽기 전용. 기준 HEAD `b5111df`.
- 실행 장소: GCE(ROS 없음). 순수 Python 모의 실험만 했다. PC·장비 접속 없음, 저장소 파일 변경 없음.
- 표기: **[확인]** = GCE에서 production 코드를 실행해 본 것, **[코드]** = 소스 읽기로 확인, **[추론]** = 근거 있는 추정, **[미확인]** = 확인하지 못함.
- 모의 실험 스크립트(저장소 밖): `/tmp/claude-1000/-home-ubuntu-orca-projects-conveyor-tracker-bot/c36ff206-6eb6-4b6c-b1f5-90db5b87ffb5/scratchpad/` 의 `waitsim.py`, `fixA.py`, `r1check.py`, `arm.py`.

## 0. 요약

| 구분 | 항목 |
|---|---|
| **지금 고칠 것** (작고, GCE 순수 시험으로 검증 가능) | ① `RosSkillPort._phase_done`: result가 `phase_done` feedback보다 먼저 보이면 즉시 None(`STEP_TIMEOUT`)을 낸다(HEAD 동작 확인). rclpy가 실제로 result를 먼저 넘길 수 있는지는 미확인이므로 PC 명령 ③을 먼저 보고, 가능하면 짧은 대기 추가(1.4절 수정안 A). ② 실제 `robot_controller_node` 해제 경로의 남은 경쟁 2건(해제와 `reset_stop` 사이 Stop, 실행 callback 초입 예외 시 예약 고착) → 몇 줄 수정(3.3절). ③ I0 nominal 실패 때 마지막 단계·소요시간이 잘리지 않게 실패 메시지 순서만 바꿈(1.4절 수정안 B). |
| **G0 이후(또는 G0에서)로 미룰 것** | 감시 노드 기동 과도의 코드 대책(연속 N회 수신 뒤 무장) — OFF-2에서 실기 재현 여부를 본 뒤 결정. 보호정지 입력 연결(4절). F4 중 그리퍼 감시 연결(G0), 관측 신선도·벨트 이벤트 연결(G3 첫 실기 동적 시도 전), 시각 역행 무효화(G3, 뒤로 미뤄도 됨). |
| **그대로 둘 것** | 제품 단계 제한시간·heartbeat 0.3 s(실측 전 변경 금지 방침 유지). I0 기동 1.5 s 안정 대기와 시험의 해제 절차. WSL에서의 nominal 간헐 실패는 원인 단계가 확인될 때까지 "환경 영향 의심, 미확정"으로 두고 반복 측정으로 좇지 않는다. 감시 노드 별도 executor 스레드(효과 없음, 2.2절). |

핵심 판단 세 가지:

1. I0 `STEP_TIMEOUT`의 결과 코드 4는 `FAILURE`가 아니라 **`SAFE_ABORT`(L3)** 다. 따라서 실패 단계는 MOVE_STANDBY·APPROACH·SYNCHRONIZE·DESCEND·RELEASE 중 하나이고, 여유가 가장 작은 **RELEASE(제한 1.0 s, 대역 phase 0.36 s)** 가 가장 유력하다. 이미 알려진 0.5~0.8 s 시계 역행만으로는 지금 코드에서 어떤 단계도 끊기지 않는다 **[확인]**. 남는 경로는 (a) 0.65 s 이상 feedback 전달 지연, (b) 0.65~1.0 s 이상 시계 전진, (c) result 선도착 세 가지이며, 실패 단계와 그 소요시간 한 줄이면 가려진다. 원문 로그가 없어 **원인 미확정**이다.
2. 감시 노드 기동 과도는 감시 타이머가 정상이었으므로 executor 문제가 아니라 **DDS 수준의 새 연결 직후 수신 지연**으로 본다(확신도 중간). 시연에는 운용 절차(기동 후 확인·사유 남기고 해제)로 충분하다.
3. R1 P1·P2 수정은 **올바르다 [확인]**. 다만 좁은 경쟁 3건이 남는다(2건은 몇 줄로 닫힘, 1건은 설계상 수용).

---

## 1. I0 nominal 간헐 `STEP_TIMEOUT`

### 1.1 실패 형태에서 확정되는 것

- 시험은 `실패: {res.result} ({res.reason_code})`로 숫자를 찍는다(`src/drop_bringup/test/test_i0_integration.py:240-243`). `DropTask.action:10-15`에서 4 = `SAFE_ABORT`. coordinator 답변으로도 확인(`DropTask_Result(result=4, reason_code='STEP_TIMEOUT')`). **[코드]**
- `SAFE_ABORT`는 L3다(`cycle_runner.py:477`). L3는 공깃돌을 들고 있거나 동적 구간일 때다(`abort_policy.py:60-68`). 문자 그대로 `STEP_TIMEOUT`은 `_timeout()` 기본값, 즉 **포트가 None을 돌려준 경우**뿐이다(`cycle_runner.py:156, 314-323, 379, 386, 398, 408`). **[코드]**
- 그래서 제외되는 것: DETECT·PICK(들기 전이라 L2 = 결과 1), RETREAT(None이면 `RETREAT_FAILED` → L4 = 결과 5, `cycle_runner.py:417-418`), 동기 유지시간 부족(사유가 `TIMEOUT`, `cycle_runner.py:391-392`), PLAN(`PLAN_DEADLINE`), 사이클 제한시간·heartbeat·정지 유지 등 latch된 장애(사유가 그 장애 이름으로 바뀜, `cycle_runner.py:247-250`).
- coordinator 답변(원문 일부만 남음): 실패한 소스는 tar `84e50a7b`(P1·P2 수정 전, 기동 1.5 s 안정 대기 포함), I0 묶음 4회 중 2회차 seed 7·3회차 seed 18. **실패한 2·3회차에는 '기동 과도 감시 노드 Stop' 출력이 없었고**, 실패 사이클의 PRECHECK heartbeat 나이는 0.038 s·0.090 s로 정상이었다. → 기동 과도·정지 유지 경로와 heartbeat 지연은 이 실패의 원인이 아니다(정지 유지였다면 사유도 `ROBOT_STOPPED`였을 것).
- 과거 기록의 같은 증상: 2026-11-09 seed 9 RELEASE(시계 역행 + steady 상한 1배, 수정됨), 2026-11-10 seed 7 RELEASE·seed 13 SYNCHRONIZE(`records/worklog/2026-11-10.md:38`). 오늘 seed 7·18은 단계 미기록.

포트가 None을 내는 경로는 네 가지뿐이다(`orchestrator_node.py`). **[코드]**

| 경로 | 위치 | 특징(StepRecord `t_end − t_start`) |
|---|---|---|
| ROS deadline 도달 | `_wait` :183 | 단계 제한시간과 거의 같음(시계 전진이면 훨씬 짧음) |
| steady 상한(남은 ROS 시간 × 2) 도달 | `_wait` :159-164, :181 | ROS 경과는 제한보다 짧고 steady는 2배 |
| result가 끝났는데 해당 phase의 `phase_done` feedback이 없음 | `_phase_done` :443-452 | 매우 짧음(대기 0 s) |
| 앞선 goal 미정리로 새 goal 거부·서버 미준비 | `_send` :199-203 | APPROACH에서만, 즉시 |

### 1.2 단계별 여유와 모의 실험

`duration_scale_ratio` 0.12에서 대역 소요시간(`scenario_engine.py:516-519`, `nominal_20mms.yaml:53-59`): PickStone 0.48 s, MoveToPose 0.24 s, **DynamicPlace는 0.36 s가 아니라 1.8 s로 바닥 고정**(원값 ≥ 1.8 s이고 압축값 < 1.8 s이면 1.8 s) → 5개 phase 각 0.36 s. 대역의 경과 시간은 **ROS 시각** 기준이다(`mock_robot_controller_node.py:129, 156`, `ros_common.py:199-200`). `t0` offset −2.5 s는 병뚜껑 0이 시야 시작점에 막 들어오게 하는 값으로, 계획 여유가 약 19 s라 시간 예산과 무관하다(`scenario_engine.py:458-459`). 사이클은 약 3 s, 사이클 제한 15 s. **[코드]**

| 단계 | 대역 소요 | 단계 제한(ROS) | steady 상한 | 여유(ROS) |
|---|---:|---:|---:|---:|
| MOVE_STANDBY | 0.24 s | 10.0 s | 20 s | 9.8 s |
| APPROACH | 0.36 s | 3.0 s | 6 s | 2.6 s |
| SYNCHRONIZE | 0.36 s | 2.0 s | 4 s | 1.6 s |
| DESCEND | 0.36 s | 1.5 s | 3 s | 1.1 s |
| **RELEASE** | 0.36 s | **1.0 s** | 2 s | **0.64 s** |

production `RosSkillPort._wait`·`_phase_done`를 AST로 그대로 뽑아 가짜 시계로 돌린 결과(`waitsim.py`, 단계 진입 0.10 s 뒤 시계 점프, feedback 전달 지연 10 ms). **[확인]**

| 조건 | RELEASE | DESCEND | SYNCHRONIZE | APPROACH |
|---|---|---|---|---|
| 역행 −0.5 / −0.8 s (WSL 실측 범위) | 통과(steady 0.88 / 1.18 s) | 통과 | 통과 | 통과 |
| 역행 −1.6 / −1.7 s | 통과 / **실패**(steady 상한 2 s) | 통과 | 통과 | 통과 |
| 전진 +0.7 / +0.9 s | 통과 / **실패**(즉시) | 통과 | 통과 | 통과 |
| 전진 +1.7 s | 실패 | **실패** | 통과 | 통과 |
| 점프 없음, feedback 전달 지연 0.60 / 0.65 s | 통과 / **실패**(1.0 s) | — | — | — |
| result done인데 RELEASE `phase_done` 미도착 | **즉시 None**(대기 0.0 s) | 동일 | 동일 | 동일 |

읽는 법:

- **0.5~0.8 s 역행만으로는 실패하지 않는다.** 11-09에 넣은 배율 2.0이 그 범위를 덮는다. 역행으로 실패하려면 1.64 s 넘게 뛰어야 한다.
- 시계가 **앞으로** 뛰면 ROS deadline이 바로 지난다. 대역 경과도 같이 뛰어 phase_done을 곧 보내지만, 오케스트레이터는 다음 5 ms 폴링에서 deadline을 먼저 본다. RELEASE는 전진 0.64~1.0 s 이상에서 실패한다. WSL에서 전진 점프가 있는지는 **[미확인]**(기록은 "약 30 s마다 0.5~0.8 s 역행"뿐, `docs/learn/2026-11-09-ci-failures-and-robustness.md:27`).
- 실패 seed의 사이클 시각(사이클당 약 3 s: seed 7 ≈ 21~24 s, seed 9 ≈ 27~30 s, seed 18 ≈ 54~57 s)이 약 30 s 주기 근처다. **상관 정황일 뿐 기전의 증거는 아니다. [추론]**

### 1.3 원인 판단

- **가장 그럴듯한 원인(확신도 중간)**: WSL의 순간 교란(시계 보정 순간의 지연, 또는 프로세스 기상 지연)으로 **RELEASE의 `phase_done` feedback이 0.64 s 넘게 늦게 처리**됐거나 시계가 앞으로 뛰어, 0.12 압축에서 유일하게 여유가 1 s 미만인 RELEASE가 끊겼다. 근거: (1) L3 + `STEP_TIMEOUT`으로 단계 후보가 5개로 좁혀짐, (2) 과거 세 번 중 두 번이 RELEASE, (3) 여유 표에서 RELEASE가 최소, (4) 같은 PC에서 시계 점프와 0.3 s급 콜백 지연이 이미 측정됨(`records/worklog/2026-11-09.md:25, 42`). 이 경우 **제품 결함이 아니라 시험 환경(WSL)과 압축 시험의 여유 문제**다. 실제 구동 장비 LEGION(네이티브 Ubuntu)에서 같은 일이 생긴다는 근거는 없다.
- **두 번째 후보(확신도 낮음)**: result 선도착. `_phase_done`의 `done()`은 "해당 phase_done feedback이 있음 **또는** result가 끝남"이고, result가 끝났는데 feedback이 아직 목록에 없으면 그 자리에서 None이다(`orchestrator_node.py:443-452`). feedback 처리가 0.36 s 이상 밀린 채 result가 먼저 처리되면 RELEASE에서, 1.08 s 이상이면 SYNCHRONIZE에서 이렇게 된다. 밀릴 수 있는 이유: action client가 노드 기본 callback group(상호 배제)에 있어 30 Hz 구독 3개·서비스 응답·상태 타이머와 한 줄로 처리된다(`orchestrator_node.py:583-595`, 구독은 group 미지정). 또 대역은 RETREAT의 `phase_done`을 **항상 result 직전에** 보낸다(`mock_robot_controller_node.py:170-172`). rclpy action client가 한 번에 feedback 1개만 꺼내고 result를 feedback보다 먼저 처리한다는 것, feedback 구독 QoS가 KEEP_LAST 10이라는 것은 기억에 근거한 것으로 **[미확인]** 이다(아래 PC 명령 ③). 이 경로는 **실기에서도 그대로 남는 제품 쪽 약점**이다: 실제 controller도 마지막 phase_done 뒤 곧바로 result를 보낸다(`robot_controller_node.py:253-306`). RETREAT에서 걸리면 L4(`RETREAT_FAILED`)가 된다.
  - 이 후보에 불리한 정황: 같은 경합은 `phase_done`이 늘 result 바로 앞에 오는 RETREAT에서 가장 잘 생겨야 하고, 그때는 결과 5(`PROTECTIVE_STOP`)·사유 `STEP_TIMEOUT`이 된다. 그런 실패는 기록에 없다. 앞으로 결과 5 + `STEP_TIMEOUT`이 보이면 이 후보가 맞다는 신호다.
- **나머지(확신도 낮음)**: 1.64 s 넘는 역행, feedback 유실 등.

동기 유지 판정은 별개의 좁은 여유다: 유지시간은 feedback **수신** ROS 시각으로 재는데(`orchestrator_node.py:209, 462`), 대역 SYNCHRONIZE phase가 0.36 s라 `hold_s` 0.3 s 대비 여유가 0.06 s뿐이다(`step_judges.py:289-308`). 여유 0.06 s는 **[코드]**. 이것은 `TIMEOUT` 사유로 나타나므로 오늘 실패와는 다른 실패이고, 11-09의 "SYNCHRONIZE TIMEOUT 1회"가 이것이었으리라는 것은 **[추론]** 이다.

### 1.4 수정안(작은 순)

**A. `_phase_done`에 짧은 feedback 대기 추가 — PC 명령 ③으로 result 선처리가 가능함을 본 뒤 적용(제품 코드 6줄, GCE 순수 시험 가능).** result가 정상(OK)으로 끝났는데 phase_done이 아직 없을 때만 최대 0.2 s 더 기다린다. 정지·보호정지 결과에는 기다리지 않으므로 L4 반응 시간은 그대로다.

```diff
--- a/src/orchestrator/orchestrator/orchestrator_node.py
+++ b/src/orchestrator/orchestrator/orchestrator_node.py
@@ POLL_S = 0.005
+FEEDBACK_DRAIN_S = 0.2   # result가 마지막 phase_done feedback보다 먼저 처리될 때 기다리는 시간
@@ def _phase_done(self, phase, deadline):
-        def done():
-            return any(fb.phase == phase and fb.phase_done for _, fb in self._feedback) or (
-                self._result_future is not None and self._result_future.done())
+        def has_fb():
+            return any(fb.phase == phase and fb.phase_done for _, fb in self._feedback)
+
+        def done():
+            return has_fb() or (self._result_future is not None and self._result_future.done())
         if not self._wait(done, deadline):
             return None
+        if not has_fb() and self._result_future.result().result.result == 0:
+            self._wait(has_fb, min(deadline, self.clock.ros_now() + FEEDBACK_DRAIN_S))
         fb = next((fb for _, fb in self._feedback if fb.phase == phase and fb.phase_done), None)
```

소스 문자열에 이 변경을 적용한 스크래치 사본으로 본 결과(`fixA.py`, 저장소 무변경) **[확인]**: 정상 result + feedback 50 ms 지연 → HEAD는 0.000 s에 None, 변경안은 0.052 s에 metrics 반환. 정상 result + feedback 끝내 없음 → 0.204 s 뒤 None. `STOPPED` result → 두 쪽 모두 즉시 None + `ROBOT_STOPPED` latch(L4 반응 시간 불변).

회귀 시험은 `src/orchestrator/test/test_node_failure_regressions.py`의 `port()` 틀로 쓴다: result done + feedback 없음 → 다른 스레드가 0.05 s 뒤 feedback 추가 → metrics 반환, feedback이 끝내 없으면 0.2 s 뒤 None. SPEC-100 4.10절(포트 대기 계약)에 한 줄을 더한다. 한계: rclpy가 result 처리 때 feedback callback을 지운다면 이 대기로는 못 받는다(PC 명령 ③으로 확인).

**B. 실패 메시지 순서(시험만, 2줄).** 지금은 PRECHECK metrics가 앞에 와서 잘린다. 마지막 단계와 소요시간을 앞에 둔다.

```diff
--- a/src/drop_bringup/test/test_i0_integration.py
+++ b/src/drop_bringup/test/test_i0_integration.py
@@ test_i0_nominal_20_cycles_and_records
+            cyc = [m for _, m in step_msgs if task_msgs and m.cycle_id == task_msgs[-1][1].cycle_id]
+            last = [(m.step_name, m.reason_code, round((m.t_end.sec - m.t_start.sec) + (m.t_end.nanosec - m.t_start.nanosec) * 1e-9, 3)) for m in cyc[-2:]]
             assert res.result == DropTask.Result.SUCCESS, (
-                f'seed={seed} 실패: {res.result} ({res.reason_code}); '
+                f'seed={seed} 실패: {res.result} ({res.reason_code}); last={last}; '
```

**C. action client를 전용 callback group으로(선택, 2줄).** feedback·result가 30 Hz 데이터 콜백 뒤에 줄 서지 않게 한다. 순서 보존을 위해 Reentrant가 아니라 전용 `MutuallyExclusiveCallbackGroup` 하나를 쓴다: `self.action_group = MutuallyExclusiveCallbackGroup()`, `ActionClient(self, typ, name, callback_group=self.action_group)`(`orchestrator_node.py:593`). 효과는 PC에서만 확인 가능 **[미확인]**. A를 넣고도 재발할 때 적용한다.

**D. 하지 않는 것.** `step_timeout.release_s` 등 제품 값 완화(TBD·G0 측정 대상), 재시도로 덮기, WSL 반복 측정. 사용자 지침(시험을 과하게 하지 않음)에 맞춰, 판정용 I0는 LEGION이 켜져 있을 때 LEGION에서 1회 돌리는 것으로 충분하다. WSL에서 다시 실패하면 B의 `last=` 한 줄로 아래 표에 대입한다.

| `last=`에 보이는 것 | 뜻 | 조치 |
|---|---|---|
| RELEASE, 소요 ≈ 1.0 s | feedback 전달이 0.64 s 넘게 지연(환경 교란) | 환경 문제로 기록, C 검토 |
| 소요가 제한보다 훨씬 짧음(≤ 0.4 s) + 시계 표본에 전진 점프 | 시계 전진 | 환경 문제로 기록(WSL 한정) |
| 소요가 매우 짧고 시계 점프 없음 | result 선도착 | A로 해결되는지 확인 |
| ROS 소요 < 제한, steady ≈ 2배 | 1.64 s 넘는 역행 | 환경 문제로 기록 |

### 1.5 PC에서 돌릴 확인 명령(coordinator가 실행, officepc WSL 시험 사본 기준)

①과 ②는 **한 번의 SSH 호출(스크립트 하나)** 로 돌린다. WSL은 SSH 세션이 끝나면 백그라운드 프로세스를 정리한다. 수정안 B를 PC 사본에 넣지 않아도 되도록 단계 소요시간은 pytest 임시 폴더의 단계 기록 JSONL에서 읽는다(pytest는 최근 3회 실행분을 남긴다).

```bash
# ① 시계 점프 표본(양방향). I0와 같은 시간대에 돌린다.
python3 - > /tmp/clk.log <<'P' &
import time
m0 = time.monotonic(); last = time.time() - time.monotonic()
while time.monotonic() - m0 < 420:
    time.sleep(0.005); off = time.time() - time.monotonic()
    if abs(off - last) > 0.005:
        print(f'wall={time.time():.3f} mono={time.monotonic()-m0:7.2f} jump={off-last:+.3f}', flush=True)
    last = off
P
# ② nominal만 3회, 출력은 자르지 않고 파일로(각 약 70 s, 600 s 안에 끝남)
for i in 1 2 3; do
  python3 -m pytest -q src/drop_bringup/test/test_i0_integration.py -k nominal_20 > /tmp/i0-nom-$i.log 2>&1; echo "run $i exit $?"
done
grep -ho "seed=[0-9]* 실패: [0-9] ([A-Z_]*)" /tmp/i0-nom-*.log
grep -h "SAFE_ABORT STEP_TIMEOUT" /tmp/i0-nom-*.log | cut -c1-300   # orchestrator 로그의 상태 열(마지막 정상 단계 → SAFE_ABORT)
# 실패 사이클의 단계별 ROS 소요시간(1.4절 표에 대입). 시계 표본의 wall 값과 t_start_s를 맞춰 본다.
python3 - <<'P'
import glob, json, os
for f in glob.glob(f'/tmp/pytest-of-{os.environ["USER"]}/pytest-*/**/*.jsonl', recursive=True):
    rows = [json.loads(l) for l in open(f, encoding='utf-8') if l.strip()]
    if any(r.get('reason_code') == 'STEP_TIMEOUT' and 'step_name' in r for r in rows):
        print(f)
        for r in rows:
            print(f"  {r['step_name']:14} {r.get('reason_code',''):14} t_start={r['t_start_s']:.3f} dur={r['t_end_s']-r['t_start_s']:.3f}")
P
kill %1 2>/dev/null; cat /tmp/clk.log
# ③ rclpy action client의 꺼내기·처리 순서와 feedback QoS(읽기만)
python3 - <<'P'
import inspect, rclpy.action.client as c
print(inspect.signature(c.ActionClient.__init__))
print(inspect.getsource(c.ActionClient.take_data)); print(inspect.getsource(c.ActionClient.execute))
P
```

---

## 2. 감시 노드 기동 과도(`CONTROLLER_HEARTBEAT_LOST` Stop)

### 2.1 원인 판단(DDS 수준 수신 지연: 확신도 중간, 세부 기전은 미확인)

코드로 가려지는 것: **[코드]**

- 감시 노드는 `rclpy.spin` 단일 스레드이고(`safety_supervisor_node.py:166`), 손실 판정은 **콜백이 실행된 steady 시각** 기준이다(`watchdogs.py:15-20`). 문제 순간에 0.05 s 감시 타이머는 정상 간격이었다(coordinator 측정). 같은 executor가 타이머는 제때 돌리면서 구독 콜백만 0.3 s 밀릴 수는 없다. → **단일 스레드 executor의 대기열 순서는 원인이 아니고, 별도 executor 스레드도 대책이 아니다.** 대역 쪽 heartbeat callback group 분리가 효과 없었던 것과도 맞는다.
- 보내는 쪽은 연속이었다(독립 구독자 stamp·수신 공백 ≤ 0.10 s). 대역 heartbeat는 steady 타이머다(`mock_robot_controller_node.py:81`).
- QoS 불일치 아님: 발행 RELIABLE·VOLATILE depth 10(`ros_common.py:23`, 실제 노드 `robot_controller_node.py:40`), 감시 구독 RELIABLE·VOLATILE depth 20(`safety_supervisor_node.py:26`). 호환된다. 불일치였다면 한 번도 못 받았을 것이다.
- `_controller_seen` 때문에 첫 수신 전에는 손실을 보지 않는다(`safety_supervisor_node.py:63`). 그래서 과도는 "첫 heartbeat는 받았고, 그 다음 것들이 늦게" 올 때만 드러난다. 대역은 생성자에서 `exclusive_check_s` 1 s 동안 막힌 뒤 발행자를 만든다(`ros_common.py:170-177`). 그래서 감시 노드의 구독이 먼저 떠 있고 **이미 있던 reader에 새 writer가 붙는 순간**에 생긴다고 본다(launch 기동 순서는 읽지 않았으므로 **[추론]**).

남는 후보: **[추론]**

1. reliable 연결 초기의 재전송 대기(가장 유력): writer가 reader를 먼저 match하고 보낸 첫 표본은 도착했지만, 이어진 1~2개가 reader 쪽 match·전송 경로(SHM/UDP) 준비 중에 빠지고, reliable 순서 보장 때문에 뒤 표본이 재전송까지 묶였다가 한꺼번에 전달됐다. 0.30~0.34 s(heartbeat 3회분)와 "R1 이전에도 4회 중 1회"에 맞는다.
2. SHM↔UDP 전환 자체: 11-09에 UDP 전용으로도 다른 증상이 남았다는 기록만 있고 이 증상에 대해서는 **[미확인]**.
3. 프로세스 기동 부하로 인한 OS 수준 지연: 타이머가 정상이었으므로 가능성 낮음.

가르는 방법(PC, 필요할 때만): 감시 노드가 받은 controller heartbeat의 `seq`와 수신 steady 시각을 기동 후 3 s 동안 찍는다. 공백 뒤 `seq`가 연속이고 여러 개가 같은 순간에 들어오면 후보 1이다. 추가로 PC 시험 사본에서만 감시 노드 heartbeat 구독을 `BEST_EFFORT`로 바꿔(한 줄) 과도가 사라지는지 본다. 기저 발생률이 4회 중 1회 정도라 5회 연속 무발생은 우연으로도 4번에 1번쯤 나온다. **결정적인 것은 `seq`·수신 시각 기록**이고 BEST_EFFORT 실험은 보조다.

```bash
# PC 시험 사본에서만(저장소 반영 아님). 5회 기동해 '기동 과도 감시 노드 Stop' 출력 횟수를 센다.
sed -i 's/^HB_QOS = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE/HB_QOS = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT/' \
  src/orchestrator/orchestrator/safety_supervisor_node.py
colcon build --symlink-install --packages-select orchestrator && source install/setup.bash
for i in 1 2 3 4 5; do python3 -m pytest -q -s src/drop_bringup/test/test_i0_integration.py -k protective_stop 2>&1 | grep -c "기동 과도"; done
```

### 2.2 대책 비교

| 대책 | 내용 | 장점 | 단점·안전 의미 |
|---|---|---|---|
| **운용 절차(권장, 코드 변경 없음)** | 기동 → 2 s 대기 → controller heartbeat `error_code` 확인. 101이면 Stop 이력 사유가 `CONTROLLER_HEARTBEAT_LOST`이고 로봇이 정지·대기 자세임을 눈으로 확인한 뒤 `ros2 param set /robot_controller_node stop_release_reason "기동 과도"` → 필요하면 orchestrator `resume_reason` | 안전 쪽으로만 실패(가짜 Stop은 멈춘 로봇을 멈춘 채로 둠). REQ-ABT-010 그대로. I0가 이미 같은 절차 | 기동마다 사람 손 1회. 실제 노드는 `on_activate`에서 발행자를 만들므로(`robot_controller_node.py:187-194`) **재활성화 때마다** 생길 수 있음 |
| 연속 N회 수신 뒤 무장 | controller heartbeat가 0.3 s 미만 간격으로 5회 이어진 뒤부터 `CONTROLLER_HEARTBEAT_LOST`를 판정. 보호정지·통신 단절 판정은 즉시 유지 | 가짜 Stop 제거. 순수 Python 약 10줄, GCE 시험 가능 | **REQ-ABT-010 의미 변화**: 첫 수신 뒤 약 0.5 s 동안 controller heartbeat 손실을 감시 노드가 못 본다. 지금도 첫 수신 전에는 못 보므로(`_controller_seen`) 기존 공백을 0.5 s 넓히는 것. 그 사이 방어선: orchestrator PRECHECK(heartbeat 나이 ≤ 0.3 s, 사이클 시작 차단)와 사이클 중 `_watch`. SPEC-110 REQ-ABT-010에 "무장 뒤" 단서, TC-ABT-009-02 입력을 "1회 후 끊김"에서 "5회 후 끊김"으로 바꿔야 함 |
| 감시 구독을 BEST_EFFORT로 | heartbeat는 최신만 의미 있으므로 재전송 대기를 없앰 | 원인이 후보 1이면 근본 해결, 한 줄 | 원인 미확인. SPEC-010 6절 QoS 표(Reliable)와 계약 시험 문구 조정 필요. 유실이 곧 공백이 되므로 실기 망 품질에 민감 |
| 별도 executor 스레드·callback group | — | — | 타이머가 정상이었으므로 **효과 없음**. 하지 않는다 |

"연속 5회 뒤 무장"을 `SupervisorMonitor`의 하위 클래스로 만들어 가짜 시계로 본 결과(`arm.py`): **[확인]**

| 입력 | HEAD | 무장 방식 |
|---|---|---|
| 첫 수신 뒤 0.33 s 공백 → 정상 6회 → 끊김 | 0.30 s에 가짜 Stop, 끊김 0.30 s 뒤 Stop | 가짜 Stop 없음, 끊김 0.30 s 뒤 Stop |
| 정상 8회 → 끊김 | 끊김 0.30 s 뒤 Stop | 같음 |
| 1회 수신 → 끊김(TC-ABT-009-02 입력) | 0.30 s 뒤 Stop | **Stop 없음**(시험 입력 변경 필요) |

권고: 시연까지는 **운용 절차**로 간다. OFF-2에서 LEGION 실기 기동 5회 중 가짜 Stop이 한 번이라도 나오고 번거로우면 "연속 5회 뒤 무장"을 넣는다. 변경안 요지:

```python
# SupervisorMonitor (safety_supervisor_node.py)
ARM_BEATS = 5
def on_heartbeat(self, node_name, error_code=0):
    if node_name == 'robot_controller_node':
        ok = self._controller_seen and not self._hb.lost(node_name)     # 직전 수신과의 간격 < timeout
        self._streak = self._streak + 1 if ok else 1
        self._armed = self._armed or self._streak >= ARM_BEATS           # 한 번 무장하면 유지
        ...
def active_fault(self):
    ...
    if self._armed and self._hb.lost('robot_controller_node'):
        return 'CONTROLLER_HEARTBEAT_LOST'
```

---

## 3. R1 검수 P1·P2 수정(commit `4ac036f`) 독립 확인

검수 보고서(`r1-review-2026-11-11.md:84-124`)의 재현 틀을 그대로 써서 HEAD의 production `_on_goal`·`_on_params`·`_execute`를 AST로 뽑아 실제 `StaticSkillExecutor`·`SimulatedBackend`와 돌렸다(`r1check.py`). ROS executor 스케줄과 실기는 재현하지 않았다.

### 3.1 수정 확인 **[확인]**

| 항목 | 수정 전(검수 보고서) | HEAD |
|---|---|---|
| P1: 정지 유지 중 goal 수락 → 실행 전 해제 → 실행 | 해제 성공, `move_line` 1회, 결과 OK | 수락 시 `_active_goal=True`, 해제 **거부(`GOAL_ACTIVE`)**, 결과 `CANCELED/STOPPED`, backend 호출 0회, 정지 유지 계속 |
| P2: 유지 중이 아닐 때 빈 사유 해제 | backend 정지 True → False | **True 유지**, 응답 성공(무동작) |
| 수락 뒤 Stop → 실행(해제 없음) | — | `CANCELED/STOPPED`, backend 호출 0회 |

코드 위치: 수락 시점 예약 `robot_controller_node.py:227-233`, 실제 해제 때만 `reset_stop` `:378-389`. `_execute`가 `_stop_requested`를 False로 덮어도(`:242`) `cancel()`이 `stop_latch.latched`를 함께 보고(`:251`), skill은 첫 지령 전에 cancel을 확인하므로(`skill_logic.py:185`) Stop이 사라지지 않는다. 대역은 원래 수락 때 `_busy`를 예약한다(`mock_robot_controller_node.py:88-93`). 수정은 올바르다.

### 3.2 남은 경쟁 **[확인: 호출 순서를 강제로 끼워 넣어 재현]**

| # | 순서 | 결과 | 심각도 |
|---|---|---|---|
| B | 해제 승인(`release()`) → **새 Stop**(latch + `backend.stop()`) → `reset_stop()` | 정지 유지 True인데 backend 정지 False. goal은 정지 유지가 막으므로 움직이지는 않는다. 다만 실기 backend에서 `reset_stop`이 정지 상태를 푸는 호출이면 Stop 직후 그 호출이 나간다 | P3(작게 닫힘) |
| D | goal 수락(예약) → `_execute`가 `try` 전에 예외(`:243-248`) | `_active_goal`이 True로 남아 이후 모든 goal 거부·해제 `GOAL_ACTIVE` 거부. 안전 쪽 고착이지만 노드 재시작 전까지 복구 불가 | P3(작게 닫힘) |
| C | 유지 중 아님 → 작업자가 해제 설정 → `was_latched` 읽은 직후 **새 Stop** → `release()` | 방금 들어온 Stop의 정지 유지가 풀림(backend 정지는 남음) | P3(설계상 수용) |

D와 같은 고착이 "수락 뒤 비활성화로 실행 callback이 아예 불리지 않는" 경우에도 생길 수 있으나 이것은 재현하지 않았다 **[추론]**. 비활성화 때 예약을 지우는 식으로 고치면 실행 중인 goal의 예약까지 지워져 P1 구멍이 다시 열리므로 그렇게 고치지 않는다(재시작으로 복구).

C는 lock으로 순서를 고정해도 "Stop 직후 도착한 해제 요청이 그 Stop을 푼다"는 결과가 같다. 막으려면 해제 요청에 정지 세대 번호를 실어야 하는데, 사람이 수동으로 넣는 해제와 Stop이 같은 수 ms에 겹칠 때만 생기므로 1회성 시연 범위에서는 **수용**하고 SPEC-010 4.5절에 한 줄 적는 정도면 된다.

### 3.3 수정안(B·D, 실제 노드만)

```diff
--- a/src/robot_control/robot_control/robot_controller_node.py
+++ b/src/robot_control/robot_control/robot_controller_node.py
@@ def _execute(self, name, goal_handle):
-        with self._lock:
-            self._active_goal = True
-            latched = self.stop_latch.latched
-        self._stop_requested = False
-        goal = goal_handle.request
-        typ = ACTIONS[name][0]
-        deadline_s = None
-        raw_deadline = ...
+        try:
+            return self._execute_reserved(name, goal_handle)   # 기존 본문 그대로
+        finally:
+            with self._lock:
+                self._active_goal = False                      # 예외로 빠져도 예약을 푼다
@@ def _on_params(self, params):
             reset = getattr(self.backend, 'reset_stop', None)
             if reset is not None:
                 reset()
+                if self.stop_latch.latched:                    # 해제와 reset 사이에 Stop이 들어왔으면 다시 멈춘다
+                    self.backend.stop()
```

기존 본문과 그 안의 `finally`는 그대로 두고 바깥에 한 겹만 더한다. 회귀 시험은 `src/robot_control/test/test_robot_controller_stop.py:161` 옆에 두 개: `request`가 예외를 내는 goal 뒤 다음 goal 수락, `reset_stop` 안에서 Stop을 끼워 넣은 뒤 `backend.is_stopped()` True. rclpy가 필요하므로 PC에서 `python3 -m pytest -q src/robot_control/test/test_robot_controller_stop.py`.

---

## 4. 실제 `robot_controller_node`의 보호정지 입력과 해제 거부(G0 연결 방안)

현재 상태 **[코드]**: `msg.protective_stop = False` 고정(`robot_controller_node.py:345, 354`), `ready`·heartbeat 코드·해제 판정이 모두 `protective_stop=False` 상수를 받는다(`:353, :366, :380-381`). 순수 판정(`stop_latch.py:43-66`)은 이미 보호정지 우선(100)·해제 거부(`PROTECTIVE_STOP`)를 구현했고 시험이 있다. 빠진 것은 **입력 한 줄기**다.

연결 방안(G0에서, 작은 순):

1. backend 규약에 `read_protective_stop() -> bool` 하나를 더한다(`skill_logic.py:81-86` Protocol, `fake_backend.py`에는 시험용 setter).
2. 노드에 `_pstop()` 하나를 두고 네 곳이 모두 이것만 쓴다: `_on_status`(`protective_stop`, `ready`), `_on_heartbeat`(코드 100), `_on_params`(해제 거부), `_execute`(실행 전 보호정지면 지령 없이 `FAULT/PROTECTIVE_STOP`, `cancel()`에도 포함). **읽기 실패·예외는 보호정지로 본다**(안전 쪽).
3. 보호정지를 보면 노드가 스스로 `stop_latch.latch('PROTECTIVE_STOP')`을 건다. 제어기에서 보호정지를 풀어도 작업자가 사유를 남기기 전에는 goal을 받지 않게 된다.
4. `reset_stop`은 프로젝트 쪽 정지 표시만 푼다. **제어기의 보호정지 복구(티치 펜던트 조작)를 코드가 대신하지 않는다.**
5. G0에서 확인할 것 **[미확인]**: Doosan 드라이버에서 로봇 상태를 주는 토픽·서비스 이름과 보호정지·비상정지·SAFE_OFF에 해당하는 상태 값, 갱신 주기(10 Hz heartbeat에 실을 수 있는지), 보호정지 중 `move_line` 호출의 반환. 시험: fake backend로 TC-ROB-014-02(보호정지 중 해제 거부, 코드 100 우선)를 실제 노드 C 시험으로, 실기는 OFF-2에서 보호정지 1회 → `ready=false`·코드 100·해제 거부·제어기 복구 뒤 사유 해제 → 다음 사이클 PRECHECK 통과를 눈으로 확인.

---

## 5. F4(감시 ROS 연결): G3~G6 전에 먼저 할 것과 미뤄도 되는 것

현재 `orchestrator_node._watch`는 로봇 상태·heartbeat만 본다(`orchestrator_node.py:643-659`). `OBS_STALE`·`CLOCK_JUMP`·`BELT_UNSTEADY`·`GRIPPER_FAULT`는 이벤트 정의와 U 판정만 있고 노드에서 발생시키는 곳이 없다(이 파일에 해당 이름 0건) **[코드]**. 게이트 배정은 `TEST_SPEC.md:552-556`: ABT-014-02 = G0, ABT-011-02 = G3·G4, ABT-012-02·013-02 = G3.

| 구분 | 항목 | 이유 |
|---|---|---|
| **먼저(실기에서 처음 공깃돌을 드는 G0 사이클 전)** | 그리퍼 감시 연결 ABT-014-02 | 모든 사이클이 그리퍼로 든다. 필요한 값(`gripper_error_bits`·`gripper_response_s`·`gripper_alive`)은 이미 `RobotStatus`로 캐시되고 `GripperMonitor`가 있다(`watchdogs.py:49-79`). `_watch`에 몇 줄 |
| **먼저(실기 벨트 위 첫 동적 해제 시도 전, G3 끝~G4 시작)** | 관측 신선도 ABT-011-02, 벨트 급변 ABT-013-02 | 이동 투입의 핵심 위험 두 가지(오래된 track으로 해제, 속도 변한 벨트에 해제). `self.tracked`·`self.belt` 캐시와 `FreshnessMonitor`가 이미 있어 `_watch`에서 동적 상태일 때만 판정하면 된다. C 시험은 대역 `occlusion`·`speed_change` 이벤트로 가능 |
| **미뤄도 됨** | 시각 역행 무효화 ABT-012-02 | planner 노드는 역행 때 계획을 `CLOCK_RESET`으로 만료한다(`intercept_planner_node.py:129`). 실제 구동 장비는 네이티브 Linux라 WSL식 주기 역행이 없다고 예상되나 **[미확인]** 이므로 OFF 세션에서 1.5절 ① 표본을 한 번 돌려 확인하고, 점프가 없으면 G3 뒤로 미룬다 |
| **미뤄도 됨** | 영상 freshness(P2-6, G1), 통계적 반복 검증 | G1 범위이고, 사용자 지침상 반복·엄격 검증은 하지 않음 |
| **대역 I0에는 불필요** | 위 전부 | M0 통과 판정에 영향 없음 |

순서 권고: G0 작업에 그리퍼 연결과 4절(보호정지 입력)을 같이 넣고, G3에서 tracker 실제 노드를 붙일 때 신선도·벨트 연결을 같은 커밋 묶음으로 넣는다. 그 전에 실기에서 `MODE_PREDICTED`·`MODE_SYNCHRONIZED` 해제를 시도하지 않는다.

---

## 6. 확인하지 못한 것(모음)

- I0 실패의 실제 단계·소요시간(원문 로그 없음). 1.3절 원인 순위는 조건부다.
- WSL 시계의 전진 점프 유무, 1.6 s 넘는 역행 유무.
- rclpy(Jazzy) action client의 result·feedback 처리 순서, feedback 구독 QoS 깊이, result 뒤 feedback callback 유지 여부.
- 감시 노드 과도의 DDS 세부 기전(재전송 대기인지, 전송 경로 전환인지), LEGION 네이티브 환경에서의 재현 여부.
- 수정안은 모두 제안이며 저장소에 적용하지 않았다. A와 "5회 뒤 무장"만 스크래치 사본에서 동작을 봤고, B·C·3.3절은 실행해 보지 않았다.
- 실제 노드의 ROS executor 수준 경쟁(3.2절은 호출 순서를 손으로 끼워 넣은 재현).
- Doosan 보호정지 상태 값·토픽 이름.
