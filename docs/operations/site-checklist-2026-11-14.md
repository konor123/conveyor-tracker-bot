# 현장 점검표 (2026-11-14)

- 작성: coordinator(Claude Opus 5.5), Codex·OpenCode 검토 의견과 advisor 종합. 안전 절차 전체는 [현장 안전 절차](safety-procedure.md)이고, 이 문서는 11-13 현장 점검(`records/sessions/OFF-4-2026-11-13.md`)과 그 뒤 원격 작업에서 나온 오늘 확인할 항목만 모았다.
- 수치는 저장소 기록에 있는 값만 쓴다. 이 문서에서 새 값을 정하지 않는다.

## 1. 정적 투입 시연의 기준 코드

- 실기에서 정적 투입 1사이클이 성공한 코드는 `e509c87`이다(`records/sessions/OFF-4-2026-11-13.md` 시작 SHA).
- 정적 투입(`static_pick_place`)이 import하는 모듈(`static_pick_place`, `vision`, `calibration`, `robot_control`의 `drl_common`·`motion_adapter`)이 그 뒤 바뀌었는지 현장 사본에서 확인한다.

```bash
git diff --stat e509c87 HEAD -- src/static_pick_place src/vision src/calibration \
  src/robot_control/robot_control/drl_common.py src/robot_control/robot_control/motion_adapter.py
```

- 출력이 비어 있으면 정적 경로 코드는 검증한 그대로다. 비어 있지 않으면 바뀐 파일과 그 커밋의 시험 기록(`records/worklog/`)을 보고, 의심되면 `e509c87`을 받은 별도 사본에서 정적 투입을 실행한다.

## 2. 시작 전에 확인할 것

| 항목 | 확인 방법 | 근거 |
|---|---|---|
| 이전에 띄운 카메라 노드가 장치를 잡고 있지 않다 | `ros2 node list`에 camera 노드가 없고 RealSense 드라이버 프로세스가 없는지 본다. 있으면 끄고 D455 확인을 다시 한다 | 11-13 첫 D455 확인에서 4일 전 노드 때문에 `Device or resource busy`, 같은 이름 노드 중복 |
| 큰 공유 메모리 프로파일 | 카메라 드라이버와 영상을 받는 노드 모두 `FASTRTPS_DEFAULT_PROFILES_FILE`에 `src/drop_bringup/config/fastdds_large_shm.xml`을 준다(`camera.launch.py`는 자기 노드에 자동 적용) | 프로파일 없이 1280×800 영상이 최대 2 s씩 끊김(11-13) |
| 발견 범위 | `echo $ROS_AUTOMATIC_DISCOVERY_RANGE` 값을 세션 기록에 적는다 | 발행 0.33 s 정지 미세 재현은 이 값이 비었을 때만 됐다(`records/reviews/publish-stall-repro-2026-11-13.md`) |
| 기동 직후 heartbeat | 안전 절차대로 기동 → 2 s 대기 → controller heartbeat `error_code` 확인. 다만 원격 관찰의 정지는 기동 2.5~3.9 s 뒤였으므로 2 s 대기 뒤에도 4 s 무렵까지 `CONTROLLER_HEARTBEAT_LOST` Stop이 날 수 있다. 그때도 같은 해제 절차다 | `docs/operations/safety-procedure.md` 2절, `records/reviews/heartbeat-stall-evidence-2026-11-13.md` |
| 비상정지 담당 입회 | 시작 전 "입회 중" 확인을 세션 기록에 남긴다 | 11-13 기록에는 이 확인이 따로 남지 않았다 |

## 3. 알려진 장비·설정 상태

- 벨트 진행 방향은 robot_base **+x**다(11-07 영상은 거의 +y). `T_base_conveyor`는 11-13에 임시 평행이동 (0.65, 0.08, 0.061) m만 넣었고 측정값이 아니다. 위치 정확도를 판정하는 데 쓰지 않는다.
- 팔을 벨트 위에서 기다리게 하면 카메라에서 벨트 상류가 가려져 대상 선택이 되지 않는다. 벨트 옆(y 0.20 m)에서 기다린다.
- 벨트는 저속에서 저절로 멈춰 손으로 다시 돌려야 했다. 다시 돌린 직후에는 속도가 바뀌는 중이라 예측이 크게 틀린다(11-13 비접촉 시험: 예측 34 mm/s, 실제 약 47 mm/s, 통과 시각 −0.81 s. 계획 직전 같은 track의 계획이 `UNCERTAIN`으로 거절됨). 벨트를 다시 돌렸으면 속도가 안정된 뒤 계획을 받는다.
- 그리퍼 DRL 끝마다 제어기 알람 level 1·index 2007이 뜬다. 11-07·11-09·11-13 모두 같고 동작은 정상이었다.
- 드라이버 기동 때 `Could not enable FIFO RT scheduling` 경고는 `movel` 정지 투입에 영향이 없었다.
- `static_frames_publisher`가 뜬 launch를 Ctrl-C로 끄면 traceback과 종료 코드 −2가 나올 수 있다. 종료 신호 처리를 아직 넣지 않은 알려진 경우이고 장애가 아니다(`specs/SPEC-000-system.md` 4.5절 대상 밖 진입점).

## 4. 실시간 추적을 띄울 때

- `main`의 vision은 1280×800 프레임 하나에 약 160 ms(6.6 Hz)다. 검출 지연이 tracker의 `input_timeout_s`(0.2 s)를 넘으면 tracker가 스스로 회복하지 못한다. 그때는 tracker lifecycle을 다시 시작해야 했다(11-13). 이 두 문제의 수정이 `main`에 들어갔는지는 `records/patches/README.md`의 반영 상태 칸에서 확인한다.
- 실시간 추적은 로봇 지령 없이 `replay.launch.py`를 bag 없이 `use_sim_time:=false`로 띄운다(11-13 구성).

## 5. 예측 해제 지점 비접촉 시험 도구

`tools/robot/hover_predicted.py`는 `main`에 없다(11-13 실험 사본, `records/patches/2026-11-13-vision-crop-tracker-late-hover.patch`의 2·3번 커밋). 실제 팔을 움직이는 도구라 원격에서 검증할 수 없어 반영하지 않았다. 실험 사본에서 다시 쓰면 아래를 지킨다(`records/reviews/patch-review-2026-11-14.md`).

- 시작 자세가 알려진 안전 자세(park)인지 확인한 뒤 실행한다. 도구는 현재 자세에서 대기·정지 지점까지의 직선 경로를 검사하지 않는다.
- 이동 높이·속도·가속은 정적 투입 설정을 그대로 쓰고, park 자세의 IK는 도구가 미리 확인하지 않는다.
- `move_line` 실행 중 Ctrl+C로는 팔이 서지 않는다. 비상정지·펜던트로 멈춘다. 로봇 오류로 끝나면 도구는 park로 돌아가지 않으므로 펜던트로 복구한다.
- 통과 시각은 모든 뚜껑 검출을 모아 계산하므로 벨트에 뚜껑을 하나만 둔다.

## 6. 오늘 바꾸지 않는 것

DDS 발행 설정, 감시 노드 손실 판정 0.3 s, 기동 대기 시간, RELEASE 단계 제한 1.0 s(실제 controller I0 정상 0.88~0.91 s), 그리퍼 통신 설정·펌웨어. 근거와 후보는 `records/reviews/publish-stall-repro-2026-11-13.md`, `records/reviews/release-step-timeout-2026-11-13.md`다.
