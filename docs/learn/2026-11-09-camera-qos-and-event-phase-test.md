# 카메라 QoS와 이벤트 위상 시험 (2026-11-09)

두 문제는 겉보기엔 다르지만 같은 질문이다. "받는 쪽이 못 따라가거나 보내는 쪽이 늦게 깼을 때, 시스템과 시험은 무엇을 약속하는가?"

## 1. 녹화 중 영상 프레임이 빠졌다 (LEGION, 현장)

- **증상:** D455 컬러 영상 15 s bag에 411장만 들어갔다(기대 약 450). CameraInfo는 435개로 빠짐이 없었다. 50 ms 넘는 간격이 22번 나왔다.
- **확인 방법:** `~/ctb-field-2026-11-09/gaps.py`(또는 `tools/bench/bag_integrity.py`)로 토픽별 개수·간격·`기록 시각 − stamp`를 센다. 카메라 stamp는 빠짐이 없는데 영상 토픽만 비면, 카메라 문제가 아니라 전송·구독 문제다.
- **가설을 하나씩 지웠다:** depth를 끄고 RGB만 녹화해도 빠졌다(depth 부하 아님). discovery를 LOCALHOST로 제한해도, 기본값이어도 빠졌다(탐색 문제 아님).
- **원인:** `realsense2_camera`의 `color_qos`·`depth_qos` 기본이 `SYSTEM_DEFAULT`(reliable, history depth 1)다. 깊이 1이면 녹화기가 읽기 전에 다음 프레임이 덮어쓴다. `color_qos:=DEFAULT depth_qos:=DEFAULT`(depth 10)로 바꾸니 같은 12 s 시험에서 누락이 0이 되었고 수신 지연 p50은 21~27 ms에서 18 ms로 줄었다.
- **깊이를 키우면 느려지나?** 새 프레임은 깊이와 상관없이 바로 나간다. 깊이는 아직 확인받지 못한 프레임을 몇 장 쥐고 있느냐일 뿐이다. 다만 받는 쪽이 처리 속도를 못 따라가면 큐에 쌓인 옛 프레임을 차례로 처리해 지연이 커진다. 그래서 녹화기는 깊이 10, 실시간 노드는 "최신 하나만"(구독 depth 1)으로 나눠 쓴다.
- **함정:** 선두 영상 한 장이 160 s 전 값이면 `TRANSIENT_LOCAL` 발행자가 마지막 샘플을 새 구독자에게 다시 준 것이다(retained 샘플). 시각 역행이 아니다. 간격 통계는 그 한 장을 빼고도 따로 본다.
- **자료:** [worklog](../../records/worklog/2026-11-09.md)의 "OFF-1 평면 기준판 bag과 카메라 QoS" 절, [bags.csv](../../records/bags.csv).

### 1.1 다시 확인하는 방법

```bash
# 카메라(현장 스크립트 cam_up.sh와 같은 인자). QoS를 빼면 기본값 SYSTEM_DEFAULT로 돌아간다
source /opt/ros/jazzy/setup.bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
       FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/conveyor-tracker-bot/src/drop_bringup/config/fastdds_large_shm.xml
ros2 launch realsense2_camera rs_launch.py rgb_camera.color_profile:=1280,800,30 \
  depth_module.depth_profile:=1280,720,30 color_qos:=DEFAULT depth_qos:=DEFAULT

# 녹화(경로·SHA-256을 records/bags.csv에 남긴다)
tools/eval/record_trial.sh -n <이름> -p static -d 10 -l current -m "<메모>" \
  -t "/camera/camera/color/image_raw /camera/camera/color/camera_info /camera/camera/depth/image_rect_raw /camera/camera/depth/camera_info /tf_static"

# 무결성: 토픽별 개수, 도착 순서 기준 stamp 역행·반복, 50 ms 초과 간격, 선두 retained 샘플, 영상·CameraInfo 쌍
python3 tools/bench/bag_integrity.py --out /tmp/integrity.json <bag 폴더>
ros2 topic info -v /camera/camera/color/image_raw    # 발행자·구독자 QoS(Reliability, Durability, History depth) 확인
```

### 1.2 수치 비교

| 조건 | 녹화 | RGB / CameraInfo | 50 ms 초과 간격 | 수신 − stamp p50 |
|---|---|---|---|---|
| 기본 QoS, RGB+depth | 15 s 기준판 bag | 411 / 435 | 22 | 21~27 ms |
| 기본 QoS, RGB만 / discovery 변경 | 시험 녹화 | 5~10% 누락 | 남음 | — |
| `color_qos:=DEFAULT depth_qos:=DEFAULT` | 12 s 시험 녹화 | 347 / 346 | 0 | 18.4 ms |
| 같은 설정, 정지 영상 세트 | 10 s × 5 | 276~287장, 선두 retained 제외 누락 0 | 0 | 약 18 ms |
| 같은 설정, 이동 take 2 | 110 s | 3279 | 1(66.7 ms) | — |

실시간 노드(`vision_node`)는 이와 별개로 구독을 RELIABLE/VOLATILE depth 1로 두고 최신 프레임만 처리한다. 녹화는 "다 받기", 실시간 처리는 "최신만"이 목표라 같은 토픽이라도 구독 QoS를 다르게 둔다.

## 2. 이벤트 직후 첫 프레임이 늦게 보인다 (CI, TC-MCK-005-01)

- **증상:** GitHub CI(2코어)에서만 "예정 3.0 s 이벤트의 첫 수신 stamp가 3.10·3.20 s"로 실패했다. PC에서는 통과했다.
- **원인:** 제품 정책(SPEC-700 4.1)은 타이머가 늦게 깨면 **그 순간의 최신 프레임만** 발행하고 놓친 프레임은 다시 내지 않는다. 그런데 시험은 "예정 시각 바로 다음 프레임을 반드시 받는다"를 요구했다. 두 약속이 부딪친다. 시험이 틀린 기준을 가지고 있었다.
- **재현:** 가짜 시계로 같은 지연(3.1, 3.2, 6.2667)을 주입하면 기존 판정은 결정적으로 실패한다. 이렇게 재현하면 CI에서만 나던 문제를 로컬에서 고정해 놓고 고칠 수 있다. 실제 PC에서 CPU 2개로 제한하고 부하를 줬더니 발행 이전 단계에서 프레임이 빠졌다(콜백이 고른 인덱스에도 없음). DDS 손실이 아니라 늦게 깬 콜백이다.
- **해결(기준을 완화하지 않고 층을 나눔):** (1) 시계를 한 프레임씩 진행하는 시험에서 `ceil(t_event × 30)` 첫 프레임에 효과가 나타나는지 1/30 s 허용 오차로 검사한다. (2) 실시간 시험은 받은 모든 프레임의 stamp와 이벤트 상태가 일치하는지, 이벤트 뒤 상태를 1번 이상 받았는지만 본다. 놓친 프레임 수는 합격 조건이 아니라 진단으로 남긴다.
- **배운 점:** 간헐 실패를 "재시도"나 "허용 오차 완화"로 덮지 않는다. 제품이 약속한 것(최신 프레임)과 시험이 요구한 것(모든 프레임)이 맞는지 먼저 확인한다. 푸시 뒤 CI에서 통과해야 이 항목이 끝난다.
- **자료:** [SPEC-700](../../specs/SPEC-700-mocks.md) 4.1·REQ-MCK-005, [TEST_SPEC](../../TEST_SPEC.md) TC-MCK-005-01/02, `src/drop_mocks/test/frame_timing.py`.

## 3. 도구를 믿기 전에 실제 입력으로 돌린다

OpenCode가 만든 `bag_integrity.py`는 단위 시험 10개가 통과했지만, 실제 bag을 읽는 CLI는 가짜 메시지 1개를 분석하는 임시 코드였다. 시간 역행도 정렬한 뒤에 세어서 찾을 수 없었다. 실제 bag에 돌려 이미 아는 값(`gaps.py`)과 대조했기 때문에 드러났다. 시험이 통과한다는 사실은 시험이 덮는 범위 안에서만 의미가 있다.

## 4. 이후 CI에서 본 같은 유형의 실패

TC-MCK-005-01을 고친 뒤에도 GitHub CI의 `drop_mocks` 실시간 시험은 가끔 실패했다. 3e29bf8·849b7d3에서는 `test_injected_events_appear_at_scheduled_time`(PickStone 사건 1.066 s, 기대 0.9 ± 0.1 s / 예정 2.5 s 이벤트를 창에서 못 받음), 4fa5b8d에서는 계약 시험 `test_tracker_heartbeat`(간격 0.362 s)이다. 이 커밋들은 vision·static_pick_place·문서만 바꿨고, 같은 시험이 ab0fd4b에서는 통과했다. 2코어 러너의 실시간 지연이라는 같은 유형으로 보이지만 원인은 확정하지 않았다. 1절과 같은 방법(가짜 시계로 지연을 재현해 기준이 제품 약속과 맞는지 확인)으로 다룰 대상이다.
