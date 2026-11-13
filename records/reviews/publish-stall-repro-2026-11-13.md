# 발행 정지 0.33 s 미세 재현과 native 스택 (2026-11-13)

- 수행: coordinator(Claude Opus 5.5). 실행 장소: 사무실 PC WSL(Ubuntu 24.04, ROS Jazzy, `rmw_fastrtps_cpp` 8.4.4, Fast DDS 2.14.7, rclpy 7.1.12, `RMW_IMPLEMENTATION` 미설정 = Fast DDS).
- 앞선 관찰(`records/reviews/heartbeat-stall-evidence-2026-11-13.md`): 대역 세계 프로세스가 기동 2.5~3.2 s 뒤 rclpy `Publisher.publish` 안에서 0.31~0.38 s 멈췄고(CPU 약 0.01 s), CI의 대역 controller heartbeat 공백도 기동 3.0~3.25 s 뒤 0.33~0.43 s였다. rclpy `publish`는 GIL을 놓지 않으므로 발행이 막히면 프로세스 전체가 멈춘다.
- 표기: **[확인]** = 이번 실행으로 확인, **[추론]** = 근거 있는 추정.

## 1. 방법

저장소 밖 노드 없이 rclpy만 쓰는 두 스크립트(`records/reviews/evidence/publish-stall-2026-11-13/`):

- `pub_probe.py`: 노드 하나가 reliable·volatile·depth 20(감시 노드 heartbeat QoS와 같음) 토픽에 64바이트 문자열을 100 Hz로 12 s 발행하고, `publish()` 한 번이 20 ms 이상 걸린 시각·시간을 남긴다.
- `joiner.py`: 새 프로세스·참여자 하나. `sub`는 같은 토픽을 구독, `other`는 다른 토픽을 구독하고 4 s 동안 spin한다.
- 시나리오(`run_stall.sh`): `spaced` = 발행자 준비 뒤 1 s `sub`, 3 s `other`, 5 s `sub`, 7 s `sub`×3, 9 s `other`×3. `burst` = 준비 직후 `sub`×3·`other`×3을 한꺼번에(launch가 노드를 함께 띄우는 모양).
- 설정: `default`(동기 발행, 공유 메모리 + UDP), `async`(`RMW_FASTRTPS_PUBLICATION_MODE=ASYNCHRONOUS`), `udp`(`FASTDDS_BUILTIN_TRANSPORTS=UDPv4`, 공유 메모리 끔). 환경 변수 이름과 기본값은 rmw_fastrtps jazzy README·`participant.cpp`와 Fast DDS 2.14 문서의 인용으로 정했다(OpenCode 조사). 변수가 실제로 적용됐는지는 따로 확인하지 않았고, 설정마다 결과가 달랐다는 것만 확인했다. 표의 18회는 `ROS_AUTOMATIC_DISCOVERY_RANGE`를 비운(기본) 상태다. LOCALHOST면 rmw가 전송을 직접 정해 `FASTDDS_BUILTIN_TRANSPORTS`가 듣지 않으므로 udp 대조를 위해 비웠다. I0·CI는 LOCALHOST를 쓰므로 그 설정으로 default·burst를 5회 더 돌렸다.
- 설정 3 × 시나리오 2 × 3회 = 18회(`run_all.sh`, 결과 `summary.txt`). 다른 시험이 돌지 않을 때 실행했다.

## 2. 결과 [확인]

| 설정 | spaced(3회) | burst(3회) |
|---|---|---|
| default | 최악 0.6~1.0 ms, 20 ms 이상 0회 | **2회**: `publish()` 한 번이 0.330 s(준비 3.97 s 뒤)·0.328 s(3.90 s 뒤). 나머지 1회 최악 0.6 ms |
| async | 최악 0.2~0.3 ms, 0회 | 최악 0.2~0.4 ms, 0회 |
| udp | 최악 0.5~0.7 ms, 0회 | 최악 0.5 ms, 0회 |

- `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`(I0·CI 설정)에서 default·burst는 **5회 중 0회**(최악 0.3~0.4 ms)였다.
- burst의 참여자는 준비 0.4~0.5 s 뒤 생겼다. 정지는 그 약 3.4 s 뒤였다. 정지 시간 0.33 s와 "참여자가 몰린 뒤 약 3 s" 모양이 I0·CI 관찰(0.31~0.43 s, 기동 2.5~3.25 s 뒤)과 같다.
- 정지한 2회는 발행 횟수가 1168·1169회(정상 1200회)로, 정지 동안 발행이 그만큼 빠졌다.

## 3. native 스택 [확인]

`run_pyspy.sh`: 같은 burst·default를 py-spy 0.4.3(PyPI, 이 실험 폴더의 임시 venv)이 부모로 띄워 200 Hz로 기록(`--native --idle`)했다. 5회 중 2회 재현(0.326 s, 0.330 s). `spy_stack.py`로 `Publisher.publish` 안에 머문 가장 긴 연속 표본(58개·69개 = 0.29·0.35 s)을 보면 표본 전부가 같은 곳이다.

```text
<module> [pub_probe.py]
Publisher.publish [publisher.py]
(_rclpy_pybind11) → (librcl) → rmw_fastrtps_shared_cpp::__rmw_publish
eprosima::fastdds::dds::DataWriterImpl::write_w_timestamp
eprosima::fastdds::dds::DataWriterImpl::perform_create_new_change
eprosima::fastrtps::rtps::StatefulWriter::unsent_change_added_to_history
eprosima::fastrtps::rtps::RTPSMessageGroup::~RTPSMessageGroup
eprosima::fastrtps::rtps::RTPSMessageGroup::send
eprosima::fastrtps::rtps::LocatorSelectorSender::send
eprosima::fastrtps::rtps::RTPSWriter::send_nts
pthread_mutex_lock [libc.so.6]          ← 58/58, 69/69 표본
```

동기 발행은 호출한 스레드가 직접 보낸다. 그 보내기 경로의 mutex 하나를 0.3 s 기다렸다.

## 4. 해석과 한계

- 확인한 것: 이 재현에서 막힌 곳은 rclpy·노드 코드가 아니라 Fast DDS 동기 전송 경로의 mutex 대기다. 공유 메모리를 끄거나 비동기 발행으로 바꾸면 같은 조건에서 3회 중 0회였다.
- 재현 조건의 한계: 발견 범위 기본(멀티캐스트 발견)에서만 재현됐고, I0·CI가 쓰는 LOCALHOST에서는 0/5였다. 따라서 I0·CI의 정지가 같은 원인인지는 이 실험으로 정해지지 않는다. 같은 것은 정지 시간(약 0.33 s), 시점(참여자가 몰린 뒤 약 3 s), 막힌 곳이 rclpy `publish` 안이라는 것까지다. 현장 bringup이 발견 범위를 어떻게 두는지는 확인하지 않았다.
- 모르는 것 **[추론]**: 그 mutex를 0.3 s 쥔 스레드와 그 스레드가 한 일. py-spy는 Python 스레드만 기록해 Fast DDS 내부 스레드는 보지 못했다. 새 참여자들의 공유 메모리 포트·세그먼트를 열거나 정리하는 일이 그 mutex 안에서 이뤄진다고 보지만 확인하지 않았다. 표본이 작다(정지 2/3, 대조 0/3·0/3).
- I0·CI 실행들의 native 스택은 없다.

## 5. 바꾸지 않은 것과 후보

현장 전이라 DDS 설정·감시 판정 한도(0.3 s)는 바꾸지 않았다. 다음은 후보이며, 각 후보에 확인할 것이 남아 있다.

| 후보 | 효과(이 실험) | 걸리는 것 |
|---|---|---|
| heartbeat·상태 발행 노드만 비동기 발행 | 0/3 | 발행 스레드 한 번 더 거침(지연·지터 측정 필요), 노드별 설정 방법(XML 프로파일) |
| 공유 메모리 끄기(UDP만) | 0/3 | 카메라 영상(약 3 MB)은 큰 공유 메모리 프로파일(`src/drop_bringup/config/fastdds_large_shm.xml`)이 필요해 전 노드 일괄 적용 불가 |
| 노드를 한꺼번에 띄우지 않고 나눠 기동 | 미시험 | 기동 시간 |
| 감시 노드가 controller heartbeat를 몇 번 받은 뒤부터 손실 판정(REQ-ABT-010 변경안) | 해당 없음(증상 완화) | 기동 직후 실제 손실을 늦게 봄 |

현장 운용 절차(기동 → 2 s 대기 → 사유를 남기고 해제)와 I0 시험의 기동 안정 대기(1.5 s)는 바꾸지 않았다. 다만 이 재현과 I0·CI 관찰의 정지는 기동 2.5~3.9 s 뒤라 두 대기보다 늦다. 대기를 늘리는 것도 후보다.
