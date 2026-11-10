# 용어집

[학습 문서 목록](README.md)으로 돌아가기. 정의는 이 프로젝트를 읽는 데 필요한 범위이며, 계약·기대값의 원본은 [PLAN](../../PLAN.md), [TEST_SPEC](../../TEST_SPEC.md), [SPEC-010](../../specs/SPEC-010-interfaces.md), [SPEC-110](../../specs/SPEC-110-abort-timeout.md)이다.

## ROS 2와 통신

| 용어 | 뜻과 프로젝트 예 |
|---|---|
| 노드(node) | ROS 통신에 참여하는 실행 단위. orchestrator_node는 단계 실행, safety_supervisor_node는 독립 안전 감시를 맡는다. |
| 토픽(topic) | 발행자가 메시지 흐름을 보내고 구독자가 받는 통신 경로. robot/status·heartbeat처럼 계속 갱신되는 상태에 사용한다. |
| 서비스(service) | 요청 하나에 응답을 받는 호출. 프로젝트의 DetectObjects·Stop 같은 계약에 사용하며 짧게 끝나는 요청에 적합하다. |
| 액션(action) | 수락·진행 feedback·최종 결과·cancel을 갖는 장시간 작업 계약. DropTask와 로봇 skill이 예다. cancel 요청 전송은 취소 완료와 다르다. |
| lifecycle | 노드의 configure·activate·deactivate·cleanup 등 관리 상태를 제어하는 방식. 프로젝트의 관리 상태 설계와 실제 구현 여부는 PLAN·Progress를 확인한다. 모든 노드가 자동으로 lifecycle 노드인 것은 아니다. |
| QoS | 신뢰성·보관 방식·queue 깊이 등 전달 계약. Reliable/Best Effort, Transient Local/Volatile 조합이 메시지 수신과 선두 샘플에 영향을 준다. |
| DDS discovery | 노드들이 상대의 publisher·subscriber·service를 발견하는 과정. 프로세스가 시작돼도 상대가 즉시 보이지 않을 수 있다. |
| retained sample | Transient Local 발행자가 보관했다가 늦게 연결한 구독자에게 전달하는 샘플. 현장 bag의 오래된 선두 영상 분석에 등장한다. |
| remap | 실행할 때 토픽·서비스 이름을 다른 이름으로 바꾸는 것. 중복 provider 감시도 해석된 이름을 확인해야 한다. |
| ROS_DOMAIN_ID | DDS 통신 영역 구분 값. PC mock 시험을 현장 장비 통신과 분리하는 데 사용한다. |

## 시간과 실행

| 용어 | 뜻과 프로젝트 예 |
|---|---|
| ROS 시각 | 데이터 stamp·작업 deadline 등 ROS에서 쓰는 시간. 기본 시스템 시각 또는 시뮬레이션 /clock을 따르며 정지·역행 가능성을 다룬다. |
| steady 시각 | 역행하지 않는 경과시간 기준. heartbeat 손실·중단 경로의 유한 상한을 재는 데 사용한다. Python의 monotonic이 대응 예다. |
| use_sim_time | 노드가 ROS 시각을 /clock에서 받게 하는 설정. /clock 정지가 안전 감시까지 정지시키지 않도록 steady 감시를 분리한다. |
| stamp | 메시지에 기록한 사건 시각. 영상 노출 시각과 호스트 수신 시각의 의미를 구별해야 한다. |
| deadline | 작업을 마쳐야 할 절대 시각. 해제 후 RETREAT·VERIFY는 원 사이클 deadline과 독립적인 유한 예산을 갖는다. |
| executor | 준비된 ROS callback을 선택해 실행하는 실행기. MultiThreadedExecutor도 대기 집합 관리·스레드 경합 비용이 있다. |
| callback group | callback 간 동시 실행 허용 규칙. mutually exclusive는 같은 그룹의 동시 실행을 막고 reentrant는 허용한다. 공유 데이터 보호는 별도 설계가 필요하다. |
| heartbeat | 노드 생존 상태를 주기적으로 알리는 메시지. 수신 공백은 steady로 판단하며 정상 heartbeat만으로 모든 기능 정상은 보장하지 않는다. |
| epoch / revision | epoch는 시간·추적 세계의 세대 구분, revision은 같은 계획의 갱신 구분. 오래된 track·plan을 다시 쓰지 않게 하는 계약에 사용한다. |
| p95 / p99 | 표본의 95%/99%가 이 값 이하인 분위수. 지령 간격·지연의 꼬리를 보여 주지만 최대값이나 절대 보장은 아니다. |

## 중단과 안전

| 용어 | 뜻과 프로젝트 예 |
|---|---|
| L0 LOCAL_STOP | controller가 stale·범위 밖 지령에 로컬 정지하고 이후 지령을 거부하는 경로. 다른 노드의 판단을 기다리지 않는다. |
| L1 RETRY(제거) | 예전에 검출·파지 실패를 같은 단계부터 다시 하던 등급. 2026-11-10에 제거했다. 값 1은 비워 두고 검출·파지 실패는 L2로 끝난다([설명](2026-11-10-scope-trim-success-path.md)). |
| L2 ABORT_CYCLE | 공깃돌을 들지 않고 동적 구간이 아닌 상황 등에서 사이클을 중단하고 안전 자세 후 작업자 대기로 간다. |
| L3 SAFE_ABORT | 파지 중 또는 동적 구간의 실패에서 제어된 후퇴·안전 자세·공깃돌 내려놓기를 거쳐 작업자 대기로 간다. |
| L4 PROTECTIVE_STOP | 하드 이상·controller heartbeat 손실·후퇴 실패 등에 Stop을 요청하고 작업자를 기다린다. 자동 reset·원점 복귀를 하지 않는다. |
| 해제 후 정리 | 이미 놓은 공깃돌을 되돌릴 수 없어 비하드 이상에도 RETREAT·VERIFY를 수행하는 정책. 하드 이상은 여전히 L4가 우선이다. |
| fail-closed | 검증에 필요한 조건이 없으면 정상으로 통과시키지 않는 방식. logger가 schema를 찾지 못할 때 검증 성공으로 취급하지 않는 사례다. |
| latch | 사건 상태를 기억해 후속 처리를 유지하는 장치. Stop 전송 실패까지 “완료”로 latch하면 재시도를 막는 결함이 된다. |

## 시험·자료·좌표

| 용어 | 뜻과 프로젝트 예 |
|---|---|
| SPEC / REQ / TC | 명세 문서 / 요구사항 / 시험 케이스. req·tc 마커로 테스트 함수와 연결한다. |
| TDD | 기대값을 정하고 실패하는 시험을 먼저 만든 뒤 최소 구현·통과·리팩터링하는 흐름. |
| U / C | 순수 모듈 단위 시험 / 노드 외부 계약 시험. ROS 없는 U 통과와 ROS 통신 C 통과를 구분한다. |
| I0 | 실제 오케스트레이터와 세계·로봇 대역의 통합. 정상·장애·중단을 장비 없이 시험한다. |
| I1 / I2 | 실데이터 replay의 검출·추적 / fixture 또는 replay 기반 계획·시간 예산 통합. |
| I3 / I4 / I5 | 합성 관측과 실제 로봇 / 실제 인식과 로봇 대역 / 전체 실제 경로 통합. 실제 장비가 들어가는 현장 단계다. |
| mock / fake / fixture | 외부 역할을 대신하는 대역 / 단순화한 시험 구현 / 고정 시험 입력. mock_world, FakeClock, 합성 궤적이 각각 예다. |
| oracle | 시험의 정답 기준. 구현 상수를 공유하면 같은 누락을 놓칠 수 있어 SPEC 기반 독립 metric 집합을 사용한다. |
| rosbag / replay | ROS 메시지·시각을 저장한 자료 / 저장 메시지 재생. 원본은 저장소 밖에 보존하고 bags.csv에 경로·해시를 기록한다. |
| ON / ON* / OFF | 장비 없는 작업 / 현장 자료가 필요한 온라인 분석·회귀 / 실기 지령·실시간 센서·물리 측정. 원격 접속 여부로 나누지 않는다. |
| gate / OFF-n | 합격 조건 / 현장 작업 세션 번호. OFF-1 자료가 있어도 G1 보정 검증은 별도로 남을 수 있다. |
| TF / hand-eye | 좌표계 사이 변환 / 로봇 자세와 영상 표식을 이용한 카메라·로봇 관계 보정. 후보 변환의 영상 잔차와 독립 접촉 오차를 구별한다. |
| TCP / ZYZ | 공구 중심점 / 드라이버가 사용하는 회전 표현. 프로젝트 내부 단위와 드라이버 mm·deg·ZYZ 변환은 adapter에 모은다. |
| pulse / DRL / RS-485 | 그리퍼 위치 레지스터 단위 / 로봇 제어기의 프로그램 언어 / 직렬 통신 물리 계층. 이 프로젝트는 플랜지 RS-485와 DRL 경로를 사용한다. |
| QoS (Reliability·Durability·History depth) | DDS 전달 약속. RELIABLE은 재전송, TRANSIENT_LOCAL은 늦게 온 구독자에게 마지막 샘플을 다시 줌(retained 샘플), depth는 확인받지 못한 샘플을 몇 개 쥐는지. 녹화는 depth 10, 실시간 처리는 최신 1개. |
| Lab 거리(ΔE) | 사람 눈의 색 차이에 가깝게 만든 Lab 색공간에서의 거리. 공깃돌은 ROI 배경과 ΔE ≥ 30이면 전경. 색상(H)이 비슷해도 밝기가 다르면 크다. |
| 원형도 / 원 적합 | `4π·면적/둘레²`(원 = 1). 고리가 끊긴 뚜껑은 바깥 경계 점에 최소제곱 원을 맞추고 각도 덮임 비율로 채택한다. |
| 광선-평면 교차 | 픽셀을 왜곡 보정해 카메라에서 나가는 광선을 만들고, 보정한 평면(매트·벨트 + 대상물 높이)과 만나는 점을 위치로 삼는다. depth를 쓰지 않는다. |
| 플랜지 / TCP 미등록 | 로봇 손목 끝 원판. 공구를 등록하지 않으면 제어기가 보고·지령하는 위치는 플랜지다. 손가락 끝은 그 아래 약 160 mm. |
| 역기구학(ikin) / J3 = 0° | 목표 자세 → 관절각 계산. E0509에서 J3(팔꿈치)가 0°면 팔을 다 편 도달 경계라 제어기가 NOT REACHABLE로 거절한다. |
| drl_state | DRL 프로그램 상태. 0 PLAY, 1 STOP, 2 HOLD, 3 LAST(연결 뒤 실행한 적 없음). 결과를 쓴 직후에도 잠시 PLAY라 그때 보낸 이동은 무시된다. |
| 로봇 모드 / 제어 권한 | MANUAL(0)·AUTONOMOUS(1). 수동 모드에서는 DRL이 거부된다. 펜던트가 자동·대기가 아니면 ROS 드라이버의 제어 권한 요청도 거부된다(ACCESS_CONTROL_DENY). |
| 파지 판정(막힘) | 닫기 목표에 못 가고 위치가 멈춘 상태. 이동이 있었거나 전류가 한도의 80% 이상이면 대상을 쥔 것으로 본다. 목표까지 닫히면 빈손. |
| GT | Ground Truth, 비교용 독립 기준값. 픽셀 이동이나 추적기의 자체 출력만으로 실측 mm/s GT를 대체하지 않는다. |

현장 예는 [OFF-1·OFF-2 설명](2026-11-07-site-session-off1-off2.md), 시계·경쟁 조건 예는 [CI 설명](2026-11-09-ci-failures-and-robustness.md), 검출·좌표는 [검출 설명](2026-11-09-stone-cap-detection.md), 로봇·그리퍼는 [정지 pick-and-place 설명](2026-11-09-static-pick-and-place.md)을 참조한다.
