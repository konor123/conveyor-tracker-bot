# 카메라 보정 인계 — 2026-11-07

- 실행: 이 호스트 academylegion, Codex coordinator 직접 수행. 실기 동작 및 판단 작업으로 worker 배정 없음. coordinator 및 타 에이전트 사용량·잔여량·초기화 시각은 미확인.
- 인계 근거: Claude 카메라 세션 774da4f2-5ee7-4834-8c14-8427f65e1c13. 기존 ChArUco 도구 완료, 실제 외부 보정 없음. 마지막 사용자 대화는 그리퍼에 ArUco를 물리고 다자세로 카메라↔로봇/표식 오프셋을 추정하는 방안. 해당 방식의 실제 수집 자료 및 완성된 전용 실행 도구는 확인하지 못함.
- 사용자가 그리퍼 표식 장착 후 회전 및 카메라 보정 인계를 지시. joint7 표기를 질의하여 마지막 손목축 joint_6으로 확정했고 상대 +90° 지시를 받음.
- 현장 준비는 기존 gripper-takeover 세션의 사용자 비상정지 준비 확인 기록을 승계. 새 서비스 시작·설정 변경·그리퍼 개폐 없음.
- preflight: robot_state=1, DRL=0, 마지막 alarm level=1/group=1/index=2007. 관절 6개, joint_6≈0°. TCP 위치 약 [365.995, -0.005, 404.631] mm.
- /dsr01/motion/move_joint: relative [0,0,0,0,0,90] deg, velocity=5 deg/s, acceleration=10 deg/s², sync. 응답 success=True.
- postflight: joint_6=89.9999466°, 나머지 관절 사실상 동일, robot_state=1. 원시 응답과 요약은 records/calib/2026-11-07-takeover/.
- D455 직접 촬영: before-rotation.png, after-rotation.png, capture-log.csv. DICT_4X4_50 검출: 회전 전 없음, 회전 후 ID24만 검출(매트 우상단). 화면 좌상단의 별도 표식은 상단이 잘림. 그리퍼 표식 ID와 실제 검은 변 길이는 사용자 응답 대기.
- 보정 결과·G1 합격 없음. 단일 시점/미확정 표식을 외부 보정 대응점으로 사용하지 않음. 표식 가시성·ID·실측 크기를 확인한 뒤 다양한 회전축을 포함하는 수집 및 독립 검증 절차 준비 필요.
- 기존 미커밋 파일 보존. 로봇 자동 복귀·추가 이동 없음. 카메라 촬영 프로세스 종료.

## 수동 제어 인계 (2026-11-07T18:34:59.616770+09:00)

- 표식 한 변 40 mm라는 사용자 답변. 숫자 ID는 미확정.
- 사용자 추가 지시: joint_6 상대 −180°. 시작값 +90°, STANDBY, DRL STOP(1) 확인 후 5°/s, 10°/s²로 실행. MoveJoint success=True.
- 예상 종점 −90°는 계산값이며 실측값으로 기록하지 않는다. 사용자가 회전 후 자동 제어 중지를 지시하여 후속 자세 조회·촬영·추가 지령 없이 수동 제어로 인계했다.
- 사용자가 그리퍼 표식을 카메라 FOV 안으로 직접 옮길 예정. 재개 지시까지 자동 동작 중지.

## 수동 제어 문제와 지정 상대 이동 (2026-11-07T18:39:05.240448+09:00)

- 원인 확인: robot_mode=1(AUTONOMOUS), 드라이버 연결 유지. dsr_controller2.cpp의 OnMonitoringAccessControlCB는 REQUEST에 RESPONSE_NO를 보내므로 외부 제어권 요청을 거절한다. 앞선 '자동 제어 중지'는 지령 중지였고 연결 해제가 누락됐다.
- 수동 모드(0) 설정·재조회 성공 후 기존 launch 172283 및 자식 172289/172291 종료. 프로세스 및 로봇 TCP 연결 부재 확인.
- 사용자가 수동 조정 대신 상대 X +250 mm, Z −150 mm 이동을 지시. 베이스 기준, 자세 유지로 해석하고 대화에 명시. 드라이버를 재기동(launch PID231608, hardware231631); 시작 robot_state=1, robot_mode=1, alarm=0.
- 시작 TCP [365.993133545, −0.004879135, 404.572601318] mm. joint_6 −90.0000305°로 이전 회전 종점 확인.
- MoveLine ref=0, mode=1, pos=[250,0,−150,0,0,0], velocity=[20,5], acceleration=[20,10], synchronous. success=True. 종료 TCP [615.993347168, −0.004725540, 254.571807861] mm, robot_state=1.
- after-translate.png 및 검출 결과 저장. ID24만 검출(매트 표식), 그리퍼 표식은 영상에서 확인되지 않음. 보정 결과 없음.
- 앞선 수동 인계 요청에 따라 robot_mode=0 설정·조회 성공 후 이번 launch 및 자식 종료. PID 부재, 110.120.1.38 TCP 연결 부재 확인. 자동 재연결 없음. 실제 펜던트 수동 제어 가능 여부는 사용자 확인 대기.

## 카메라 높이 450 mm FOV 확인 (2026-11-07T18:49:15.095739+09:00)

- 사용자 보고: 카메라 높이를 450 mm로 조정. 높이는 사용자 제공값이며 별도 실측하지 않음. 카메라 위치 변경 뒤 새 영상으로 확인.
- 로봇 재연결·제어 지령 없음. capture_d455.py로 camera-height450-check.png 저장.
- DICT_4X4_50 검출: ID24(매트), ID23(벨트 부근). 상단 중앙 표식은 영상 위쪽에 잘려 있고, 그리퍼 일부가 상단 우측 가장자리에 보임. 그리퍼 표식의 완전 가시성 및 ID는 확보하지 못함.
- camera-height450-markers.json, camera-height450-annotated.png에 검출 결과 저장. 카메라 위치를 옮긴 상태이므로 이전 영상과 동일 외부 변환으로 묶지 않음. 보정 결과 없음.

## 카메라 높이 600 mm FOV 확인 (2026-11-07T18:51:03.114860+09:00)

- 사용자 보고 높이 600 mm. 로봇 연결·제어 지령 없이 D455 영상 camera-height600-check.png 촬영.
- 그리퍼에 물린 표식 전체가 영상 안에 보임. DICT_4X4_50에서 ID25 검출. 네 꼭짓점 [(581,102),(582,154),(530,155),(529,103)] px, 변 길이 약52 px, 상단 여유102 px. 고정 표식 ID23·24도 검출.
- 사용자 제공 표식 변 길이40 mm. 가시성 확인이며 외부 보정·정확도 합격 판정 아님. 카메라 이동 후 영상은 600 mm 별도 배치 자료로 취급.
- camera-height600-markers.json 및 camera-height600-annotated.png 저장. 로봇은 앞선 수동 인계 상태 유지.

## Hand-Eye 多자세 수집

- 카메라 위치 확정(사용자 높이600 mm), 새 기물 배치, 사용자 준비 확인 후 진행.
- Agy Argon 독립 수치 검토: B_i X = Y C_i, cv2.calibrateHandEye에 B_i^-1와 C_i를 입력하면 Y=base<-camera. 단일 축·이동만으로는 퇴화, 작은 표식 PnP 법선 모호성 확인. 원문과 임시 수치 분석 파일 검토함.
- 초기 +Z30 mm 후 +X60 mm 이동에서 작업 범위에 재진입한 손을 확인하여 MoveStop(stop_mode1) success. +X60 이동 자체는 완료된 상태였고 postflight STANDBY. 사용자 재차 준비 확인 뒤 최신 영상으로 손이 범위 밖임을 확인.
- +X60 종점은 elbow 약29.3°로 singularity region 경고3205 발생. 이후 X−40 mm로 안쪽 이동. 기준 TCP [635.95703125,-0.01290254,284.44219971] mm. 기준 관절3 약46.88°. 안전 정지 자동리셋·알람삭제 없음.
- 계산용30+검증용20 자세를 역기구학 sol_space2로 사전 점검. 관절3>35°, 관절5 절댓값20..160° 조건으로50개 선택,28후보 제외. 위치 범위와 회전 목표는 pose-plan.json. 실제 이동은 기존 MoveLine 서비스,10 mm/s 및5°/s(첫 자세3°/s), 매 자세 동기 완료 후 네이티브TCP·회전행렬·컬러5장 저장. 제품 코드 수정·신규 제어노드 배포 없음.
- 50자세250장 수집,400개 원시 파일 SHA256 보존. 계산용0..29와 검증용30..49 분리. 계산용 자료 해시를 검증 촬영 전에 고정.
- 중간QC: 초기16자세80장 모두ID25 검출, corner std최대0.341 px. 고정ID23의37장면 corner drift p95=0.106px,max=0.151px. 계산용 자세 translation block rank6,condition5.53.
- 마지막 상태STANDBY, 마지막 alarm level1/group2/index3206(Inside->Outside). 수동모드0 설정·재조회 후 launch242070 및 자식 종료. 실제로봇TCP연결·PID부재 확인. 자동복귀·그리퍼개폐 없음.
- 자료: records/calib/2026-11-07-final-layout/. 현장TCP접촉 독립검증0점, G1판정없음. 현재 단계는오프라인변환계산.

## Hand-Eye 후보 계산·보관 검증 완료 (2026-11-07T19:49:25.890206+09:00)

- 계산: Agy gemini-4-argon-high(Orca ctx_1b62d40c3546), 변환검증·보고서사실확인·보관검증: Codex coordinator. Opus선행호출은현재인증방식미지원으로실패했고Argonfallback을사용했다.
- B_i X=Y C_i에서PARK초기화→분리형병진LS→고정K/왜곡의FIT-only LM픽셀정밀화. FIT30자세, 별도검증20자세. 250/250 ID25검출. Y=base_link<-camera, X=controller_tcp<-marker.
- Y병진[m]=[0.8465827425227948,0.05628050882349247,0.5924757616195993], X병진[m]=[-0.00024538,-0.03951986,0.14718508]. 전체정밀4×4·쿼터니언·역행렬은candidate YAML과report.json.
- FIT RMS0.977437px, 검증RMS1.147313px,p95=1.924749px,max=2.324480px. 자세별5장평균코너기준. Coordinator가원본250장을독립재검출하고실제팔행렬·고정K로직접투영해RMS일치(차이<1e-5px),역행렬·행렬체인확인.
- 원본400파일·결과158파일초기해시확인. worker보고서의전역최적성표현,깊이범위,고정표식변길이·지터,베이스Z와테이블높이비교,holdout평가횟수문구를실제데이터에맞게수정. 원본worker결과는result-worker-original.tar.gz로보존.
- 분석스냅샷의자기복사오류를고쳐프로젝트내분석본으로재실행. 숫자추정알고리즘·목적함수·분리·K불변,4×4요소최대차이0. 최종분석SHA256=0314f29e9eb719c442191bdc92669f59f6f7fffceb21ae2d13b9818613fe10a3.
- 프로젝트세션폴더records/calib/2026-11-07-final-layout/:250image-pose대응JSONL/CSV,50actualB행렬,관측PnP후보·모델예측C·base<-marker각50세트,최종X/Y및역행렬,분석소스·수치의존모듈·환경버전·K스냅샷·보고서·오차그림보관. 최종결과163파일해시,전체세션699파일session-manifest.sha256. 사용자추가지시한기록·보관완료.
- 현장독립TCP접촉0점, verify.pass=false,G1_pass=false. 실기1mm정확도합격미판정·운용설정미적용. 다음물리검증은기등록정밀포인터TCP와작업면독립접촉점20개이상필요. 추가실기지령·재연결없음.

## 낮은높이 과거영상 프로젝트내보관 (2026-11-07T20:04:09.488274+09:00)

- 사용자추가지시로초기ChArUco자료를 /home/umm/drop_data/calib/2026-11-07 및Claude기존scratchpad의cap/cap2/cap3/cap4에서회수·복사. 원래위치파일유지. 로봇·카메라접속없음.
- 이전인계촬영3장과450mm확인원본·표시영상·검출JSON,캡처로그와존재하는원래팔자세기록도복사.
- 프로젝트경로records/calib/2026-11-07-lower-heights/. 복사37파일, 이미지25개(카메라원본사본14개=고유촬영11장,표시7개,확대2개,인쇄참고2개). 파생/참고자료를원본카메라촬영과구분.
- 초기350mm는site-survey L01사용자제공공칭값임을표시.450mm변경전인계사진의정확한영상별높이와미기록팔자세는미확정으로표시.
- provenance.json·image-index.csv·summary.json·README.md와전체41파일SHA256목록보관·전부읽어검증. 최종600mm Hand-Eye자료및해시불변.
- 직접수행근거: 원본출처/파생자료판단과작은정형복사검증으로위임왕복보다작음. 새worker배정없음.
