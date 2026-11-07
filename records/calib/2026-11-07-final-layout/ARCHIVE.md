# Hand-Eye 세션 보관 자료

카메라 위치 확정 후 새 기물 배치에서 수집했다. 사용자 제공 높이600 mm, ArUco DICT_4X4_50 ID25 검은 변40 mm. 로봇실제제어는 종료했고 수동모드 설정 후 연결을 해제했다.

- 원본 영상: `pose-00-00.png`부터 `pose-49-04.png`,50자세×5장=250장.
- 영상↔팔 자세 목록: `image_pose_index.csv`(250행), 상세 대응 및 캡처메타데이터:`image_pose_pairs.jsonl`.
- 실제 팔 자세 및4×4행렬: `arm_poses.json`(50개), `matrices/base_from_controller_tcp/pose-NN.yaml`.
- 팔 원본응답: `pose-NN-posx.txt`(베이스XYZ mm,ZYZ deg,solution space), `pose-NN-rotm.txt`(실제회전행렬3행). TCP이름은빈문자열로보고됐으며 임의의tool0 TF와동일하다고가정하지않는다.
- 계산용:`pose-00`..`pose-29`,검증용:`pose-30`..`pose-49`. 명령목표가아닌실제측정자세를사용한다.
- 시간대응: 동기이동완료후정지상태에서팔자세를조회하고카메라버스트를촬영했다. 하드웨어공유시각동기화자료는아니다.
- 원본400개파일SHA-256:`all-data-manifest.sha256`. 검증촬영전에고정한계산용자료해시:`fit-data-manifest.sha256`.
- 카메라↔로봇과TCP↔표식의계산결과는`result/`에보관한다. 행렬의방향과단위는각결과파일에기록한다. 후보값과독립현장검증합격값을구분하며현재독립TCP접촉검증은0점이다.


## 최종 보관된 계산 결과

- 후보 카메라 변환: `result/camera_extrinsics_candidate.yaml`, 방향 `base_link <- camera_color_optical_frame`, 병진단위m, quaternion순서xyzw, 4×4행렬·역행렬 포함.
- 후보 표식 오프셋: `result/marker_offset_candidate.yaml`, 방향 `controller_tcp <- aruco_marker_25`, 단위m, 4×4행렬·역행렬 포함.
- 자세별 관측PnP행렬·모델예측행렬·코너: `result/observations.json`, `result/matrices/observed_pnp_camera_from_marker/`, `result/matrices/predicted_camera_from_marker/`, `result/matrices/predicted_base_from_marker/`.
- 오차표: `result/per_pose_errors.csv`. 계산용30자세 RMS0.9774px, 미사용검증20자세 RMS1.1473px, 검증p95=1.9247px, 최대2.3245px. 각자세5장평균코너 기준.
- 해석보고서: `result/report.md`, `result/report.json`. 분석소스와동결된수치의존모듈: `result/analysis/`. 실제사용K/왜곡스냅샷: `inputs/d455_color_1280x800.yaml`.
- 조정자 독립재계산 및무결성검증: `coordinator-verification.json`. 보고서사실관계와재현실행수정기록: `coordinator-corrections.json`. 수정전worker산출물: `result-worker-original.tar.gz`.
- 결과파일SHA256: `result/result-manifest.sha256`. 전체세션목록: `session-manifest.sha256`(목록자체제외).
- 독립현장TCP접촉검증은0점이다. `verify.pass=false`, `G1_pass=false`. 후보변환은실제운용설정에적용되지않았다. 카메라위치를바꾸면재수집대상이다.
