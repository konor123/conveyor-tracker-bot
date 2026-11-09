# 2026-11-09 잠정 평면

`static_pick_place`가 쓰는 매트·벨트 평면과 현장 설정. 모두 **잠정값**이며 G1 보정 검증을 대신하지 않는다.

| 파일 | 내용 | 근거 |
|---|---|---|
| `mat_plane.yaml` | robot_base 기준 매트면(n·p = offset_m) | bag `2026-11-09_194454_off1-plane-ref-mat-pos1`의 ChArUco 5×7 보드 40프레임 PnP(재투영 중앙 0.13 px, offset 표준편차 0.57 mm) × 후보 외부 변환 `cal-2026-11-07-handeye-01`. depth 미사용 |
| `belt_plane.yaml` | 벨트면 | 매트면 + 93 mm(사용자 실측: 작업대 → 벨트 100 mm, 매트 두께 7 mm), 매트와 평행 가정 |
| `site.yaml` | `pick_place --config`용 덮어쓰기 | 경로는 저장소 루트 기준 상대 경로 |
| `vision_node.yaml` | `vision_node` 현장 파라미터(보정·평면 경로, 실측 높이 15·22 mm) | `drop.launch.py use_mock_vision:=false vision_params_file:=records/calib/2026-11-09-planes/vision_node.yaml` |

- `frame: robot_base`는 Doosan `base_link`(외부 변환 parent_frame)와 같은 좌표계다. `calibration_id`는 외부 변환과 일치해야 노드가 받아들인다.
- 21:30경 컨베이어·매트를 로봇 쪽으로 옮겼다. 매트는 같은 작업대 위라 그대로 썼지만 다시 재지 않았다.
- 경로 규칙: 설정 안 파일 경로의 상대 경로는 **저장소 루트 기준**이다(SPEC-000 4.4절, REQ-SYS-005). 실행 위치와 무관하다. 다른 곳에 둔 사본을 쓰려면 `DROP_REPO_ROOT`로 루트를 준다.
- 사용: `python3 -m static_pick_place.pick_place --config records/calib/2026-11-09-planes/site.yaml`
- 기록: `records/sessions/OFF-4-2026-11-09.md`, `records/worklog/2026-11-09.md`
