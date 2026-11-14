# 2026-11-14 현장 영상(간이 스크립트의 이동 뚜껑 투입)

간이 투입 스크립트 시도 4회의 상단 카메라(D455 RGB) 녹화를 H.264 MP4로 압축했다. 영상 합계 5.45 MB. 압축본과 원본의 프레임 수가 `.frames.csv` 행 수와 같은지 확인했다.

- 원본은 rosbag이 아니다. 시도마다 `run_recorder.py`(검토 대기 패치 `records/patches/2026-11-14-g4-simple-demo.patch`)가 `/camera/camera/color/image_raw`를 작업 구역(x 240..1060, y 0..520 px)만 잘라 mp4v 30 fps로 쓴 파일이다. 원본은 현장 노트북(호스트 `fastcampus`)의 저장소 밖 데모 사본에 보존한다. 원본 파일 이름·크기·SHA-256은 [manifest.json](manifest.json)에 있다.
- 영상은 고정 **30 fps 재생용 복사본**이다. 프레임 사이 실제 시간 간격은 영상 재생 시간으로 재현하지 않는다. 각 `.frames.csv`가 프레임 번호, 원래 header stamp(s), 수신 시각(s)을 보존한다. 시간 분석은 이 CSV와 `records/trials/2026-11-14-g4-simple/run-NN.dets.jsonl`을 쓴다.
- 영상 왼쪽 위 숫자는 프레임 header stamp의 초 단위 아래 세 자리와 밀리초다.
- FFmpeg/libx264, CRF 26, preset medium, yuv420p. 버전·명령·SHA-256·프레임 수는 [manifest.json](manifest.json).
- 깊이 미리보기는 없다(녹화기가 RGB만 받았다).

| 시도 | 판정 | 영상 | 원래 프레임 시각 | 프레임 | 크기(MB) |
|---|---|---|---|---:|---:|
| `records/trials/2026-11-14-g4-simple/run-11` | 실패(돌이 뚜껑 43 mm 뒤 벨트에 떨어짐) | [MP4](2026-11-14_125016_g4-simple-run-11-miss.rgb.mp4) | [CSV](2026-11-14_125016_g4-simple-run-11-miss.rgb.frames.csv) | 1093 | 1.46 |
| `records/trials/2026-11-14-g4-simple/run-13` | 성공(쥐고 있던 돌) | [MP4](2026-11-14_131043_g4-simple-run-13-success.rgb.mp4) | [CSV](2026-11-14_131043_g4-simple-run-13-success.rgb.frames.csv) | 1053 | 1.21 |
| `records/trials/2026-11-14-g4-simple/run-14` | 성공(매트에서 집기부터) | [MP4](2026-11-14_131152_g4-simple-run-14-success.rgb.mp4) | [CSV](2026-11-14_131152_g4-simple-run-14-success.rgb.frames.csv) | 1238 | 1.38 |
| `records/trials/2026-11-14-g4-simple/run-15` | 성공(매트에서 집기부터) | [MP4](2026-11-14_131625_g4-simple-run-15-success.rgb.mp4) | [CSV](2026-11-14_131625_g4-simple-run-15-success.rgb.frames.csv) | 1279 | 1.40 |

판정 근거와 수치는 [현장 기록](../../sessions/OFF-2-2026-11-14.md)과 각 시도의 `run-NN.analysis.json`·`run-NN.sheet.jpg`에 있다. 이 영상은 간이 스크립트의 결과이며 G4 합격 기준(저속 30회)의 측정 자료가 아니다.
