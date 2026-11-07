# 2026-11-07 현장 영상

원본 rosbag 8개(80.08 GB)의 RGB 8개·깊이 미리보기 6개를 H.264 MP4로 압축했다. 영상 합계 302.35 MB. 전체 디코딩, 원본 메시지 수와 영상 프레임 수 일치, 첫·마지막 프레임 읽기를 확인했다.

- 원본 경로와 파일 목록 해시: [bags.csv](../../bags.csv). 원본 rosbag과 16-bit 깊이는 로컬에 보존한다.
- 아래 영상은 고정 **30 fps 재생용 복사본**이다. 메시지 사이 실제 시간 간격·누락·retained 선두 샘플은 영상 재생 시간으로 재현하지 않는다.
- 각 `.frames.csv`는 프레임 번호, 재생 시각, 원래 header stamp(ns), bag 수신 시각(ns), 수신 지연(s)을 보존한다. 계측·시간 분석은 이 CSV와 원본을 사용한다.
- 깊이 영상은 uint16 값 0..2000을 회색조로 표시한 손실 압축 미리보기다. 깊이 수치 복원·정밀 보정에 사용하지 않는다. 범위를 넘는 값은 표시상 포화된다.
- FFmpeg/libx264, CRF 26, preset medium, yuv420p. 버전·전체 명령·SHA-256·프레임 수는 [manifest.json](manifest.json), 재현 소스는 [export_rgbd.py](export_rgbd.py).
- 단일 현재 조명 자료다. 수치 속도·로봇 허용 최고속도·운용 보정 합격을 이 영상으로 확정하지 않는다. 품질 제한은 [속도 조건 목록](../../bench/moving-conditions-2026-11-07.csv)과 [현장 기록](../../sessions/OFF-1-2026-11-07.md)을 따른다.

| 원본 bag / 종류 | 영상 | 원래 프레임 시각 | 프레임 | 크기(MB) |
|---|---|---|---:|---:|
| 2026-11-07_195156_off1-camera-replay-smoke / rgb | [MP4](2026-11-07_195156_off1-camera-replay-smoke.rgb.mp4) | [CSV](2026-11-07_195156_off1-camera-replay-smoke.rgb.frames.csv) | 286 | 0.30 |
| 2026-11-07_195455_off2-stream-vision-recording-load / rgb | [MP4](2026-11-07_195455_off2-stream-vision-recording-load.rgb.mp4) | [CSV](2026-11-07_195455_off2-stream-vision-recording-load.rgb.frames.csv) | 8832 | 8.14 |
| 2026-11-07_200054_off2-loaded-stale-rgbd / rgb | [MP4](2026-11-07_200054_off2-loaded-stale-rgbd.rgb.mp4) | [CSV](2026-11-07_200054_off2-loaded-stale-rgbd.rgb.frames.csv) | 1295 | 0.97 |
| 2026-11-07_200054_off2-loaded-stale-rgbd / depth-preview | [MP4](2026-11-07_200054_off2-loaded-stale-rgbd.depth-preview.mp4) | [CSV](2026-11-07_200054_off2-loaded-stale-rgbd.depth-preview.frames.csv) | 1294 | 34.82 |
| 2026-11-07_200632_off1-moving-analog-stable-current / rgb | [MP4](2026-11-07_200632_off1-moving-analog-stable-current.rgb.mp4) | [CSV](2026-11-07_200632_off1-moving-analog-stable-current.rgb.frames.csv) | 1735 | 1.59 |
| 2026-11-07_200632_off1-moving-analog-stable-current / depth-preview | [MP4](2026-11-07_200632_off1-moving-analog-stable-current.depth-preview.mp4) | [CSV](2026-11-07_200632_off1-moving-analog-stable-current.depth-preview.frames.csv) | 1736 | 47.03 |
| 2026-11-07_201012_off1-moving-analog-fast-1 / depth-preview | [MP4](2026-11-07_201012_off1-moving-analog-fast-1.depth-preview.mp4) | [CSV](2026-11-07_201012_off1-moving-analog-fast-1.depth-preview.frames.csv) | 1776 | 47.06 |
| 2026-11-07_201012_off1-moving-analog-fast-1 / rgb | [MP4](2026-11-07_201012_off1-moving-analog-fast-1.rgb.mp4) | [CSV](2026-11-07_201012_off1-moving-analog-fast-1.rgb.frames.csv) | 1775 | 1.65 |
| 2026-11-07_201202_off1-moving-analog-fast-1-long / rgb | [MP4](2026-11-07_201202_off1-moving-analog-fast-1-long.rgb.mp4) | [CSV](2026-11-07_201202_off1-moving-analog-fast-1-long.rgb.frames.csv) | 1933 | 1.46 |
| 2026-11-07_201202_off1-moving-analog-fast-1-long / depth-preview | [MP4](2026-11-07_201202_off1-moving-analog-fast-1-long.depth-preview.mp4) | [CSV](2026-11-07_201202_off1-moving-analog-fast-1-long.depth-preview.frames.csv) | 1933 | 50.67 |
| 2026-11-07_201739_off1-moving-analog-fast-2 / depth-preview | [MP4](2026-11-07_201739_off1-moving-analog-fast-2.depth-preview.mp4) | [CSV](2026-11-07_201739_off1-moving-analog-fast-2.depth-preview.frames.csv) | 1930 | 53.40 |
| 2026-11-07_201739_off1-moving-analog-fast-2 / rgb | [MP4](2026-11-07_201739_off1-moving-analog-fast-2.rgb.mp4) | [CSV](2026-11-07_201739_off1-moving-analog-fast-2.rgb.frames.csv) | 1930 | 1.30 |
| 2026-11-07_202213_off1-moving-analog-maximum / rgb | [MP4](2026-11-07_202213_off1-moving-analog-maximum.rgb.mp4) | [CSV](2026-11-07_202213_off1-moving-analog-maximum.rgb.frames.csv) | 1918 | 1.70 |
| 2026-11-07_202213_off1-moving-analog-maximum / depth-preview | [MP4](2026-11-07_202213_off1-moving-analog-maximum.depth-preview.mp4) | [CSV](2026-11-07_202213_off1-moving-analog-maximum.depth-preview.frames.csv) | 1918 | 52.25 |
