#!/usr/bin/env bash
# D455 연결·프로파일·시각 확인. 로봇을 움직이지 않는다.
#
#   tools/bench/check_d455.sh            # RealSense 노드를 띄워서 확인 후 그대로 둔다(Ctrl+C로 종료)
#   tools/bench/check_d455.sh --no-launch  # 이미 떠 있는 노드를 확인만 한다
#
# 확인 항목:
#   1) 장치·펌웨어·지원 프로파일(rs-enumerate-devices 있으면)
#   2) 컬러 1280x800 / 30 fps, CameraInfo 크기·왜곡 모델·내부 행렬
#   3) 실제 수신 주기(ros2 topic hz)
#   4) 노출·화이트밸런스·global time 관련 파라미터 값(ros2 param dump)
#   5) header.stamp − 수신 시각 분포(시계 영역·지연 대략값, 100 프레임)
# 결과: records/bench/d455-check-<날짜>-<시각>.txt
set -o pipefail   # set -u 금지: ROS setup.bash

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
CAMERA_NS="${CAMERA_NS:-/camera/camera}"
NODE="${CAMERA_NODE:-$CAMERA_NS}"
LAUNCH=1
[[ "${1:-}" == "--no-launch" ]] && LAUNCH=0

command -v ros2 >/dev/null 2>&1 || source /opt/ros/jazzy/setup.bash
OUT="$REPO/records/bench/d455-check-$(date +%F-%H%M%S).txt"
mkdir -p "$(dirname "$OUT")"
exec > >(tee "$OUT") 2>&1

sec() { printf '\n===== %s =====\n' "$*"; }

sec "환경"
date -Is; hostname; echo "ROS_DISTRO=$ROS_DISTRO CAMERA_NS=$CAMERA_NS"
ros2 pkg prefix realsense2_camera 2>/dev/null && \
  (dpkg -l 2>/dev/null | awk '/realsense/{print $2, $3}')

sec "장치"
if command -v rs-enumerate-devices >/dev/null 2>&1; then
  rs-enumerate-devices -s
  echo "--- 컬러 1280x800 프로파일"
  rs-enumerate-devices 2>/dev/null | grep -E "Color|RGB" | grep -E "1280x800" | head -20
else
  echo "rs-enumerate-devices 없음(librealsense2-utils). 펌웨어는 노드 로그에서 확인"
fi

LPID=""
if (( LAUNCH )); then
  sec "RealSense 노드 기동"
  # 버전에 따라 rgb_camera.color_profile(4.54 이후) 또는 rgb_camera.profile을 쓴다. 둘 다 넘긴다.
  ros2 launch realsense2_camera rs_launch.py \
    rgb_camera.color_profile:=1280,800,30 rgb_camera.profile:=1280,800,30 \
    enable_color:=true enable_depth:=true \
    depth_module.depth_profile:=1280,720,30 depth_module.profile:=1280,720,30 \
    align_depth.enable:=false pointcloud.enable:=false \
    >/tmp/d455_launch.log 2>&1 &
  LPID=$!
  echo "launch pid $LPID, 로그 /tmp/d455_launch.log"
  for _ in $(seq 1 30); do
    ros2 topic list 2>/dev/null | grep -qx "$CAMERA_NS/color/image_raw" && break
    sleep 1
  done
  grep -iE "firmware|device|serial|warn|error" /tmp/d455_launch.log | head -30
fi

sec "토픽"
ros2 topic list | grep -i camera || echo "카메라 토픽 없음"
ros2 topic list | grep -qx "$CAMERA_NS/color/image_raw" || { echo "실패: $CAMERA_NS/color/image_raw 없음"; exit 1; }

sec "CameraInfo"
timeout 10 ros2 topic echo --once "$CAMERA_NS/color/camera_info" | grep -E "^(height|width|distortion_model):"
timeout 10 ros2 topic echo --once "$CAMERA_NS/color/camera_info" | sed -n '/^d:/,/^r:/p'

sec "수신 주기(12 s)"
timeout 12 ros2 topic hz "$CAMERA_NS/color/image_raw" 2>&1 | tail -4
timeout 8 ros2 topic hz "$CAMERA_NS/color/camera_info" 2>&1 | tail -2

sec "파라미터(노출·화이트밸런스·시간·프로파일)"
ros2 param dump "$NODE" >/tmp/d455_params.yaml 2>/dev/null || ros2 param dump "$(ros2 node list | grep -m1 camera)" >/tmp/d455_params.yaml
grep -iE "exposure|white_balance|global_time|profile|auto_exposure|gain|frames_queue|enable_" /tmp/d455_params.yaml
cp /tmp/d455_params.yaml "${OUT%.txt}-params.yaml"
echo "전체 파라미터: ${OUT%.txt}-params.yaml"

sec "header.stamp − 수신 시각(100 프레임)"
python3 - "$CAMERA_NS/color/image_raw" <<'PY'
import sys, time, statistics
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
topic = sys.argv[1]
rclpy.init()
n = Node("d455_stamp_check")
d, gaps, last = [], [], [None]
def cb(m):
    now = n.get_clock().now().nanoseconds * 1e-9
    st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
    d.append(now - st)
    if last[0] is not None:
        gaps.append(st - last[0])
    last[0] = st
n.create_subscription(Image, topic, cb, qos_profile_sensor_data)
t_end = time.time() + 15
while len(d) < 100 and time.time() < t_end:
    rclpy.spin_once(n, timeout_sec=0.1)
if not d:
    print("수신 없음"); sys.exit(1)
q = statistics.quantiles(d, n=100) if len(d) > 2 else d
print(f"N={len(d)}  (수신−stamp) p50={statistics.median(d)*1e3:.1f} ms  p95={q[94]*1e3 if len(q)>94 else max(d)*1e3:.1f} ms  min={min(d)*1e3:.1f} ms  max={max(d)*1e3:.1f} ms")
if gaps:
    print(f"stamp 간격 p50={statistics.median(gaps)*1e3:.2f} ms  max={max(gaps)*1e3:.2f} ms  역행={sum(g<=0 for g in gaps)}회")
print("판정 참고: 값이 수 ms~수십 ms면 같은 시계 영역(global time 또는 시스템 시각). 음수이거나 초 단위로 크면 시계 영역이 다르다.")
n.destroy_node(); rclpy.shutdown()
PY

sec "끝"
echo "결과 파일: $OUT"
if [[ -n "$LPID" ]]; then
  echo "RealSense 노드는 계속 실행 중(pid $LPID). 노출·화이트밸런스는 자동 설정 그대로 둔다."
  echo "종료: kill -INT $LPID"
fi
