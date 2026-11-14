#!/usr/bin/env bash
# 현장 PC 소프트웨어 버전·Doosan 인터페이스 목록 수집(조회만, 로봇을 움직이지 않는다).
#
#   DOOSAN_WS=~/doosan_ws tools/bench/collect_versions.sh
#
# 드라이버(model:=e0509 mode:=real)가 떠 있으면 서비스·토픽 목록과 그리퍼 관련 서비스도 기록한다.
# 결과: records/bench/versions-<날짜>-<시각>.txt  (site-survey의 V01~V04 항목 근거)
set -o pipefail   # set -u 금지: ROS setup.bash

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
DOOSAN_WS="${DOOSAN_WS:-}"
DSR_NS="${DSR_NS:-/dsr01}"
OUT="$REPO/records/bench/versions-$(date +%F-%H%M%S).txt"
mkdir -p "$(dirname "$OUT")"
exec > >(tee "$OUT") 2>&1
sec() { printf '\n===== %s =====\n' "$*"; }

command -v ros2 >/dev/null 2>&1 || source /opt/ros/jazzy/setup.bash

if [[ -z "$DOOSAN_WS" ]]; then
  # dsr_msgs2 소스가 있는 워크스페이스를 찾는다.
  DOOSAN_WS="$(find "$HOME" -maxdepth 4 -type d -name dsr_msgs2 -path '*/src/*' 2>/dev/null | head -1 | sed 's#/src/.*##')"
fi

sec "시스템"
date -Is; hostname; uname -r; lsb_release -ds 2>/dev/null
echo "ROS_DISTRO=$ROS_DISTRO  RMW=${RMW_IMPLEMENTATION:-기본}  ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}"
python3 --version

sec "doosan-robot2 워크스페이스: ${DOOSAN_WS:-찾지 못함}"
if [[ -n "$DOOSAN_WS" ]]; then
  FOUND=0
  while IFS= read -r g; do
    d="$(dirname "$g")"; FOUND=1
    printf '%s  %s  %s  dirty=%s\n' "${d#"$DOOSAN_WS"/}" \
      "$(git -C "$d" rev-parse HEAD)" \
      "$(git -C "$d" rev-parse --abbrev-ref HEAD)" \
      "$(git -C "$d" status --porcelain | wc -l)"
    git -C "$d" remote -v | head -1
  done < <(find "$DOOSAN_WS" -maxdepth 4 -name .git -not -path '*/build/*' -not -path '*/install/*' 2>/dev/null | sort)
  (( FOUND )) || echo "git 저장소 없음(소스 사본). 출처·버전을 따로 확인한다"
  if [[ -f "$DOOSAN_WS/install/setup.bash" ]]; then
    source "$DOOSAN_WS/install/setup.bash"
  else
    echo "경고: $DOOSAN_WS/install/setup.bash 없음(빌드 안 됨 또는 colcon 외 방식). dsr 패키지는 not found로 나온다"
  fi
  echo "--- DRFL 라이브러리"
  find "$DOOSAN_WS" -iname '*DRFL*' \( -name '*.so*' -o -name '*.a' -o -name '*.h' \) 2>/dev/null | head -10
  grep -rhoE 'DRCF_VERSION[^;]*|DRFL_VERSION[^;]*' "$DOOSAN_WS/src" 2>/dev/null | sort -u | head -5
fi

sec "ROS 패키지"
for p in dsr_bringup2 dsr_controller2 dsr_msgs2 dsr_hardware2 realsense2_camera realsense2_camera_msgs dsr_gripper dsr_gripper_interfaces; do
  printf '%-26s %s\n' "$p" "$(ros2 pkg prefix "$p" 2>/dev/null || echo 'not found')"
done
dpkg -l 2>/dev/null | awk '/realsense|ros-jazzy-rclpy |ros-jazzy-rosbag2 /{print $2, $3}'

sec "프로젝트 패키지 이름 충돌(underlay만)"
( source /opt/ros/jazzy/setup.bash
  for p in drop_interfaces vision tracking intercept_planner robot_control orchestrator calibration drop_mocks drop_bringup drop_sim; do
    printf '%-18s %s\n' "$p" "$(ros2 pkg prefix "$p" 2>/dev/null || echo 'not found')"
  done )

sec "스트림·이동·그리퍼 인터페이스 정의"
for i in dsr_msgs2/msg/ServolStream dsr_msgs2/msg/SpeedlStream dsr_msgs2/msg/ServolRtStream dsr_msgs2/msg/SpeedlRtStream \
         dsr_msgs2/srv/MoveLine dsr_msgs2/srv/DrlStart dsr_msgs2/srv/SetOutputRegisterInt dsr_msgs2/srv/GetOutputRegisterInt \
         dsr_gripper_interfaces/srv/GripperCmd; do
  echo "--- $i"; ros2 interface show "$i" 2>&1 | grep -v '^\s*#' | sed '/^\s*$/d'
done

sec "실행 중 그래프(드라이버가 떠 있을 때만 의미 있음)"
ros2 node list 2>/dev/null
echo "--- 서비스(${DSR_NS})"
ros2 service list -t 2>/dev/null | grep -E "^${DSR_NS}/" | grep -E "drl|register|motion/move_line|realtime|system/get|aux_control/get|gripper"
echo "--- 스트림 토픽"
ros2 topic list -t 2>/dev/null | grep -E "stream|joint_states|robot_state|error"

sec "로봇 상태 조회(서비스 있으면, 읽기 전용)"
for s in system/get_robot_state system/get_robot_mode system/get_current_tool system/get_current_tcp; do
  if ros2 service list 2>/dev/null | grep -qx "$DSR_NS/$s"; then
    t="$(ros2 service type "$DSR_NS/$s")"
    echo "--- $s ($t)"; timeout 5 ros2 service call "$DSR_NS/$s" "$t" "{}" 2>&1 | tail -3
  fi
done

sec "그리퍼 노드·DRL 코드 사본"
GRIP_SRC="$(ros2 pkg prefix dsr_gripper 2>/dev/null)"
if [[ -n "$GRIP_SRC" ]]; then
  f="$(find "$GRIP_SRC" "$DOOSAN_WS" -name 'gripper_service*.py' 2>/dev/null | head -3)"
  echo "$f"
  for x in $f; do sha256sum "$x"; grep -nE "REG_|clamp|750|1150|min\(|max\(|baudrate|slave_id|DRL_START|SET_REG" "$x" | head -25; done
fi

echo; echo "결과 파일: $OUT"
