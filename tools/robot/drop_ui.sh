#!/usr/bin/env bash
# 움직이는 뚜껑 투입 버튼 창을 띄운다(실제 로봇 지령, 입회 필수). 먼저 떠 있어야 하는 것: 로봇 드라이버(dsr_bringup2, mode:=real),
# 카메라, vision_node. 같은 ROS_DOMAIN_ID를 쓴다.
#   tools/robot/drop_ui.sh
# 바꿀 수 있는 환경 변수: DSR_WS(dsr_msgs2가 있는 워크스페이스, 기본 ~/ros2_ws), ROS_DOMAIN_ID(기본 40)
set -e
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
set +u
source /opt/ros/jazzy/setup.bash
source "${DSR_WS:-$HOME/ros2_ws}/install/setup.bash"
source "$REPO/install/setup.bash"
set -e
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-40}"
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}"
export FASTRTPS_DEFAULT_PROFILES_FILE="$REPO/src/drop_bringup/config/fastdds_large_shm.xml"
export PYTHONPATH="$REPO:$PYTHONPATH"
cd "$REPO"
exec python3 tools/robot/drop_ui.py "$@"
