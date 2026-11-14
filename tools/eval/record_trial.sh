#!/usr/bin/env bash
# rosbag2 녹화 + 무결성 기록.
#
#   tools/eval/record_trial.sh -n <이름> -p <목적> [-d <초>] [-b <다이얼>] [-l <조명>] [-t "<토픽 ...>"] [-m "<메모>"]
#
#   -n  bag 이름(예: off1-moving-dial3-lightA). 날짜·시각 접두가 자동으로 붙는다
#   -p  목적: moving | static | plane_ref | occlusion | misc
#   -d  녹화 시간 [s]. 생략하면 Ctrl+C로 끝낸다
#   -b  벨트 다이얼 설정(정지면 0)
#   -l  조명 조건(A, B ...)
#   -t  녹화 토픽(공백 구분). 기본값은 RealSense 컬러 영상·카메라 정보(필수)
#       + /tf_static·/tf·컬러 metadata·extrinsics(발행 중이면 함께 녹화)
#   -m  메모
#
# 환경 변수:
#   BAG_ROOT      bag 저장 위치(저장소 밖). 기본 ~/drop_bags
#   CAMERA_NS     RealSense 토픽 접두. 기본 /camera/camera
#   CAMERA_MOUNT  카메라 거치 식별자(사진 파일명 등). 기본 unset
#
# 결과: $BAG_ROOT/<날짜>/<이름>/ 에 bag, $BAG_ROOT/bags.csv 에 한 줄(경로·SHA-256·크기).
set -o pipefail   # set -u 금지: ROS setup.bash가 미정의 변수를 참조한다.

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
BAG_ROOT="${BAG_ROOT:-$HOME/drop_bags}"
CAMERA_NS="${CAMERA_NS:-/camera/camera}"
INDEX="$BAG_ROOT/bags.csv"

NAME=""; PURPOSE=""; DURATION=""; DIAL=""; LIGHT=""; TOPICS=""; NOTES=""
while getopts "n:p:d:b:l:t:m:h" opt; do
  case "$opt" in
    n) NAME="$OPTARG" ;;
    p) PURPOSE="$OPTARG" ;;
    d) DURATION="$OPTARG" ;;
    b) DIAL="$OPTARG" ;;
    l) LIGHT="$OPTARG" ;;
    t) TOPICS="$OPTARG" ;;
    m) NOTES="$OPTARG" ;;
    h|*) sed -n '2,22p' "${BASH_SOURCE[0]}"; exit 2 ;;
  esac
done

die() { echo "오류: $*" >&2; exit 1; }
[[ -n "$NAME" ]] || die "-n <이름> 필요"
[[ "$NAME" =~ ^[A-Za-z0-9._-]+$ ]] || die "이름에는 영문·숫자·._- 만 쓴다: $NAME"
case "$PURPOSE" in moving|static|plane_ref|occlusion|misc) ;; *) die "-p 는 moving|static|plane_ref|occlusion|misc" ;; esac
if [[ -n "$DURATION" ]]; then [[ "$DURATION" =~ ^[0-9]+$ ]] || die "-d 는 정수 초"; fi

OPTIONAL=""
if [[ -z "$TOPICS" ]]; then
  TOPICS="$CAMERA_NS/color/image_raw $CAMERA_NS/color/camera_info"
  OPTIONAL="/tf_static /tf $CAMERA_NS/color/metadata $CAMERA_NS/extrinsics/depth_to_color"
fi

if ! command -v ros2 >/dev/null 2>&1; then
  [[ -f /opt/ros/jazzy/setup.bash ]] || die "ROS 2 Jazzy가 없다"
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
fi

# 토픽이 실제로 발행 중인지 먼저 확인(오타·네임스페이스 차이로 빈 bag을 만드는 일을 막는다).
# daemon이 막 기동했거나 발견(discovery)이 덜 끝나면 첫 목록이 비어 있을 수 있어 몇 번 다시 본다.
MISSING=()
for try in 1 2 3 4 5; do
  LIVE="$(timeout 10 ros2 topic list 2>/dev/null)"
  MISSING=()
  for t in $TOPICS; do
    grep -qx -- "$t" <<<"$LIVE" || MISSING+=("$t")
  done
  (( ${#MISSING[@]} == 0 )) && break
  sleep 2
done
for t in $OPTIONAL; do
  grep -qx -- "$t" <<<"$LIVE" && TOPICS="$TOPICS $t"
done
if (( ${#MISSING[@]} )); then
  echo "발행되지 않는 토픽: ${MISSING[*]}" >&2
  echo "현재 카메라 관련 토픽:" >&2; grep -i camera <<<"$LIVE" >&2
  die "토픽을 확인한 뒤 다시 실행(CAMERA_NS 또는 -t)"
fi

DAY="$(date +%F)"
STAMP="$(date +%H%M%S)"
OUT="$BAG_ROOT/$DAY/${DAY}_${STAMP}_${NAME}"
mkdir -p "$BAG_ROOT/$DAY" || die "저장 위치 생성 실패: $BAG_ROOT/$DAY"
[[ -e "$OUT" ]] && die "이미 있음: $OUT"

FREE_KB="$(df -Pk "$BAG_ROOT" | awk 'NR==2{print $4}')"
if [[ -n "$FREE_KB" ]] && (( FREE_KB < 50*1024*1024 )); then
  echo "경고: 남은 디스크 $((FREE_KB/1024/1024)) GB (1280x800 raw 30 fps ≈ 92 MB/s, 60 s ≈ 5.5 GB)" >&2
fi

echo "녹화: $OUT"
echo "토픽: $TOPICS"
[[ -n "$DURATION" ]] && echo "시간: ${DURATION} s" || echo "Ctrl+C로 종료"
T0="$(date +%s.%N)"
# shellcheck disable=SC2086
if [[ -n "$DURATION" ]]; then
  timeout -s INT -k 10 "$DURATION" ros2 bag record -o "$OUT" $TOPICS
else
  trap ':' INT
  ros2 bag record -o "$OUT" $TOPICS
  trap - INT
fi
RC=$?
T1="$(date +%s.%N)"
# timeout이 SIGINT로 끝내면 124, 사용자 Ctrl+C는 130 등. bag 디렉터리와 metadata로 성공을 판단한다.
[[ -f "$OUT/metadata.yaml" ]] || die "metadata.yaml 없음(종료 코드 $RC). bag이 정상 종료되지 않았다: $OUT"

SHA="$(cd "$OUT" && find . -type f -print0 | sort -z | xargs -0 sha256sum | sha256sum | awk '{print $1}')"
SIZE="$(du -sb "$OUT" | awk '{print $1}')"
DUR="$(awk -v a="$T0" -v b="$T1" 'BEGIN{printf "%.1f", b-a}')"


echo
INFO="$(ros2 bag info "$OUT" 2>/dev/null)"
sed -n '1,30p' <<<"$INFO"
# 영상 프레임 수 검사: 녹화기가 디스크·USB 속도를 못 따라가면 영상만 빠진다.
BAG_SEC="$(awk '/^Duration:/{gsub(/s/,"",$2); print $2}' <<<"$INFO")"
IMG_N="$(grep -F "Topic: $CAMERA_NS/color/image_raw " <<<"$INFO" | sed -n 's/.*Count: \([0-9]*\).*/\1/p')"
FPS_NOTE=""
if [[ -n "$BAG_SEC" && -n "$IMG_N" ]]; then
  FPS="$(awk -v n="$IMG_N" -v s="$BAG_SEC" 'BEGIN{ if (s>0) printf "%.1f", n/s; else print 0 }')"
  echo "영상 프레임: $IMG_N / ${BAG_SEC} s = ${FPS} fps (기대 ${EXPECT_FPS:-30})"
  if awk -v f="$FPS" -v e="${EXPECT_FPS:-30}" 'BEGIN{exit !(f < 0.95*e)}'; then
    echo "경고: 영상 프레임 손실. 디스크(외장 SSD 직결)·USB 대역·CPU 부하를 확인하고 다시 녹화한다" >&2
    FPS_NOTE="fps=${FPS} 프레임 손실"
  fi
fi
echo
mkdir -p "$BAG_ROOT"
[[ -f "$INDEX" ]] || echo "date,bag_name,purpose,belt_dial,lighting,duration_s,path,sha256,size_bytes,camera_mount,notes" >"$INDEX"
csv() { local s="${1//\"/\"\"}"; printf '"%s"' "$s"; }
{
  printf '%s,%s,%s,%s,%s,%s,' "$DAY" "$(basename "$OUT")" "$PURPOSE" "$DIAL" "$LIGHT" "$DUR"
  printf '%s,%s,%s,' "$(csv "$OUT")" "$SHA" "$SIZE"
  printf '%s,%s\n' "$(csv "${CAMERA_MOUNT:-}")" "$(csv "${NOTES}${FPS_NOTE:+ [$FPS_NOTE]}")"
} >>"$INDEX"
echo "기록: $INDEX"
echo "SHA-256(파일 목록 해시): $SHA"
