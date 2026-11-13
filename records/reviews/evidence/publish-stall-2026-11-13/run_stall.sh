#!/bin/bash
# 발행 정지 미세 재현. 사용: run_stall.sh <variant> <scenario> <rep>
#  variant: default | async | udp     scenario: spaced | burst

V=$1; SC=$2; REP=$3
D=${STALL_DIR:-$PWD}; cd $D   # pub_probe.py·joiner.py가 있는 폴더
source /opt/ros/jazzy/setup.bash
unset RMW_FASTRTPS_PUBLICATION_MODE FASTDDS_BUILTIN_TRANSPORTS ROS_AUTOMATIC_DISCOVERY_RANGE
export ROS_DOMAIN_ID=$((150 + REP))
case $V in
  async) export RMW_FASTRTPS_PUBLICATION_MODE=ASYNCHRONOUS ;;
  udp) export FASTDDS_BUILTIN_TRANSPORTS=UDPv4 ;;
esac
tag=$V-$SC-$REP
python3 pub_probe.py 12 res-$tag.json > pub-$tag.out 2>&1 &
PUB=$!
# 발행자가 준비될 때까지
for i in $(seq 1 50); do grep -q PUB_READY pub-$tag.out 2>/dev/null && break; sleep 0.1; done
T0=$(date +%s.%N)
: > join-$tag.out
if [ "$SC" = spaced ]; then
  # 시각(발행 준비 기준)과 종류: 1 s sub, 3 s other, 5 s sub, 7 s sub×3, 9 s other×3
  plan="1:sub:a 3:other:b 5:sub:c 7:sub:d1 7:sub:d2 7:sub:d3 9:other:e1 9:other:e2 9:other:e3"
else
  plan="0:sub:a 0:sub:b 0:sub:c 0:other:d 0:other:e 0:other:f"
fi
for item in $plan; do
  at=${item%%:*}; rest=${item#*:}; kind=${rest%%:*}; name=${rest#*:}
  ( now=$(date +%s.%N); w=$(echo "$T0 + $at - $now" | bc -l); [ "$(echo "$w > 0" | bc -l)" = 1 ] && sleep $w
    python3 joiner.py $kind 4 $name >> join-$tag.out 2>&1 ) &
done
wait $PUB
wait
python3 - "$tag" <<'PY'
import json, sys
tag = sys.argv[1]
r = json.load(open(f'res-{tag}.json'))
joins = []
for line in open(f'join-{tag}.out'):
    p = line.split()
    if p and p[0] == 'JOIN':
        joins.append((p[1], p[2], round(float(p[3]) - r['t0'], 2)))
print(f"{tag}: n={r['n']} worst={r['worst_s']*1000:.1f}ms slow(>=20ms)={r['slow']} joins={sorted(joins, key=lambda j: j[2])}")
PY
