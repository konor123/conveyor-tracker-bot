#!/bin/bash
# burst 기본 설정을 py-spy(부모로 띄움, native·idle 포함)로 기록한다. 임시 venv는 이 폴더 안에만 둔다.
D=${STALL_DIR:-$PWD}; cd $D   # pub_probe.py·joiner.py가 있는 폴더
if [ ! -x $D/pyspy-venv/bin/py-spy ]; then
  python3 -m venv $D/pyspy-venv && $D/pyspy-venv/bin/pip -q install py-spy > $D/pyspy-install.log 2>&1
fi
$D/pyspy-venv/bin/py-spy --version
source /opt/ros/jazzy/setup.bash
unset RMW_FASTRTPS_PUBLICATION_MODE FASTDDS_BUILTIN_TRANSPORTS ROS_AUTOMATIC_DISCOVERY_RANGE
for rep in 1 2 3 4 5; do
  export ROS_DOMAIN_ID=$((160 + rep)); tag=spy-$rep
  $D/pyspy-venv/bin/py-spy record --native --idle --threads --rate 200 --format speedscope -o spy-$rep.json \
      -- python3 pub_probe.py 8 res-$tag.json > pub-$tag.out 2>&1 &
  SPY=$!
  for i in $(seq 1 80); do grep -q PUB_READY pub-$tag.out 2>/dev/null && break; sleep 0.1; done
  for n in a b c d e f; do (python3 joiner.py $([ $n \< d ] && echo sub || echo other) 4 $n > /dev/null 2>&1 &) ; done
  wait $SPY
  python3 -c "import json;r=json.load(open('res-$tag.json'));print('$tag', 'worst_ms=%.1f'%(r['worst_s']*1000), r['slow'])"
done
