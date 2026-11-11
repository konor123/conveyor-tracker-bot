#!/bin/bash
cd ~/ctb-codex
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_DOMAIN_ID=183 PYTHONPATH=$PWD/tools/spec:${PYTHONPATH:-}
mkdir -p "$HOME/ctb-codex/nominal-results"
status=0
for n in $(seq 1 8); do
 python3 -m pytest -q src/drop_bringup/test/test_i0_integration.py::test_i0_nominal_20_cycles_and_records --basetemp="$HOME/ctb-codex/nominal-results/$n" > /mnt/c/tmp/codex-w7/final-nom-$n.log 2>&1
 code=$?
 echo "nominal $n exit=$code"
 [ "$code" -eq 0 ] || status=1
done
exit $status
