#!/bin/bash
set -e
cd ~/ctb-codex
source /opt/ros/jazzy/setup.bash
export PATH=$HOME/ctb-work/colcon-venv/bin:$PATH ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_DOMAIN_ID=183
colcon build --symlink-install > /mnt/c/tmp/codex-w7/final-build.log 2>&1
source install/setup.bash
export PYTHONPATH=$PWD/tools/spec:${PYTHONPATH:-}
status=0
run(){ name=$1; shift; "$@" > /mnt/c/tmp/codex-w7/final-$name.log 2>&1; code=$?; echo "$name=$code"; [ "$code" -eq 0 ] || status=1; }
set +e
run spec python3 -m pytest -q tools/spec
run eval python3 -m pytest -q tools/eval
run orch python3 -m pytest -q src/orchestrator/test
run mocks python3 -m pytest -q src/drop_mocks/test --ignore=src/drop_mocks/test/contract
run robot python3 -m pytest -q src/robot_control/test
run contract python3 -m pytest -q src/drop_mocks/test/contract
run bringup python3 -m pytest -q src/drop_bringup/test
run trace python3 tools/spec/trace.py --gate M0 --check
exit $status
