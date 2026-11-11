#!/bin/bash
set -eo pipefail
export ROS_DOMAIN_ID=169 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
mkdir -p "$HOME/ctb-codex9"
cd "$HOME/ctb-codex9"
tar xzf /mnt/c/tmp/codex-w9/source.tar.gz
git init -q
git -c user.name=LocalTest -c user.email=local@localhost add .
git -c user.name=LocalTest -c user.email=local@localhost commit -qm snapshot
source /opt/ros/jazzy/setup.bash
if ! command -v colcon >/dev/null; then
 python3 -m venv --system-site-packages .venv
 .venv/bin/pip -q install colcon-common-extensions
 export PATH="$PWD/.venv/bin:$PATH"
fi
colcon build --symlink-install
source install/setup.bash
export PYTHONPATH="$PWD/tools/spec:${PYTHONPATH:-}"
python3 -m pytest -q tools/spec
python3 -m pytest -q tools/eval
python3 -m pytest -q src/orchestrator/test
python3 -m pytest -q src/drop_mocks/test --ignore=src/drop_mocks/test/contract
python3 -m pytest -q src/robot_control/test
python3 -m pytest -q src/drop_mocks/test/contract
python3 -m pytest -q src/drop_bringup/test
python3 tools/spec/trace.py --gate M0 --check
for i in $(seq 1 8); do
 echo "NOMINAL_RUN=$i"
 python3 -m pytest -q src/drop_bringup/test/test_i0_integration.py -k nominal
done
colcon build --build-base build-copy --install-base install-copy
# Use a clean subprocess so copy installation does not inherit symlink paths.
env -u PYTHONPATH -u AMENT_PREFIX_PATH -u COLCON_PREFIX_PATH -u CMAKE_PREFIX_PATH bash /mnt/c/tmp/codex-w9/copy.sh
