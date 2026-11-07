#!/bin/bash
set -e
bench_dir="$(cd -- "$(dirname -- "$(readlink -f -- "$0")")" && pwd)"
source /opt/ros/jazzy/setup.bash
source "$bench_dir/../../install/setup.bash"
exec python3 "$bench_dir/gripper_once.py" "$@"
