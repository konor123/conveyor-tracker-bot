#!/bin/bash
set -e
source /opt/ros/jazzy/setup.bash
source "$HOME/ctb-codex9/install-copy/setup.bash"
cd /tmp
python3 - <<'PY'
from pathlib import Path
from orchestrator.record_logger_node import _find_schema_path, validate_trial_record, build_trial_record
import orchestrator.record_logger_node as logger
p = _find_schema_path()
print('COPY_MODULE', logger.__file__)
print('COPY_SCHEMA', p)
assert '/install-copy/' in logger.__file__
assert '/install-copy/' in p
rec = build_trial_record({'trial_id': 'copy-test'}, env='mock', gate='M0', git_sha='test', assumed_params=['test'])
errors = validate_trial_record(rec)
print('COPY_ERRORS', errors)
assert not errors
print('COPY_INSTALL_OK')
PY
