#!/bin/bash
cd ~/ctb-codex
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_DOMAIN_ID=183 PYTHONPATH=$PWD/tools/spec:${PYTHONPATH:-}
python3 - <<'PY'
import tarfile
with tarfile.open('/mnt/c/tmp/codex-w7/src.tar.gz') as t:
 for name,out in [('src/drop_mocks/test/test_mock_nodes.py','src/drop_mocks/test/test_w7_old_clock.py'),('src/drop_bringup/test/test_i0_integration.py','src/drop_bringup/test/test_w7_old_hb.py')]:
  s=t.extractfile(name).read().decode()
  if 'mock_nodes' in name:
   s+='''\n@pytest.fixture(autouse=True)
def w7_host_rollback(monkeypatch):
    start = time.monotonic()
    original_now = Harness.now
    monkeypatch.setattr(Harness, 'now', lambda self: original_now(self) - (0.8 if time.monotonic()-start >= 4.0 else 0.0))
    for cls in (MockWorldNode, MockRobotControllerNode):
        original = cls.ros_now
        monkeypatch.setattr(cls, 'ros_now', lambda self, fn=original: fn(self) - (0.8 if time.monotonic()-start >= 4.0 else 0.0))
'''
  else:
   s+='''\n_original_send_goal = _send_goal
_send_count = 0
def _send_goal(*args, **kwargs):
    global _send_count
    _send_count += 1
    if _send_count == 1:
        time.sleep(0.6)
    return _original_send_goal(*args, **kwargs)
'''
  open(out,'w').write(s)
PY
python3 -m pytest -q src/drop_mocks/test/test_w7_old_clock.py::test_injected_events_appear_at_scheduled_time > /mnt/c/tmp/codex-w7/controlled-old-mock.log 2>&1
echo old-mock=$?
python3 -m pytest -q src/drop_bringup/test/test_w7_old_hb.py::test_i0_heartbeat_loss_all_roles > /mnt/c/tmp/codex-w7/controlled-old-hb.log 2>&1
echo old-hb=$?
rm src/drop_mocks/test/test_w7_old_clock.py src/drop_bringup/test/test_w7_old_hb.py
