"""mock_world_node가 tracker 계약을 통과한다."""
import sys
import time

import pytest
from drop_ros import Target
from tracker_contract import *  # noqa: F401,F403



@pytest.fixture(scope='module')
def tracker_target():
    return Target.from_env('tracker', Target(
        cmd=[sys.executable, '-m', 'drop_mocks.mock_world_node'], node='mock_world_node',
        ros_args=['-p', 'role:=tracking', '-p', 'scenario:=nominal_20mms', '-p', f't0_s:={time.time() - 5.0}']))
