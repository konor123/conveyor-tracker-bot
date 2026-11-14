"""mock_world_node가 vision 계약을 통과한다."""
import sys
import time

import pytest
from drop_ros import Target
from vision_contract import *  # noqa: F401,F403



@pytest.fixture(scope='module')
def vision_target():
    return Target.from_env('vision', Target(
        cmd=[sys.executable, '-m', 'drop_mocks.mock_world_node'], node='mock_world_node',
        ros_args=['-p', 'role:=vision', '-p', 'scenario:=nominal_20mms', '-p', f't0_s:={time.time() - 5.0}']))
