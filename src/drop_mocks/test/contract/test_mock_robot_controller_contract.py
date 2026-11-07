"""mock_robot_controller_node가 robot_controller 계약을 통과한다(TC-MCK-004-01)."""
import os
import sys

import pytest
from drop_ros import Target
from robot_controller_contract import *  # noqa: F401,F403

pytestmark = [
    pytest.mark.req('REQ-MCK-004'),
    pytest.mark.tc('TC-MCK-004-01'),
    pytest.mark.req('REQ-IF-005'),
    pytest.mark.tc('TC-IF-005-01'),
    pytest.mark.level('C'),
]
SCENARIO = os.path.join(os.path.dirname(__file__), os.pardir, 'data', 'contract_robot.yaml')


@pytest.fixture(scope='module')
def robot_target():
    return Target.from_env('robot_controller', Target(
        cmd=[sys.executable, '-m', 'drop_mocks.mock_robot_controller_node'], node='mock_robot_controller_node',
        ros_args=['-p', f'scenario:={os.path.abspath(SCENARIO)}']))
