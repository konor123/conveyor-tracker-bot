import os
import sys

_HERE = os.path.dirname(__file__)
_SRC = os.path.join(_HERE, os.pardir, os.pardir)
sys.path.insert(0, os.path.join(_HERE, *[os.pardir] * 3, 'tools', 'testing'))
from drop_pytest import *  # noqa: E402,F401,F403

# 소스 트리에서 직접 돌릴 때 형제 패키지(vision, robot_control)를 찾는다
for _pkg in ('static_pick_place', 'vision', 'robot_control', 'calibration'):
    sys.path.insert(0, os.path.join(_SRC, _pkg))
