import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), *[os.pardir] * 3, 'tools', 'testing'))
from drop_pytest import *  # noqa: E402,F401,F403

sys.path.insert(0, os.path.join(os.path.dirname(__file__), os.pardir))
sys.path.insert(0, os.path.dirname(__file__))
