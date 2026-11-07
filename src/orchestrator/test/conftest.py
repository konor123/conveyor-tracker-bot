import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), *[os.pardir] * 3, 'tools', 'testing'))
from drop_pytest import *  # noqa: E402,F401,F403

import pytest  # noqa: E402

sys.path.insert(0, os.path.dirname(__file__))
from fakes import FakeClock  # noqa: E402


@pytest.fixture
def fake_clock():
    return FakeClock()


@pytest.fixture
def orch_cfg():
    import yaml
    path = os.path.join(os.path.dirname(__file__), os.pardir, 'config', 'orchestrator.yaml')
    with open(path) as fh:
        return yaml.safe_load(fh)
