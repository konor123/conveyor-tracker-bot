import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), *[os.pardir] * 3, 'tools', 'testing'))
from drop_pytest import *  # noqa: E402,F401,F403

import pytest  # noqa: E402

PKG = os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir))
SCENARIOS = os.path.join(PKG, 'scenarios')
DATA = os.path.join(os.path.dirname(__file__), 'data')


def scenario_path(name):
    for d in (SCENARIOS, DATA):
        p = os.path.join(d, name + '.yaml')
        if os.path.exists(p):
            return p
    raise FileNotFoundError(name)


@pytest.fixture
def load_scenario():
    import yaml

    def _load(name):
        with open(scenario_path(name), encoding='utf-8') as fh:
            return yaml.safe_load(fh)
    return _load
