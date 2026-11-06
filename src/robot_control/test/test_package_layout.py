import importlib

import pytest


@pytest.mark.level('U')
def test_package_importable():
    assert importlib.import_module('robot_control') is not None
