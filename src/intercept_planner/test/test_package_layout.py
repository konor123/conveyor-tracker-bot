import importlib



def test_package_importable():
    assert importlib.import_module('intercept_planner') is not None
