import importlib



def test_package_importable():
    assert importlib.import_module('vision') is not None
