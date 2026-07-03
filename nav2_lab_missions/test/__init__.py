import os


def load_tests(loader, _tests, _pattern):
    return loader.discover(os.path.dirname(__file__), pattern='test_*.py')
