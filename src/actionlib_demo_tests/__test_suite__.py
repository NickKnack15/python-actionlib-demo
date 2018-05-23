import unittest
import os


class TestSuite(unittest.TestSuite):

    def __init__(self):
        super(TestSuite, self).__init__()

        test_directory = os.path.dirname(os.path.abspath(os.path.join(__file__, os.pardir)))
        loader = unittest.TestLoader()
        self.addTests(loader.discover(test_directory, '*_test.py'))
