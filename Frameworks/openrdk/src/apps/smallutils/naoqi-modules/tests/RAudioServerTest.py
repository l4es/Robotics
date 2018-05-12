"""
  UnitTests for RAudioServer Module

"""

import unittest
import naoqi
from naoqi import ALProxy

IP = "127.0.0.1"
PORT = 9559

class RAudioServerTest(unittest.TestCase):
    """
    All methods that begin with 'test' are consisdered as tests
    Methods are run in sorted order
    """

    def setUp(self):
       """ Code that will run before every test """
       pass

    def tearDown(self):
        """ Code that will run after every test """
        pass

    def test_innerTest_00(self):
        global proxy
        result = proxy.innerTest()


     # ... add more tests here ...

def main():
    global proxy
    proxy = ALProxy("RAudioServer", IP, PORT)
    suite = unittest.TestLoader().loadTestsFromTestCase(RAudioServerTest)
    results = unittest.TextTestRunner(verbosity=2).run(suite)
    return results.wasSuccessful()

if __name__ == "__main__":
    main()
