import unittest

from ldesign.path import Segment


class MyTestCase(unittest.TestCase):
    def test_something(self):
        self.assertRaises(TypeError, lambda: Segment("gg", 10))
        self.assertRaises(ValueError, lambda: Segment((1, 2), -10))
        t = Segment(10j, 5)
        self.assertEqual(t.point, 10j)
        self.assertEqual(t.radius, 5)


if __name__ == '__main__':
    unittest.main()
