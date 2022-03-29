import unittest

from ldesign.path import SegmentOp


class MyTestCase(unittest.TestCase):
    def test_something(self):
        self.assertRaises(TypeError, lambda: SegmentOp("gg", 10))  # type: ignore
        self.assertRaises(ValueError, lambda: SegmentOp((1, 2), -10))  # type: ignore
        t = SegmentOp(10j, 5)
        self.assertEqual(t.point, 10j)
        self.assertEqual(t.radius, 5)


if __name__ == "__main__":
    unittest.main()
