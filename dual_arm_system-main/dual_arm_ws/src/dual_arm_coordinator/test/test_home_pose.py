import unittest
import math
from dual_arm_coordinator.home_pose import convert_home


class HomePoseTests(unittest.TestCase):
    def test_unknown_zero_is_rejected(self):
        with self.assertRaises(ValueError):
            convert_home([90,567,712,825],[None]*4,[1]*4)

    def test_unknown_direction_is_rejected(self):
        with self.assertRaises(ValueError):
            convert_home([90,567,712,825],[512]*4,[None]*4)

    def test_signed_offsets(self):
        actual=convert_home([510,514,512,612],[512]*4,[1,-1,1,-1])
        factor=math.radians(300)/1023
        for a,b in zip(actual,[-2*factor,-2*factor,0,-100*factor]):
            self.assertAlmostEqual(a,b)

    def test_invalid_values_are_rejected(self):
        for raw,zero,direction in [([1024]*4,[512]*4,[1]*4),([512]*3,[512]*4,[1]*4),
                                   ([512]*4,[512]*4,[0]*4),([True]*4,[512]*4,[1]*4)]:
            with self.assertRaises(ValueError):
                convert_home(raw,zero,direction)
