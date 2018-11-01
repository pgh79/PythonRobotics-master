from unittest import TestCase

from SLAM.iterative_closest_point import iterative_closest_point as m

print(__file__)


class Test(TestCase):

    def test1(self):
        m.show_animation = False
        m.main()
