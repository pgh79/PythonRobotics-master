from unittest import TestCase

import sys
sys.path.append("../PathPlanning/RRTStarDubins/")
# import os
# BASE_DIR = os.path.dirname(os.path.abspath(__file__))
# sys.path.append(BASE_DIR)
# sys.path.append(os.path.join(BASE_DIR, '../PathPlanning/RRTStarDubins/'))

from PathPlanning.RRTStarDubins import rrt_star_dubins as m

print(__file__)


class Test(TestCase):

    def test1(self):
        m.show_animation = False
        m.main()
