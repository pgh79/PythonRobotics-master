from sympy import *
from sympy.abc import x, y, z, a, b, c, d
from math import *

aa = solve([x - y ** 2, (z - 9) ** 2 + x - (y - z) ** 2 - 9, 2 * (y - z) ** 2 - x], [x, y, z])
print(aa)
