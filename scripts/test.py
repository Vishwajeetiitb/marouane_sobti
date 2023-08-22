#!/usr/bin/env python3

import numpy as np
from math import pi
from math import atan,acos,asin,sin,cos,tan
x = 0.1
y = 0.1
l0 = 0.125
l1 = 0.125
q1 = acos((x**2 + y**2 - l0**2 - l1**2)/(2*l0*l1))
q0 = atan(x/y) - atan((l1*sin(q1))/(l0+l1*cos(q1)))

