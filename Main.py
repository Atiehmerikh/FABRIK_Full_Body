from FABRIKARM import FABRIKARM
from UpperChain import ClosedLoop

import numpy as np

## in this part you can build an arm with specifying constraint types and joint position and target

constraintType=["ball","hinge","ball"]
targetPosition=[4.0,3.0,1.0]
originPosition=[0.0,0.0,0.0]
orientationAngleLimit=0.5

manipulator = FABRIKARM(np.loadtxt("joints.txt"),targetPosition,originPosition,np.loadtxt("theta.txt"),orientationAngleLimit,constraintType)
manipulator.solve()


## in this part you can build an UpperBody with specifying constraint types and joint position and target
##hint: it should make a loop so the first joint coordinate should be the same to last joint coordinate!

targetPosition=[4.0,1.0,1.0]
orientationAngleLimit=0.5

manipulator = ClosedLoop(np.loadtxt("jointsClosed.txt"),targetPosition,np.loadtxt("theta.txt"),orientationAngleLimit)
manipulator.solve()
