from FABRIKARM import FABRIKARM
from UpperChain import ClosedLoop
import numpy as np

## in this part you can build an arm with specifying constraint types and joint position and target
##
##constraintType=["ball","hinge","ball"]
##targetPosition=[4.0,3.0,1.0]
##originPosition=[0.0,0.0,0.0]
##orientationAngleLimit=0.5
##
manipulator = FABRIKARM(np.loadtxt("joints.txt"),targetPosition,originPosition,orientationAngleLimit,np.loadtxt("length.txt"))
manipulator.solve()


## in this part you can build an UpperBody with specifying constraint types and joint position and target
##hint: it should make a loop so the first joint coordinate should be the same to last joint coordinate!

##targetPosition=[7.0,1.0,1.0]
##originPosition=[3.0,2.0,1.0]
##
### End Effectors of Upper chain
##fixed_joint=[3.0, 2.0, 1.0]
##
##
##
##orientationAngleLimit=0.5
##
##manipulator = ClosedLoop(np.loadtxt("jointsClosed.txt"),targetPosition,originPosition,fixed_joint,np.loadtxt("theta.txt"),orientationAngleLimit,np.loadtxt("length.txt"))
##manipulator.solve()


