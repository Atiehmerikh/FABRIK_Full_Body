targetPosition=[2.2,4.0,6.0]
originPosition=[0.0,0.0,0.0]
orientationAngleLimit=0.5
constraintType = ["ball","ball","hinge","ball","ball","ball","ball","ball","ball","ball","ball","ball"
                 ,"ball","ball","ball","ball","ball"]
manipulator = FABRIK(np.loadtxt("joints.txt"),targetPosition,originPosition,orientationAngleLimit,np.loadtxt("length.txt"),np.loadtxt("Theta.txt"),constraintType)
manipulator.solve()