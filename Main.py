import FABRIK as FABRIK
import numpy as np

targetPosition = [5.2, 6.0, 5.0]
orientationAngleLimit = 0.5
manipulator = FABRIK.FABRIK(np.loadtxt("joints.txt"), targetPosition, orientationAngleLimit,
                            np.loadtxt("length.txt"), np.loadtxt("Theta.txt"))
manipulator.solve()
