import FABRIK as FABRIK
import numpy as np

targetPosition = [-0.65, 0.3, -0.2]
constraint_type = ["ballAndSocket", "ballAndSocket", "hinge", "ballAndSocket", "ballAndSocket", "ballAndSocket",
                   "hinge", "ballAndSocket", "ballAndSocket", "ballAndSocket", "ballAndSocket", "hinge",
                   "ballAndSocket", "ballAndSocket", "hinge", "ballAndSocket", "ballAndSocket", "ballAndSocket",
                   "ballAndSocket"]
orientationAngleLimit = 0.5
manipulator = FABRIK.FABRIK(np.loadtxt("joints2.txt"), targetPosition, orientationAngleLimit,
                            np.loadtxt("length2.txt"), np.loadtxt("Theta.txt"), constraint_type)
manipulator.solve()
