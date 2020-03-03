import FABRIK_v2
import numpy as np
import RebaOptimizer

targetPosition = [0.52, -0.95, 0.52]

constraint_type = ["ballAndSocket", "ballAndSocket", "hinge", "ballAndSocket", "endEffector",
                   "ballAndSocket","hinge", "ballAndSocket", "endEffector",
                   "ballAndSocket", "ballAndSocket", "hinge",
                   "ballAndSocket", "ballAndSocket", "hinge", "ballAndSocket", "ballAndSocket", "ballAndSocket",
                   "ballAndSocket"]
orientationAngleLimit = 0.5
# initial_reba_score = RebaOptimizer.__init__(np.loadtxt("length1.txt"))

manipulator = FABRIK_v2.FABRIK(np.loadtxt("joints1.txt"), targetPosition, orientationAngleLimit,
                            np.loadtxt("length1.txt"), np.loadtxt("theta-reba.txt"), constraint_type)

manipulator.solve()
