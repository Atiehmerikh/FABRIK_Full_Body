import FABRIK_v2
import numpy as np
import RebaCalculator


targetPosition = [-0.62, 0.551, 0.52]

constraint_type = ["ballAndSocket", "ballAndSocket", "hinge", "ballAndSocket", "endEffector",
                   "ballAndSocket","hinge", "ballAndSocket", "endEffector",
                   "ballAndSocket", "ballAndSocket", "hinge",
                   "ballAndSocket", "ballAndSocket", "hinge", "ballAndSocket", "ballAndSocket", "ballAndSocket",
                   "ballAndSocket"]
orientationAngleLimit = 0.5
initial_reba_score = RebaCalculator.__init__(np.loadtxt("length2.txt"))
print('the initial Reba Score:', initial_reba_score)


manipulator = FABRIK_v2.FABRIK(np.loadtxt("joints2.txt"), targetPosition, orientationAngleLimit,
                            np.loadtxt("length2.txt"), np.loadtxt("theta-reba.txt"), constraint_type)

manipulator.solve()
