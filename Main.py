import FABRIK_IK
import numpy as np

targetPosition = [-0.45, 0.3, 0.5]

# constraint of each joint

constraint_type = ["ballAndSocket", "ballAndSocket", "ballAndSocket", "ballAndSocket", "endEffector",
                   "ballAndSocket","ballAndSocket", "ballAndSocket", "endEffector",
                   "ballAndSocket", "ballAndSocket",
                   "hinge","ballAndSocket", "ballAndSocket",
                   "hinge", "ballAndSocket", "ballAndSocket",
                   "ballAndSocket","ballAndSocket"]
orientationAngleLimit = 0

manipulator = FABRIK_IK.FABRIK(np.loadtxt("joints_position.txt"), targetPosition, orientationAngleLimit,
                               np.loadtxt("joints_position_fixed.txt"), np.loadtxt("joints_constraint.txt"), constraint_type)

manipulator.solve()
