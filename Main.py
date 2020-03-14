import FABRIK_IK
import numpy as np

targetPosition = [-0.45, 0.1, 0.7]
targetOrientation = [0.71, 0, 0, 0.71]

# constraint of each joint

constraint_type = ["ballAndSocket", "ballAndSocket", "hinge", "ballAndSocket", "endEffector",
                   "ballAndSocket", "hinge", "ballAndSocket", "endEffector",
                   "ballAndSocket", "ballAndSocket",
                   "hinge", "ballAndSocket", "endEffector",
                   "hinge", "ballAndSocket", "endEffector",
                   "ballAndSocket", "endEffector"]

# rotational angle limit of each joint in degree
rotationAngleLimit = 40

manipulator = FABRIK_IK.FABRIK(np.loadtxt("joints_position.txt"), np.loadtxt("orientation.txt"), targetPosition,
                               targetOrientation,
                               np.loadtxt("joints_position_fixed.txt"), np.loadtxt("joints_constraint.txt"),
                               constraint_type, rotationAngleLimit)

manipulator.solve()
