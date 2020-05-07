import FABRIK_IK
import numpy as np


def main():
    target_position = [-0.45, 0.4, 0.7]
    target_orientation = [0.71, 0, 0, 0.71]
    clone_joints_position()
    with open('constraint_type.txt') as f:
        constraint_type = [line.rstrip() for line in f]

    manipulator = FABRIK_IK.FABRIK(np.loadtxt("joints_position.txt"),np.loadtxt("joints_position_fixed.txt"), np.loadtxt("orientation.txt"), target_position,
                                   target_orientation, np.loadtxt("joints_constraint.txt"),
                                   constraint_type,np.loadtxt("bone_twist_constraints.txt"))

    manipulator.solve()


def clone_joints_position():
    with open("joints_position.txt") as f:
        with open("joints_position_fixed.txt", "w") as f1:
            for line in f:
                f1.write(line)



if __name__ == "__main__":
    main()