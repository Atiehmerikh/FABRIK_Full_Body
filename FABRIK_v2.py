import numpy as np
from pycg3d.cg3d_point import CG3dPoint
from pycg3d import utils
from Constraints import constraints
from draw import Draw
import RebaOptimizer


def distance_calculation(i, j):
    i_point = CG3dPoint(i[0], i[1], i[2])
    j_point = CG3dPoint(j[0], j[1], j[2])

    return utils.distance(i_point, j_point)


class FABRIK:

    def __init__(self, joints, target, orientation, length, theta, constraint_type):
        self.n = len(joints)
        self.joints = joints
        self.orientation = orientation
        self.rightArmIndex = [0, 1, 2, 3, 4]
        self.leftArmIndex = [0, 5, 6, 7, 8]
        self.upperChain = [0, 1, 5, 0]
        self.lowerChain = [0, 9, 10, 0]
        self.rightLeg = [13, 12, 11, 9]
        self.leftLeg = [16, 15, 14, 10]
        self.neck = [0, 17]
        self.head = [17, 18]
        self.constraint_type = constraint_type

        self.Theta = theta
        self.firstPos = length
        self.tolerance = 0.01
        self.target = target

    def laterality(self):
        right_upper_body = distance_calculation(self.firstPos[4], self.target)
        left_upper_body = distance_calculation(self.firstPos[8], self.target)

        # ON THE RIGHT SIDE OR LEFT relative to base point
        if right_upper_body <= left_upper_body:
            return "right"
        if right_upper_body > left_upper_body:
            return "left"

    def forward_arm(self, arm_index):
        # set end effector as target
        n = len(arm_index)
        self.joints[arm_index[n - 1]] = self.target
        for i in range(n - 2, -1, -1):
            r = distance_calculation(self.joints[arm_index[i]], self.joints[arm_index[i + 1]])
            landa = distance_calculation(self.firstPos[arm_index[i]], self.firstPos[arm_index[i + 1]]) / r
            # find new joint position
            pos = (1 - landa) * self.joints[arm_index[i + 1]] + landa * self.joints[arm_index[i]]
            self.joints[arm_index[i]] = pos

    def update_upper_chain(self, target_position):
        if target_position == "right":
            r = distance_calculation(self.joints[self.upperChain[1]], self.joints[self.upperChain[2]])
            landa = distance_calculation(self.firstPos[self.upperChain[1]], self.firstPos[self.upperChain[2]]) / r
            # find new joint position
            pos = (1 - landa) * self.joints[self.upperChain[1]] + landa * self.joints[self.upperChain[2]]
            self.joints[self.upperChain[2]] = pos
            self.backward_arm(self.leftArmIndex)
        else:
            r = distance_calculation(self.joints[self.upperChain[1]], self.joints[self.upperChain[2]])
            landa = distance_calculation(self.firstPos[self.upperChain[1]], self.firstPos[self.upperChain[2]]) / r
            # find new joint position
            pos = (1 - landa) * self.joints[self.upperChain[2]] + landa * self.joints[self.upperChain[1]]
            self.joints[self.upperChain[1]] = pos
            self.backward_arm(self.rightArmIndex)

    def update_lower_chain_f(self):
        r = distance_calculation(self.joints[self.lowerChain[0]], self.joints[self.lowerChain[1]])
        landa = distance_calculation(self.firstPos[self.lowerChain[0]], self.firstPos[self.lowerChain[1]]) / r
        # find new joint position
        pos = (1 - landa) * self.joints[self.lowerChain[0]] + landa * self.joints[self.lowerChain[1]]
        self.joints[self.lowerChain[1]] = pos

        r = distance_calculation(self.joints[self.lowerChain[0]], self.joints[self.lowerChain[2]])
        landa = distance_calculation(self.firstPos[self.lowerChain[0]], self.firstPos[self.lowerChain[2]]) / r
        # find new joint position
        pos = (1 - landa) * self.joints[self.lowerChain[0]] + landa * self.joints[self.lowerChain[2]]
        self.joints[self.lowerChain[2]] = pos

    def update_lower_chain_b(self):
        r = distance_calculation(self.joints[self.lowerChain[0]], self.joints[self.lowerChain[1]])
        landa = distance_calculation(self.firstPos[self.lowerChain[0]], self.firstPos[self.lowerChain[1]]) / r
        # find new joint position
        pos = (1 - landa) * self.joints[self.lowerChain[1]] + landa * self.joints[self.lowerChain[0]]
        self.joints[self.lowerChain[0]] = pos

    def forward_leg(self):
        # set end effector of leg as target
        n = len(self.rightLeg)
        for i in range(n - 2, -1, -1):
            # Right Leg
            r_right = distance_calculation(self.joints[self.rightLeg[i]], self.joints[self.rightLeg[i + 1]])
            landa_right = distance_calculation(self.firstPos[self.rightLeg[i]],
                                               self.firstPos[self.rightLeg[i + 1]]) / r_right
            pos_right = (1 - landa_right) * self.joints[self.rightLeg[i + 1]] + landa_right * self.joints[
                self.rightLeg[i]]

            r_left = distance_calculation(self.joints[self.leftLeg[i]], self.joints[self.leftLeg[i + 1]])
            landa_left = distance_calculation(self.firstPos[self.leftLeg[i]],
                                              self.firstPos[self.leftLeg[i + 1]]) / r_left
            pos_left = (1 - landa_left) * self.joints[self.leftLeg[i + 1]] + landa_left * self.joints[self.leftLeg[i]]

            # find new joint position

            self.joints[self.rightLeg[i]] = pos_right
            self.joints[self.leftLeg[i]] = pos_left

    def backward_arm(self, arm_index):
        # set root as initial position
        n = len(arm_index)
        for i in range(1, n):
            r = distance_calculation(self.joints[arm_index[i]], self.joints[arm_index[i - 1]])
            landa = distance_calculation(self.firstPos[arm_index[i]], self.firstPos[arm_index[i - 1]]) / r
            # find new joint position
            pos = (1 - landa) * self.joints[arm_index[i - 1]] + landa * self.joints[arm_index[i]]
            # if i < n:
            #     # constraint_obj = constraints(self.joints, arm_index, i - 1, self.Theta[arm_index[i - 1]],
            #                                  self.firstPos,
            #                                  self.constraint_type[arm_index[i - 1]])
            #     constraint_return = constraint_obj.joint_location_calculator()
            #     if constraint_return[0] == 0:
            #         self.joints[arm_index[i]] = pos
            #     else:
            #         for j in range(3):
            #             self.joints[arm_index[i]][j] = constraint_return[j]
            # else:
            #     self.joints[arm_index[i]] = pos
            self.joints[arm_index[i]] = pos

    def backward_leg(self):
        # set root as initial position
        self.joints[self.rightLeg[0]] = self.firstPos[self.rightLeg[0]]
        self.joints[self.rightLeg[1]] = self.firstPos[self.rightLeg[1]]
        self.joints[self.leftLeg[0]] = self.firstPos[self.leftLeg[0]]
        self.joints[self.leftLeg[1]] = self.firstPos[self.leftLeg[1]]
        n = len(self.rightLeg)
        for i in range(2, n):
            # for Right leg
            r = distance_calculation(self.joints[self.rightLeg[i]], self.joints[self.rightLeg[i - 1]])
            landa = distance_calculation(self.firstPos[self.rightLeg[i]], self.firstPos[self.rightLeg[i - 1]]) / r
            # find new joint position
            pos = (1 - landa) * self.joints[self.rightLeg[i - 1]] + landa * self.joints[self.rightLeg[i]]

            self.joints[self.rightLeg[i]] = pos
            # for left leg
            r = distance_calculation(self.joints[self.leftLeg[i]], self.joints[self.leftLeg[i - 1]])
            landa = distance_calculation(self.firstPos[self.leftLeg[i]], self.firstPos[self.leftLeg[i - 1]]) / r
            # find new joint position
            pos = (1 - landa) * self.joints[self.leftLeg[i - 1]] + landa * self.joints[self.leftLeg[i]]
            self.joints[self.leftLeg[i]] = pos

    def solve(self):
        sum_l = 0
        target_pos = self.laterality()
        if target_pos == "right":
            # arm length
            for i in range(len(self.rightArmIndex) - 1):
                sum_l = sum_l + distance_calculation(self.firstPos[self.rightArmIndex[i]],
                                                     self.firstPos[self.rightArmIndex[i + 1]])
            # chain length
            sum_l = sum_l + distance_calculation(self.firstPos[0], self.firstPos[self.rightLeg[len(self.rightLeg) - 1]])
            # Leg length
            for i in range(len(self.rightLeg) - 1):
                sum_l = sum_l + distance_calculation(self.firstPos[self.rightLeg[i]],
                                                     self.firstPos[self.rightLeg[i + 1]])
            if sum_l < distance_calculation(self.firstPos[self.rightLeg[0]], self.target):
                print("target is out of reach!!!!!!!")
                return
        else:
            # arm length
            for i in range(len(self.leftArmIndex) - 1):
                sum_l = sum_l + distance_calculation(self.firstPos[self.leftArmIndex[i]],
                                                     self.firstPos[self.leftArmIndex[i + 1]])
            # chain length
            sum_l = sum_l + distance_calculation(self.firstPos[0], self.firstPos[self.leftLeg[len(self.leftLeg) - 1]])
            # Leg length
            for i in range(len(self.leftLeg) - 1):
                sum_l = sum_l + distance_calculation(self.firstPos[self.leftLeg[i]], self.firstPos[self.leftLeg[i + 1]])
            if sum_l < distance_calculation(self.firstPos[self.leftLeg[0]], self.target):
                print("target is out of reach!!!!!!!")
                return

            # target is in reach
        counter = 0
        if target_pos == "right":
            n = len(self.rightArmIndex)
            dif = distance_calculation(self.joints[self.rightArmIndex[n - 1]], self.target)
        else:
            n = len(self.leftArmIndex)
            dif = distance_calculation(self.joints[self.leftArmIndex[n - 1]], self.target)

        while dif > self.tolerance:
            if target_pos == "right":
                self.backward_arm(self.rightArmIndex)
            else:
                self.backward_arm(self.leftArmIndex)
            self.update_upper_chain(target_pos)
            self.update_lower_chain_b()
            self.backward_leg()

            if target_pos == "right":
                self.forward_arm(self.rightArmIndex)
            else:
                self.forward_arm(self.leftArmIndex)
            self.update_upper_chain(target_pos)
            self.update_lower_chain_f()
            self.forward_leg()
            if target_pos == "right":
                dif = distance_calculation(self.joints[self.rightArmIndex[n - 1]], self.target)
            else:
                dif = distance_calculation(self.joints[self.leftArmIndex[n - 1]], self.target)
            counter = counter + 1
            if counter > 10:
                break

        RebaOptimizer.__init__(self.joints)
        draw_obj = Draw(self.joints, self.target, np.loadtxt("length1.txt"), self.rightArmIndex, self.leftArmIndex,
                        self.upperChain,
                        self.lowerChain, self.rightLeg, self.leftLeg, self.neck, self.head)
        draw_obj.draw_final()
