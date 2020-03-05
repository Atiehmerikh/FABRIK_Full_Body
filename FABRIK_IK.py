import numpy as np
from pycg3d.cg3d_point import CG3dPoint
from pycg3d import utils
from Constraints import constraints
from draw import Draw
# This is solver of Inverse kinematic of a whole chain of human body with foot on the ground
# For more info about the procedure refer to https://www.sciencedirect.com/science/article/pii/S1524070311000178
# and for the constraints refer to https://www.researchgate.net/publication/271771862_Extending_FABRIK_with_model_constraints

def distance_calculation(i, j):
    i_point = CG3dPoint(i[0], i[1], i[2])
    j_point = CG3dPoint(j[0], j[1], j[2])

    return utils.distance(i_point, j_point)


class FABRIK:

    def __init__(self, joints, target, orientation, length, theta, constraint_type):
        self.n = len(joints)
        self.joints = joints
        self.orientation = orientation
        self.rightArmIndex = [17, 1, 2, 3, 4]
        self.leftArmIndex = [17, 5, 6, 7, 8]

        self.inverseLeftArmIndex = [8, 7, 6, 5]
        self.inverseRightArmIndex = [4, 3, 2, 1]

        self.upperChain = [0, 1, 5, 17, 0]
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

        # ON THE RIGHT SIDE OR LEFT relative to how close to end effectors
        if right_upper_body <= left_upper_body:
            return "right"
        if right_upper_body > left_upper_body:
            return "left"

    def position_calculator(self, i, j, body_part):
        r = distance_calculation(self.joints[body_part[i]], self.joints[body_part[j]])
        landa = distance_calculation(self.firstPos[body_part[i]], self.firstPos[body_part[j]]) / r
        pos = (1 - landa) * self.joints[body_part[i]] + landa * self.joints[body_part[j]]
        self.joints[body_part[j]] = pos

    def forward_arm(self, arm_index, target):
        n = len(arm_index)
        # set end effector as target
        self.joints[arm_index[n - 1]] = target
        for i in range(n - 2, -1, -1):
            self.position_calculator(i + 1, i, arm_index)

    def update_upper_chain_f(self, target_position):
        if target_position == "right":
            self.position_calculator(1, 2, self.upperChain)
            self.position_calculator(2, 0, self.upperChain)
            self.position_calculator(1, 0, self.upperChain)
            self.position_calculator(0, 2, self.upperChain)
            # for tuning left arm:
            self.forward_arm(self.inverseLeftArmIndex, self.joints[self.leftArmIndex[1]])
        else:
            self.position_calculator(2, 0, self.upperChain)
            self.position_calculator(0, 1, self.upperChain)
            self.position_calculator(2, 1, self.upperChain)
            self.position_calculator(1, 0, self.upperChain)
            # for tuning right arm:
            self.forward_arm(self.inverseRightArmIndex, self.joints[self.rightArmIndex[1]])

    def update_upper_chain_b(self, target_position):
        self.position_calculator(0, 2, self.upperChain)
        self.position_calculator(2, 1, self.upperChain)
        self.position_calculator(0, 1, self.upperChain)
        self.position_calculator(1, 2, self.upperChain)
        if target_position == 'right':
            self.forward_arm(self.inverseLeftArmIndex, self.joints[self.leftArmIndex[1]])
        else:
            self.forward_arm(self.inverseRightArmIndex, self.joints[self.rightArmIndex[1]])
        # Neck
        self.joints[self.upperChain[3]] = (self.joints[self.upperChain[2]] + self.joints[self.upperChain[1]]) / 2

    def update_lower_chain_f(self):
        self.position_calculator(0, 1, self.lowerChain)
        self.position_calculator(1, 2, self.lowerChain)
        self.position_calculator(0, 2, self.lowerChain)
        self.position_calculator(2, 1, self.lowerChain)

    def update_lower_chain_b(self):
        self.position_calculator(1, 0, self.lowerChain)
        self.position_calculator(2, 0, self.lowerChain)

    def forward_leg(self):
        # set end effector of leg as target
        n = len(self.rightLeg)
        for i in range(n - 2, -1, -1):
            self.position_calculator(i + 1, i, self.rightLeg)
            self.position_calculator(i + 1, i, self.leftLeg)

    def backward_arm(self, arm_index):
        # set root as initial position
        n = len(arm_index)
        for i in range(2, n):
            self.position_calculator(i - 1, i, arm_index)
            constraint_obj = constraints(self.joints, arm_index, i - 1, self.Theta[arm_index[i - 1]],
                                         self.firstPos,
                                         self.constraint_type[arm_index[i - 1]])
            constraint_return = constraint_obj.joint_location_calculator()
            if constraint_return[0] != 0:
                for j in range(3):
                    self.joints[arm_index[i]][j] = constraint_return[j]

    def backward_leg(self):
        # set root as initial position
        self.joints[self.rightLeg[0]] = self.firstPos[self.rightLeg[0]]
        self.joints[self.rightLeg[1]] = self.firstPos[self.rightLeg[1]]
        self.joints[self.leftLeg[0]] = self.firstPos[self.leftLeg[0]]
        self.joints[self.leftLeg[1]] = self.firstPos[self.leftLeg[1]]
        n = len(self.rightLeg)
        for i in range(2, n):
            # for Right leg
            self.position_calculator(i - 1, i, self.rightLeg)
            if i > 2:
                constraint_obj = constraints(self.joints, self.rightLeg, i - 1, self.Theta[self.rightLeg[i - 1]],
                                             self.firstPos,
                                             self.constraint_type[self.rightLeg[i - 1]])
                constraint_return = constraint_obj.joint_location_calculator()
                if constraint_return[0] != 0:
                    for j in range(3):
                        self.joints[self.rightLeg[i]][j] = constraint_return[j]

            # for left leg
            self.position_calculator(i - 1, i, self.leftLeg)
            if i > 2:
                constraint_obj = constraints(self.joints, self.leftLeg, i - 1, self.Theta[self.leftLeg[i - 1]],
                                             self.firstPos,
                                             self.constraint_type[self.leftLeg[i - 1]])
                constraint_return = constraint_obj.joint_location_calculator()
                if constraint_return[0] != 0:
                    for j in range(3):
                        self.joints[self.leftLeg[i]][j] = constraint_return[j]

    def solve(self):
        counter = 0
        sum_l = 0
        target_pos = self.laterality()
        ###################################################################
        # is target in reach or not
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
                n = len(self.rightArmIndex)
                dif = distance_calculation(self.joints[self.rightArmIndex[n - 1]], self.target)

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
            else:
                n = len(self.leftArmIndex)
                dif = distance_calculation(self.joints[self.leftArmIndex[n - 1]], self.target)

            # target is in reach
        ##########################################################################
        while dif > self.tolerance:
            if target_pos == "right":
                self.forward_arm(self.rightArmIndex, self.target)
            else:
                self.forward_arm(self.leftArmIndex, self.target)
            self.update_upper_chain_f(target_pos)
            self.update_lower_chain_f()
            self.forward_leg()

            self.backward_leg()
            self.update_lower_chain_b()
            self.update_upper_chain_b(target_pos)
            if target_pos == "right":
                self.backward_arm(self.rightArmIndex)
            else:
                self.backward_arm(self.leftArmIndex)

            if target_pos == "right":
                dif = distance_calculation(self.joints[self.rightArmIndex[n - 1]], self.target)
            else:
                dif = distance_calculation(self.joints[self.leftArmIndex[n - 1]], self.target)
            counter = counter + 1
            if counter > 10:
                break

        draw_obj = Draw(self.joints, self.target, np.loadtxt("joints_position_fixed.txt"), self.rightArmIndex, self.leftArmIndex,
                        self.upperChain,
                        self.lowerChain, self.rightLeg, self.leftLeg, self.neck, self.head)
        draw_obj.draw_final()
