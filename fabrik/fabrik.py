import numpy as np
import math
from fabrik.constraints import Constraints
from fabrik.draw import Draw
from fabrik.angle_calculator.angle_calculator import AngleCalculator
import fabrik.util as util
from fabrik.input_reader import *
from fabrik.output_writer import *
# This is solver of Inverse kinematic of a whole chain of human body with foot on the ground
# For more info about the procedure refer to https://www.sciencedirect.com/science/article/pii/S1524070311000178
# and for the constraints refer to https://www.researchgate.net/publication/271771862_Extending_FABRIK_with_model_constraints


class FABRIK:

    def __init__(self, joints, initial_joints_position, orientation, target, target_orientation, theta, constraint_type,
                 bone_orientation_limit):
        self.n = len(joints)
        self.joints = joints
        self.firstPos = initial_joints_position
        self.orientation = orientation
        self.target = target
        self.targetOrientation = target_orientation
        self.constraint_type = constraint_type
        self.bone_orientation_limit = bone_orientation_limit
        self.Theta = theta

        self.solve_distance_threshold = 0.01

        self.rightArmIndex = [17, 1, 2, 3, 4]
        self.right_end_effector_index = 4
        self.leftArmIndex = [17, 5, 6, 7, 8]
        self.left_end_effector_index = 8
        self.upperChain = [0, 1, 5, 17, 0]
        self.lowerChain = [0, 9, 10, 0]
        self.rightLeg = [13, 12, 11, 9]
        self.leftLeg = [16, 15, 14, 10]
        self.neck = [0, 17]
        self.head = [17, 18]

        self.inverseLeftArmIndex = [8, 7, 6, 5]
        self.inverseRightArmIndex = [4, 3, 2, 1]

    # Finding the laterality of the human whole chain
    def laterality(self):
        """
        Determines right-handed or left-handed person
        """
        right_upper_body = util.distance(self.firstPos[self.right_end_effector_index], self.target)
        left_upper_body = util.distance(self.firstPos[self.left_end_effector_index], self.target)

        # ON THE RIGHT SIDE OR LEFT relative to how close to end effectors
        if right_upper_body <= left_upper_body:
            return "right"
        if right_upper_body > left_upper_body:
            return "left"

    # Finding the next point position in forward/backward iteration of FABRIK
    def solve_for_distance(self, i, j, body_part):
        r = util.distance(self.joints[body_part[i]], self.joints[body_part[j]])
        landa = util.distance(self.firstPos[body_part[i]], self.firstPos[body_part[j]]) / r
        pos = (1 - landa) * self.joints[body_part[i]] + landa * self.joints[body_part[j]]
        self.joints[body_part[j]] = pos

    def solve_for_orientation(self, outer_joint, inner_joint, body_part):
        q1 = self.orientation[body_part[outer_joint]]
        q2 = self.orientation[body_part[inner_joint]]
        # finding the rotor that express rotation between two orientational frame(between outer and inner joint)
        rotor = self.find_rotation_quaternion(q1, q2)
        needed_rotation = math.acos(rotor[0]) * 2 * (180 / np.pi)

        if needed_rotation <= self.bone_orientation_limit[body_part[outer_joint]]:
            # if the rotation is inside the limited
            self.orientation[body_part[inner_joint]] = self.multiply_two_quaternion(rotor, self.orientation[
                body_part[outer_joint]])
        else:
            # the maximum allowed rotation angle
            theta = (self.bone_orientation_limit[body_part[outer_joint]]) * (np.pi / 180)
            # the rotation axis
            v1 = np.dot(rotor[1:], (1 / math.sqrt(1 - rotor[0] ** 2)))
            w = math.cos(theta / 2)
            x = v1[0] * math.sin(theta / 2)
            y = v1[1] * math.sin(theta / 2)
            z = v1[2] * math.sin(theta / 2)
            self.orientation[body_part[inner_joint]] = [w, x, y, z]

    # Finding the rotation rotor between outer joint and inner joint of each FABRIK iteration
    def find_rotation_quaternion(self, outer_quaternion, inner_quaternion):
        conjucate = [outer_quaternion[0], -outer_quaternion[1], -outer_quaternion[2], -outer_quaternion[3]]
        length = math.sqrt(outer_quaternion[0] ** 2 + outer_quaternion[1] ** 2 +
                           outer_quaternion[2] ** 2 + outer_quaternion[3] ** 2)
        inverse = np.dot(conjucate, (1 / length))
        rotation = self.multiply_two_quaternion(inner_quaternion, inverse)
        return rotation

    # multiplication of two quaternion(representing the orientation of joints)
    def multiply_two_quaternion(self, q1, q2):
        a = q1[0]
        b = q1[1]
        c = q1[2]
        d = q1[3]
        e = q2[0]
        f = q2[1]
        g = q2[2]
        h = q2[3]

        m0 = round(a * e - b * f - c * g - d * h, 2)
        m1 = round(b * e + a * f + c * h - d * g, 2)
        m2 = round(a * g - b * h + c * e + d * f, 2)
        m3 = round(a * h + b * g - c * f + d * e, 2)
        return [m0, m1, m2, m3]

    def forward_arm(self, arm_index, target, target_orientation):
        n = len(arm_index)
        # set end effector as target
        self.joints[arm_index[n - 1]] = target
        self.orientation[arm_index[n - 1]] = target_orientation
        for i in range(n - 2, -1, -1):
            self.solve_for_distance(i + 1, i, arm_index)
            self.solve_for_orientation(i + 1, i, arm_index)
            if i < n - 2:
                mconstraint = Constraints(self.joints, arm_index, i + 1, self.Theta[arm_index[i + 1]], self.firstPos,
                                          self.constraint_type[arm_index[i + 1]], self.orientation[arm_index[i + 1]])
                constraint_return = mconstraint.rotational_constraint()
                if constraint_return[0] != 0:
                    for j in range(3):
                        self.joints[arm_index[i]][j] = constraint_return[j]

    def update_upper_chain_f(self, target_position):
        if target_position == "right":
            self.solve_for_distance(1, 2, self.upperChain)
            self.solve_for_distance(2, 0, self.upperChain)
            self.solve_for_distance(1, 0, self.upperChain)
            self.solve_for_distance(0, 2, self.upperChain)
            # for tuning left arm:
            self.forward_arm(self.inverseLeftArmIndex, self.joints[self.leftArmIndex[1]],
                             self.orientation[self.leftArmIndex[1]])
        else:
            self.solve_for_distance(2, 0, self.upperChain)
            self.solve_for_distance(0, 1, self.upperChain)
            self.solve_for_distance(2, 1, self.upperChain)
            self.solve_for_distance(1, 0, self.upperChain)
            # for tuning right arm:
            self.forward_arm(self.inverseRightArmIndex, self.joints[self.rightArmIndex[1]],
                             self.orientation[self.rightArmIndex[1]])

    def update_upper_chain_b(self, target_position):
        self.solve_for_distance(0, 2, self.upperChain)
        self.solve_for_distance(2, 1, self.upperChain)
        self.solve_for_distance(0, 1, self.upperChain)
        self.solve_for_distance(1, 2, self.upperChain)
        if target_position == 'right':
            self.forward_arm(self.inverseLeftArmIndex, self.joints[self.leftArmIndex[1]],
                             self.orientation[self.leftArmIndex[1]])
        else:
            self.forward_arm(self.inverseRightArmIndex, self.joints[self.rightArmIndex[1]],
                             self.orientation[self.rightArmIndex[1]])
        # Neck
        self.joints[self.upperChain[3]] = (self.joints[self.upperChain[2]] + self.joints[self.upperChain[1]]) / 2

    def update_lower_chain_f(self):
        self.solve_for_distance(0, 1, self.lowerChain)
        self.solve_for_distance(1, 2, self.lowerChain)
        self.solve_for_distance(0, 2, self.lowerChain)
        self.solve_for_distance(2, 1, self.lowerChain)

    def update_lower_chain_b(self):
        self.solve_for_distance(1, 0, self.lowerChain)
        self.solve_for_distance(2, 0, self.lowerChain)

    def forward_leg(self):
        # set end effector of leg as target
        n = len(self.rightLeg)
        for i in range(n - 2, -1, -1):
            self.solve_for_distance(i + 1, i, self.rightLeg)
            self.solve_for_distance(i + 1, i, self.leftLeg)
            self.solve_for_orientation(i + 1, i, self.rightLeg)
            self.solve_for_orientation(i + 1, i, self.leftLeg)
            if i < n - 2:
                mconstraint = Constraints(self.joints, self.rightLeg, i + 1, self.Theta[self.rightLeg[i + 1]],
                                          self.firstPos,
                                          self.constraint_type[self.rightLeg[i + 1]],
                                          self.orientation[self.rightLeg[i + 1]])
                constraint_return = mconstraint.rotational_constraint()
                if constraint_return[0] != 0:
                    for j in range(3):
                        self.joints[self.rightLeg[i]][j] = constraint_return[j]

                mconstraint = Constraints(self.joints, self.leftLeg, i + 1, self.Theta[self.leftLeg[i + 1]],
                                          self.firstPos,
                                          self.constraint_type[self.leftLeg[i + 1]],
                                          self.orientation[self.leftLeg[i + 1]])
                constraint_return = mconstraint.rotational_constraint()
                if constraint_return[0] != 0:
                    for j in range(3):
                        self.joints[self.leftLeg[i]][j] = constraint_return[j]

    def backward_arm(self, arm_index):
        # set root as initial position
        n = len(arm_index)
        for i in range(2, n):
            self.solve_for_distance(i - 1, i, arm_index)

    def backward_leg(self):
        # set root as initial position
        self.joints[self.rightLeg[0]] = self.firstPos[self.rightLeg[0]]
        self.joints[self.rightLeg[1]] = self.firstPos[self.rightLeg[1]]
        self.joints[self.leftLeg[0]] = self.firstPos[self.leftLeg[0]]
        self.joints[self.leftLeg[1]] = self.firstPos[self.leftLeg[1]]
        n = len(self.rightLeg)
        for i in range(2, n):
            # for Right leg
            self.solve_for_distance(i - 1, i, self.rightLeg)

            # for left leg
            self.solve_for_distance(i - 1, i, self.leftLeg)

    def solve(self):
        counter = 0
        sum_l = 0
        target_pos = self.laterality()
        ###################################################################
        # is target in reach or not
        if target_pos == "right":
            # arm length
            for i in range(len(self.rightArmIndex) - 1):
                sum_l = sum_l + util.distance(self.firstPos[self.rightArmIndex[i]],
                                              self.firstPos[self.rightArmIndex[i + 1]])
            # chain length
            sum_l = sum_l + util.distance(self.firstPos[0], self.firstPos[self.rightLeg[len(self.rightLeg) - 1]])
            # Leg length
            for i in range(len(self.rightLeg) - 1):
                sum_l = sum_l + util.distance(self.firstPos[self.rightLeg[i]],
                                              self.firstPos[self.rightLeg[i + 1]])
            if sum_l < util.distance(self.firstPos[self.rightLeg[0]], self.target):
                print("target is out of reach!!!!!!!")
                return
            else:
                n = len(self.rightArmIndex)
                dif = util.distance(self.joints[self.rightArmIndex[n - 1]], self.target)

        else:
            # arm length
            for i in range(len(self.leftArmIndex) - 1):
                sum_l = sum_l + util.distance(self.firstPos[self.leftArmIndex[i]],
                                              self.firstPos[self.leftArmIndex[i + 1]])
            # chain length
            sum_l = sum_l + util.distance(self.firstPos[0], self.firstPos[self.leftLeg[len(self.leftLeg) - 1]])
            # Leg length
            for i in range(len(self.leftLeg) - 1):
                sum_l = sum_l + util.distance(self.firstPos[self.leftLeg[i]], self.firstPos[self.leftLeg[i + 1]])
            if sum_l < util.distance(self.firstPos[self.leftLeg[0]], self.target):
                print("target is out of reach!!!!!!!")
                return
            else:
                n = len(self.leftArmIndex)
                dif = util.distance(self.joints[self.leftArmIndex[n - 1]], self.target)

            # target is in reach
        ##########################################################################

        while dif > self.solve_distance_threshold:
            if target_pos == "right":
                self.forward_arm(self.rightArmIndex, self.target, self.targetOrientation)
            else:
                self.forward_arm(self.leftArmIndex, self.target, self.targetOrientation)
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
                dif = util.distance(self.joints[self.rightArmIndex[n - 1]], self.target)
            else:
                dif = util.distance(self.joints[self.leftArmIndex[n - 1]], self.target)
            counter = counter + 1
            if counter > 10:
                break

        m_angle = AngleCalculator(OutputWriter().angle_writer(), self.joints)
        m_angle.calculate()
        draw_obj = Draw(self.joints, self.target, InputReader().initial_joints_position(), self.rightArmIndex,
                        self.leftArmIndex,
                        self.upperChain,
                        self.lowerChain, self.rightLeg, self.leftLeg, self.neck, self.head)
        draw_obj.draw_final()
