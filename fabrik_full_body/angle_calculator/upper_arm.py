import math
import numpy as np
import fabrik_full_body.angle_calculator.body_part_numbering as bodyNum


class UpperArm:
    def __init__(self, joints, file):
        self.joints = joints
        self.file = file

    def trunk_plane(self):
        m_body_number = bodyNum.BodyPartNumber()
        trunk_joint_numbers = m_body_number.trunk_upper_body()

        # finding a plane of upper body
        u = self.joints[trunk_joint_numbers[1]] - self.joints[trunk_joint_numbers[0]]
        v = self.joints[trunk_joint_numbers[3]] - self.joints[trunk_joint_numbers[0]]

        normal_plane = np.cross(u, v)
        return normal_plane

    def upper_arm_flex(self):
        m_body_number = bodyNum.BodyPartNumber()
        right_upper_arm_joint_numbers = m_body_number.right_arm()
        left_upper_arm_joint_numbers = m_body_number.left_arm()

        right_upper_arm_vector = self.joints[right_upper_arm_joint_numbers[2]] - self.joints[
            right_upper_arm_joint_numbers[1]]

        left_upper_arm_vector = self.joints[left_upper_arm_joint_numbers[2]] - self.joints[
            left_upper_arm_joint_numbers[1]]

        normal_trunk_plane = self.trunk_plane()

        flex_right_upper_arm = 90 - math.degrees(math.acos(np.dot(right_upper_arm_vector, normal_trunk_plane) /
                                                           (math.sqrt(
                                                               np.dot(normal_trunk_plane,
                                                                      normal_trunk_plane)) * math.sqrt(
                                                               np.dot(right_upper_arm_vector,
                                                                      right_upper_arm_vector)))))

        flex_left_upper_arm = 90 - math.degrees(math.acos(np.dot(left_upper_arm_vector, normal_trunk_plane) /
                                                          (math.sqrt(
                                                              np.dot(normal_trunk_plane,
                                                                     normal_trunk_plane)) * math.sqrt(
                                                              np.dot(left_upper_arm_vector,
                                                                     left_upper_arm_vector)))))

        self.file.write("Upper right arm flexion \n")
        self.file.write(str(flex_right_upper_arm))
        self.file.write("\n")

        self.file.write("Upper left arm flexion \n")
        self.file.write(str(flex_left_upper_arm))
        self.file.write("\n")

    def upper_arm_side_bending(self):
        m_body_number = bodyNum.BodyPartNumber()
        right_upper_arm_joint_numbers = m_body_number.right_arm()
        left_upper_arm_joint_numbers = m_body_number.left_arm()

        trunk_joint_numbers = m_body_number.trunk_upper_body()
        right_upper_arm_vector = self.joints[right_upper_arm_joint_numbers[2]] - self.joints[
            right_upper_arm_joint_numbers[1]]
        left_upper_arm_vector = self.joints[left_upper_arm_joint_numbers[2]] - self.joints[
            left_upper_arm_joint_numbers[1]]

        normal_trunk_plane = self.trunk_plane()

        proj_right_upperarm_on_plane = right_upper_arm_vector - np.dot(right_upper_arm_vector,
                                                                       normal_trunk_plane) * normal_trunk_plane

        proj_left_upperarm_on_plane = left_upper_arm_vector - np.dot(left_upper_arm_vector,
                                                                     normal_trunk_plane) * normal_trunk_plane

        spine_vector = self.joints[trunk_joint_numbers[0]] - self.joints[trunk_joint_numbers[2]]

        right_side_degree = np.dot(spine_vector, proj_right_upperarm_on_plane) / (
                math.sqrt(np.dot(spine_vector, spine_vector)) * math.sqrt(
            np.dot(proj_right_upperarm_on_plane, proj_right_upperarm_on_plane)))

        left_side_degree = np.dot(spine_vector, proj_left_upperarm_on_plane) / (
                math.sqrt(np.dot(spine_vector, spine_vector)) * math.sqrt(
            np.dot(proj_left_upperarm_on_plane, proj_left_upperarm_on_plane)))

        if np.dot(np.cross(spine_vector, right_upper_arm_vector), normal_trunk_plane) < 0:
            # if the arm go to the body: adduction
            right_side_degree *= -1

        if np.dot(np.cross(spine_vector, left_upper_arm_vector), normal_trunk_plane) > 0:
            left_side_degree *= -1

        self.file.write("Upper right arm abduction \n")
        self.file.write(str(right_side_degree))
        self.file.write("\n")

        self.file.write("Upper left arm abduction \n")
        self.file.write(str(left_side_degree))
        self.file.write("\n")

    def shoulder_rise(self):
        m_body_number = bodyNum.BodyPartNumber()
        trunk_joint_numbers = m_body_number.trunk_upper_body()
        right_shoulder_joint_numbers = m_body_number.right_shoulder()
        left_shoulder_joint_numbers = m_body_number.left_shoulder()
        spine_vector = self.joints[trunk_joint_numbers[0]] - self.joints[trunk_joint_numbers[2]]
        right_shoulder_vector = self.joints[right_shoulder_joint_numbers[1]] - self.joints[
            right_shoulder_joint_numbers[0]]
        left_shoulder_vector = self.joints[left_shoulder_joint_numbers[1]] - self.joints[left_shoulder_joint_numbers[0]]

        right_shoulder_rise_degree = math.degrees(math.acos(np.dot(spine_vector, right_shoulder_vector) / (
                math.sqrt(np.dot(right_shoulder_vector, right_shoulder_vector)) * math.sqrt(
            np.dot(spine_vector, spine_vector)))))
        left_shoulder_rise_degree = math.degrees(math.acos(np.dot(spine_vector, left_shoulder_vector) / (
                math.sqrt(np.dot(left_shoulder_vector, left_shoulder_vector)) * math.sqrt(
            np.dot(spine_vector, spine_vector)))))

        self.file.write("Upper right arm raise \n")
        self.file.write(str(right_shoulder_rise_degree))
        self.file.write("\n")

        self.file.write("Upper left arm raise \n")
        self.file.write(str(left_shoulder_rise_degree))
        self.file.write("\n")

    def upper_arm_rotation(self):
        # TODO: twist should be coded
        self.file.write("Upper right arm rotation \n")
        self.file.write(str(0))
        self.file.write("\n")

        self.file.write("Upper left arm rotation \n")
        self.file.write(str(0))
        self.file.write("\n")
