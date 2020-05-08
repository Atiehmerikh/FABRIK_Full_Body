import numpy as np
import math
import fabrik_full_body.angle_calculator.body_part_numbering as bodyNum


class Wrist:
    def __init__(self, joints,file):
        self.joints = joints
        self.file = file

    def wrist_flex(self):
        m_body = bodyNum.BodyPartNumber()
        right_arm_joint_number = m_body.right_arm()
        left_arm_joint_number = m_body.left_arm()

        right_shoulder_elbow_vector = self.joints[right_arm_joint_number[1]] - self.joints[right_arm_joint_number[0]]
        left_shoulder_elbow_vector = self.joints[left_arm_joint_number[1]] - self.joints[left_arm_joint_number[0]]
        right_elbow_wrist_vector = self.joints[right_arm_joint_number[2]] - self.joints[right_arm_joint_number[1]]
        left_elbow_wrist_vector = self.joints[left_arm_joint_number[2]] - self.joints[left_arm_joint_number[1]]
        right_wrist_finger_vector = self.joints[right_arm_joint_number[3]] - self.joints[right_arm_joint_number[2]]
        left_wrist_finger_vector = self.joints[left_arm_joint_number[3]] - self.joints[left_arm_joint_number[2]]

        right_plane_normal_vec = np.cross(right_shoulder_elbow_vector, right_elbow_wrist_vector)
        left_plane_normal_vec = np.cross(left_shoulder_elbow_vector, left_elbow_wrist_vector)

        right_wrist_flex = math.degrees(math.acos(np.dot(right_elbow_wrist_vector, right_wrist_finger_vector) / (
                math.sqrt(np.dot(right_elbow_wrist_vector, right_elbow_wrist_vector)) * math.sqrt(
            np.dot(right_wrist_finger_vector, right_wrist_finger_vector)))))
        left_wrist_flex = math.degrees(math.acos(np.dot(left_elbow_wrist_vector, left_wrist_finger_vector) / (
                math.sqrt(np.dot(left_elbow_wrist_vector, left_elbow_wrist_vector)) * math.sqrt(
            np.dot(left_wrist_finger_vector, left_wrist_finger_vector)))))

        if right_plane_normal_vec[0] != 0 or right_plane_normal_vec[1] != 0 or right_plane_normal_vec[2] != 0:
            if np.dot(np.cross(right_wrist_finger_vector, right_elbow_wrist_vector), right_plane_normal_vec) > 0:
                # means extend of wrist
                right_wrist_flex *= -1

        if left_plane_normal_vec[0] != 0 or left_plane_normal_vec[1] != 0 or left_plane_normal_vec[2] == 0:
            if np.dot(np.cross(left_wrist_finger_vector, left_elbow_wrist_vector), left_plane_normal_vec) > 0:
                # means extend of wrist
                left_wrist_flex *= -1

        self.file.write("Right hand flexion \n")
        self.file.write(str(right_wrist_flex))
        self.file.write("\n")

        self.file.write("Left hand flexion \n")
        self.file.write(str(left_wrist_flex))
        self.file.write("\n")

    def wrist_side(self):
        m_body = bodyNum.BodyPartNumber()
        right_arm_joint_number = m_body.right_arm()
        left_arm_joint_number = m_body.left_arm()

        right_shoulder_elbow_vector = self.joints[right_arm_joint_number[1]] - self.joints[right_arm_joint_number[0]]
        left_shoulder_elbow_vector = self.joints[left_arm_joint_number[1]] - self.joints[left_arm_joint_number[0]]
        right_elbow_wrist_vector = self.joints[right_arm_joint_number[2]] - self.joints[right_arm_joint_number[1]]
        left_elbow_wrist_vector = self.joints[left_arm_joint_number[2]] - self.joints[left_arm_joint_number[1]]
        right_wrist_finger_vector = self.joints[right_arm_joint_number[3]] - self.joints[right_arm_joint_number[2]]
        left_wrist_finger_vector = self.joints[left_arm_joint_number[3]] - self.joints[left_arm_joint_number[2]]

        right_plane_normal_vec = np.cross(right_shoulder_elbow_vector, right_elbow_wrist_vector)
        left_plane_normal_vec = np.cross(left_shoulder_elbow_vector, left_elbow_wrist_vector)

        if right_plane_normal_vec[0] != 0 or right_plane_normal_vec[1] != 0 or right_plane_normal_vec[2] != 0:
            right_side_bent_degree = 90 - math.degrees(
                math.acos(np.dot(right_plane_normal_vec, right_wrist_finger_vector) / (
                        math.sqrt(np.dot(right_wrist_finger_vector, right_wrist_finger_vector)) * math.sqrt(
                    np.dot(right_plane_normal_vec, right_plane_normal_vec)))))
        else:
            right_side_bent_degree = 0
        if left_plane_normal_vec[0] != 0 or left_plane_normal_vec[1] != 0 or left_plane_normal_vec[2] == 0:
            left_side_bent_degree = 90 - math.degrees(
                math.acos(np.dot(left_plane_normal_vec, left_wrist_finger_vector) / (
                        math.sqrt(np.dot(left_wrist_finger_vector, left_wrist_finger_vector)) * math.sqrt(
                    np.dot(left_plane_normal_vec, left_plane_normal_vec)))))
        else:
            left_side_bent_degree = 0

        self.file.write("Right hand bending \n")
        self.file.write(str(right_side_bent_degree))
        self.file.write("\n")

        self.file.write("Left hand bending \n")
        self.file.write(str(left_side_bent_degree))
        self.file.write("\n")

    def wrist_torsion(self):
        # TODO: twist should be considered
        right_torsion = 0
        left_torsion = 0

        self.file.write("Right hand twist \n")
        self.file.write(str(right_torsion))
        self.file.write("\n")

        self.file.write("Left hand twist \n")
        self.file.write(str(left_torsion))
        self.file.write("\n")
