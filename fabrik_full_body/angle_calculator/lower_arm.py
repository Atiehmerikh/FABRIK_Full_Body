import math
import numpy as np
import fabrik_full_body.angle_calculator.body_part_numbering as bodyNum


class LowerArm:
    def __init__(self, joints,file):
        self.file = file
        self.joints = joints

    def lower_arm_degree(self):
        m_body_number = bodyNum.BodyPartNumber()
        right_arm_joint_numbers = m_body_number.right_arm()
        left_arm_joint_numbers = m_body_number.left_arm()

        right_shoulder_elbow_vector = self.joints[right_arm_joint_numbers[1]] - self.joints[right_arm_joint_numbers[0]]
        left_shoulder_elbow_vector = self.joints[left_arm_joint_numbers[1]] - self.joints[left_arm_joint_numbers[0]]

        right_elbow_wrist_vector = self.joints[right_arm_joint_numbers[2]] - self.joints[right_arm_joint_numbers[1]]
        left_elbow_wrist_vector = self.joints[left_arm_joint_numbers[2]] - self.joints[left_arm_joint_numbers[1]]

        # right and left arm degree in saggital plane
        right_degree = math.degrees(math.acos(np.dot(right_shoulder_elbow_vector, right_elbow_wrist_vector)) / (
                    (math.sqrt(np.dot(right_shoulder_elbow_vector, right_shoulder_elbow_vector))) * (
                math.sqrt(np.dot(right_elbow_wrist_vector, right_elbow_wrist_vector)))))
        left_degree = math.degrees(math.acos(np.dot(left_shoulder_elbow_vector, left_elbow_wrist_vector)) / (
                    (math.sqrt(np.dot(left_shoulder_elbow_vector, left_shoulder_elbow_vector))) * (
                math.sqrt(np.dot(left_elbow_wrist_vector, left_elbow_wrist_vector)))))

        self.file.write("Lower right arm flexion \n")
        self.file.write(str(right_degree))
        self.file.write("\n")

        self.file.write("Lower left arm flexion \n")
        self.file.write(str(left_degree))
        self.file.write("\n")
