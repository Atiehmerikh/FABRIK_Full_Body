import numpy as np
import math
import fabrik_full_body.angle_calculator.body_part_numbering as bodyNum


class Leg:
    def __init__(self, joints,file):
        self.joints = joints
        self.file = file

    def leg_degree(self):
        m_body_number = bodyNum.BodyPartNumber()
        right_leg_joint_numbers = m_body_number.right_leg()
        left_leg_joint_numbers = m_body_number.left_leg()
        right_knee_hip_vector = self.joints[right_leg_joint_numbers[0]] - self.joints[right_leg_joint_numbers[1]]
        right_ankle_knee_vector = self.joints[right_leg_joint_numbers[1]] - self.joints[right_leg_joint_numbers[2]]

        left_knee_hip_vector = self.joints[left_leg_joint_numbers[0]] - self.joints[left_leg_joint_numbers[1]]
        left_ankle_knee_vector = self.joints[left_leg_joint_numbers[1]] - self.joints[left_leg_joint_numbers[2]]

        # knee degree
        right_knee_degree = math.degrees(math.acos(np.dot(right_knee_hip_vector, right_ankle_knee_vector) / (
                (math.sqrt(np.dot(right_knee_hip_vector, right_knee_hip_vector))) * (
            math.sqrt(np.dot(right_ankle_knee_vector, right_ankle_knee_vector))))))

        left_knee_degree = math.degrees(math.acos(np.dot(left_knee_hip_vector, left_ankle_knee_vector) / (
                (math.sqrt(np.dot(left_knee_hip_vector, left_knee_hip_vector))) * (
            math.sqrt(np.dot(left_ankle_knee_vector, left_ankle_knee_vector))))))

        self.file.write("Upper right leg flexion \n")
        self.file.write(str(right_knee_degree))
        self.file.write("\n")

        self.file.write("Upper left leg flexion \n")
        self.file.write(str(left_knee_degree))
        self.file.write("\n")

        # we consider both feet are fixed on ground
        self.file.write("Upper right leg abduction \n")
        self.file.write(str(0))
        self.file.write("\n")

        self.file.write("Upper left leg abduction \n")
        self.file.write(str(0))
        self.file.write("\n")

        self.file.write("Lower right leg flexion \n")
        self.file.write(str(0))
        self.file.write("\n")

        self.file.write("Lower left leg flexion \n")
        self.file.write(str(0))
        self.file.write("\n")

        self.file.write("Right foot flexion \n")
        self.file.write(str(0))
        self.file.write("\n")

        self.file.write("Left foot flexion \n")
        self.file.write(str(0))
        self.file.write("\n")

        self.file.write("Right foot bending \n")
        self.file.write(str(0))
        self.file.write("\n")

        self.file.write("Left foot bending \n")
        self.file.write(str(0))
        self.file.write("\n")

        # rotation should be written
        self.file.write("Right foot twist \n")
        self.file.write(str(0))
        self.file.write("\n")

        self.file.write("Left foot twist \n")
        self.file.write(str(0))
        self.file.write("\n")

        self.file.write("Upper right leg rotation \n")
        self.file.write(str(0))
        self.file.write("\n")

        self.file.write("Upper left leg rotation \n")
        self.file.write(str(0))
        self.file.write("\n")