from pycg3d.cg3d_point import CG3dPoint
from pycg3d.cg3d_vector import CG3dVector
import math
from pycg3d import utils


def lower_arm_score(joints,file):
    right_shoulder_index = 1
    left_shoulder_index = 5
    right_elbow_index = 2
    left_elbow_index = 6
    right_wrist_index = 3
    left_wrist_index = 7
    neck_index = 17

    right_shoulder_point = CG3dPoint(joints[right_shoulder_index][0], joints[right_shoulder_index][1],
                                     joints[right_shoulder_index][2])
    left_shoulder_point = CG3dPoint(joints[left_shoulder_index][0], joints[left_shoulder_index][1],
                                    joints[left_shoulder_index][2])
    right_elbow_point = CG3dPoint(joints[right_elbow_index][0], joints[right_elbow_index][1],
                                  joints[right_elbow_index][2])
    left_elbow_point = CG3dPoint(joints[left_elbow_index][0], joints[left_elbow_index][1],
                                 joints[left_elbow_index][2])
    right_wrist_point = CG3dPoint(joints[right_wrist_index][0], joints[right_wrist_index][1],
                                  joints[right_wrist_index][2])
    left_wrist_point = CG3dPoint(joints[left_wrist_index][0], joints[left_wrist_index][1],
                                 joints[left_wrist_index][2])

    right_shoulder_elbow_vector = CG3dVector(right_elbow_point[0] - right_shoulder_point[0],
                                             right_elbow_point[1] - right_shoulder_point[1],
                                             right_elbow_point[2] - right_shoulder_point[2])
    left_shoulder_elbow_vector = CG3dVector(left_elbow_point[0] - left_shoulder_point[0],
                                            left_elbow_point[1] - left_shoulder_point[1],
                                            left_elbow_point[2] - left_shoulder_point[2])

    right_elbow_wrist_vector = CG3dVector(right_wrist_point[0] - right_elbow_point[0],
                                          right_wrist_point[1] - right_elbow_point[1],
                                          right_wrist_point[2] - right_elbow_point[2])
    left_elbow_wrist_vector = CG3dVector(left_wrist_point[0] - left_elbow_point[0],
                                         left_wrist_point[1] - left_elbow_point[1],
                                         left_wrist_point[2] - left_elbow_point[2])

    # right and left arm degree in saggital plane
    right_flexion = math.degrees(math.acos((right_shoulder_elbow_vector * right_elbow_wrist_vector) / (
            utils.distance(right_shoulder_point, right_elbow_point) * utils.distance(right_wrist_point,
                                                                                     right_elbow_point))))
    left_flexion = math.degrees(math.acos((left_shoulder_elbow_vector * left_elbow_wrist_vector) / (
            utils.distance(left_shoulder_point, left_elbow_point) * utils.distance(left_wrist_point,
                                                                                   left_elbow_point))))
    lower_arm_reba_score = 0
    if right_flexion >= left_flexion:
        if 0 <= right_flexion < 60:
            lower_arm_reba_score = lower_arm_reba_score + 2
        if 60 <= right_flexion < 100:
            lower_arm_reba_score = lower_arm_reba_score + 1
        if 100 <= right_flexion:
            lower_arm_reba_score = lower_arm_reba_score + 1
        file.write(str(right_flexion)+ ',')
    if right_flexion < left_flexion:
        if 0 <= left_flexion < 60:
            lower_arm_reba_score = lower_arm_reba_score + 2
        if 60 <= left_flexion < 100:
            lower_arm_reba_score = lower_arm_reba_score + 1
        if 100 <= left_flexion:
            lower_arm_reba_score = lower_arm_reba_score + 1
        file.write(str(left_flexion)+ ',')
    return lower_arm_reba_score
