from pycg3d.cg3d_point import CG3dPoint
from pycg3d.cg3d_vector import CG3dVector
import math
from pycg3d import utils


def upper_arm_score(joints,file):
    right_shoulder_index = 1
    right_elbow_index = 2
    left_shoulder_index = 5
    left_elbow_index = 6
    neck_index = 17
    head_index = 18
    base_index = 0

    neck_point = CG3dPoint(joints[neck_index][0], joints[neck_index][1], joints[neck_index][2])
    head_point = CG3dPoint(joints[head_index][0], joints[head_index][1], joints[head_index][2])

    base_point = CG3dPoint(joints[base_index][0], joints[base_index][1],
                           joints[base_index][2])
    right_shoulder_point = CG3dPoint(joints[right_shoulder_index][0], joints[right_shoulder_index][1],
                                     joints[right_shoulder_index][2])
    left_shoulder_point = CG3dPoint(joints[left_shoulder_index][0], joints[left_shoulder_index][1],
                                    joints[left_shoulder_index][2])
    right_elbow_point = CG3dPoint(joints[right_elbow_index][0], joints[right_elbow_index][1],
                                  joints[right_elbow_index][2])
    left_elbow_point = CG3dPoint(joints[left_elbow_index][0], joints[left_elbow_index][1],
                                 joints[left_elbow_index][2])

    right_shoulder_elbow_vector = CG3dVector(right_elbow_point[0] - right_shoulder_point[0],
                                             right_elbow_point[1] - right_shoulder_point[1],
                                             right_elbow_point[2] - right_shoulder_point[2])
    left_shoulder_elbow_vector = CG3dVector(left_elbow_point[0] - left_shoulder_point[0],
                                            left_elbow_point[1] - left_shoulder_point[1],
                                            left_elbow_point[2] - left_shoulder_point[2])
    neck_base_vector = CG3dVector(base_point[0] - neck_point[0],
                                  base_point[1] - neck_point[1],
                                  base_point[2] - neck_point[2])

    right_shoulder_neck_vector = CG3dVector(neck_point[0] - right_shoulder_point[0],
                                            neck_point[1] - right_shoulder_point[1],
                                            neck_point[2] - right_shoulder_point[2])
    left_shoulder_neck_vector = CG3dVector(neck_point[0] - left_shoulder_point[0],
                                           neck_point[1] - left_shoulder_point[1],
                                           neck_point[2] - left_shoulder_point[2])

    neck_head_vector = CG3dVector(head_point[0] - neck_point[0],
                                  head_point[1] - neck_point[1],
                                  head_point[2] - neck_point[2])

    # flexion or extension degree
    right_flexion = math.degrees(math.acos((neck_base_vector * right_shoulder_elbow_vector) / (
            utils.distance(neck_point, base_point) * utils.distance(right_shoulder_point, right_elbow_point))))
    left_flexion = math.degrees(math.acos((neck_base_vector * left_shoulder_elbow_vector) / (
            utils.distance(neck_point, base_point) * utils.distance(left_shoulder_point, left_elbow_point))))
    # adduction or abduction
    right_side_abduction = math.degrees(math.acos((right_shoulder_neck_vector * right_shoulder_elbow_vector) / (
            utils.distance(neck_point, right_shoulder_point) * utils.distance(right_shoulder_point,
                                                                              right_elbow_point)))) - 90
    left_side_abduction = math.degrees(math.acos((left_shoulder_neck_vector * left_shoulder_elbow_vector) / (
            utils.distance(neck_point, left_shoulder_point) * utils.distance(left_shoulder_point,
                                                                             left_elbow_point)))) - 90
    right_shoulder_rise_degree = math.degrees(math.acos((right_shoulder_neck_vector * neck_head_vector) / (
            utils.distance(neck_point, head_point) * utils.distance(right_shoulder_point,
                                                                    neck_point))))
    left_shoulder_rise_degree = math.degrees(math.acos((left_shoulder_neck_vector * neck_head_vector) / (
            utils.distance(neck_point, head_point) * utils.distance(left_shoulder_point,
                                                                    neck_point))))
    upper_arm_reba_score = 0
    if right_flexion >= left_flexion:
        if 0 <= right_flexion < 20:
            upper_arm_reba_score = upper_arm_reba_score + 1
        if 20 <= right_flexion < 45:
            upper_arm_reba_score = upper_arm_reba_score + 2
        if 45 <= right_flexion < 90:
            upper_arm_reba_score = upper_arm_reba_score + 3
        if 90 <= right_flexion:
            upper_arm_reba_score = upper_arm_reba_score + 4
        file.write(str(right_flexion)+ ',')
    if right_flexion < left_flexion:
        if 0 <= left_flexion < 20:
            upper_arm_reba_score = upper_arm_reba_score + 1
        if 20 <= left_flexion < 45:
            upper_arm_reba_score = upper_arm_reba_score + 2
        if 45 <= left_flexion < 90:
            upper_arm_reba_score = upper_arm_reba_score + 3
        if 90 <= left_flexion:
            upper_arm_reba_score = upper_arm_reba_score + 4
        file.write(str(left_flexion)+ ',')
    if right_side_abduction != 90 or left_side_abduction != 90:
        upper_arm_reba_score = upper_arm_reba_score + 1
    file.write(str(right_side_abduction)+ ',')
    if right_shoulder_rise_degree != 90 or left_shoulder_rise_degree != 90:
        upper_arm_reba_score = upper_arm_reba_score + 1
    file.write(str(right_shoulder_rise_degree)+ ',')

    return upper_arm_reba_score
