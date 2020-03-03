from pycg3d.cg3d_point import CG3dPoint
from pycg3d.cg3d_vector import CG3dVector
import math
from pycg3d import utils


def neck_score(joints,file):
    neck_reba_score = 0

    neck_index = 17
    head_index = 18
    core_index = 0
    toe_index = 13
    ankle_index = 12
    right_shoulder_index = 1

    neck_point = CG3dPoint(joints[neck_index][0], joints[neck_index][1], joints[neck_index][2])
    base_point = CG3dPoint(joints[core_index][0], joints[core_index][1], joints[core_index][2])
    ankle_point = CG3dPoint(joints[ankle_index][0], joints[ankle_index][1], joints[ankle_index][2])
    head_point = CG3dPoint(joints[head_index][0], joints[head_index][1], joints[head_index][2])
    toe_point = CG3dPoint(joints[toe_index][0], joints[toe_index][1], joints[toe_index][2])
    shoulder_point = CG3dPoint(joints[right_shoulder_index][0], joints[right_shoulder_index][1],
                               joints[right_shoulder_index][2])

    neck_head_vector = CG3dVector(head_point[0] - neck_point[0], head_point[1] - neck_point[1],
                                  head_point[2] - neck_point[2])
    base_neck_vector = CG3dVector(neck_point[0] - base_point[0], neck_point[1] - base_point[1],
                                  neck_point[2] - base_point[2])

    ankle_toe_vector = CG3dVector(toe_point[0] - ankle_point[0], toe_point[1] - ankle_point[1],
                                  toe_point[2] - ankle_point[2])
    neck_shoulder_vector = CG3dVector(shoulder_point[0] - neck_point[0], shoulder_point[1] - neck_point[1],
                                      shoulder_point[2] - neck_point[2])

    # neck extension
    flex_or_ext = math.degrees(math.acos((ankle_toe_vector * base_neck_vector) / (
            utils.distance(neck_point, head_point) * utils.distance(toe_point, base_point))))
    if flex_or_ext > 90:
        # extension of neck
        neck_flexion = -(flex_or_ext-90)
        neck_reba_score = neck_reba_score + 2
    # neck flexion
    else:
        neck_flexion = math.degrees(math.acos((neck_head_vector * base_neck_vector) / (
                utils.distance(neck_point, head_point) * utils.distance(neck_point, base_point))))
        if 0 <= neck_flexion < 20:
            neck_reba_score = neck_reba_score + 1
        if 20 <= neck_flexion:
            neck_reba_score = neck_reba_score + 2

    # neck_bending:
    neck_bending = math.degrees(math.acos((neck_head_vector * neck_shoulder_vector) / (
            utils.distance(neck_point, head_point) * utils.distance(neck_point, shoulder_point))))
    if neck_bending != 90:
        neck_reba_score = neck_reba_score + 1
    file.write(str(neck_flexion)+ ',')
    file.write(str(neck_bending)+ ',')
    return neck_reba_score
