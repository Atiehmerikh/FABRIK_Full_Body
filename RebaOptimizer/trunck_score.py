from pycg3d.cg3d_point import CG3dPoint
from pycg3d.cg3d_vector import CG3dVector
import math
from pycg3d import utils


def trunk_score(joints,file):
    trunk_reba_score = 0

    neck_index = 17
    base_index = 0
    left_hip_index = 10
    right_hip_index = 9
    left_ankle_index = 15

    neck_point = CG3dPoint(joints[neck_index][0], joints[neck_index][1], joints[neck_index][2])
    base_point = CG3dPoint(joints[base_index][0], joints[base_index][1], joints[base_index][2])
    left_hip_point = CG3dPoint(joints[left_hip_index][0], joints[left_hip_index][1], joints[left_hip_index][2])
    right_hip_point = CG3dPoint(joints[right_hip_index][0], joints[right_hip_index][1], joints[right_hip_index][2])
    left_ankle_point = CG3dPoint(joints[left_ankle_index][0], joints[left_ankle_index][1], joints[left_ankle_index][2])

    base_neck_vector = CG3dVector(neck_point[0] - base_point[0], neck_point[1] - base_point[1],
                                  neck_point[2] - base_point[2])
    ankle_hip_vector = CG3dVector(left_hip_point[0] - left_ankle_point[0], left_hip_point[1] - left_ankle_point[1],
                                  left_hip_point[2] - left_ankle_point[2])
    right_left_hip_vector = CG3dVector(right_hip_point[0] - left_hip_point[0], right_hip_point[1] - left_hip_point[1],
                                right_hip_point[2] - left_hip_point[2])

    # trunk flexion or extension
    trunk_flexion = math.degrees(math.acos((base_neck_vector * ankle_hip_vector) / (
            utils.distance(neck_point, base_point) * utils.distance(left_ankle_point, left_hip_point))))
    if trunk_flexion == 0:
        trunk_reba_score = trunk_reba_score + 1
    if 0 < trunk_flexion < 20:
        trunk_reba_score = trunk_reba_score + 2
    if 20 <= trunk_flexion < 60:
        trunk_reba_score = trunk_reba_score + 3
    if 60 <= trunk_flexion:
        trunk_reba_score = trunk_reba_score + 4

    # trunk side bending:
    trunk_side = math.degrees(math.acos((base_neck_vector * right_left_hip_vector) / (
            utils.distance(neck_point, base_point) * utils.distance(right_hip_point, left_hip_point))))

    if trunk_side != 90:
        trunk_reba_score = trunk_reba_score + 1

    file.write(str(trunk_flexion)+ ',')
    file.write(str(trunk_side)+ ',')

    return trunk_reba_score
