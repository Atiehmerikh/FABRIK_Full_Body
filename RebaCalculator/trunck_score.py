from pycg3d.cg3d_point import CG3dPoint
from pycg3d.cg3d_vector import CG3dVector
import math
from pycg3d import utils


def trunk_score(joints):
    neck_index = [17]
    lower_body_index = [0, 9, 10]

    neck_point = CG3dPoint(joints[neck_index[0]][0], joints[neck_index[0]][1], joints[neck_index[0]][2])
    base_point = CG3dPoint(joints[lower_body_index[0]][0], joints[lower_body_index[0]][1],
                           joints[lower_body_index[0]][2])
    middle_legs_point = CG3dPoint((joints[lower_body_index[1]][0] + joints[lower_body_index[2]][0]) / 2,
                                  (joints[lower_body_index[1]][1] + joints[lower_body_index[2]][1]) / 2,
                                  (joints[lower_body_index[1]][2] + joints[lower_body_index[2]][2]) / 2)

    base_lower_point_vector = CG3dVector(middle_legs_point[0] - base_point[0], middle_legs_point[1] - base_point[1],
                                         middle_legs_point[2] - base_point[2])
    base_neck_vector = CG3dVector(neck_point[0] - base_point[0], neck_point[1] - base_point[1],
                                  neck_point[2] - base_point[2])

    # the degree between neck and trunc
    alpha = 180 - math.degrees(math.acos((base_lower_point_vector * base_neck_vector) / (
            utils.distance(neck_point, base_point) * utils.distance(middle_legs_point, base_point))))
    trunk_reba_score = 0
    if (alpha == 0):
        trunk_reba_score = trunk_reba_score + 1
    if (0 < alpha < 20):
        trunk_reba_score = trunk_reba_score + 2
    if (20 <= alpha < 60):
        trunk_reba_score = trunk_reba_score + 3
    if (60 <= alpha):
        trunk_reba_score = trunk_reba_score + 4

    return trunk_reba_score
