from pycg3d.cg3d_point import CG3dPoint
from pycg3d.cg3d_vector import CG3dVector
import math
from pycg3d import utils


def neck_score(joints):
    neck_head_index = [17, 18]
    neck_core_index = [17, 0]

    neck_point = CG3dPoint(joints[neck_head_index[0]][0], joints[neck_head_index[0]][1], joints[neck_head_index[0]][2])
    base_point = CG3dPoint(joints[neck_core_index[1]][0], joints[neck_core_index[1]][1], joints[neck_core_index[1]][2])
    head_point = CG3dPoint(joints[neck_head_index[1]][0], joints[neck_head_index[1]][1], joints[neck_head_index[1]][2])

    neck_head_vector = CG3dVector(head_point[0] - neck_point[0], head_point[1] - neck_point[1],
                                  head_point[2] - neck_point[2])
    neck_base_vector = CG3dVector(base_point[0] - neck_point[0], base_point[1] - neck_point[1],
                                  base_point[2] - neck_point[2])

    # the degree between neck and trunc
    alpha = 180 - math.degrees(math.acos((neck_head_vector * neck_base_vector) / (
            utils.distance(neck_point, head_point) * utils.distance(neck_point, base_point))))
    neck_reba_score = 0
    if (0 <= alpha < 20):
        neck_reba_score = neck_reba_score + 1
    if (20 <= alpha):
        neck_reba_score = neck_reba_score + 2

    return neck_reba_score