from pycg3d.cg3d_point import CG3dPoint
from pycg3d.cg3d_vector import CG3dVector
import math
from pycg3d import utils


def wrist_score(joints,file):
    right_wrist_index = 3
    right_finger_index = 4
    left_wrist_index = 7
    left_finger_index = 8
    right_elbow_index = 2
    left_elbow_index = 6

    right_wrist_point = CG3dPoint(joints[right_wrist_index][0], joints[right_wrist_index][1],
                                  joints[right_wrist_index][2])
    left_wrist_point = CG3dPoint(joints[left_wrist_index][0], joints[left_wrist_index][1],
                                 joints[left_wrist_index][2])
    right_finger_point = CG3dPoint(joints[right_finger_index][0], joints[right_finger_index][1],
                                   joints[right_finger_index][2])
    left_finger_point = CG3dPoint(joints[left_finger_index][0], joints[left_finger_index][1],
                                  joints[left_finger_index][2])
    right_elbow_point = CG3dPoint(joints[right_elbow_index][0], joints[right_elbow_index][1],
                                  joints[right_elbow_index][2])
    left_elbow_point = CG3dPoint(joints[left_elbow_index][0], joints[left_elbow_index][1],
                                 joints[left_elbow_index][2])

    right_elbow_wrist_vector = CG3dVector(right_wrist_point[0] - right_elbow_point[0],
                                          right_wrist_point[1] - right_elbow_point[1],
                                          right_wrist_point[2] - right_elbow_point[2])
    left_elbow_wrist_vector = CG3dVector(left_wrist_point[0] - left_elbow_point[0],
                                         left_wrist_point[1] - left_elbow_point[1],
                                         left_wrist_point[2] - left_elbow_point[2])

    right_wrist_finger_vector = CG3dVector(right_finger_point[0] - right_wrist_point[0],
                                           right_finger_point[1] - right_wrist_point[1],
                                           right_finger_point[2] - right_wrist_point[2])
    left_wrist_finger_vector = CG3dVector(left_finger_point[0] - left_wrist_point[0],
                                          left_finger_point[1] - left_wrist_point[1],
                                          left_finger_point[2] - left_wrist_point[2])
    right_wrist = math.degrees(
        math.acos((right_elbow_wrist_vector * right_wrist_finger_vector) / (
                utils.distance(right_elbow_point, right_wrist_point) * utils.distance(right_wrist_point,
                                                                                      right_finger_point))))
    left_wrist = math.degrees(
        math.acos((left_elbow_wrist_vector * left_wrist_finger_vector) / (
                utils.distance(left_elbow_point, left_wrist_point) * utils.distance(left_wrist_point,
                                                                                    left_finger_point))))
    wrist_reba_score = 0
    if right_wrist >= left_wrist:
        if 0 <= right_wrist < 15:
            wrist_reba_score = wrist_reba_score + 1
        if 15 <= right_wrist:
            wrist_reba_score = wrist_reba_score + 2
        file.write(str(right_wrist)+ ',')
    if right_wrist < left_wrist:
        if 0 <= left_wrist < 15:
            wrist_reba_score = wrist_reba_score + 1
        if 15 <= left_wrist:
            wrist_reba_score = wrist_reba_score + 2
        file.write(str(left_wrist)+ ',')
    return wrist_reba_score
