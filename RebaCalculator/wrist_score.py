from pycg3d.cg3d_point import CG3dPoint
from pycg3d.cg3d_vector import CG3dVector
import math
from pycg3d import utils


def wrist_score(joints):
    right_arm_index = [1, 2, 3, 4]
    left_arm_index = [5, 6, 7, 8]

    right_elbow_point = CG3dPoint(joints[right_arm_index[1]][0], joints[right_arm_index[1]][1],
                                  joints[right_arm_index[1]][2])
    left_elbow_point = CG3dPoint(joints[left_arm_index[1]][0], joints[left_arm_index[1]][1],
                                 joints[left_arm_index[1]][2])
    right_wrist_point = CG3dPoint(joints[right_arm_index[2]][0], joints[right_arm_index[2]][1],
                                  joints[right_arm_index[2]][2])
    left_wrist_point = CG3dPoint(joints[left_arm_index[2]][0], joints[left_arm_index[2]][1],
                                 joints[left_arm_index[2]][2])
    right_finger_point = CG3dPoint(joints[right_arm_index[3]][0], joints[right_arm_index[3]][1],
                                   joints[right_arm_index[3]][2])
    left_finger_point = CG3dPoint(joints[left_arm_index[3]][0], joints[left_arm_index[3]][1],
                                  joints[left_arm_index[3]][2])

    right_wrist_elbow_vector = CG3dVector(right_wrist_point[0] - right_elbow_point[0],
                                          right_wrist_point[1] - right_elbow_point[1],
                                          right_wrist_point[2] - right_elbow_point[2])
    left_wrist_elbow_vector = CG3dVector(left_wrist_point[0] - left_elbow_point[0],
                                         left_wrist_point[1] - left_elbow_point[1],
                                         left_wrist_point[2] - left_elbow_point[2])

    right_finger_wrist_vector = CG3dVector(right_finger_point[0] - right_wrist_point[0],
                                           right_finger_point[1] - right_wrist_point[1],
                                           right_finger_point[2] - right_wrist_point[2])
    left_finger_wrist_vector = CG3dVector(left_finger_point[0] - left_wrist_point[0],
                                          left_finger_point[1] - left_wrist_point[1],
                                          left_finger_point[2] - left_wrist_point[2])
    right_wrist_alpha = math.degrees(
        math.acos(float("{0:.2f}".format((right_finger_wrist_vector * right_wrist_elbow_vector) / (
                utils.distance(right_elbow_point, right_wrist_point) * utils.distance(right_wrist_point,
                                                                                      right_finger_point))))))

    left_wrist_alpha = math.degrees(
        math.acos(float("{0:.2f}".format((left_finger_wrist_vector * left_wrist_elbow_vector) / (
                utils.distance(left_elbow_point, left_wrist_point) * utils.distance(left_wrist_point,
                                                                                      left_finger_point))))))
    wrist_reba_score_right =0
    if ( right_wrist_alpha ==0):
        wrist_reba_score_right = wrist_reba_score_right + 1
    if(0<right_wrist_alpha<15):
        wrist_reba_score_right=wrist_reba_score_right+2
    if (15 < right_wrist_alpha  ):
        wrist_reba_score_right = wrist_reba_score_right + 3

    wrist_reba_score_left = 0
    if (left_wrist_alpha == 0):
        wrist_reba_score_left = wrist_reba_score_left + 1
    if (0 < left_wrist_alpha < 15):
        wrist_reba_score_left = wrist_reba_score_left + 2
    if (15 < left_wrist_alpha):
        wrist_reba_score_left = wrist_reba_score_left + 3

    return max(wrist_reba_score_left,wrist_reba_score_right)
