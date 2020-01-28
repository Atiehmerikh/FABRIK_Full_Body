from pycg3d.cg3d_point import CG3dPoint
from pycg3d.cg3d_vector import CG3dVector
import math
from pycg3d import utils


def leg_score(joints):
    right_leg_index = [9, 11, 12, 13]
    left_leg_index = [10, 14, 15, 16]

    right_hip_point = CG3dPoint(joints[right_leg_index[0]][0], joints[right_leg_index[0]][1],
                                joints[right_leg_index[0]][2])
    left_hip_point = CG3dPoint(joints[left_leg_index[0]][0], joints[left_leg_index[0]][1], joints[left_leg_index[0]][2])

    right_knee_point = CG3dPoint(joints[right_leg_index[1]][0], joints[right_leg_index[1]][1],
                                 joints[right_leg_index[1]][2])
    left_knee_point = CG3dPoint(joints[left_leg_index[1]][0], joints[left_leg_index[1]][1],
                                joints[left_leg_index[1]][2])

    right_wrist_point = CG3dPoint(joints[right_leg_index[2]][0], joints[right_leg_index[2]][1],
                                  joints[right_leg_index[2]][2])
    left_wrist_point = CG3dPoint(joints[left_leg_index[2]][0], joints[left_leg_index[2]][1],
                                 joints[left_leg_index[2]][2])

    right_knee_hip_vector = CG3dVector(right_hip_point[0] - right_knee_point[0],
                                       right_hip_point[1] - right_knee_point[1],
                                       right_hip_point[2] - right_knee_point[2])
    right_knee_wrist_vector = CG3dVector(right_wrist_point[0] - right_knee_point[0],
                                         right_wrist_point[1] - right_knee_point[1],
                                         right_wrist_point[2] - right_knee_point[2])

    left_knee_hip_vector = CG3dVector(left_hip_point[0] - left_knee_point[0],
                                      left_hip_point[1] - left_knee_point[1],
                                      left_hip_point[2] - left_knee_point[2])
    left_knee_wrist_vector = CG3dVector(left_wrist_point[0] - left_knee_point[0],
                                        left_wrist_point[1] - left_knee_point[1],
                                        left_wrist_point[2] - left_knee_point[2])

    # the degree between neck and trunc
    right_alpha = 180 - math.degrees(math.acos((right_knee_hip_vector * right_knee_wrist_vector) / (utils.distance(right_hip_point,right_knee_point)*utils.distance(right_knee_point,right_wrist_point))))
    left_alpha = 180 - math.degrees(math.acos((left_knee_hip_vector * left_knee_wrist_vector) / (utils.distance(left_hip_point,left_knee_point)*utils.distance(left_knee_point,left_wrist_point))))

    leg_reba_score = 0
    if(right_alpha != left_alpha):
        leg_reba_score = leg_reba_score+2
    if (right_alpha==0 or left_alpha==0):
        leg_reba_score = leg_reba_score + 1
    if (30<=right_alpha<60 or 30<=right_alpha<60 ):
        leg_reba_score = leg_reba_score + 1
    if (60<=right_alpha or 60<=right_alpha ):
        leg_reba_score = leg_reba_score + 2

    return leg_reba_score