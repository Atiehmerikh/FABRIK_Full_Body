from pycg3d.cg3d_point import CG3dPoint
from pycg3d.cg3d_vector import CG3dVector
import math
from pycg3d import utils


def upper_arm_score(joints):
    right_upper_arm_index = [0, 1, 2]
    left_upper_arm_index = [0, 5, 6]
    neck_index = [17]
    head_index = [18]

    neck_point = CG3dPoint(joints[neck_index[0]][0], joints[neck_index[0]][1], joints[neck_index[0]][2])
    base_point = CG3dPoint(joints[right_upper_arm_index[0]][0], joints[right_upper_arm_index[0]][1],
                           joints[right_upper_arm_index[0]][2])
    right_shoulder_point = CG3dPoint(joints[right_upper_arm_index[1]][0], joints[right_upper_arm_index[1]][1],
                                     joints[right_upper_arm_index[1]][2])
    left_shoulder_point = CG3dPoint(joints[left_upper_arm_index[1]][0], joints[left_upper_arm_index[1]][1],
                                    joints[left_upper_arm_index[1]][2])
    right_elbow_point = CG3dPoint(joints[right_upper_arm_index[2]][0], joints[right_upper_arm_index[2]][1],
                                  joints[right_upper_arm_index[2]][2])
    left_elbow_point = CG3dPoint(joints[left_upper_arm_index[2]][0], joints[left_upper_arm_index[2]][1],
                                 joints[left_upper_arm_index[2]][2])
    head_point = CG3dPoint(joints[head_index[0]][0], joints[head_index[0]][1],
                           joints[head_index[0]][2])

    right_shoulder_elbow_vector = CG3dVector(right_elbow_point[0] - right_shoulder_point[0],
                                             right_elbow_point[1] - right_shoulder_point[1],
                                             right_elbow_point[2] - right_shoulder_point[2])
    left_shoulder_elbow_vector = CG3dVector(left_elbow_point[0] - left_shoulder_point[0],
                                            left_elbow_point[1] - left_shoulder_point[1],
                                            left_elbow_point[2] - left_shoulder_point[2])
    neck_base_vector = CG3dVector(base_point[0] - neck_point[0],
                                  base_point[1] - neck_point[1],
                                  base_point[2] - neck_point[2])
    neck_head_vector = CG3dVector(head_point[0] - neck_point[0],
                                  head_point[1] - neck_point[1],
                                  head_point[2] - neck_point[2])
    right_shoulder_neck_vector = CG3dVector(neck_point[0] - right_shoulder_point[0],
                                            neck_point[1] - right_shoulder_point[1],
                                            neck_point[2] - right_shoulder_point[2])
    left_shoulder_neck_vector = CG3dVector(neck_point[0] - left_shoulder_point[0],
                                           neck_point[1] - left_shoulder_point[1],
                                           neck_point[2] - left_shoulder_point[2])

    # saggital degree
    right_alpha_s = math.degrees(math.acos((neck_base_vector * right_shoulder_elbow_vector) / (
            utils.distance(neck_point, base_point) * utils.distance(right_shoulder_point, right_elbow_point))))
    left_alpha_s = math.degrees(math.acos((neck_base_vector * left_shoulder_elbow_vector) / (
            utils.distance(neck_point, base_point) * utils.distance(left_shoulder_point, left_elbow_point))))
    # coronal degree
    right_alpha_c = math.degrees(math.acos((right_shoulder_neck_vector * right_shoulder_elbow_vector) / (
            utils.distance(neck_point, right_shoulder_point) * utils.distance(right_shoulder_point,
                                                                              right_elbow_point)))) - 90
    left_alpha_c = math.degrees(math.acos((left_shoulder_neck_vector * left_shoulder_elbow_vector) / (
            utils.distance(neck_point, left_shoulder_point) * utils.distance(left_shoulder_point,
                                                                             left_elbow_point)))) - 90
    right_shoulder_rise_degree = math.degrees(math.acos((right_shoulder_neck_vector * neck_head_vector) / (
            utils.distance(neck_point, head_point) * utils.distance(right_shoulder_point,
                                                                    neck_point))))
    left_shoulder_rise_degree = math.degrees(math.acos((left_shoulder_neck_vector * neck_head_vector) / (
            utils.distance(neck_point, head_point) * utils.distance(left_shoulder_point,
                                                                    neck_point))))
    upper_arm_reba_score_right = 0
    upper_arm_reba_score_left = 0

    if (0 <= right_alpha_s < 20):
        upper_arm_reba_score_right = upper_arm_reba_score_right + 1
    if (20 <= right_alpha_s < 45):
        upper_arm_reba_score_right = upper_arm_reba_score_right + 2
    if (45 <= right_alpha_s < 90 ):
        upper_arm_reba_score_right = upper_arm_reba_score_right + 3
    if (90 <= right_alpha_s):
        upper_arm_reba_score_right = upper_arm_reba_score_right + 4


    if ( 0 <= left_alpha_s < 20):
        upper_arm_reba_score_left = upper_arm_reba_score_left + 1
    if (20 <= left_alpha_s < 45 ):
        upper_arm_reba_score_left = upper_arm_reba_score_left + 2
    if ( 45 <= left_alpha_s < 90):
        upper_arm_reba_score_left = upper_arm_reba_score_left + 3
    if (90 <= left_alpha_s):
        upper_arm_reba_score_left = upper_arm_reba_score_left + 4

    # if (right_alpha_c > 0 or left_alpha_c > 0):
    #     upper_arm_reba_score_left = upper_arm_reba_score_left + 1
    #     upper_arm_reba_score_right = upper_arm_reba_score_left + 1
    #
    # if (right_shoulder_rise_degree < 90 or left_shoulder_rise_degree < 90):
    #     upper_arm_reba_score_left = upper_arm_reba_score_left + 1
    #     upper_arm_reba_score_right = upper_arm_reba_score_left + 1


    return max(upper_arm_reba_score_right,upper_arm_reba_score_left)