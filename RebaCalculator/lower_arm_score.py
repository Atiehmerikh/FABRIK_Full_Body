from pycg3d.cg3d_point import CG3dPoint
from pycg3d.cg3d_vector import CG3dVector
import math
from pycg3d import utils


def lower_arm_score(joints):
    right_arm_index = [0, 1, 2, 3]
    left_arm_index = [0, 5, 6, 7]
    neck_index = [17]

    right_shoulder_point = CG3dPoint(joints[right_arm_index[1]][0], joints[right_arm_index[1]][1],
                                     joints[right_arm_index[1]][2])
    left_shoulder_point = CG3dPoint(joints[left_arm_index[1]][0], joints[left_arm_index[1]][1],
                                    joints[left_arm_index[1]][2])
    right_elbow_point = CG3dPoint(joints[right_arm_index[2]][0], joints[right_arm_index[2]][1],
                                  joints[right_arm_index[2]][2])
    left_elbow_point = CG3dPoint(joints[left_arm_index[2]][0], joints[left_arm_index[2]][1],
                                 joints[left_arm_index[2]][2])
    right_wrist_point = CG3dPoint(joints[right_arm_index[3]][0], joints[right_arm_index[3]][1],
                                  joints[right_arm_index[3]][2])
    left_wrist_point = CG3dPoint(joints[left_arm_index[3]][0], joints[left_arm_index[3]][1],
                                 joints[left_arm_index[3]][2])
    neck_point = CG3dPoint(joints[neck_index[0]][0], joints[neck_index[0]][1], joints[neck_index[0]][2])

    right_elbow_shoulder_vector = CG3dVector(right_shoulder_point[0] - right_elbow_point[0],
                                             right_shoulder_point[1] - right_elbow_point[1],
                                             right_shoulder_point[2] - right_elbow_point[2])
    left_elbow_shoulder_vector = CG3dVector(left_shoulder_point[0] - left_elbow_point[0],
                                            left_shoulder_point[1] - left_elbow_point[1],
                                            left_shoulder_point[2] - left_elbow_point[2])

    right_wrist_elbow_vector = CG3dVector(right_wrist_point[0] - right_elbow_point[0],
                                          right_wrist_point[1] - right_elbow_point[1],
                                          right_wrist_point[2] - right_elbow_point[2])
    left_wrist_elbow_vector = CG3dVector(left_wrist_point[0] - left_elbow_point[0],
                                         left_wrist_point[1] - left_elbow_point[1],
                                         left_wrist_point[2] - left_elbow_point[2])
    right_shoulder_neck_vector = CG3dVector(neck_point[0] - right_shoulder_point[0],
                                            neck_point[1] - right_shoulder_point[1],
                                            neck_point[2] - right_shoulder_point[2])
    left_shoulder_neck_vector = CG3dVector(neck_point[0] - left_shoulder_point[0],
                                           neck_point[1] - left_shoulder_point[1],
                                           neck_point[2] - left_shoulder_point[2])

    # right and left arm degree in saggital plane
    right_alpha_s = math.degrees(math.acos((right_elbow_shoulder_vector * right_wrist_elbow_vector) / (
            utils.distance(right_shoulder_point, right_elbow_point) * utils.distance(right_wrist_point,
                                                                                     right_elbow_point))))
    left_alpha_s = math.degrees(math.acos((left_elbow_shoulder_vector * left_wrist_elbow_vector) / (
            utils.distance(left_shoulder_point, left_elbow_point) * utils.distance(left_wrist_point,
                                                                                   left_elbow_point))))

    # right and left arm degree in transversal plane

    right_alpha_t = math.degrees(math.acos((right_shoulder_neck_vector * right_wrist_elbow_vector) / (
            utils.distance(right_shoulder_point, neck_point) * utils.distance(right_wrist_point,
                                                                              right_elbow_point))))
    left_alpha_t = 90 - math.degrees(math.acos((left_shoulder_neck_vector * left_wrist_elbow_vector) / (
            utils.distance(left_shoulder_point, neck_point) * utils.distance(left_wrist_point,
                                                                             left_elbow_point))))

    lower_arm_reba_score_right = 0
    lower_arm_reba_score_left = 0

    if (90 <= right_alpha_s < 100):
        lower_arm_reba_score_right = lower_arm_reba_score_right + 1
    if (right_alpha_s < 90 or 100 <= right_alpha_s):
        lower_arm_reba_score_right = lower_arm_reba_score_right + 2

    if (90 <= left_alpha_s < 100):
        lower_arm_reba_score_left = lower_arm_reba_score_left + 1
    if (left_alpha_s < 90 or 100 <= left_alpha_s):
        lower_arm_reba_score_left = lower_arm_reba_score_left + 2
    # if(right_alpha_t!=0 or left_alpha_t!=0):
    #     lower_arm_reba_score = lower_arm_reba_score+1

    return max(lower_arm_reba_score_left,lower_arm_reba_score_right)
