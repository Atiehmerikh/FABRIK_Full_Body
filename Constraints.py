# import math
# from pycg3d.cg3d_point import CG3dPoint
# from pycg3d.cg3d_vector import CG3dVector
# from pycg3d import utils
#
#
# class constraints:
#
#     def __init__(self,joints, body_index, i, theta, length, constraint_type,orientation):
#         self.joints = joints
#         self.body_index = body_index
#         self.i = i
#         self.pi = 22 / 7
#         self.theta = (self.pi / 180) * theta
#         self.length = length
#         self.constraint_type = constraint_type
#         #the orientation of joint
#         theta = math.acos(orientation[0])*2
#         self.si = theta
#
#     def rotational_constraint(self):
#         p_i = CG3dPoint(self.joints[self.body_index[self.i]][0], self.joints[self.body_index[self.i]][1], self.joints[self.body_index[self.i]][2])
#         p_before_i = CG3dPoint(self.joints[self.body_index[self.i - 1]][0], self.joints[self.body_index[self.i - 1]][1], self.joints[self.body_index[self.i - 1]][2])
#         p_nextt_i = CG3dPoint(self.joints[self.body_index[self.i + 1]][0], self.joints[self.body_index[self.i + 1]][1], self.joints[self.body_index[self.i + 1]][2])
#
#         p_nextt_i_first_pose = CG3dPoint(self.length[self.body_index[self.i + 1]][0], self.length[self.body_index[self.i + 1]][1],
#
#                                          self.length[self.body_index[self.i + 1]][2])
#
#         v_i_nextt = CG3dVector(p_nextt_i[0] - p_i[0], p_nextt_i[1] - p_i[1], p_nextt_i[2] - p_i[2])
#         v_before_i = CG3dVector(p_i[0] - p_before_i[0], p_i[1] - p_before_i[1], p_i[2] - p_before_i[2])
#
#         if (self.constraint_type == "hinge"):
#             # defining a line l_1 in paper:
#             l_i_nextt_firstpose = utils.distance(p_i, p_nextt_i_first_pose)
#             v_i_nextt_firstPose = CG3dVector((-p_i[0] + p_nextt_i_first_pose[0]) / l_i_nextt_firstpose,
#                                              (-p_i[1] + p_nextt_i_first_pose[1]) / l_i_nextt_firstpose,
#                                              (-p_i[2] + p_nextt_i_first_pose[2]) / l_i_nextt_firstpose)
#             l_i_firstpose_beforee_firstpose = utils.distance(p_i, p_before_i)
#             v_beforee_firstpose_i_firstpose = CG3dVector(
#                 (-p_before_i[0] + p_i[0]) / l_i_firstpose_beforee_firstpose,
#                 (-p_before_i[1] + p_i[1]) / l_i_firstpose_beforee_firstpose,
#                 (-p_before_i[2] + p_i[2]) / l_i_firstpose_beforee_firstpose)
#
#             normal_vector_phi1 = v_i_nextt_firstPose ^ v_beforee_firstpose_i_firstpose
#
#             l_before_next_firstpose = utils.distance(p_before_i, p_nextt_i_first_pose)
#             v_before_next_firstpose = CG3dVector(
#                 (p_before_i[0] - p_nextt_i_first_pose[0]) / l_before_next_firstpose,
#                 (p_before_i[1] - p_nextt_i_first_pose[1]) / l_before_next_firstpose,
#                 (p_before_i[2] - p_nextt_i_first_pose[2]) / l_before_next_firstpose)
#
#             l1 = normal_vector_phi1 ^ v_before_next_firstpose
#
#             # for phi_2
#             l_before_next = utils.distance(p_before_i, p_nextt_i_first_pose)
#             v_nextt_before = CG3dVector((p_before_i[0] - p_nextt_i_first_pose[0]) / l_before_next,
#                                         (p_before_i[1] - p_nextt_i_first_pose[1]) / l_before_next,
#                                         (p_before_i[2] - p_nextt_i_first_pose[2]) / l_before_next)
#             v_l1 = CG3dVector(l1[0], l1[1], l1[2])
#
#             normal_vector_phi2 = v_nextt_before ^ v_l1
#             # the equation to solve for p_i new position
#
#             # image of p_i on second page:
#
#             d = v_before_i * normal_vector_phi2
#             p_hat = CG3dPoint(p_i[0] - d * normal_vector_phi2[0], p_i[1] - d * normal_vector_phi2[1],
#                               p_i[2] - d * normal_vector_phi2[2])
#             # l2 which is a line between p_next and p_hat
#             l_beforee_hat = utils.distance(p_before_i, p_hat)
#             unit_l2 = CG3dVector((p_hat[0] - p_before_i[0]) / l_beforee_hat,
#                                  (p_hat[1] - p_before_i[1]) / l_beforee_hat,
#                                  (p_hat[2] - p_before_i[2]) / l_beforee_hat)
#             p_primei = p_before_i + l_i_firstpose_beforee_firstpose * unit_l2
#
#             l_prime_nextt = utils.distance(p_primei, p_nextt_i)
#             v_prime_i_nextt = CG3dVector(p_nextt_i[0] - p_primei[0], p_nextt_i[1] - p_primei[1],
#                                          p_nextt_i[2] - p_primei[2])
#             # a unit vector of a line passing p(i-1) and P'(i)
#             unit_prime_nextt = CG3dVector(v_prime_i_nextt[0] / l_prime_nextt, v_prime_i_nextt[1] / l_prime_nextt,
#                                           v_prime_i_nextt[2] / l_prime_nextt)
#             p_nextt_prime = p_primei + l_i_nextt_firstpose * unit_prime_nextt
#
#             v_prime_i_nextt_prime = CG3dVector(p_nextt_prime[0] - p_primei[0], p_nextt_prime[1] - p_primei[1],
#                                                p_nextt_prime[2] - p_primei[2])
#
#             # finding o_hinge to know it is in limited zone
#             v_before_prime_i = CG3dVector(p_primei[0] - p_before_i[0], p_primei[1] - p_before_i[1],
#                                           p_primei[2] - p_before_i[2])
#             l_before_i_prime = utils.distance(p_primei, p_before_i)
#
#             l_prime_o = (v_before_prime_i * v_prime_i_nextt_prime) / l_before_i_prime
#
#             # a unit vector of a line passing p(i-1) and P'(i)
#             unit_vec_next_prime = CG3dVector((p_primei[0] - p_before_i[0]) / l_before_i_prime,
#                                              (p_primei[1] - p_before_i[1]) / l_before_i_prime,
#                                              (p_primei[2] - p_before_i[2]) / l_before_i_prime)
#
#             v_prime_o = l_prime_o * unit_vec_next_prime
#             o_hinge = CG3dPoint(v_prime_o[0] + p_primei[0], v_prime_o[1] + p_primei[1], v_prime_o[2] + p_primei[2])
#             s_hinge = utils.distance(p_primei, o_hinge)
#             q_2 = s_hinge * math.tan(self.theta[1])
#             q_4 = s_hinge * math.tan(self.theta[3])
#
#             # finding that the p_next is on right or left  of o:
#             cross_product = v_prime_i_nextt_prime ^ v_prime_o
#             angle_io_ibefore = (math.sqrt(cross_product * cross_product)) / (l_prime_o * l_prime_nextt)
#
#             l_o_before_prime = utils.distance(o_hinge, p_nextt_prime)
#
#             # the distance between o and p_nextt_i:
#             if (angle_io_ibefore <= 0):
#                 if (l_o_before_prime <= q_2):
#                     return p_nextt_prime
#                 else:
#                     circle_center = p_primei
#                     radius_circle = utils.distance(p_primei, p_nextt_prime)
#                     U = CG3dVector((p_nextt_prime[0] - p_primei[0]) / radius_circle,
#                                    (p_nextt_prime[1] - p_primei[1]) / radius_circle,
#                                    (p_nextt_prime[2] - p_primei[2]) / radius_circle)
#                     V = normal_vector_phi2 ^ U
#                     angle_difference = angle_io_ibefore - self.theta[1]
#                     p_nextt_prime = circle_center + radius_circle * math.cos(
#                         angle_difference) * U + radius_circle * math.sin(angle_difference) * V
#                     return p_nextt_prime
#             elif (angle_io_ibefore > 0):
#                 if (l_o_before_prime <= q_4):
#                     return p_nextt_prime
#                 else:
#                     circle_center = p_primei
#                     radius_circle = utils.distance(p_primei, p_nextt_prime)
#                     U = CG3dVector((p_nextt_prime[0] - p_primei[0]) / radius_circle,
#                                    (p_nextt_prime[1] - p_primei[1]) / radius_circle,
#                                    (p_nextt_prime[2] - p_primei[2]) / radius_circle)
#                     V = normal_vector_phi2 ^ U
#                     angle_difference = self.pi / 2 + angle_io_ibefore - self.theta[3]
#                     p_nextt_prime = circle_center + radius_circle * math.cos(
#                         angle_difference) * U + radius_circle * math.sin(angle_difference) * V
#                     return p_nextt_prime
#
#         if (self.constraint_type == "ballAndSocket"):
#             dot_prod = v_i_nextt * v_before_i
#             l_i_nextt = utils.distance(p_i, p_nextt_i_first_pose)
#             l_before_i = utils.distance(p_i, p_before_i)
#
#             l_i_o = dot_prod / l_before_i
#
#             # a unit vector of a line passing p(i-1) and P(i)
#             unit_vec_beforee_i = CG3dVector((p_i[0] - p_before_i[0]) / l_before_i,
#                                             (p_i[1] - p_before_i[1]) / l_before_i,
#                                             (p_i[2] - p_before_i[2]) / l_before_i)
#             # vector from p(i) to point o
#             v_i_o = l_i_o * unit_vec_beforee_i
#
#             # points based on the paper definition
#             o = CG3dPoint(v_i_o[0] + p_i[0], v_i_o[1] + p_i[1], v_i_o[2] + p_i[2])
#             s = utils.distance(o, p_i)
#
#             # Semi ellipsoidal parameter qi (1,2,3,4)
#             q1 = s * math.tan(self.theta[0])
#             q2 = s * math.tan(self.theta[1])
#             q3 = s * math.tan(self.theta[2])
#             q4 = s * math.tan(self.theta[3])
#
#             # change the coordinate to cross section of cone and calculating the (i-1)th position in it
#
#             l_o_nextt = utils.distance(o, p_nextt_i)
#             unit_v_o_nextt = CG3dVector((p_nextt_i[0] - o[0]) / l_o_nextt, (p_nextt_i[1] - o[1]) / l_o_nextt,
#                                         (p_nextt_i[2] - o[2]) / l_o_nextt)
#
#             y_t = l_o_nextt * math.sin(self.si)
#             x_t = l_o_nextt * math.cos(self.si)
#
#             # finding the sector of the target position
#
#             if (x_t > 0 and y_t > 0):
#                 sector = 1
#             elif (x_t > 0 > y_t):
#                 sector = 4
#             elif (x_t < 0 and y_t < 0):
#                 sector = 3
#             elif (x_t < 0 and y_t > 0):
#                 sector = 2
#
#             if x_t == 0 or y_t==0:
#                 p_prime = CG3dPoint(0, 0, 0)
#                 return p_prime
#             # elif (x_t < 0 < y_t) or (x_t < 0 and y_t == 0) or (x_t == 0 and y_t > 0):
#
#             # checking that the target point is in ellipsoidal shape
#             inbound = 0
#             if ((x_t ** 2) / (q2 ** 2) + (y_t ** 2) / (q3 ** 2)) < 1 and sector == 1:
#                 inbound = 1
#                 p_prime = CG3dPoint(0, 0, 0)
#                 return p_prime
#             elif ((x_t ** 2) / (q2 ** 2) + (y_t ** 2) / (q1 ** 2)) <= 1 and sector == 2:
#                 inbound = 1
#                 p_prime = CG3dPoint(0, 0, 0)
#                 return p_prime
#             elif ((x_t ** 2) / (q4 ** 2) + (y_t ** 2) / (q1 ** 2)) <= 1 and sector == 3:
#                 inbound = 1
#                 p_prime = CG3dPoint(0, 0, 0)
#                 return p_prime
#             elif ((x_t ** 2) / (q4 ** 2) + (y_t ** 2) / (q3 ** 2)) <= 1 and sector == 4:
#                 inbound = 1
#                 p_prime = CG3dPoint(0, 0, 0)
#                 return p_prime
#             # if it is out bound of the ellipsoidal shape we should find the nearest point on ellipsoidal shape
#             if inbound == 0 and sector == 1:
#                 x_nearest_point = self.find_nearest_point(q2, q3, sector, x_t, y_t)
#                 y_nearest_point = math.sqrt(abs(q3 ** 2 - q3 ** 2 / q2 ** 2 * x_nearest_point ** 2))
#             elif inbound == 0 and sector == 2:
#                 x_nearest_point = self.find_nearest_point(q2, q1, sector, x_t, y_t)
#                 y_nearest_point = -math.sqrt(abs(q1 ** 2 - q1 ** 2 / q2 ** 2 * x_nearest_point ** 2))
#             elif inbound == 0 and sector == 3:
#                 x_nearest_point = self.find_nearest_point(q4, q1, sector, x_t, y_t)
#                 y_nearest_point = -math.sqrt(abs(q1 ** 2 - q1 ** 2 / q4 ** 2 * x_nearest_point ** 2))
#             elif inbound == 0 and sector == 4:
#                 x_nearest_point = self.find_nearest_point(q2, q3, sector, x_t, y_t)
#                 y_nearest_point = math.sqrt(abs(q3 ** 2 - q3 ** 2 / q2 ** 2 * x_nearest_point ** 2))
#
#             l_o_next_in_ellipse_plane = math.sqrt(x_t ** 2 + y_t ** 2)
#             l_o_nearest_point_in_ellipse_plane = math.sqrt(x_nearest_point ** 2 + y_nearest_point ** 2)
#             diff_z_o_np = (l_o_nearest_point_in_ellipse_plane / l_o_next_in_ellipse_plane) * abs(o[2] - p_nextt_i[2])
#             if (unit_v_o_nextt[2] >= 0):
#                 z_nearest_point = o[2] + diff_z_o_np
#             else:
#                 z_nearest_point = o[2] - diff_z_o_np
#
#             p_nearest_point_in_global = CG3dPoint(o[0] + unit_v_o_nextt[0] * x_nearest_point,
#                                                   o[1] + unit_v_o_nextt[1] * y_nearest_point,
#                                                   o[2] + unit_v_o_nextt[2] * z_nearest_point)
#
#             return p_nearest_point_in_global
#
#     def find_nearest_point(self,a, b, sector, x_t, y_t):
#         x_k1 = x_t * a * b / math.sqrt(b ** 2 * x_t ** 2 + a ** 2 * y_t ** 2)
#         if sector == 1 or sector == 2:
#             sign = 1
#         elif sector == 3 or sector == 4:
#             sign = -1
#         if abs(x_t) < a:
#             x_k2 = x_t
#         else:
#             x_k2 = a * sign
#
#         x0 = 1 / 2 * (x_k1 + x_k2)
#         x = self.newton_Raphson_nearest_point(x0, a, b, x_t, y_t)
#         return x
#
#     def func_nearest_point(self,x, a, b, x_t, y_t):
#         a4 = -(b ** 2) * ((a ** 2) - (b ** 2)) / (a ** 2)
#         a3 = 2 * (b ** 2) * x_t * ((a ** 2) - (b ** 2))
#         a2 = -(b ** 2) * (a ** 2) * (x_t ** 2) + (b ** 2) * ((a ** 2) - (b ** 2)) ** 2 - (b ** 4) * (y_t ** 2)
#         a1 = -2 * (a ** 2) * (b ** 2) * x_t * ((a ** 2) - (b ** 2))
#         a0 = (a ** 4) * (b ** 2) * (x_t ** 2)
#         return a4 * x * x * x * x + a3 * x * x * x + a2 * x * x + a1 * x + a0
#
#     def derivative_func_nearest_point(self,x, a, b, x_t, y_t):
#         a4 = -(b ** 2) * ((a ** 2) - (b ** 2)) / (a ** 2)
#         a3 = 2 * (b ** 2) * x_t * ((a ** 2) - (b ** 2))
#         a2 = -(b ** 2) * (a ** 2) * (x_t ** 2) + (b ** 2) * ((a ** 2) - (b ** 2)) ** 2 - (b ** 4) * (y_t ** 2)
#         a1 = -2 * (a ** 2) * (b ** 2) * x_t * ((a ** 2) - (b ** 2))
#         return 4 * a4 * x * x * x + 3 * a3 * x * x + 2 * a2 * x + a1
#
#     # Function to find the root
#
#     def newton_Raphson_nearest_point(self,x, a, b, x_t, y_t):
#         # if self.derivative_func_nearest_point(x, a, b, x_t, y_t)==0:
#         #     return x
#         h = self.func_nearest_point(x, a, b, x_t, y_t) / self.derivative_func_nearest_point(x, a, b, x_t, y_t)
#         counter = 0
#         while (abs(h) >= 0.0001):
#             h = self.func_nearest_point(x, a, b, x_t, y_t) / self.derivative_func_nearest_point(x, a, b, x_t, y_t)
#             x = x - h
#         return x
#
#
#
#
#

import math
from pycg3d.cg3d_point import CG3dPoint
from pycg3d.cg3d_vector import CG3dVector
from pycg3d import utils
import numpy as np


class constraints:

    def __init__(self, joints, body_index, i, theta, length, constraint_type, orientation):
        self.joints = joints
        self.body_index = body_index
        self.i = i
        self.pi = 22 / 7
        self.theta = (self.pi / 180) * theta
        self.length = length
        self.constraint_type = constraint_type
        # the orientation of joint
        theta = math.acos(orientation[0]) * 2
        self.si = theta

    def rotational_constraint(self):
        p_i = CG3dPoint(self.joints[self.body_index[self.i]][0], self.joints[self.body_index[self.i]][1],
                        self.joints[self.body_index[self.i]][2])
        p_before = CG3dPoint(self.joints[self.body_index[self.i - 1]][0], self.joints[self.body_index[self.i - 1]][1],
                             self.joints[self.body_index[self.i - 1]][2])
        p_next = CG3dPoint(self.joints[self.body_index[self.i + 1]][0], self.joints[self.body_index[self.i + 1]][1],
                           self.joints[self.body_index[self.i + 1]][2])

        v_i_next = CG3dVector(p_next[0] - p_i[0], p_next[1] - p_i[1], p_next[2] - p_i[2])
        v_before_i = CG3dVector(p_i[0] - p_before[0], p_i[1] - p_before[1], p_i[2] - p_before[2])

        dot_prod = v_i_next * v_before_i
        s = dot_prod

        # a unit vector of a line passing  from p(i+1) and P(i)
        l_next_i = utils.distance(p_next, p_i)
        unit_vec_next_i = CG3dVector((p_i[0] - p_next[0]) / l_next_i,
                                     (p_i[1] - p_next[1]) / l_next_i,
                                     (p_i[2] - p_next[2]) / l_next_i)

        # the center of cone
        o = CG3dPoint(p_i[0] + unit_vec_next_i[0] * s, p_i[1] + unit_vec_next_i[1] * s,
                      p_i[2] + unit_vec_next_i[2] * s)

        if (self.constraint_type == "hinge"):
            # normal plane vector of p(next), p(i), p(before)
            normal_plane = v_i_next ^ v_before_i
            l = math.sqrt(normal_plane * normal_plane)
            uv_normal_plane = CG3dVector(normal_plane[0] / l, normal_plane[1] / l, normal_plane[2] / l)
            # a vector from p_i to o
            l_i__o = utils.distance(p_i, o)
            unit_vec_i_o = CG3dVector((o[0] - p_i[0]) / l_i__o,
                                      (o[1] - p_i[1]) / l_i__o,
                                      (o[2] - p_i[2]) / l_i__o)

            axis_normal_plane = [uv_normal_plane[0], uv_normal_plane[1], uv_normal_plane[2]]
            unit_vec_i_o_array = np.array([[unit_vec_i_o[0]], [unit_vec_i_o[1]], [unit_vec_i_o[2]]])
            # rotating p(i)-o C.C.W to find flexion constraint(left of pi_o)
            p_down = np.dot(self.rotation_matrix(axis_normal_plane, self.theta[1]), unit_vec_i_o_array)
            p_down = CG3dVector(p_down[0, 0], p_down[1, 0], p_down[2, 0])
            # rotating p(i)-o C.W to find extension constraint(right of pi_o )
            p_up = np.dot(self.rotation_matrix(axis_normal_plane, -self.theta[3]), unit_vec_i_o_array)
            p_up = CG3dVector(p_up[0, 0], p_up[1, 0], p_up[2, 0])

            l_before_i = utils.distance(p_i, p_before)
            uv_i_before = CG3dVector((p_before[0] - p_i[0]) / l_before_i, (p_before[1] - p_i[1]) / l_before_i,
                                     (p_before[2] - p_i[2]) / l_before_i)

            if (uv_i_before ^ unit_vec_i_o) * (uv_normal_plane) > 0:
                # means it is upper side of pi_o
                if self.theta[3] != 0:
                    # angle between vec(i_before) and vec(i_o)
                    angle = math.acos(uv_i_before * unit_vec_i_o)
                    if angle <= self.theta[3]:
                        return CG3dPoint(0, 0, 0)
                    else:
                        p_nearest_point = p_i + l_before_i * p_up
                        return p_nearest_point
                else:
                    p_nearest_point = CG3dPoint(p_i[0] + unit_vec_i_o[0] * l_before_i,
                                                p_i[1] + unit_vec_i_o[1] * l_before_i,
                                                p_i[2] + unit_vec_i_o[2] * l_before_i)
                    return p_nearest_point
            else:
                # means it is down side of pi_o
                if self.theta[1] != 0:
                    # angle between vec(i_before) and vec(i_o)
                    angle = math.acos(uv_i_before * unit_vec_i_o)
                    if angle <= self.theta[1]:
                        return CG3dPoint(0, 0, 0)
                    else:
                        p_nearest_point = p_i + l_before_i * p_down
                        return p_nearest_point
                else:
                    p_nearest_point = CG3dPoint(p_i[0] + unit_vec_i_o[0] * l_before_i,
                                                p_i[1] + unit_vec_i_o[1] * l_before_i,
                                                p_i[2] + unit_vec_i_o[2] * l_before_i)
                    return p_nearest_point
        if self.constraint_type == "ballAndSocket":
            # Semi ellipsoidal parameter qi (1,2,3,4)
            q1 = round(s * math.tan(self.theta[0]), 3)
            q2 = round(s * math.tan(self.theta[1]), 3)
            q3 = round(s * math.tan(self.theta[2]), 3)
            q4 = round(s * math.tan(self.theta[3]), 3)

            # change the coordinate to cross section of cone and calculating the (i-1)th position in it
            l_o_next = utils.distance(o, p_next)

            if 0 <= round(self.si) < np.pi / 2:
                sector = 1
            elif np.pi / 2 <= round(self.si) < np.pi:
                sector = 2
            elif np.pi <= round(self.si) < 3 * np.pi / 2:
                sector = 3
            elif 3 * np.pi / 2 <= round(self.si) < np.pi / 2 * np.pi:
                sector = 4

            y_t = round(l_o_next * math.sin(self.si), 2)
            x_t = round(l_o_next * math.cos(self.si), 2)

            # checking that the target point is in ellipsoidal shape
            inbound = 0
            if round(((x_t ** 2) / (q3 ** 2) + (y_t ** 2) / (q2 ** 2))) <= 1 and sector == 1:
                inbound = 1
                p_prime = CG3dPoint(0, 0, 0)
                return p_prime
            elif round(((x_t ** 2) / (q1 ** 2) + (y_t ** 2) / (q2 ** 2))) <= 1 and sector == 2:
                inbound = 1
                p_prime = CG3dPoint(0, 0, 0)
                return p_prime
            elif round(((x_t ** 2) / (q1 ** 2) + (y_t ** 2) / (q4 ** 2))) <= 1 and sector == 3:
                inbound = 1
                p_prime = CG3dPoint(0, 0, 0)
                return p_prime
            elif round(((x_t ** 2) / (q3 ** 2) + (y_t ** 2) / (q4 ** 2))) <= 1 and sector == 4:
                inbound = 1
                p_prime = CG3dPoint(0, 0, 0)
                return p_prime

            # if it is out bound of the ellipsoidal shape we should find the nearest point on ellipsoidal shape
            if inbound == 0 and sector == 1:
                if y_t != 0:
                    result = self.find_nearest_point(q3, q2, sector, x_t, y_t)
                    x_nearest_point = result[0]
                    y_nearest_point = result[1]
                else:
                    x_nearest_point = q3
                    y_nearest_point = 0
            elif inbound == 0 and sector == 2:
                if x_t != 0:
                    result = self.find_nearest_point(q1, q2, sector, x_t, y_t)
                    x_nearest_point = result[0]
                    y_nearest_point = result[1]
                else:
                    x_nearest_point = 0
                    y_nearest_point = q2
            elif inbound == 0 and sector == 3:
                if y_t != 0:
                    result = self.find_nearest_point(q1, q4, sector, x_t, y_t)
                    x_nearest_point = result[0]
                    y_nearest_point = result[1]
                else:
                    x_nearest_point = -q1
                    y_nearest_point = 0
            elif inbound == 0 and sector == 4:
                if x_t != 0:
                    result = self.find_nearest_point(q3, q4, sector, x_t, y_t)
                    x_nearest_point = result[0]
                    y_nearest_point = result[1]
                else:
                    x_nearest_point = 0
                    y_nearest_point = -q4

            # finding nearest point global coordinate in 3d
            l_o_before = math.sqrt(x_t ** 2 + y_t ** 2)
            l_o_nearest_point = math.sqrt(x_nearest_point ** 2 + y_nearest_point ** 2)
            l_nearest_before = math.sqrt((x_t - x_nearest_point) ** 2 + (y_nearest_point - y_t) ** 2)

            # rotation angle between vector from o to p_before and o to p_nearest point
            rot_angle = (math.acos(round((l_nearest_before ** 2 - l_o_before ** 2 - l_o_nearest_point ** 2) / (
                    -2 * l_o_before * l_o_nearest_point))))

            # rotating the vector from o to p_before around vector from p_next to o

            # a vector from o to p_before
            unit_vec_o_before = CG3dVector((p_before[0] - o[0]) / l_o_before,
                                           (p_before[1] - o[1]) / l_o_before,
                                           (p_before[2] - o[2]) / l_o_before)
            unit_vec_normal_plane = [(p_i[0] - p_next[0]) / l_next_i,
                                     (p_i[1] - p_next[1]) / l_next_i,
                                     (p_i[2] - p_next[2]) / l_next_i]

            if rot_angle == 0:
                p_nearest_point_in_global = CG3dPoint(o[0] + l_o_nearest_point * unit_vec_o_before[0],
                                                      o[1] + l_o_nearest_point * unit_vec_o_before[1],
                                                      o[2] + l_o_nearest_point * unit_vec_o_before[2])
                return p_nearest_point_in_global
            else:
                # to find out nearest point is on left or right side of target point
                orient_near_to_target = x_t * y_nearest_point - x_nearest_point * y_t
                unit_vec_o_before = np.array([[unit_vec_o_before[0]],
                                              [unit_vec_o_before[1]],
                                              [unit_vec_o_before[2]]])
                if orient_near_to_target * unit_vec_next_i[2] > 0:
                    # to find nearest point should rotate the uv(o-target) in C.C.W
                    uv_o_nearest_point = np.dot(self.rotation_matrix(unit_vec_normal_plane, rot_angle),
                                                unit_vec_o_before)
                else:
                    uv_o_nearest_point = np.dot(self.rotation_matrix(unit_vec_normal_plane, -rot_angle),
                                                unit_vec_o_before)

                p_nearest_point_in_global = CG3dPoint(o[0] + l_o_nearest_point * uv_o_nearest_point[0],
                                                      o[1] + l_o_nearest_point * uv_o_nearest_point[1],
                                                      o[2] + l_o_nearest_point * uv_o_nearest_point[2])

                return p_nearest_point_in_global

    def rotation_matrix(self, axis, theta):
        """
        Return the rotation matrix associated with counterclockwise rotation about
        the given axis by theta radians.
        """
        axis = np.asarray(axis)
        axis = axis / math.sqrt(np.dot(axis, axis))
        a = math.cos(theta / 2.0)
        b, c, d = -axis * math.sin(theta / 2.0)
        aa, bb, cc, dd = a * a, b * b, c * c, d * d
        bc, ad, ac, ab, bd, cd = b * c, a * d, a * c, a * b, b * d, c * d
        return np.array([[aa + bb - cc - dd, 2 * (bc + ad), 2 * (bd - ac)],
                         [2 * (bc - ad), aa + cc - bb - dd, 2 * (cd + ab)],
                         [2 * (bd + ac), 2 * (cd - ab), aa + dd - bb - cc]])

    def find_initial_point(self, a, b, sector, x_t, y_t):
        x_k1 = x_t * a * b / math.sqrt(b ** 2 * x_t ** 2 + a ** 2 * y_t ** 2)
        y_k1 = y_t * a * b / math.sqrt(b ** 2 * x_t ** 2 + a ** 2 * y_t ** 2)
        if sector == 1 or sector == 2:
            sign_y = 1
        elif sector == 3 or sector == 4:
            sign_y = -1
        if sector == 1 or sector == 4:
            sign_x = 1
        elif sector == 3 or sector == 2:
            sign_x = -1

        if abs(x_t) < a:
            x_k2 = x_t
            y_k2 = sign_y * (b / a) * math.sqrt(a ** 2 - x_t ** 2)
        else:
            x_k2 = a * sign_x * a
            y_k2 = 0

        x0 = 0.5 * (x_k1 + x_k2)
        y0 = 0.5 * (y_k1 + y_k2)

        initial = np.array([x0, y0])
        return initial

    def q_matrix(self, x, y, a, b, x_t, y_t):
        Q = np.matrix(
            [[b ** 2 * x, a ** 2 * y], [(a ** 2 - b ** 2) * y + b ** 2 * y_t, (a ** 2 - b ** 2) * x - a ** 2 * x_t]])
        return Q

    def find_delta(self, x, y, a, b, x_t, y_t):
        f1 = (a ** 2 * y ** 2 + b ** 2 * x ** 2 - a ** 2 * b ** 2) / 2
        f2 = b ** 2 * x * (y_t - y) - a ** 2 * y * (x_t - x)
        f = np.matrix([[f1], [f2]])
        Q = self.q_matrix(x, y, a, b, x_t, y_t)
        delta = -1 * (np.linalg.inv(Q) * f)

        result = np.array([delta[0, 0], delta[1, 0]])
        return result

    def find_next_point(self, x, y, a, b, x_t, y_t):
        delta = self.find_delta(x, y, a, b, x_t, y_t)
        x = x + delta[0]
        y = y + delta[1]
        result = np.array([x, y])
        return result

    def find_nearest_point(self, a, b, sector, x_t, y_t):
        initial = self.find_initial_point(a, b, sector, x_t, y_t)
        x_initial = initial[0]
        y_initial = initial[1]
        x = x_initial
        y = y_initial
        thresh = 0.001
        result = self.find_next_point(x, y, a, b, x_t, y_t)
        while result[0] > thresh or result[1] > thresh:
            x = result[0]
            y = result[1]
            result = self.find_next_point(x, y, a, b, x_t, y_t)
        x_nearest_point = result[0]
        y_nearest_point = result[1]
        return np.array([x_nearest_point, y_nearest_point])
# def func_nearest_point(self, x, a, b, x_t, y_t):
#     a4 = -(b ** 2) * ((a ** 2) - (b ** 2)) / (a ** 2)
#     a3 = 2 * (b ** 2) * x_t * ((a ** 2) - (b ** 2))
#     a2 = -(b ** 2) * (a ** 2) * (x_t ** 2) + (b ** 2) * ((a ** 2) - (b ** 2)) ** 2 - (b ** 4) * (y_t ** 2)
#     a1 = -2 * (a ** 2) * (b ** 2) * x_t * ((a ** 2) - (b ** 2))
#     a0 = (a ** 4) * (b ** 2) * (x_t ** 2)
#     return a4 * x * x * x * x + a3 * x * x * x + a2 * x * x + a1 * x + a0
#
# def derivative_func_nearest_point(self, x, a, b, x_t, y_t):
#     a4 = -(b ** 2) * ((a ** 2) - (b ** 2)) / (a ** 2)
#     a3 = 2 * (b ** 2) * x_t * ((a ** 2) - (b ** 2))
#     a2 = -(b ** 2) * (a ** 2) * (x_t ** 2) + (b ** 2) * ((a ** 2) - (b ** 2)) ** 2 - (b ** 4) * (y_t ** 2)
#     a1 = -2 * (a ** 2) * (b ** 2) * x_t * ((a ** 2) - (b ** 2))
#     return 4 * a4 * x * x * x + 3 * a3 * x * x + 2 * a2 * x + a1
#
# # Function to find the root
#
# def newton_Raphson_nearest_point(self, x, y, a, b, x_t, y_t):
#     h = self.func_nearest_point(x, a, b, x_t, y_t) / self.derivative_func_nearest_point(x, a, b, x_t, y_t)
#     counter = 0
#     while (abs(h) >= 0.0001):
#         h = self.func_nearest_point(x, a, b, x_t, y_t) / self.derivative_func_nearest_point(x, a, b, x_t, y_t)
#         x = x - h
#     return x
