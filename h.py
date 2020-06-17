import math
from pycg3d.cg3d_point import CG3dPoint
from pycg3d.cg3d_vector import CG3dVector
from pycg3d import utils
import numpy as np


class constraints:

    def __init__(self, joints, body_index, i, theta, length, constraint_type, orientation):
        self.x_global_axis =[1,0,0]
        self.y_global_axis = [0,1,0]
        self.joints = joints
        self.body_index = body_index
        self.i = i
        self.pi = 22 / 7
        self.theta = (self.pi / 180) * theta
        self.length = length
        self.constraint_type = constraint_type
        # the orientation of joint
        theta = math.acos(orientation[0]) * 2
        self.orientation_theta = theta
        self.joint_rotation_vector = self.joint_rotation_vector(orientation)

    def joint_rotation_vector(self, orientation):
        theta = math.acos(orientation[0]) * 2
        v1 = orientation[1]/math.sin(theta/2)
        v2 = orientation[2]/math.sin(theta/2)
        v3 = orientation[3]/math.sin(theta/2)
        return CG3dVector(v1,v2,v3)

    def rotate_about_axis(self,source, angle_degs, rotation_axis):
        sin_theta = math.sin(angle_degs * math.pi / 180)
        cos_theta = math.cos(angle_degs * math.pi / 180)
        one_minus_cos_theta = 1.0 - cos_theta

        xy_one = rotation_axis[0] * rotation_axis[1] * one_minus_cos_theta
        xz_one = rotation_axis[0] * rotation_axis[2] * one_minus_cos_theta
        yz_one = rotation_axis[1] * rotation_axis[2] * one_minus_cos_theta

        m00 = rotation_axis[0] * rotation_axis[0] * one_minus_cos_theta + cos_theta
        m01 = xy_one + rotation_axis[2] * sin_theta
        m02 = xz_one - rotation_axis[1] * sin_theta

        m10 = xy_one - rotation_axis[2] * sin_theta
        m11 = rotation_axis[1] * rotation_axis[1] * one_minus_cos_theta + cos_theta
        m12 = yz_one + rotation_axis[0] * sin_theta

        m20 = xz_one + rotation_axis[1] * sin_theta
        m21 = yz_one - rotation_axis[0] * sin_theta
        m22 = rotation_axis[2] * rotation_axis[2] * one_minus_cos_theta + cos_theta

        rotation_matrix = [[m00, m01, m02],
                           [m10, m11, m12],
                           [m20, m21, m22]]
        result_times = []
        for i in range(0, 3):
            result_row = 0
            for j in range(0, 3):
                result_row += rotation_matrix[i][j] * source[j]
            result_times.append(result_row)
        result_times = CG3dVector(result_times[0],result_times[1],result_times[2])
        return result_times

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
            l_o_before = utils.distance(o, p_before)
            v_o_before = CG3dVector(p_before[0] - o[0], p_before[1] - o[1], p_before[2] - o[2])
            uv_o_before = CG3dVector(v_o_before[0]/l_o_before,v_o_before[1]/l_o_before,v_o_before[2]/l_o_before)

            # joint reference plane coordinate axis:
            rotation_axis = self.joint_rotation_vector
            x_reference_axis = self.normalization(self.rotate_about_axis(self.x_global_axis,self.orientation_theta,rotation_axis))
            y_reference_axis = self.normalization(self.rotate_about_axis(self.y_global_axis, self.orientation_theta, rotation_axis))

            x_t = v_o_before*x_reference_axis
            y_t = v_o_before*y_reference_axis

            if x_t>0 and y_t>=0:
                sector = 1
            elif x_t<=0 and y_t>0:
                sector = 2
            elif x_t<0 and y_t<=0:
                sector = 3
            elif x_t>=0 and y_t<0:
                sector = 4

            # checking that the target point is in ellipsoidal shape
            inbound = 0
            if x_t==0 and y_t==0:
                inbound = 1
                p_prime = CG3dPoint(0, 0, 0)
                return p_prime

            if round(((x_t ** 2) / (q2 ** 2) + (y_t ** 2) / (q3 ** 2))) <= 1 and sector == 1:
                inbound = 1
                p_prime = CG3dPoint(0, 0, 0)
                return p_prime
            elif round(((x_t ** 2) / (q2 ** 2) + (y_t ** 2) / (q1 ** 2))) <= 1 and sector == 2:
                inbound = 1
                p_prime = CG3dPoint(0, 0, 0)
                return p_prime
            elif round(((x_t ** 2) / (q4 ** 2) + (y_t ** 2) / (q1 ** 2))) <= 1 and sector == 3:
                inbound = 1
                p_prime = CG3dPoint(0, 0, 0)
                return p_prime
            elif round(((x_t ** 2) / (q4 ** 2) + (y_t ** 2) / (q3 ** 2))) <= 1 and sector == 4:
                inbound = 1
                p_prime = CG3dPoint(0, 0, 0)
                return p_prime

            # if it is out bound of the ellipsoidal shape we should find the nearest point on ellipsoidal shape
            if inbound == 0 and sector == 1:
                if y_t != 0:
                    result = self.find_nearest_2(x_t, y_t ,q2, q3)
                    x_nearest_point = result[0]
                    y_nearest_point = result[1]
                else:
                    x_nearest_point = q2
                    y_nearest_point = 0
            elif inbound == 0 and sector == 2:
                if x_t != 0:
                    result = self.find_nearest_2(x_t, y_t ,q2, q1)
                    x_nearest_point = result[0]
                    y_nearest_point = result[1]
                else:
                    x_nearest_point = 0
                    y_nearest_point = -q1
            elif inbound == 0 and sector == 3:
                if y_t != 0:
                    result = self.find_nearest_2(x_t, y_t ,q4, q1)
                    x_nearest_point = result[0]
                    y_nearest_point = result[1]
                else:
                    x_nearest_point = -q4
                    y_nearest_point = 0
            elif inbound == 0 and sector == 4:
                if x_t != 0:
                    result = self.find_nearest_2(x_t, y_t ,q4, q3)
                    x_nearest_point = result[0]
                    y_nearest_point = result[1]
                else:
                    x_nearest_point = 0
                    y_nearest_point = q3


            l_o_nearest_point = math.sqrt(x_nearest_point**2 +y_nearest_point**2)
            nearest_point_in_global = CG3dPoint(o[0]+uv_o_before[0]*l_o_nearest_point,
                                                o[1]+uv_o_before[1]*l_o_nearest_point,
                                                o[2]+uv_o_before[2]*l_o_nearest_point)
            # finding nearest point global coordinate in 3d
            # l_o_before = math.sqrt(x_t ** 2 + y_t ** 2)
            # l_o_nearest_point = math.sqrt(x_nearest_point ** 2 + y_nearest_point ** 2)
            # l_nearest_before = math.sqrt((x_t - x_nearest_point) ** 2 + (y_nearest_point - y_t) ** 2)
            #
            # # rotation angle between vector from o to p_before and o to p_nearest point
            # rot_angle = (math.acos(round((l_nearest_before ** 2 - l_o_before ** 2 - l_o_nearest_point ** 2) / (
            #         -2 * l_o_before * l_o_nearest_point))))
            #
            # # rotating the vector from o to p_before around vector from p_next to o
            #
            # # a vector from o to p_before
            # unit_vec_o_before = CG3dVector((p_before[0] - o[0]) / l_o_before,
            #                                (p_before[1] - o[1]) / l_o_before,
            #                                (p_before[2] - o[2]) / l_o_before)
            # unit_vec_normal_plane = [(p_i[0] - p_next[0]) / l_next_i,
            #                          (p_i[1] - p_next[1]) / l_next_i,
            #                          (p_i[2] - p_next[2]) / l_next_i]
            #
            # if rot_angle == 0:
            #     p_nearest_point_in_global = CG3dPoint(o[0] + l_o_nearest_point * unit_vec_o_before[0],
            #                                           o[1] + l_o_nearest_point * unit_vec_o_before[1],
            #                                           o[2] + l_o_nearest_point * unit_vec_o_before[2])
            #     return p_nearest_point_in_global
            # else:
            #     # to find out nearest point is on left or right side of target point
            #     orient_near_to_target = x_t * y_nearest_point - x_nearest_point * y_t
            #     unit_vec_o_before = np.array([[unit_vec_o_before[0]],
            #                                   [unit_vec_o_before[1]],
            #                                   [unit_vec_o_before[2]]])
            #     if orient_near_to_target * unit_vec_next_i[2] > 0:
            #         # to find nearest point should rotate the uv(o-target) in C.C.W
            #         uv_o_nearest_point = np.dot(self.rotation_matrix(unit_vec_normal_plane, rot_angle),
            #                                     unit_vec_o_before)
            #     else:
            #         uv_o_nearest_point = np.dot(self.rotation_matrix(unit_vec_normal_plane, -rot_angle),
            #                                     unit_vec_o_before)
            #
            #     p_nearest_point_in_global = CG3dPoint(o[0] + l_o_nearest_point * uv_o_nearest_point[0],
            #                                           o[1] + l_o_nearest_point * uv_o_nearest_point[1],
            #                                           o[2] + l_o_nearest_point * uv_o_nearest_point[2])

            return nearest_point_in_global

    def find_nearest_2(self,x_t,y_t,a,b):
        m = x_t/y_t
        y_nearest_point = math.sqrt(abs(1- (m**2/a**2+1/b**2)))
        x_nearest_point = m*y_nearest_point
        return [x_nearest_point,y_nearest_point]
















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
        counter =0
        while result[0] > thresh or result[1] > thresh and counter<1000:
            x = result[0]
            y = result[1]
            result = self.find_next_point(x, y, a, b, x_t, y_t)
            counter+=1
        x_nearest_point = result[0]
        y_nearest_point = result[1]
        return np.array([x_nearest_point, y_nearest_point])

    def normalization(self,vector):
        l = math.sqrt(vector[0] ** 2 + vector[1] ** 2 + vector[2] ** 2)
        if l == 0:
            l += 0.001
        normal_vector = CG3dVector(vector[0] / l, vector[1] / l, vector[2] / l)
        return normal_vector