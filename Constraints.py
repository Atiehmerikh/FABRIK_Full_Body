import math
from pycg3d.cg3d_point import CG3dPoint
from pycg3d.cg3d_vector import CG3dVector
from pycg3d import utils

counterLimit = 10000
si = 0.0


def constraint(joints, body_index, i, theta, length, constraint_type):
    pi = 22 / 7
    theta = (pi / 180) * theta

    p_i = CG3dPoint(joints[body_index[i]][0], joints[body_index[i]][1], joints[body_index[i]][2])
    p_before_i = CG3dPoint(joints[body_index[i - 1]][0], joints[body_index[i - 1]][1], joints[body_index[i - 1]][2])
    p_nextt_i = CG3dPoint(joints[body_index[i + 1]][0], joints[body_index[i + 1]][1], joints[body_index[i + 1]][2])

    p_i_first_pose = CG3dPoint(length[body_index[i]][0], length[body_index[i]][1], length[body_index[i]][2])
    p_before_i_first_pose = CG3dPoint(length[body_index[i - 1]][0], length[body_index[i - 1]][1],
                                      length[body_index[i - 1]][2])
    p_nextt_i_first_pose = CG3dPoint(length[body_index[i + 1]][0], length[body_index[i + 1]][1],

                                     length[body_index[i + 1]][2])

    v_i_nextt = CG3dVector(p_nextt_i[0] - p_i[0], p_nextt_i[1] - p_i[1], p_nextt_i[2] - p_i[2])
    v_before_i = CG3dVector(p_i[0] - p_before_i[0], p_i[1] - p_before_i[1], p_i[2] - p_before_i[2])

    if (constraint_type == "hinge"):
        # defining a line l_1 in paper:
        l_i_nextt_firstpose = utils.distance(p_i, p_nextt_i_first_pose)
        v_i_nextt_firstPose = CG3dVector((-p_i[0] + p_nextt_i_first_pose[0]) / l_i_nextt_firstpose,
                                         (-p_i[1] + p_nextt_i_first_pose[1]) / l_i_nextt_firstpose,
                                         (-p_i[2] + p_nextt_i_first_pose[2]) / l_i_nextt_firstpose)
        l_i_firstpose_beforee_firstpose = utils.distance(p_i, p_before_i)
        v_beforee_firstpose_i_firstpose = CG3dVector(
            (-p_before_i[0] + p_i[0]) / l_i_firstpose_beforee_firstpose,
            (-p_before_i[1] + p_i[1]) / l_i_firstpose_beforee_firstpose,
            (-p_before_i[2] + p_i[2]) / l_i_firstpose_beforee_firstpose)

        normal_vector_phi1 = v_i_nextt_firstPose ^ v_beforee_firstpose_i_firstpose

        l_before_next_firstpose = utils.distance(p_before_i, p_nextt_i_first_pose)
        v_before_next_firstpose = CG3dVector(
            (p_before_i[0] - p_nextt_i_first_pose[0]) / l_before_next_firstpose,
            (p_before_i[1] - p_nextt_i_first_pose[1]) / l_before_next_firstpose,
            (p_before_i[2] - p_nextt_i_first_pose[2]) / l_before_next_firstpose)

        l1 = normal_vector_phi1 ^ v_before_next_firstpose

        # for phi_2
        l_before_next = utils.distance(p_before_i, p_nextt_i_first_pose)
        v_nextt_before = CG3dVector((p_before_i[0] - p_nextt_i_first_pose[0]) / l_before_next,
                                    (p_before_i[1] - p_nextt_i_first_pose[1]) / l_before_next,
                                    (p_before_i[2] - p_nextt_i_first_pose[2]) / l_before_next)
        v_l1 = CG3dVector(l1[0], l1[1], l1[2])

        normal_vector_phi2 = v_nextt_before ^ v_l1
        # the equation to solve for p_i new position

        # image of p_i on second page:

        d = v_before_i * normal_vector_phi2
        p_hat = CG3dPoint(p_i[0] - d * normal_vector_phi2[0], p_i[1] - d * normal_vector_phi2[1],
                          p_i[2] - d * normal_vector_phi2[2])
        # l2 which is a line between p_next and p_hat
        l_beforee_hat = utils.distance(p_before_i, p_hat)
        unit_l2 = CG3dVector((p_hat[0] - p_before_i[0]) / l_beforee_hat,
                             (p_hat[1] - p_before_i[1]) / l_beforee_hat,
                             (p_hat[2] - p_before_i[2]) / l_beforee_hat)
        p_primei = p_before_i + l_i_firstpose_beforee_firstpose * unit_l2

        l_prime_nextt = utils.distance(p_primei, p_nextt_i)
        v_prime_i_nextt = CG3dVector(p_nextt_i[0] - p_primei[0], p_nextt_i[1] - p_primei[1],
                                     p_nextt_i[2] - p_primei[2])
        # a unit vector of a line passing p(i-1) and P'(i)
        unit_prime_nextt = CG3dVector(v_prime_i_nextt[0] / l_prime_nextt, v_prime_i_nextt[1] / l_prime_nextt,
                                      v_prime_i_nextt[2] / l_prime_nextt)
        p_nextt_prime = p_primei + l_i_nextt_firstpose * unit_prime_nextt

        v_prime_i_nextt_prime = CG3dVector(p_nextt_prime[0] - p_primei[0], p_nextt_prime[1] - p_primei[1],
                                           p_nextt_prime[2] - p_primei[2])

        # finding o_hinge to know it is in limited zone
        v_before_prime_i = CG3dVector(p_primei[0] - p_before_i[0], p_primei[1] - p_before_i[1],
                                      p_primei[2] - p_before_i[2])
        l_before_i_prime = utils.distance(p_primei, p_before_i)

        l_prime_o = (v_before_prime_i * v_prime_i_nextt_prime) / l_before_i_prime

        # a unit vector of a line passing p(i-1) and P'(i)
        unit_vec_next_prime = CG3dVector((p_primei[0] - p_before_i[0]) / l_before_i_prime,
                                         (p_primei[1] - p_before_i[1]) / l_before_i_prime,
                                         (p_primei[2] - p_before_i[2]) / l_before_i_prime)

        v_prime_o = l_prime_o * unit_vec_next_prime
        o_hinge = CG3dPoint(v_prime_o[0] + p_primei[0], v_prime_o[1] + p_primei[1], v_prime_o[2] + p_primei[2])
        s_hinge = utils.distance(p_primei, o_hinge)
        q_2 = s_hinge * math.tan(theta[1])
        q_4 = s_hinge * math.tan(theta[3])

        # finding that the p_next is on right or left  of o:
        cross_product = v_prime_i_nextt_prime ^ v_prime_o
        angle_io_ibefore = (math.sqrt(cross_product * cross_product)) / (l_prime_o * l_prime_nextt)

        l_o_before_prime = utils.distance(o_hinge, p_nextt_prime)

        # the distance between o and p_nextt_i:
        if (angle_io_ibefore <= 0):
            if (l_o_before_prime <= q_2):
                return p_nextt_prime
            else:
                circle_center = p_primei
                radius_circle = utils.distance(p_primei, p_nextt_prime)
                U = CG3dVector((p_nextt_prime[0] - p_primei[0]) / radius_circle,
                               (p_nextt_prime[1] - p_primei[1]) / radius_circle,
                               (p_nextt_prime[2] - p_primei[2]) / radius_circle)
                V = normal_vector_phi2 ^ U
                angle_difference = angle_io_ibefore - theta[1]
                p_nextt_prime = circle_center + radius_circle * math.cos(
                    angle_difference) * U + radius_circle * math.sin(angle_difference) * V
                return p_nextt_prime
        elif (angle_io_ibefore > 0):
            if (l_o_before_prime <= q_4):
                return p_nextt_prime
            else:
                circle_center = p_primei
                radius_circle = utils.distance(p_primei, p_nextt_prime)
                U = CG3dVector((p_nextt_prime[0] - p_primei[0]) / radius_circle,
                               (p_nextt_prime[1] - p_primei[1]) / radius_circle,
                               (p_nextt_prime[2] - p_primei[2]) / radius_circle)
                V = normal_vector_phi2 ^ U
                angle_difference = 3.14 / 2 + angle_io_ibefore - theta[3]
                p_nextt_prime = circle_center + radius_circle * math.cos(
                    angle_difference) * U + radius_circle * math.sin(angle_difference) * V
                return p_nextt_prime

    if (constraint_type == "ballAndSocket"):
        dot_prod = v_i_nextt * v_before_i
        l_i_nextt = utils.distance(p_i, p_nextt_i_first_pose)
        l_before_i = utils.distance(p_i, p_before_i)

        l_i_o = dot_prod / l_before_i

        # a unit vector of a line passing p(i-1) and P(i)
        unit_vec_beforee_i = CG3dVector((p_i[0] - p_before_i[0]) / l_before_i, (p_i[1] - p_before_i[1]) / l_before_i,
                                        (p_i[2] - p_before_i[2]) / l_before_i)
        # vector from p(i) to point o
        v_i_o = l_i_o * unit_vec_beforee_i

        # points based on the paper definition
        o = CG3dPoint(v_i_o[0] + p_i[0], v_i_o[1] + p_i[1], v_i_o[2] + p_i[2])
        s = utils.distance(o, p_i)


        # Semi ellipsoidal parameter qi (1,2,3,4)
        q1 = s * math.tan(theta[0])
        q2 = s * math.tan(theta[1])
        q3 = s * math.tan(theta[2])
        q4 = s * math.tan(theta[3])



        # change the coordinate to cross section of cone and calculating the (i-1)th position in it

        l_o_nextt = utils.distance(o, p_nextt_i)
        unit_v_o_nextt = CG3dVector((p_nextt_i[0] - o[0]) / l_o_nextt, (p_nextt_i[1] - o[1]) / l_o_nextt,
                                    (p_nextt_i[2] - o[2]) / l_o_nextt)

        y_t = l_o_nextt * math.sin(si)
        x_t = l_o_nextt * math.cos(si)

        # finding the sector of the target position

        if (x_t >= 0 and y_t > 0):
            sector = 1
        elif (x_t > 0 >= y_t):
            sector = 4
        elif (x_t <= 0 and y_t < 0) :
            sector = 3
        elif (x_t < 0 and y_t >= 0):
            sector = 2


        if(x_t==0,y_t==0):
            p_prime = CG3dPoint(0, 0, 0)
            return p_prime

        # elif (x_t < 0 < y_t) or (x_t < 0 and y_t == 0) or (x_t == 0 and y_t > 0):

        # checking that the target point is in ellipsoidal shape
        inbound = 0
        if ((x_t ** 2) / (q2 ** 2) + (y_t ** 2) / (q3 ** 2)) < 1 and sector == 1:
            inbound = 1
            p_prime = CG3dPoint(0, 0, 0)
            return p_prime
        elif ((x_t ** 2) / (q2 ** 2) + (y_t ** 2) / (q1 ** 2)) <= 1 and sector == 2:
            inbound = 1
            p_prime = CG3dPoint(0, 0, 0)
            return p_prime
        elif ((x_t ** 2) / (q4 ** 2) + (y_t ** 2) / (q1 ** 2)) <= 1 and sector == 3:
            inbound = 1
            p_prime = CG3dPoint(0, 0, 0)
            return p_prime
        elif ((x_t ** 2) / (q4 ** 2) + (y_t ** 2) / (q3 ** 2)) <= 1 and sector == 4:
            inbound = 1
            p_prime = CG3dPoint(0, 0, 0)
            return p_prime
        # if it is out bound of the ellipsoidal shape we should find the nearest point on ellipsoidal shape
        if inbound == 0 and sector == 1:
            x_nearest_point = find_nearest_point(q2, q3, sector, x_t, y_t)
            y_nearest_point = math.sqrt(abs(q3 ** 2 - q3 ** 2 / q2 ** 2 * x_nearest_point ** 2))
        elif inbound == 0 and sector == 2:
            x_nearest_point = find_nearest_point(q2, q1, sector, x_t, y_t)
            y_nearest_point = -math.sqrt(abs(q1 ** 2 - q1 ** 2 / q2 ** 2 * x_nearest_point ** 2))
        elif inbound == 0 and sector == 3:
            x_nearest_point = find_nearest_point(q4, q1, sector, x_t, y_t)
            y_nearest_point = -math.sqrt(abs(q1 ** 2 - q1 ** 2 / q4 ** 2 * x_nearest_point ** 2))
        elif inbound == 0 and sector == 4:
            x_nearest_point = find_nearest_point(q2, q3, sector, x_t, y_t)
            y_nearest_point = math.sqrt(abs(q3 ** 2 - q3 ** 2 / q2 ** 2 * x_nearest_point ** 2))

        l_o_next_in_ellipse_plane = math.sqrt(x_t ** 2 + y_t ** 2)
        l_o_nearest_point_in_ellipse_plane = math.sqrt(x_nearest_point ** 2 + y_nearest_point ** 2)
        diff_z_o_np = (l_o_nearest_point_in_ellipse_plane / l_o_next_in_ellipse_plane) * abs(o[2] - p_nextt_i[2])
        if (unit_v_o_nextt[2] >= 0):
            z_nearest_point = o[2] + diff_z_o_np
        else:
            z_nearest_point = o[2] - diff_z_o_np

        p_nearest_point_in_global = CG3dPoint(o[0] + unit_v_o_nextt[0] * x_nearest_point,
                                              o[1] + unit_v_o_nextt[1] * y_nearest_point,
                                              o[2] + unit_v_o_nextt[2] * z_nearest_point)


        return p_nearest_point_in_global




def func_point(x, a2, a1, a0):
    return a2 * x * x + a1 * x + a0


def derivative_func_point(x, a2, a1):
    return 2 * a2 * x + a1


def find_nearest_point(a, b, sector, x_t, y_t):
    x_k1 = x_t * a * b / math.sqrt(b ** 2 * x_t ** 2 + a ** 2 * y_t ** 2)
    if sector == 1 or sector == 2:
        sign = 1
    elif sector == 3 or sector == 4:
        sign = -1
    if abs(x_t) < a:
        x_k2 = x_t
    else:
        x_k2 = a * sign

    x0 = 1 / 2 * (x_k1 + x_k2)
    x = newton_Raphson_nearest_point(x0, a, b, x_t, y_t)
    return x


def func_nearest_point(x, a, b, x_t, y_t):
    a4 = -(b ** 2) * ((a ** 2) - (b ** 2)) / (a ** 2)
    a3 = 2 * (b ** 2) * x_t * ((a ** 2) - (b ** 2))
    a2 = -(b ** 2) * (a ** 2) * (x_t ** 2) + (b ** 2) * ((a ** 2) - (b ** 2)) ** 2 - (b ** 4) * (y_t ** 2)
    a1 = -2 * (a ** 2) * (b ** 2) * x_t * ((a ** 2) - (b ** 2))
    a0 = (a ** 4) * (b ** 2) * (x_t ** 2)
    return a4 * x * x * x * x + a3 * x * x * x + a2 * x * x + a1 * x + a0


def derivative_func_nearest_point(x, a, b, x_t, y_t):
    a4 = -(b ** 2) * ((a ** 2) - (b ** 2)) / (a ** 2)
    a3 = 2 * (b ** 2) * x_t * ((a ** 2) - (b ** 2))
    a2 = -(b ** 2) * (a ** 2) * (x_t ** 2) + (b ** 2) * ((a ** 2) - (b ** 2)) ** 2 - (b ** 4) * (y_t ** 2)
    a1 = -2 * (a ** 2) * (b ** 2) * x_t * ((a ** 2) - (b ** 2))
    return 4 * a4 * x * x * x + 3 * a3 * x * x + 2 * a2 * x + a1


# Function to find the root
def newton_Raphson_nearest_point(x, a, b, x_t, y_t):
    h = func_nearest_point(x, a, b, x_t, y_t) / derivative_func_nearest_point(x, a, b, x_t, y_t)
    counter = 0
    while (abs(h) >= 0.0001 ):
        h = func_nearest_point(x, a, b, x_t, y_t) / derivative_func_nearest_point(x, a, b, x_t, y_t)
        x = x - h
        counter = counter + 1
    return x


