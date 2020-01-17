import math
from pycg3d.cg3d_point import CG3dPoint
from pycg3d.cg3d_vector import CG3dVector
from pycg3d import utils


def constraint(joints, body_index, i, theta, length, constraint_type):
    pi=22/7
    theta = (pi/180)*theta
    si = 0.5
    p_i = CG3dPoint(joints[body_index[i]][0], joints[body_index[i]][1], joints[body_index[i]][2])
    p_next_i = CG3dPoint(joints[body_index[i + 1]][0], joints[body_index[i + 1]][1], joints[body_index[i + 1]][2])
    p_before_i = CG3dPoint(joints[body_index[i - 1]][0], joints[body_index[i - 1]][1], joints[body_index[i - 1]][2])

    p_i_first_pose = CG3dPoint(length[body_index[i]][0], length[body_index[i]][1], length[body_index[i]][2])
    p_next_i_first_pose = CG3dPoint(length[body_index[i + 1]][0], length[body_index[i + 1]][1],
                                    length[body_index[i + 1]][2])
    p_before_i_first_pose = CG3dPoint(length[body_index[i - 1]][0], length[body_index[i - 1]][1],
                                      length[body_index[i - 1]][2])

    v_i_before = CG3dVector(p_before_i[0] - p_i[0], p_before_i[1] - p_i[1], p_before_i[2] - p_i[2])
    v_next_i = CG3dVector(p_i[0] - p_next_i[0], p_i[1] - p_next_i[1], p_i[2] - p_next_i[2])

    if (constraint_type == "hinge"):
        # defining a line l_1 in paper:
        l_before_i_firstpose = utils.distance(p_i_first_pose, p_before_i_first_pose)
        v_before_i_firstpose = CG3dVector((p_i_first_pose[0] - p_before_i_first_pose[0]) / l_before_i_firstpose,
                                          (p_i_first_pose[1] - p_before_i_first_pose[1]) / l_before_i_firstpose,
                                          (p_i_first_pose[2] - p_before_i_first_pose[2]) / l_before_i_firstpose)
        l_i_firstpose_next_firstpose = utils.distance(p_i_first_pose, p_next_i_first_pose)
        v_i_firstpose_next_firstpose = CG3dVector(
            (p_next_i_first_pose[0] - p_i_first_pose[0]) / l_i_firstpose_next_firstpose,
            (p_next_i_first_pose[1] - p_i_first_pose[1]) / l_i_firstpose_next_firstpose,
            (p_next_i_first_pose[2] - p_i_first_pose[2]) / l_i_firstpose_next_firstpose)

        normal_vector_phi1 = v_before_i_firstpose ^ v_i_firstpose_next_firstpose

        l_before_next_firstpose = utils.distance(p_next_i_first_pose, p_before_i_first_pose)
        v_before_next_firstpose = CG3dVector(
            (p_next_i_first_pose[0] - p_before_i_first_pose[0]) / l_before_next_firstpose,
            (p_next_i_first_pose[1] - p_before_i_first_pose[1]) / l_before_next_firstpose,
            (p_next_i_first_pose[2] - p_before_i_first_pose[2]) / l_before_next_firstpose)

        l1 = normal_vector_phi1 ^ v_before_next_firstpose

        # for phi_2
        l_before_next = utils.distance(p_next_i, p_before_i_first_pose)
        v_before_next = CG3dVector((p_next_i[0] - p_before_i_first_pose[0]) / l_before_next,
                                   (p_next_i[1] - p_before_i_first_pose[1]) / l_before_next,
                                   (p_next_i[2] - p_before_i_first_pose[2]) / l_before_next)
        v_l1 = CG3dVector(l1[0], l1[1], l1[2])

        normal_vector_phi2 = v_before_next ^ v_l1
        # the equation to solve for p_i new position

        # image of p_i on second page:
        d = v_before_i_firstpose * normal_vector_phi2
        v_before_hat = v_before_i_firstpose - d * normal_vector_phi2
        p_hat = CG3dPoint(v_before_hat[0] + p_before_i_first_pose[0], v_before_hat[1] + p_before_i_first_pose[1],
                          v_before_hat[2] + p_before_i_first_pose[2])
        # l2 which is a line between p_next and p_hat
        l_next_hat = utils.distance(p_next_i, p_hat)
        unit_l2 = CG3dVector((p_hat[0] - p_next_i[0]) / l_next_hat,
                             (p_hat[0] - p_next_i[0]) / l_next_hat,
                             (p_hat[0] - p_next_i[0]) / l_next_hat)
        p_primei = p_next_i + l_i_firstpose_next_firstpose * unit_l2

        l_prime_before = utils.distance(p_primei, p_before_i)
        v_prime_i_before = CG3dVector(p_before_i[0] - p_primei[0], p_before_i[1] - p_primei[1],
                                      p_before_i[2] - p_primei[2])
        # a unit vector of a line passing p(i-1) and P'(i)
        unit_prime_before = CG3dVector(v_prime_i_before[0] / l_prime_before, v_prime_i_before[1] / l_prime_before,
                                       v_prime_i_before[2] / l_prime_before)
        p_before_prime = p_primei + l_before_i_firstpose * unit_prime_before

        v_prime_i_before_prime = CG3dVector(p_before_prime[0] - p_primei[0], p_before_prime[1] - p_primei[1],
                                            p_before_prime[2] - p_primei[2])

        # finding o_hinge to know it is in limited zone
        v_next_prime_i = CG3dVector(p_primei[0] - p_next_i[0], p_primei[1] - p_next_i[1], p_primei[2] - p_next_i[2])
        l_next_prime = utils.distance(p_primei, p_next_i)

        l_prime_o = (v_next_prime_i * v_prime_i_before_prime) / l_next_prime

        # a unit vector of a line passing p(i+1) and P'(i)
        unit_vec_next_prime = CG3dVector((p_primei[0] - p_next_i[0]) / l_next_prime,
                                         (p_primei[1] - p_next_i[1]) / l_next_prime,
                                         (p_primei[2] - p_next_i[2]) / l_next_prime)

        v_prime_o = l_prime_o * unit_vec_next_prime
        o_hinge = CG3dPoint(v_prime_o[0] + p_primei[0], v_prime_o[1] + p_primei[1], v_prime_o[2] + p_primei[2])
        s_hinge = utils.distance(p_primei, o_hinge)
        q_2 = s_hinge * math.tan(theta[1])
        q_4 = s_hinge * math.tan(theta[3])

        # finding that the p_before is on right or left  of o:
        cross_product = v_prime_i_before_prime ^ v_prime_o
        angle_io_ibefore = (math.sqrt(cross_product * cross_product)) / (l_prime_o * l_prime_before)

        l_o_before_prime = utils.distance(o_hinge, p_before_prime)

        # the distance between o and p_before_i:
        if (angle_io_ibefore <= 0):
            if (l_o_before_prime <= q_2):
                return p_before_prime
            else:
                circle_center = p_primei
                radius_circle = utils.distance(p_primei, p_before_prime)
                U = CG3dVector((p_before_prime[0] - p_primei[0]) / radius_circle,
                               (p_before_prime[1] - p_primei[1]) / radius_circle,
                               (p_before_prime[2] - p_primei[2]) / radius_circle)
                V = normal_vector_phi2 ^ U
                angle_difference = angle_io_ibefore - theta[1]
                p_before_prime = circle_center + radius_circle * math.cos(
                    angle_difference) * U + radius_circle * math.sin(angle_difference) * V
                return p_before_prime
        elif (angle_io_ibefore > 0):
            if (l_o_before_prime <= q_4):
                return p_before_prime
            else:
                circle_center = p_primei
                radius_circle = utils.distance(p_primei, p_before_prime)
                U = CG3dVector((p_before_prime[0] - p_primei[0]) / radius_circle,
                               (p_before_prime[1] - p_primei[1]) / radius_circle,
                               (p_before_prime[2] - p_primei[2]) / radius_circle)
                V = normal_vector_phi2 ^ U
                angle_difference = 3.14 / 2 + angle_io_ibefore - theta[3]
                p_before_prime = circle_center + radius_circle * math.cos(
                    angle_difference) * U + radius_circle * math.sin(angle_difference) * V
                return p_before_prime
    if (constraint_type == "ballAndSocket"):
        dot_prod = v_i_before * v_next_i
        l_i_before = utils.distance(p_i_first_pose, p_before_i_first_pose)
        l_next_i = utils.distance(p_i_first_pose, p_next_i_first_pose)

        l_i_o = dot_prod / l_next_i

        # a unit vector of a line passing p(i+1) and P(i)
        unit_vec_next_i = CG3dVector((p_i[0] - p_next_i[0]) / l_next_i, (p_i[1] - p_next_i[1]) / l_next_i,
                                     (p_i[2] - p_next_i[2]) / l_next_i)
        # vector from p(i) to point o
        v_i_o = l_i_o * unit_vec_next_i

        # points based on the paper definition
        o = CG3dPoint(v_i_o[0] + p_i[0], v_i_o[1] + p_i[1], v_i_o[2] + p_i[2])
        s = utils.distance(o, p_i)

        # Semi ellipsoidal parameter qi (1,2,3,4)
        q1 = s * math.tan(theta[0])
        q2 = s * math.tan(theta[1])
        q3 = s * math.tan(theta[2])
        q4 = s * math.tan(theta[3])

        # change the coordinate to cross section of cone and calculating the (i-1)th position in it

        l_o_before = utils.distance(o, p_before_i)
        unit_v_o_before = CG3dVector((p_before_i[0] - o[0]) / l_o_before, (p_before_i[1] - o[1]) / l_o_before,
                                     (p_before_i[2] - o[2]) / l_o_before)

        y_t = l_o_before * math.cos(si)
        x_t = l_o_before * math.sin(si)

        # finding the sector of the target position

        if (x_t > 0 and y_t > 0) or (x_t >= 0 and y_t == 0) or (x_t == 0 and y_t > 0):
            sector = 1
        elif (x_t > 0 > y_t) or (x_t > 0 and y_t == 0) or (x_t == 0 and y_t < 0):
            sector = 2
        elif (x_t < 0 and y_t < 0) or (x_t < 0 and y_t == 0) or (x_t == 0 and y_t < 0):
            sector = 3
        else:
            sector = 4
        # elif (x_t < 0 < y_t) or (x_t < 0 and y_t == 0) or (x_t == 0 and y_t > 0):

        # checking that the target point is in ellipsoidal shape
        inbound = 0
        if ((x_t ** 2) / (q2 ** 2) + (y_t ** 2) / (q3 ** 2)) < 1 and sector == 1:
            inbound = 1
            p_prime = CG3dPoint(0, 0, 0)
            return p_prime
        elif ((x_t ** 2) / (q2 ** 2) + (y_t ** 2) / (q1 ** 2)) < 1 and sector == 2:
            inbound = 1
            p_prime = CG3dPoint(0, 0, 0)
            return p_prime
        elif ((x_t ** 2) / (q4 ** 2) + (y_t ** 2) / (q1 ** 2)) < 1 and sector == 3:
            inbound = 1
            p_prime = CG3dPoint(0, 0, 0)
            return p_prime
        elif ((x_t ** 2) / (q4 ** 2) + (y_t ** 2) / (q3 ** 2)) < 1 and sector == 4:
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

        # finding 2 point on 2d cross section plane
        point1 = points_on_2d_cone_cross_section(o, p_i, p_before_i, unit_v_o_before, x_t, y_t, 1)
        point2 = points_on_2d_cone_cross_section(o, p_i, p_before_i, unit_v_o_before, x_t, y_t, 2)

        # finding two vector in 2D cross section plane:
        l_o_point1 = utils.distance(o, point1)
        l_o_point2 = utils.distance(o, point2)
        v1 = CG3dVector((-point1[0] + o[0]) / l_o_point1, (-point1[1] + o[1]) / l_o_point1,
                        (-point1[2] + o[2]) / l_o_point1)
        v2 = CG3dVector((-point2[0] + o[0]) / l_o_point2, (-point2[1] + o[1]) / l_o_point2,
                        (-point2[2] + o[2]) / l_o_point2)

        # 3D coordinate of nearest point in ellipse:
        x_phat = o[0] + x_nearest_point * v1[0] + y_nearest_point * v2[0]
        y_phat = o[1] + x_nearest_point * v1[1] + y_nearest_point * v2[1]
        z_phat = o[2] + x_nearest_point * v1[2] + y_nearest_point * v2[2]

        # unit vector from p(i)to Phat(i-1)
        p_hat = CG3dPoint(x_phat, y_phat, z_phat)
        l_i_p_hat = utils.distance(p_i, p_hat)
        unit_vector_i_p_hat = CG3dVector((p_hat[0] - p_i[0]) / l_i_p_hat, (p_hat[1] - p_i[1]) / l_i_p_hat,
                                         (p_hat[2] - p_i[2]) / l_i_p_hat)

        # position of p'(i-1).......

        p_prime = CG3dPoint(p_i[0] + l_i_before * unit_vector_i_p_hat[0], p_i[1] + l_i_before * unit_vector_i_p_hat[1],
                            p_i[2] + l_i_before * unit_vector_i_p_hat[2])

        return p_prime


def points_on_2d_cone_cross_section(o, p_i, p_target, unit_vector_o_target, x_t, y_t, i):
    # The cone cross section is a plane Ax+By+Cz+D=0
    # any point in this plane (x1,y1,z1) if lies on the x axis of the 2d coordinate in plane should:
    # be on plane
    # vector from o to this point dot product unit vector has some known value
    # this point distance to o is y_t
    si = 0.5
    l_o_i = utils.distance(o, p_i)

    normal_plane = CG3dVector((p_i[0] - o[0]) / l_o_i, (p_i[1] - o[1]) / l_o_i, (p_i[2] - o[2]) / l_o_i)

    A = normal_plane[0]
    B = normal_plane[1]
    C = normal_plane[2]
    D = -(A * p_target[0] + B * p_target[1] + C * p_target[2])

    F1 = unit_vector_o_target[0] - unit_vector_o_target[2] * A / C
    F2 = unit_vector_o_target[1] - unit_vector_o_target[2] * B / C
    F3 = -A / C + (B / C) * (F1 / F2)
    a2 = 1 + (F1 / F2) ** 2 + F3 ** 2

    # The first point on y axis

    if i == 1:
        M1 = -unit_vector_o_target[0] * o[0] - unit_vector_o_target[1] * o[1] - unit_vector_o_target[2] * o[2] - \
             unit_vector_o_target[2] * D / C - y_t * math.cos(si)
        M2 = (B / C) * (M1 / F2) - D / C
        a1 = -2 * o[0] + 2 * (F1 / F2) * (o[1] + M1 / F2) + 2 * F3 * (M2 - o[2])
        a0 = o[0] ** 2 + (o[1] + M1 / F2) ** 2 + (M2 - o[2]) ** 2 - y_t ** 2

        x1 = newton_raphson_point_on_cross_section_cone(o[0], a2, a1, a0)
        y1 = -(M1 / F2) - (F1 / F2) * x1
        z1 = F3 * x1 + M2

        point1 = CG3dPoint(x1, y1, z1)
        return point1
    else:
        M1 = -unit_vector_o_target[0] * o[0] - unit_vector_o_target[1] * o[1] - unit_vector_o_target[2] * o[2] - \
             unit_vector_o_target[
                 2] * D / C - x_t * math.sin(si)
        M2 = (B / C) * (M1 / F2) - D / C
        a1 = -2 * o[0] + 2 * (F1 / F2) * (o[1] + M1 / F2) + 2 * F3 * (M2 - o[2])
        a0 = o[0] ** 2 + (o[1] + M1 / F2) ** 2 + (M2 - o[2]) ** 2 - x_t ** 2
        # The second point on x axis
        x2 = newton_raphson_point_on_cross_section_cone(o[0], a2, a1, a0)
        y2 = -(M1 / F2) - (F1 / F2) * x2
        z2 = F3 * x2 + M2
        point2 = CG3dPoint(x2, y2, z2)
        return point2


def newton_raphson_point_on_cross_section_cone(x, a2, a1, a0):
    h = func_point(x, a2, a1, a0) / derivative_func_point(x, a2, a1)
    while abs(h) >= 0.0001:
        h = func_point(x, a2, a1, a0) / derivative_func_point(x, a2, a1)
        x = x - h
    return x


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
    while abs(h) >= 0.0001:
        h = func_nearest_point(x, a, b, x_t, y_t) / derivative_func_nearest_point(x, a, b, x_t, y_t)
        x = x - h
    return x
