import math
import numpy as np
import Mat as Mat


# Finding the distance between two arbitrary point
def distance(i, j):
    return math.sqrt((i[0] - j[0]) ** 2 + (i[1] - j[1]) ** 2 + (i[2] - j[2]) ** 2)

def find_rotation_quaternion(outer_quaternion, inner_quaternion):
        conjucate = [outer_quaternion[0], -outer_quaternion[1], -outer_quaternion[2], -outer_quaternion[3]]
        length = math.sqrt(outer_quaternion[0] ** 2 + outer_quaternion[1] ** 2 +
                           outer_quaternion[2] ** 2 + outer_quaternion[3] ** 2)
        inverse = np.dot(conjucate, (1 / length))
        rotation = Mat.multiply_two_quaternion(inner_quaternion, inverse)
        return rotation
