import math


# Finding the distance between two arbitrary point
def distance(i, j):
    return math.sqrt((i[0] - j[0]) ** 2 + (i[1] - j[1]) ** 2 + (i[2] - j[2]) ** 2)
