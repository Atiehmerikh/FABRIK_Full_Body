def multiply_two_quaternion(q1, q2):
    a = q1[0]
    b = q1[1]
    c = q1[2]
    d = q1[3]
    e = q2[0]
    f = q2[1]
    g = q2[2]
    h = q2[3]

    m0 = round(a * e - b * f - c * g - d * h, 2)
    m1 = round(b * e + a * f + c * h - d * g, 2)
    m2 = round(a * g - b * h + c * e + d * f, 2)
    m3 = round(a * h + b * g - c * f + d * e, 2)
    return [m0, m1, m2, m3]