# Step two : Arm and wrist analysis
from RebaCalculator.neck_score import neck_score
from RebaCalculator.trunck_score import trunk_score
from RebaCalculator.leg_score import leg_score
from RebaCalculator.upper_arm_score import upper_arm_score
from RebaCalculator.lower_arm_score import lower_arm_score
from RebaCalculator.wrist_score import wrist_score


def __init__(joints):
    table_a = {1: [[1, 2, 3, 4],
                   [2, 3, 4, 5],
                   [2, 4, 5, 6],
                   [3, 5, 6, 7],
                   [4, 6, 7, 8]],
               2: [[1, 2, 3, 4],
                   [3, 4, 5, 6],
                   [4, 5, 6, 7],
                   [5, 6, 7, 8],
                   [6, 7, 8, 9]],
               3: [[3, 3, 5, 6],
                   [4, 5, 6, 7],
                   [5, 6, 7, 8],
                   [6, 7, 8, 9],
                   [7, 8, 9, 9]]
               }
    table_b = {1: [[1, 2, 2],
                   [1, 2, 3],
                   [3, 4, 5],
                   [4, 5, 5],
                   [6, 7, 8],
                   [7, 8, 8]],
               2: [[1, 2, 3],
                   [2, 3, 4],
                   [4, 5, 5],
                   [5, 6, 7],
                   [7, 8, 8],
                   [8, 9, 9]],
               }
    table_c = [[1, 1, 1, 2, 3, 3, 4, 5, 6, 7, 7, 7],
               [1, 2, 2, 3, 4, 4, 5, 6, 6, 7, 7, 8],
               [2, 3, 3, 3, 4, 5, 6, 7, 7, 8, 8, 8],
               [3, 4, 4, 4, 5, 6, 7, 8, 8, 9, 9, 9],
               [4, 4, 4, 5, 6, 7, 8, 8, 9, 9, 9, 9],
               [6, 6, 6, 7, 8, 8, 9, 10, 10, 10, 10],
               [7, 7, 7, 8, 9, 9, 9, 10, 10, 11, 11, 11],
               [8, 8, 8, 9, 10, 10, 10, 10, 10, 11, 11, 11],
               [9, 9, 9, 10, 10, 10, 11, 11, 11, 12, 12, 12],
               [10, 10, 10, 11, 11, 11, 11, 12, 12, 12, 12, 12],
               [11, 11, 11, 11, 12, 12, 12, 12, 12, 12, 12, 12],
               [12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12]]

    # step1: locate neck position
    neck = neck_score(joints)
    # step2: locate trunck posture
    trunk = trunk_score(joints)
    # step3: locate legs
    leg = leg_score(joints)
    # step 4: Look up score in table _A
    posture_score_A = table_a[neck][trunk - 1][leg - 1]
    # step 5: load score
    # load = input("what is the load(in lbs) ")
    load = 0
    if (11 <= int(load) < 22):
        posture_score_A = posture_score_A + 1
    if (22 <= int(load)):
        posture_score_A = posture_score_A + 2
    # step 7: upper arm score
    upper_arm = upper_arm_score(joints)
    # step 8: lower arm score
    lower_arm = lower_arm_score(joints)
    # step 9: lower arm score
    wrist = wrist_score(joints)
    # step 10: Look up score in table _B
    posture_score_B = table_b[lower_arm][upper_arm - 1][wrist - 1]
    # step 11: coupling score
    # coupling = input("what is the coupling condition?(good(0) or fair(1) or poor(2) or unacceptable(3)? ")
    coupling = 0
    posture_score_B = posture_score_B + int(coupling)
    # step 12: look up score in table C
    posture_score_C = table_c[posture_score_A - 1][posture_score_B - 1]
    # step 13: Activity score
    # activity = input("what is activity level?(static or repeated(small action more than 4 min) or rapid or none?")
    activity ='static'
    if (activity == 'static' or activity == 'repeated' or activity == 'rapid'):
        posture_score_C = posture_score_C + 1
    return posture_score_C



