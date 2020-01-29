import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d, Axes3D
from pycg3d.cg3d_point import CG3dPoint
from pycg3d import utils
import numpy as np


class Draw:
    def __init__(self, joints, target, first_pos, rightArmIndex, leftArmIndex, upperChain, lowerChain, rightLeg,
                 leftLeg, neck, head):
        self.joints = joints
        self.first_pose = first_pos
        self.target = target
        self.right_arm_index = rightArmIndex
        self.left_arm_index = leftArmIndex
        self.upper_chain_index = upperChain
        self.lower_chain_index = lowerChain
        self.right_leg_index = rightLeg
        self.left_leg_index = leftLeg
        self.neck_index = neck
        self.head_index = head

    def set_axes_radius(self, ax, origin, radius):
        ax.set_xlim3d([origin[0] - radius, origin[0] + radius])
        ax.set_ylim3d([origin[1] - radius, origin[1] + radius])
        ax.set_zlim3d([origin[2] - radius, origin[2] + radius])

    def set_axes_equal(self, ax):
        limits = np.array([
            ax.get_xlim3d(),
            ax.get_ylim3d(),
            ax.get_zlim3d(),
        ])

        origin = np.mean(limits, axis=1)
        radius = 0.5 * np.max(np.abs(limits[:, 1] - limits[:, 0]))
        self.set_axes_radius(ax, origin, radius)

    def fill_array(self, body_part_index, joints):
        w, h = 3, len(body_part_index)
        coordinate = [[0 for x in range(w)] for y in range(h)]
        # coordinate =[][len(body_part_index)]
        x = []
        y = []
        z = []
        for i in range(len(body_part_index)):
            x.append(joints[body_part_index[i]][0])
            y.append(joints[body_part_index[i]][1])
            z.append(joints[body_part_index[i]][2])
        coordinate[0][:] = x
        coordinate[1][:] = y
        coordinate[2][:] = z
        return coordinate

    def draw_final(self):
        # The Second Pose
        coordinate = self.fill_array(self.right_arm_index, self.joints)
        x_prime_right_arm = coordinate[0]
        y_prime_right_arm = coordinate[1]
        z_prime_right_arm = coordinate[2]

        coordinate = self.fill_array(self.left_arm_index, self.joints)
        x_primeLArm = coordinate[0]
        y_primeLArm = coordinate[1]
        z_primeLArm = coordinate[2]

        coordinate = self.fill_array(self.upper_chain_index, self.joints)
        x_primeU = coordinate[0]
        y_primeU = coordinate[1]
        z_primeU = coordinate[2]

        coordinate = self.fill_array(self.lower_chain_index, self.joints)
        x_primeL = coordinate[0]
        y_primeL = coordinate[1]
        z_primeL = coordinate[2]

        coordinate = self.fill_array(self.right_leg_index, self.joints)
        x_primeRLeg = coordinate[0]
        y_primeRLeg = coordinate[1]
        z_primeRLeg = coordinate[2]

        coordinate = self.fill_array(self.left_leg_index, self.joints)
        x_primeLLeg = coordinate[0]
        y_primeLLeg = coordinate[1]
        z_primeLLeg = coordinate[2]

        # # ...............................................................................................
        coordinate = self.fill_array(self.right_arm_index, self.first_pose)
        x_RArm = coordinate[0]
        y_RArm = coordinate[1]
        z_RArm = coordinate[2]

        coordinate = self.fill_array(self.left_arm_index, self.first_pose)
        x_LArm = coordinate[0]
        y_LArm = coordinate[1]
        z_LArm = coordinate[2]

        coordinate = self.fill_array(self.upper_chain_index, self.first_pose)
        x_U = coordinate[0]
        y_U = coordinate[1]
        z_U = coordinate[2]

        coordinate = self.fill_array(self.lower_chain_index, self.first_pose)
        x_L = coordinate[0]
        y_L = coordinate[1]
        z_L = coordinate[2]

        coordinate = self.fill_array(self.right_leg_index, self.first_pose)
        x_RLeg = coordinate[0]
        y_RLeg = coordinate[1]
        z_RLeg = coordinate[2]

        coordinate = self.fill_array(self.left_leg_index, self.first_pose)
        x_LLeg = coordinate[0]
        y_LLeg = coordinate[1]
        z_LLeg = coordinate[2]

        fig = plt.figure()
        ax = fig.gca(projection='3d')

        ax.plot3D(x_RArm, y_RArm, z_RArm, color='red')
        ax.plot3D(x_LArm, y_LArm, z_LArm, color='red')
        ax.plot3D(x_U, y_U, z_U, color='red')
        ax.plot3D(x_L, y_L, z_L, color='red')
        ax.plot3D(x_RLeg, y_RLeg, z_RLeg, color='red')
        ax.plot3D(x_LLeg, y_LLeg, z_LLeg, color='red')
        # # ax.plot3D(x_Neck, y_Neck, z_Neck, color='red')
        # # ax.plot3D(x_head, y_head, z_head, color='red')

        ax.plot3D(x_prime_right_arm, y_prime_right_arm,
                  z_prime_right_arm, color='green')
        ax.plot3D(x_primeLArm, y_primeLArm, z_primeLArm, color='green')
        ax.plot3D(x_primeU, y_primeU, z_primeU, color='green')
        ax.plot3D(x_primeL, y_primeL, z_primeL, color='green')
        ax.plot3D(x_primeRLeg, y_primeRLeg, z_primeRLeg, color='green')
        ax.plot3D(x_primeLLeg, y_primeLLeg, z_primeLLeg, color='green')
        # ax.plot3D(x_primeNeck, y_primeNeck, z_primeNeck, color='green')
        # ax.plot3D(x_primeHead, y_primeHead, z_primeHead, color='green')

        # FrightUpperArmLength = distance_calculation(joints[0], joints[9])
        # LrightUpperArmLength = distance_calculation(first_pos[0], first_pos[9])
        #
        # # ax.scatter3D(joints[head[1]][0], joints[head[1]][1], joints[head[1]][2],edgecolors='green')
        ax.scatter3D(self.target[0], self.target[1], self.target[2])
        #
        self.set_axes_equal(ax)
        plt.show()

    def distance_calculation(i, j):
        i_point = CG3dPoint(i[0], i[1], i[2])
        j_point = CG3dPoint(j[0], j[1], j[2])

        return utils.distance(i_point, j_point)
