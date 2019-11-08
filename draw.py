import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d, Axes3D

def draw(joints,target,first_pos):
        x_prime=[]
        y_prime=[]
        z_prime=[]
        x=[]
        y=[]
        z=[]
        n=len(first_pos)
        for i in range(n):
            x_prime.append(joints[i][0])
            y_prime.append(joints[i][1])
            z_prime.append(joints[i][2])
            x.append(first_pos[i][0])
            y.append(first_pos[i][1])
            z.append(first_pos[i][2])
        fig = plt.figure()
        ax=Axes3D(fig)
        ax.plot3D(x,y,z, marker = 'o')
        ax.plot3D(x_prime,y_prime,z_prime)
        ax.scatter3D(target[0],target[1],target[2])
        plt.show()
