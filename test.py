import numpy as np
import matplotlib.pyplot as plt
from pycg3d.cg3d_point import CG3dPoint
from pycg3d.cg3d_vector import CG3dVector
import math  


joints=np.loadtxt("joints.txt")
i=2
p_i = CG3dPoint(joints[i][0],joints[i][1],joints[i][2])
p_nextI= CG3dPoint(joints[i+1][0],joints[i+1][1],joints[i+1][2])
p_target=CG3dPoint(joints[i-1][0],joints[i-1][1],joints[i-1][2])

        # some vector
v_ptarget=CG3dVector(p_target[0]-p_i[0],p_target[1]-p_i[1],p_target[2]-p_i[2])
v_nextp=CG3dVector(p_i[0]-p_nextI[0],p_i[1]-p_nextI[1],p_i[2]-p_nextI[2])
dotProd =  v_ptarget*  v_nextp
L_Vptarget=math.sqrt(v_ptarget*v_ptarget)
L_Vnextp=math.sqrt(v_nextp*v_nextp)

L_pO = dotProd/L_Vnextp

l=3
        #a unit vector of a line passing p(i+1) and P(i)
unitVec=CG3dVector((p_i[0]-p_nextI[0])/l,(p_i[1]-p_nextI[1])/l,(p_i[2]-p_nextI[2])/l)
po= L_pO*unitVec

o=CG3dPoint(po[0]+p_i[0],po[1]+p_i[1],po[2]+p_i[2])
print(math.tan(20))
print(o)
print(joints)
