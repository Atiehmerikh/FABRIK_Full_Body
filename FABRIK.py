import math  
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d, Axes3D
from pycg3d.cg3d_point import CG3dPoint
from pycg3d.cg3d_vector import CG3dVector
from pycg3d import utils
from pycg3d.cg3d_plane import CG3dPlanePN


class FABRIK:
    def __init__(self,joints,target,Theta,orientation):
        self.n=len(joints)
        self.joints=joints
        self.Theta=Theta
        self.orientation=orientation
        lengths=[]
        for i in range(1,len(joints)):
            length = math.sqrt((joints[i][0]-joints[i-1][0])**2+
                               (joints[i][1]-joints[i-1][1])**2+
                                (joints[i][2]-joints[i-1][1])**2)
            lengths.append(length)
        self.tolerance=0.1
        self.target=target
        self.lengths=lengths
        self.origin=joints[0]
        self.totallength=sum(lengths)
    def forward(self):
        # set end effector as target
        print("Forward begins")
        print("number of joints: ",self.n)
        self.joints[self.n-1]=self.target;
        for i in range(self.n-2,0,-1):
            r = math.sqrt((self.joints[i][0]-self.joints[i+1][0])**2+
                          (self.joints[i][1]-self.joints[i+1][1])**2+
                          (self.joints[i][2]-self.joints[i+1][2])**2)
            landa = self.lengths[i]/r
            # find new joint position
            pos=(1-landa)*self.joints[i+1]+landa*self.joints[i]
            constraintReturn=self.constraint(i,self.Theta)
            if constraintReturn==0:
                self.joints[i]=pos
            else:
                self.joints[i]=self.constraint(i,self.Theta)
    def backward(self):
        # set root as initial position
        self.joints[0]=self.origin;
        for i in range(0,self.n-2):
            r = math.sqrt((self.joints[i][0]-self.joints[i+1][0])**2+
                               (self.joints[i][1]-self.joints[i+1][1])**2+
                          (self.joints[i][2]-self.joints[i+1][2])**2)
            landa = self.lengths[i]/r
            # find new joint position
            pos=(1-landa)*self.joints[i]+landa*self.joints[i+1]
            self.joints[i+1]=pos;
    
    def solve(self):
        distance = math.sqrt((self.joints[0][0]-self.target[0])**2+
                             (self.joints[0][1]-self.target[1])**2+
                             (self.joints[0][2]-self.target[2])**2)
        if distance > self.totallength:
        # target is out of reach
            for i in range(1,self.n-1):
                r=math.sqrt((self.joints[i][0]-self.target[0])**2+
                             (self.joints[i][1]-self.target[1])**2+
                            (self.joints[i][2]-self.target[2])**2)
                landa=self.lengths[i-1]/r;
        # find new joint position
                self.joints[i]= [a1 + a2 for a1, a2 in zip([(1-landa)*x for x in self.joints[i-1]], [landa*x for x in self.target])]
                print("target is out of reach!!!!!!!")

        else:
        # target is in reach
            bcount=0;
            dif=math.sqrt((self.joints[self.n-1][0]-self.target[0])**2+
                             (self.joints[self.n-1][1]-self.target[1])**2+
                          (self.joints[self.n-1][2]-self.target[2])**2)
            while dif > self.tolerance:
                self.forward()
                self.backward()
                dif = math.sqrt((self.joints[self.n-1][0]-self.target[0])**2+
                             (self.joints[self.n-1][1]-self.target[1])**2+
                                (self.joints[self.n-1][2]-self.target[2])**2)
                bcount=bcount+1
                if bcount>10:
                    break
            print("new joints position: ",self.joints)
            self.draw()
    def draw(self):
        first_pos = np.loadtxt("joints.txt")
        x_prime=[]
        y_prime=[]
        z_prime=[]
        x=[]
        y=[]
        z=[]
        for i in range(self.n):
            x_prime.append(self.joints[i][0])
            y_prime.append(self.joints[i][1])
            z_prime.append(self.joints[i][2])
            x.append(first_pos[i][0])
            y.append(first_pos[i][1])
            z.append(first_pos[i][2])
        fig = plt.figure()
        ax=Axes3D(fig)
        ax.plot3D(x,y,z, marker = 'o')
        ax.plot3D(x_prime,y_prime,z_prime)
        ax.scatter3D(self.target[0],self.target[1],self.target[2])
        plt.show()
    def constraint(self,i,Theta):
        #define points
        p_i = CG3dPoint(self.joints[i][0],self.joints[i][1],self.joints[i][2])
        p_nextI= CG3dPoint(self.joints[i+1][0],self.joints[i+1][1],self.joints[i+1][2])
        p_target=CG3dPoint(self.joints[i-1][0],self.joints[i-1][1],self.joints[i-1][2])

        # some vector
        v_ptarget=CG3dVector(p_target[0]-p_i[0],p_target[1]-p_i[1],p_target[2]-p_i[2])
        v_nextp=CG3dVector(p_i[0]-p_nextI[0],p_i[1]-p_nextI[1],p_i[2]-p_nextI[2])
        dotProd =  v_ptarget*  v_nextp
        L_Vptarget=math.sqrt(v_ptarget*v_ptarget)
        L_Vnextp=math.sqrt(v_nextp*v_nextp)

        L_po = dotProd/L_Vnextp

        
        #a unit vector of a line passing p(i+1) and P(i)
        unitVec=CG3dVector((p_i[0]-p_nextI[0])/self.lengths[i],(p_i[1]-p_nextI[1])/self.lengths[i],(p_i[2]-p_nextI[2])/self.lengths[i])
        po= L_po*unitVec


        o=CG3dPoint(po[0]+p_i[0],po[1]+p_i[1],po[2]+p_i[2])
        s=utils.distance(o,p_i)

        #Semi ellipsoidal parameter qi (1,2,3,4)
        q1=s*math.tan(Theta[0])
        q2=s*math.tan(Theta[1])
        q3=s*math.tan(Theta[2])
        q4=s*math.tan(Theta[3])


        # change the coordinate to cross section of cone and calculating the (i-1)th position in it

        si=self.orientation

        L_otarget = utils.distance(o,p_target)
        unitVoTarget = CG3dVector((p_target[0]-o[0])/L_otarget,(p_target[1]-o[1])/L_otarget,(p_target[2]-o[2])/L_otarget)
        
        y_t=L_otarget*math.cos(si)
        x_t=L_otarget*math.sin(si)


        # The cone cross section is a plane Ax+By+Cz+D=0
        # any point in this plane (x1,y1,z1) if lies on the x axis of the 2d coordinate in plane should:
        # be on plane
        # vector from o to this point dot product unitvec has some known value
        # this point distance to o is y_t
        Lopi=utils.distance(o,p_i)
        
        unitPlane=CG3dVector((p_i[0]-o[0])/Lopi,(p_i[1]-o[1])/Lopi,(p_i[2]-o[2])/Lopi)
        
        
        A=unitPlane[0]
        B=unitPlane[1]
        C=unitPlane[2]
        D=-(A*p_target[0]+B*p_target[1]+C*p_target[2])


        F1 = unitVoTarget[0]-unitVoTarget[2]*A/C
        F2 = unitVoTarget[1]-unitVoTarget[2]*B/C
    


        F3 =  -A/C+(B/C)*(F1/F2)
        M1 = -unitVoTarget[0]*o[0]-unitVoTarget[1]*o[1]-unitVoTarget[2]*o[2]-unitVoTarget[2]*D/C-y_t*math.cos(si)
        M2 = (B/C)*(M1/F2)-D/C
        a2=1+(F1/F2)**2+F3**2
        a1=-2*o[0]+2*(F1/F2)*(o[1]+M1/F2)+2*F3*(M2-o[2])
        a0=o[0]**2+(o[1]+M1/F2)**2+(M2-o[2])**2-y_t**2
        
        # The first point on y axis
        x1=self.newtonRaphsonPointOnCrossSectionCone(o[0],a2,a1,a0)
        y1=-(M1/F2)-(F1/F2)*x1
        z1=F3*x1+M2

        point1=CG3dPoint(x1,y1,z1)

        M1 = -unitVoTarget[0]*o[0]-unitVoTarget[1]*o[1]-unitVoTarget[2]*o[2]-unitVoTarget[2]*D/C-x_t*math.sin(si)
        M2 = (B/C)*(M1/F2)-D/C

        a1=-2*o[0]+2*(F1/F2)*(o[1]+M1/F2)+2*F3*(M2-o[2])
        a0=o[0]**2+(o[1]+M1/F2)**2+(M2-o[2])**2-x_t**2


    
        # The second point on x axis 
        x2=self.newtonRaphsonPointOnCrossSectionCone(o[0],a2,a1,a0)
        y2=-(M1/F2)-(F1/F2)*x2
        z2=F3*x2+M2


        point2=CG3dPoint(x2,y2,z2)

        
        # finding the sector of the target position
        
        if (x_t>0 and y_t>0)or(x_t>=0 and y_t==0)or(x_t==0 and y_t>0 ):
            sector=1
        elif (x_t>0 and y_t<0)or(x_t>0 and y_t==0)or(x_t==0 and y_t<0 ):
            sector=2
        elif (x_t<0 and y_t<0)or(x_t<0 and y_t==0)or(x_t==0 and y_t<0 ):
            sector=3
        elif (x_t<0 and y_t>0)or(x_t<0 and y_t==0)or(x_t==0 and y_t>0 ):
            sector=4


        #cheking that the target point is in ellipsoidal shape
        inbound=0;
        if (((x_t**2)/(q2**2)+(y_t**2)/(q3**2))<1 and sector==1):
            inbound=1
            return 0
        elif ((x_t**2)/(q2**2)+(y_t**2)/(q1**2))<1 and sector==2:
            inbound=1
            return 0
        elif ((x_t**2)/(q4**2)+(y_t**2)/(q1**2))<1 and sector==3:
            inbound=1
            return 0
        elif ((x_t**2)/(q4**2)+(y_t**2)/(q3**2))<1 and sector==4:
            inbound=1
            return 0
        # if it is out bound of the ellipsoidal shape we should find the nearest point on ellipsoidal shape
        if inbound==0 and sector==1:
            xNP=self.findNearestPoint(q2,q3,sector,x_t,y_t)
            yNP=math.sqrt(abs(q3**2-q3**2/q2**2*xNP**2))
        elif inbound==0 and sector==2:
            xNP=self.findNearestPoint(q2,q1,sector,x_t,y_t)
            yNP=-math.sqrt(abs(q1**2-q1**2/q2**2*xNP**2))
        elif inbound==0 and sector==3:
            xNP=self.findNearestPoint(q4,q1,sector,x_t,y_t)
            yNP=-math.sqrt(abs(q1**2-q1**2/q4**2*xNP**2))
        elif inbound==0 and sector==4:
            xNP=self.findNearestPoint(q2,q3,sector,x_t,y_t)
            yNP=math.sqrt(abs(q3**2-q3**2/q2**2*xNP**2))


        # finding two vector in 2D cross section plane:
        L_op1=utils.distance(o,point1)
        L_op2=utils.distance(o,point2)
        v1=CG3dVector((-point1[0]+o[0])/L_op1,(-point1[1]+o[1])/L_op1,(-point1[2]+o[2])/L_op1)
        v2=CG3dVector((-point2[0]+o[0])/L_op2,(-point2[1]+o[1])/L_op2,(-point2[2]+o[2])/L_op2)



        #3D coordinate of nearest point in ellipse:
        x_phat= o[0]+xNP*v1[0]+yNP*v2[0]
        y_phat= o[1]+xNP*v1[1]+yNP*v2[1]
        z_phat= o[2]+xNP*v1[2]+yNP*v2[2]
        # unit vector from p(i)to Phat(i-1)
        p_hat=CG3dPoint(x_phat,y_phat,z_phat)
        L_PPhat=utils.distance(p_i,p_hat)
        u_PPhat=CG3dVector((p_hat[0]-p_i[0])/L_PPhat,(p_hat[1]-p_i[1])/L_PPhat,(p_hat[2]-p_i[2])/L_PPhat)

        # position of p'(i-1)
        pprime = CG3dPoint(p_i[0]+self.lengths[i-1]*u_PPhat[0],p_i[1]+self.lengths[i-1]*u_PPhat[1],p_i[2]
                               +self.lengths[i-1]*u_PPhat[2])
        return pprime

    def newtonRaphsonPointOnCrossSectionCone(self,x, a2,a1,a0):
        h = self.funcPoint(x,a2,a1,a0) / self.derivFuncPoint(x,a2,a1) 
        while abs(h) >= 0.0001: 
            h = self.funcPoint(x,a2,a1,a0) / self.derivFuncPoint(x,a2,a1)  
            x = x - h 
        return x   
    def funcPoint(self,x,a2,a1,a0):
        return a2*  x *x+ a1* x+ a0
    def derivFuncPoint(self, x,a2,a1):
        return 2*a2* x  + a1


    def findNearestPoint(self,a,b,sector,x_t,y_t):
        x_k1=x_t*a*b/math.sqrt(b**2*x_t**2+a**2*y_t**2)
        if sector==1 or sector==2:
            Sign=1
        elif sector==3 or sector==4:
            Sign=-1
        if abs(x_t)<a:
            x_k2=x_t
        else:
            x_k2=a*Sign

        x0=1/2*(x_k1+x_k2)
        x=self.newtonRaphsonNP(x0,a,b,x_t,y_t)
        return x
    def funcNP(self,x,a,b,x_t,y_t):
        a4=-(b**2)*((a**2)-(b**2))/(a**2)
        a3=2*(b**2)*x_t*((a**2)-(b**2))
        a2=-(b**2)*(a**2)*(x_t**2)+(b**2)*((a**2)-(b**2))**2-(b**4)*(y_t**2)
        a1=-2*(a**2)*(b**2)*x_t*((a**2)-(b**2))
        a0=(a**4)*(b**2)*(x_t**2)
        return a4* x * x * x *x+ a3* x*x*x+a2*x*x +a1*x +a0
    def derivFuncNP(self, x,a,b,x_t,y_t ):
        a4=-(b**2)*((a**2)-(b**2))/(a**2)
        a3=2*(b**2)*x_t*((a**2)-(b**2))
        a2=-(b**2)*(a**2)*(x_t**2)+(b**2)*((a**2)-(b**2))**2-(b**4)*(y_t**2)
        a1=-2*(a**2)*(b**2)*x_t*((a**2)-(b**2))
        return 4*a4* x * x * x + 3*a3* x*x + 2*a2*x +a1
    # Function to find the root 
    def newtonRaphsonNP(self, x,a,b,x_t,y_t ): 
        h = self.funcNP(x,a,b,x_t,y_t) / self.derivFuncNP(x,a,b,x_t,y_t) 
        while abs(h) >= 0.0001: 
            h = self.funcNP(x,a,b,x_t,y_t) / self.derivFuncNP(x,a,b,x_t,y_t) 
            x = x - h 
        return x
            
            
manipulator = FABRIK(np.loadtxt("joints.txt"),[10.0,5.0,1.0],np.loadtxt("theta.txt"),0.5)
manipulator.solve()
