import math  
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d, Axes3D
from pycg3d.cg3d_point import CG3dPoint
from pycg3d.cg3d_vector import CG3dVector
from pycg3d import utils
from pycg3d.cg3d_plane import CG3dPlanePN
from draw import draw 


class FABRIKARM:
    def __init__(self,joints,target,origin,Theta,orientation,constraintType):
        self.n=len(joints)
        self.joints = joints
        self.Theta = Theta
        self.orientation = orientation
        self.constraintType = constraintType
        lengths=[]
        for i in range(1,len(joints)):
            length = math.sqrt((joints[i][0]-joints[i-1][0])**2+
                               (joints[i][1]-joints[i-1][1])**2+
                                (joints[i][2]-joints[i-1][1])**2)
            lengths.append(length)
        self.tolerance=0.01
        self.target=target
        self.lengths=lengths
        self.origin=origin
        self.totallength=sum(lengths)
    def forward(self):
        # set end effector as target
        self.joints[self.n-1]=self.target;
        for i in range(self.n-2,-1,-1):
            r = math.sqrt((self.joints[i][0]-self.joints[i+1][0])**2+
                          (self.joints[i][1]-self.joints[i+1][1])**2+
                          (self.joints[i][2]-self.joints[i+1][2])**2)
            landa = self.lengths[i]/r
            # find new joint position
            pos=(1-landa)*self.joints[i+1]+landa*self.joints[i]
            constraintReturn=self.constraint(i,self.Theta[i],self.constraintType[i])
            print(constraintReturn)
            if constraintReturn[0] == 0:
                self.joints[i]=pos
            else:
                for j in range(3):
                    self.joints[i][j]=constraintReturn[j]
        print("joints position After forward: ",self.joints)

    def backward(self):
        print("backward")
        # set root as initial position
        self.joints[0]=self.origin;
        for i in range(1,self.n):
            print("the ",i," th joint in backward")

            r = math.sqrt((self.joints[i][0]-self.joints[i-1][0])**2+
                               (self.joints[i][1]-self.joints[i-1][1])**2+
                          (self.joints[i][2]-self.joints[i-1][2])**2)
            landa = self.lengths[i-1]/r
            # find new joint position
            pos=(1-landa)*self.joints[i-1]+landa*self.joints[i]
            self.joints[i]=pos;
        print("joints position After backward: ",self.joints)       
    def solve(self):
        for q in range (0,4):
            for p in range(0,4):
                if self.Theta[q][p]>=2:
                    print("the theta is out of range!!!")
                    return
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
            lengthsN=[]
            for i in range(1,len(self.joints)):
                lengthN = math.sqrt((self.joints[i][0]-self.joints[i-1][0])**2+(self.joints[i][1]-self.joints[i-1][1])**2+(self.joints[i][2]-self.joints[i-1][1])**2)
                lengthsN.append(lengthN)
            draw(self.joints,self.target,np.loadtxt("joints.txt"))
    
    def constraint(self,i,Theta,constraintType):
            p_i = CG3dPoint(self.joints[i][0],self.joints[i][1],self.joints[i][2])
            p_nextI= CG3dPoint(self.joints[i+1][0],self.joints[i+1][1],self.joints[i+1][2])
            p_target=CG3dPoint(self.joints[i-1][0],self.joints[i-1][1],self.joints[i-1][2])

            v_ptarget=CG3dVector(p_target[0]-p_i[0],p_target[1]-p_i[1],p_target[2]-p_i[2])
            v_nextp=CG3dVector(p_i[0]-p_nextI[0],p_i[1]-p_nextI[1],p_i[2]-p_nextI[2])

            if constraintType =="hinge":
                NfirstPage=  v_ptarget^v_nextp
                # arbitrary point on first page
                p_arbit=CG3dPoint(self.joints[i-1][0]+5,self.joints[i-1][1],self.joints[i-1][2])
                p_globalTarget=CG3dPoint(self.target[0],self.target[1],self.target[2])

                v_l1=CG3dVector(p_arbit[0]-p_target[0],p_arbit[1]-p_target[1],p_arbit[2]-p_target[2])
                v_lprime1=CG3dVector(p_globalTarget[0]-p_target[0],p_globalTarget[1]-p_target[1],p_globalTarget[2]-p_target[2])

                
                NsecondPage=  v_lprime1^v_l1
                # image of p_i on second page:
                v=CG3dVector(p_target[0]-p_i[0],p_target[1]-p_i[1],p_target[2]-p_i[2])
                d=v*NsecondPage
                p_hatOnSecondPage=d*NsecondPage+v
                p_iOnSecondPage=CG3dPoint(p_hatOnSecondPage[0],p_hatOnSecondPage[1],p_hatOnSecondPage[2])

                d1=utils.distance(p_globalTarget,p_iOnSecondPage)
                unitv_l2=CG3dVector((p_globalTarget[0]-p_iOnSecondPage[0])/d1,(p_globalTarget[1]-p_iOnSecondPage[1])/d1,(p_globalTarget[2]-p_iOnSecondPage[2])/d1)
                p_primei=p_globalTarget+d1*unitv_l2

                d2=utils.distance(p_primei,p_target)
                unitv_l2prime=CG3dVector((p_primei[0]-p_target[0])/d2,(p_primei[1]-p_target[1])/d2,(p_primei[2]-p_target[2])/d2)
                p_primetarget=p_primei+d2*unitv_l2prime

                return p_primetarget


            if constraintType =="ball":
            #define points
            # some vector

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


                L_otarget = utils.distance(o,p_target)
                unitVoTarget = CG3dVector((p_target[0]-o[0])/L_otarget,(p_target[1]-o[1])/L_otarget,(p_target[2]-o[2])/L_otarget)
                
                si=self.orientation
                y_t=L_otarget*math.cos(si)
                x_t=L_otarget*math.sin(si)

                
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
                    pprime=CG3dPoint(0,0,0)
                    return pprime
                elif ((x_t**2)/(q2**2)+(y_t**2)/(q1**2))<1 and sector==2:
                    inbound=1
                    pprime=CG3dPoint(0,0,0)
                    return pprime
                elif ((x_t**2)/(q4**2)+(y_t**2)/(q1**2))<1 and sector==3:
                    inbound=1
                    pprime=CG3dPoint(0,0,0)
                    return pprime
                elif ((x_t**2)/(q4**2)+(y_t**2)/(q3**2))<1 and sector==4:
                    inbound=1
                    pprime=CG3dPoint(0,0,0)
                    return pprime
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


                #finding 2 point on 2d cross section plane
                point1=self.pointsOn2DConeCrosssection(o,p_i,p_target,unitVoTarget,x_t,y_t,1)
                point2=self.pointsOn2DConeCrosssection(o,p_i,p_target,unitVoTarget,x_t,y_t,2)


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

    def pointsOn2DConeCrosssection(self,o,p_i,p_target,unitVoTarget,x_t,y_t,i):
        
        # The cone cross section is a plane Ax+By+Cz+D=0
        # any point in this plane (x1,y1,z1) if lies on the x axis of the 2d coordinate in plane should:
        # be on plane
        # vector from o to this point dot product unitvec has some known value
        # this point distance to o is y_t
        si=self.orientation
        Lopi=utils.distance(o,p_i)
        
        unitPlane=CG3dVector((p_i[0]-o[0])/Lopi,(p_i[1]-o[1])/Lopi,(p_i[2]-o[2])/Lopi)
        
        
        A=unitPlane[0]
        B=unitPlane[1]
        C=unitPlane[2]
        D=-(A*p_target[0]+B*p_target[1]+C*p_target[2])


        F1 = unitVoTarget[0]-unitVoTarget[2]*A/C
        F2 = unitVoTarget[1]-unitVoTarget[2]*B/C
        F3 =  -A/C+(B/C)*(F1/F2)
        a2=1+(F1/F2)**2+F3**2
        
        # The first point on y axis
        

        if i==1:
            M1 = -unitVoTarget[0]*o[0]-unitVoTarget[1]*o[1]-unitVoTarget[2]*o[2]-unitVoTarget[2]*D/C-y_t*math.cos(si)
            M2 = (B/C)*(M1/F2)-D/C
            a1=-2*o[0]+2*(F1/F2)*(o[1]+M1/F2)+2*F3*(M2-o[2])
            a0=o[0]**2+(o[1]+M1/F2)**2+(M2-o[2])**2-y_t**2


            x1=self.newtonRaphsonPointOnCrossSectionCone(o[0],a2,a1,a0)
            y1=-(M1/F2)-(F1/F2)*x1
            z1=F3*x1+M2

            point1=CG3dPoint(x1,y1,z1)
            return point1
        else:
            M1 = -unitVoTarget[0]*o[0]-unitVoTarget[1]*o[1]-unitVoTarget[2]*o[2]-unitVoTarget[2]*D/C-x_t*math.sin(si)
            M2 = (B/C)*(M1/F2)-D/C
            a1=-2*o[0]+2*(F1/F2)*(o[1]+M1/F2)+2*F3*(M2-o[2])
            a0=o[0]**2+(o[1]+M1/F2)**2+(M2-o[2])**2-x_t**2
            # The second point on x axis 
            x2=self.newtonRaphsonPointOnCrossSectionCone(o[0],a2,a1,a0)
            y2=-(M1/F2)-(F1/F2)*x2
            z2=F3*x2+M2
            point2=CG3dPoint(x2,y2,z2)
            return point2

    def newtonRaphsonPointOnCrossSectionCone(self,x,a2,a1,a0):
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
