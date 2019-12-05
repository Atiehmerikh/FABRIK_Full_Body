import math  
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d, Axes3D
from pycg3d.cg3d_point import CG3dPoint
from pycg3d.cg3d_vector import CG3dVector
from pycg3d import utils
from pycg3d.cg3d_plane import CG3dPlanePN
from draw import draw
import Constraints as CF


class ClosedLoop:
    def __init__(self,joints,target,origin,fixed_joint,Theta,orientation,length):
        self.n=len(joints)
        self.joints = joints
        self.Theta = Theta
        self.orientation = orientation
        self.fixed_joint=fixed_joint
        self.length=length
        #a dictionary for lengths

        self.tolerance=0.1
        self.target=target
        self.origin=origin

    def distanceCalc(self,i,j):
        iPoint=CG3dPoint(i[0],i[1],i[2])
        jPoint=CG3dPoint(j[0],j[1],j[2])

        return (utils.distance(iPoint,jPoint))
    def findingTargetPos(self):
        rightUpperBody= self.distanceCalc(self.fixed_joint,self.joints[3])+self.distanceCalc(self.joints[3],self.joints[4])
        leftUpperBody= self.distanceCalc(self.fixed_joint,self.joints[2])+self.distanceCalc(self.joints[1],self.joints[2])     
        toTargetDistance=self.distance(self.fixed_joint,self.target)

        # MEANS IT IS ON THE RIGHT SIDE
        if self.target[0]>self.fixed_joint[0]:
            if toTargetDistance>rightUpperBody:  
                print("target is not reachable")
                return
            else:
                return self.right_effector

        # MEANS IT IS ON THE left SIDE
        if self.target[0]<self.fixed_joint[0]:
            if toTargetDistance>leftUpperBody:   
                print("target is not reachable")
                return 0
            else:
                return self.left_effector

    ## For closed loop:
    def forward(self,clsLoopJntFwdIndex,shoulderIndex,oppShoulderIndex):
        # if target is reachable on the right side ,  set end effector as target
        self.joints[shoulderIndex[1]] = self.target

        # For right/left Shoulder joints position:
        r = self.distanceCalc(self.joints[shoulderIndex[0]],self.joints[shoulderIndex[1]])
        length = self.distanceCalc(self.length[shoulderIndex[0]],self.length[shoulderIndex[1]])
        landa =(length)/r
        pos = (1-landa)*self.joints[shoulderIndex[1]]+landa*self.joints[shoulderIndex[0]]

        constraintReturn = CF.constraintsFunction.constraint(self.joints,length,shoulderIndex[0],self.Theta[shoulderIndex[0]],self.orientation)
        if constraintReturn[0] == 0:
            self.joints[shoulderIndex[0]]=pos
        else:
            for j in range(3):
                self.joints[shoulderIndex[0]][j]=constraintReturn[j]

##        self.joints[shoulderIndex[0]] = pos

        #for closed loop 
        for i in range(3,1,-1):
            r = self.distanceCalc(self.joints[clsLoopJntFwdIndex[i]],self.joints[clsLoopJntFwdIndex[i-1]])
            length = self.distanceCalc(self.length[clsLoopJntFwdIndex[i]],self.length[clsLoopJntFwdIndex[i-1]])
            landa = length/r
            # find new joint position
            pos=(1-landa)*self.joints[i]+landa*self.joints[i-1]
##            self.joints[clsLoopJntFwdIndex[i-1]]=pos

            constraintReturn=CF.constraintsFunction.constraint(self.joints,length,clsLoopJntFwdIndex[i-1],self.Theta[clsLoopJntFwdIndex[i-1]],self.orientation)
            if constraintReturn[0] == 0:
                self.joints[clsLoopJntFwdIndex[i-1]]=pos
            else:
                for j in range(3):
                    self.joints[clsLoopJntFwdIndex[i-1]][j]=constraintReturn[j]

            self.joints[clsLoopJntFwdIndex[0]]= self.joints[clsLoopJntFwdIndex[3]]
        # For left/right Shoulder joints position:
            r = self.distanceCalc(self.joints[oppShoulderIndex[0]],self.joints[oppShoulderIndex[1]])
            length = self.distanceCalc(self.length[oppShoulderIndex[0]],self.length[oppShoulderIndex[1]])
            
            landa = length/r
            pos = (1-landa)*self.joints[oppShoulderIndex[0]]+landa*self.joints[oppShoulderIndex[1]]


            constraintReturn=CF.constraintsFunction.constraint(self.joints,length,oppShoulderIndex[1],self.Theta[oppShoulderIndex[1]],self.orientation)
            if constraintReturn[0] == 0:
                self.joints[oppShoulderIndex[1]]=pos
            else:
                for j in range(3):
                    self.joints[oppShoulderIndex[1]][j]=constraintReturn[j]


##            self.joints[oppShoulderIndex[1]] = pos
    def backward(self,clsLoopJntBwdIndex,shoulderIndex,oppShoulderIndex):
        # set root as initial position
        self.joints[0]=self.fixed_joint;
        # The Close loop position
        for i in range(1,3):
            r = self.distanceCalc(self.joints[clsLoopJntBwdIndex[i]],self.joints[clsLoopJntBwdIndex[i-1]])
            length= self.distanceCalc(self.length[clsLoopJntBwdIndex[i]],self.length[clsLoopJntBwdIndex[i-1]])
            landa = length/r
            # find new joint position
            pos=(1-landa)*self.joints[clsLoopJntBwdIndex[i-1]]+landa*self.joints[clsLoopJntBwdIndex[i]]
            self.joints[clsLoopJntBwdIndex[i]] = pos

        self.joints[clsLoopJntBwdIndex[0]]= self.joints[clsLoopJntBwdIndex[3]]

        #The near target Shoulder position
        r = self.distanceCalc(self.joints[shoulderIndex[0]],self.joints[shoulderIndex[1]])
        length = self.distanceCalc(self.length[shoulderIndex[0]],self.length[shoulderIndex[1]])
        landa = length/r
        pos = (1-landa)*self.joints[shoulderIndex[0]]+landa*self.joints[shoulderIndex[1]]
        self.joints[shoulderIndex[1]] = pos

        #The opposite Shoulder position
        r = self.distanceCalc(self.joints[oppShoulderIndex[0]],self.joints[oppShoulderIndex[1]])
        length = self.distanceCalc(self.length[oppShoulderIndex[0]],self.length[oppShoulderIndex[1]])
        landa = length/r
        pos = (1-landa)*self.joints[oppShoulderIndex[0]]+landa*self.joints[oppShoulderIndex[1]]
        self.joints[oppShoulderIndex[1]] = pos

    def solve(self):
        shoulderIndex=[]
        rightShoulderIndex = [3,4]
        leftShoulderIndex = [2,1]
        # if target is on the right side
        if self.target[0]>self.fixed_joint[0]:
            clsLoopJntFwdIndex = [3,2,0,3]
            shoulderIndex=rightShoulderIndex
            oppShoulderIndex=leftShoulderIndex
            endEffectors = [4,1]
            clsLoopJntBwdIndex=[0,2,3,0]
        # if target is on the left side
        if self.target[0] <= self.fixed_joint[0]:
            clsLoopJntFwdIndex = [2,0,3,2]
            shoulderIndex=leftShoulderIndex
            oppShoulderIndex=rightShoulderIndex
            endEffectors = [4,1]
            clsLoopJntBwdIndex=[0,2,3,0]
        bcount=0
        dif = self.distanceCalc(self.joints[shoulderIndex[1]],self.target)
        while dif > self.tolerance:
            self.forward(clsLoopJntFwdIndex,shoulderIndex,oppShoulderIndex)
            self.backward(clsLoopJntBwdIndex,shoulderIndex,oppShoulderIndex)
            dif = self.distanceCalc(self.joints[shoulderIndex[1]],self.target)
            bcount=bcount+1
            if bcount>10:
                break
        print("new joints position: ",self.joints)
        closeLoopJointsIndex = [0,2,3,0]
        draw(self.joints,closeLoopJointsIndex,rightShoulderIndex,leftShoulderIndex,self.target,np.loadtxt("length.txt"))
