import math  
import numpy as np
import matplotlib.pyplot as plt
class FABRIK:
    def __init__(self,joints,target):
        self.n=len(joints)
        self.joints=joints
        lengths=[]
        for i in range(1,len(joints)):
            length = math.sqrt((joints[i][0]-joints[i-1][0])**2+
                               (joints[i][1]-joints[i-1][1])**2)
            lengths.append(length)
        self.tolerance=0.1
        self.target=target
        self.lengths=lengths
        self.origin=joints[0]
        self.totallength=sum(lengths)
    def forward(self):
        # set end effector as target
        self.joints[self.n-1]=self.target;
        for i in range(self.n-2,0,-1):
            r = math.sqrt((self.joints[i][0]-self.joints[i+1][0])**2+
                               (self.joints[i][1]-self.joints[i+1][1])**2)
            landa = self.lengths[i]/r
            # find new joint position
            pos=(1-landa)*self.joints[i+1]+landa*self.joints[i]
            self.joints[i]=pos;
        
    def backward(self):
        # set root as initial position
        self.joints[0]=self.origin;
        for i in range(0,self.n-2):
            r = math.sqrt((self.joints[i][0]-self.joints[i+1][0])**2+
                               (self.joints[i][1]-self.joints[i+1][1])**2)
            landa = self.lengths[i]/r
            # find new joint position
            pos=(1-landa)*self.joints[i]+landa*self.joints[i+1]
            self.joints[i+1]=pos;
    
    def solve(self):
        distance = math.sqrt((self.joints[0][0]-self.target[0])**2+
                             (self.joints[0][1]-self.target[1])**2)
        if distance > self.totallength:
        # target is out of reach
            for i in range(1,self.n-1):
                r=math.sqrt((self.joints[i][0]-self.target[0])**2+
                             (self.joints[i][1]-self.target[1])**2)
                landa=self.length[i-1]/r;
        # find new joint position
                self.joints[i]=(1-landa)*self.joints[i-1]+landa*self.target;
        else:
        # target is in reach
            bcount=0;
            dif=math.sqrt((self.joints[self.n-1][0]-self.target[0])**2+
                             (self.joints[self.n-1][1]-self.target[1])**2)
            while dif > self.tolerance:
                self.forward()
                self.backward()
                dif = math.sqrt((self.joints[self.n-1][0]-self.target[0])**2+
                             (self.joints[self.n-1][1]-self.target[1])**2)
                bcount=bcount+1
                if bcount>10:
                    break
            print("new joints position: ",self.joints)
            self.draw()
    def draw(self):
        first_pos = np.loadtxt("joints.txt")
        x_prime=[]
        y_prime=[]
        x=[]
        y=[]
        for i in range(self.n):
            x_prime.append(self.joints[i][0])
            y_prime.append(self.joints[i][1])
            x.append(first_pos[i][0])
            y.append(first_pos[i][1])
        plt.plot(x,y, marker = 'o')
        plt.plot(x_prime,y_prime)
        plt.scatter(self.target[0],self.target[1])
        plt.show()

manipulator = FABRIK(np.loadtxt("joints.txt"),[4,3])
manipulator.solve()
#except Exception as e:
    #print('Errorhappened!!!')
