import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d, Axes3D

def draw(joints,target,first_pos,rightArmIndex,leftArmIndex,upperChain,lowerChain,rightLeg,leftLeg):
def draw(joints,target,first_pos,rightArmIndex,leftArmIndex):
       
# The Second Pose
        x_primeRArm=[]
        y_primeRArm=[]
        z_primeRArm=[]
        
        #Right Shoulder
        for i in range(len(rightArmIndex)):
            x_primeRArm.append(joints[rightArmIndex[i]][0])
            y_primeRArm.append(joints[rightArmIndex[i]][1])
            z_primeRArm.append(joints[rightArmIndex[i]][2])
        
        x_primeLArm=[]
        y_primeLArm=[]
        z_primeLArm=[]
        
        #left Shoulder
        for i in range(len(leftArmIndex)):
            x_primeLArm.append(joints[leftArmIndex[i]][0])
            y_primeLArm.append(joints[leftArmIndex[i]][1])
            z_primeLArm.append(joints[leftArmIndex[i]][2])
    
        x_primeU=[]
        y_primeU=[]
        z_primeU=[]
        
        #Upper chain
        for i in range(len(upperChain)):
            x_primeU.append(joints[upperChain[i]][0])
            y_primeU.append(joints[upperChain[i]][1])
            z_primeU.append(joints[upperChain[i]][2])
            
        x_primeL=[]
        y_primeL=[]
        z_primeL=[]
        
         #Lower chain
        for i in range(len(lowerChain)):
            x_primeL.append(joints[lowerChain[i]][0])
            y_primeL.append(joints[lowerChain[i]][1])
            y_primeL.append(joints[lowerChain[i]][2])
        
        x_primeRLeg=[]
        y_primeRLeg=[]
        z_primeRLeg=[]
        
         #Right Leg
        for i in range(len(rightLeg)):
            x_primeRLeg.append(joints[rightLeg[i]][0])
            y_primeRLeg.append(joints[rightLeg[i]][1])
            z_primeRLeg.append(joints[rightLeg[i]][2])
        
        x_primeLLeg=[]
        y_primeLLeg=[]
        z_primeLLeg=[]
        
         #left Leg
        for i in range(len(leftLeg)):
            x_primeLLeg.append(joints[leftLeg[i]][0])
            y_primeLLeg.append(joints[leftLeg[i]][1])
            z_primeLLeg.append(joints[leftLeg[i]][2])
        
       
    #...............................................................................................
    
       # The First Pose
    
        x_RArm=[]
        y_RArm=[]
        z_RArm=[]
        
        #Right Shoulder
        for i in range(len(rightArmIndex)):
            x_RArm.append(first_pos[rightArmIndex[i]][0])
            y_RArm.append(first_pos[rightArmIndex[i]][1])
            z_RArm.append(first_pos[rightArmIndex[i]][2])
         
        x_LArm=[]
        y_LArm=[]
        z_LArm=[]
        
        #left Shoulder
        for i in range(len(leftArmIndex)):
            x_LArm.append(first_pos[leftArmIndex[i]][0])
            y_LArm.append(first_pos[leftArmIndex[i]][1])
            z_LArm.append(first_pos[leftArmIndex[i]][2])
        
       
        x_U=[]
        y_U=[]
        z_U=[]
        
        #Upper chain
        for i in range(len(upperChain)):
            x_U.append(first_pos[upperChain[i]][0])
            y_U.append(first_pos[upperChain[i]][1])
            z_U.append(first_pos[upperChain[i]][2])
            
        x_L=[]
        y_L=[]
        z_L=[]
        
         #Lower chain
        for i in range(len(lowerChain)):
            x_L.append(first_pos[lowerChain[i]][0])
            y_L.append(first_pos[lowerChain[i]][1])
            y_L.append(first_pos[lowerChain[i]][2])
        
        x_RLeg=[]
        y_RLeg=[]
        z_RLeg=[]
        
         #Right Leg
        for i in range(len(rightLeg)):
            x_RLeg.append(first_pos[rightLeg[i]][0])
            y_RLeg.append(first_pos[rightLeg[i]][1])
            z_RLeg.append(first_pos[rightLeg[i]][2])
        
        x_LLeg=[]
        y_LLeg=[]
        z_LLeg=[]
        
         #left Leg
        for i in range(len(leftLeg)):
            x_LLeg.append(first_pos[leftLeg[i]][0])
            y_LLeg.append(first_pos[leftLeg[i]][1])
            z_LLeg.append(first_pos[leftLeg[i]][2])


        fig = plt.figure()
        ax=Axes3D(fig)
        
        ax.plot3D(x_RArm,y_RArm,z_RArm,color='red')
        ax.plot3D(x_LArm,y_LArm,z_LArm,color='red')
        ax.plot3D(x_U,y_U,z_U,color='red')
        ax.plot3D(x_L,y_L,z_L,color='red')
        ax.plot3D(x_RLeg,y_RLeg,z_RLeg,color='red')
        ax.plot3D(x_LLeg,y_LLeg,z_LLeg,color='red')

        #ax.plot3D(x_primeRArm,y_primeRArm,z_primeRArm,color='green')
        #ax.plot3D(x_primeLArm,y_primeLArm,z_primeLArm,color='green')
        #ax.plot3D(x_primeU,y_primeU,z_primeU,color='green')
        #ax.plot3D(x_primeL,y_primeL,z_primeL,color='green')
        #ax.plot3D(x_primeRLeg,y_primeRLeg,z_primeRLeg,color='green')
        #ax.plot3D(x_primeLLeg,y_primeLLeg,z_primeLLeg,color='green')

   
        
          
        
        #ax.scatter3D(target[0],target[1],target[2])
        plt.show()
