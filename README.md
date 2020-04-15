# FABRIK
"This code is mostly done but needs some minor improvements that is under developement"

Description:
An inverse kinematic solver  with considering each joint constraint

This code is an inverse kinematic solver for whole human body chain. 
Consists of a FABRIk part and a Constraints part, In the constraints file each joint biomechanical constraints are considered,In the FABRIK part the main functions and step of the FABRIK inverse kinematic is described. The input of this library is the joints position, target position, target orientation, and each joint constraint. For each joint constraint, there are four angles that specify the joint flexion, extension, abduction, adduction, and orientation limits which is specified in the "main", and the output is the new joint position for reaching that target position and orientation.
Also the stick diagram of the before and after the FABRIK solver is depicted. 

This algorithm is base on the works of this paper:
https://doi.org/10.1016/j.gmod.2011.05.003



