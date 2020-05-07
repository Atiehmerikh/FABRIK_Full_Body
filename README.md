
# FULL HUMAN BODY INVERSE KINEMATIC SIMULATION BY FABRIK 

# Description:

	This library is an implementation of the FABRIK method(Forward And Backward Reaching Inverse kinematic) 
	for whole human body in the Python programming language. and is released under the MIT software license 
	and is under developement. It can simulate the human behavior in reaching a target with consideration of 
	the human joints constraints.
	NOTE: This library is under developement so there is some update in future.

# Install 
	You can install the `fabrik` library using the following command:
	```
	pip3 install git+https://github.com/Atiehmerikh/FABRIK_python.git
	```
## Usage:
	
	To use the code you should specify some values in text files:
	First you should consider the numbering in "body_number.jpg" file and rest of input files
	are based on these numbering
	1- "joits_position.txt" : This is the initial position of the joints
	2- "joints_constraint.txt": This is the constraints for each human body joints according
		to the joint reference plane and coordinate
		you should enter four number for each joint (Adduction, Abduction, Flexion, Extension)
		order of these four number is according to joints orientation in its 
		reference plane(in degree)
	3- "orientation.txt" This is the initial orientation of each joint in quaternion
	4- "bone_twist_constraints.txt" the twist limitation for each bone the limitation
		for each bone should be based on the outer bone number. 
		for any one that you don't have data let it be on its default.
	5- "constraint_type.txt" file for getting the joints constraint 
		"BALL" :for ball and socket and "hinge" : for hinge type
	6- specifying target position and orientation in "Main.py"

## OutPut:


	Out put is a text file ("angles.txt" ) which computed joint angles and also the final position in stick diagram.

## References:

	The FABRIK algorithm is explained in the following research paper:
	Aristidou, A., & Lasenby, J. (2011). FABRIK: a fast, iterative solver for the inverse kinematics problem. Graphical Models, 73(5), 243-260.

## TODO

	1- solve the algorihm for joints constraint which makes parabolic section in joints reference plane
	2- target locating on the line of kinematic chain
	3- Handling If the target is unreachable i.e. if the distance between the root and the target is less than the length of the kinematic chain.
	4- multiple end effector
	5- the human feet be able to move
	
