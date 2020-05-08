import numpy as np
from fabrik_full_body.singleton import Singleton

class InputReader(metaclass=Singleton):
    def __init__(self, base_address = "./inputs/", 
                       joints_position = "joints_position.txt",
                       initial_joints_position = "joints_position_fixed.txt",
                       orientation = "orientation.txt",
                       target = "target.txt",
                       joints_constraints = "joints_constraint.txt",
                       constraint_type = "constraint_type.txt",
                       bone_orientation_limit = "bone_twist_constraints.txt"):
        self.base_address = base_address
        self.joints_position_file_address =  self.base_address + joints_position
        self.initial_joints_position_file_address = self.base_address +  initial_joints_position
        self.orientation_file_address = self.base_address + orientation
        self.target_file_address = self.base_address + target
        self.joints_constraints_file_address = self.base_address + joints_constraints
        self.constraint_type_file_address = self.base_address + constraint_type
        self.bone_orientation_limit_file_address = self.base_address + bone_orientation_limit
        self.__clone_joints_position()

    def joints(self):
        return np.loadtxt(self.joints_position_file_address)
    
    def initial_joints_position(self):
        return np.loadtxt(self.initial_joints_position_file_address)
    
    def orientation(self):
        return np.loadtxt(self.orientation_file_address)
    
    def target_position(self):
        with open(self.target_file_address) as f:
            l = f.readline()
            return [float(i) for i in l.split(',')]

    def target_orientation(self):
        with open(self.target_file_address) as f:
            l = f.readline()
            l = f.readline()
            return [float(i) for i in l.split(',')]

    def joints_constraints(self):
        return np.loadtxt(self.joints_constraints_file_address)

    def constraint_type(self):
        with open(self.constraint_type_file_address) as f:
            return [line.rstrip() for line in f]

    def bone_orientation_limit(self):
        return np.loadtxt(self.bone_orientation_limit_file_address)

    def __clone_joints_position(self):
        with open(self.joints_position_file_address) as f:
            with open(self.initial_joints_position_file_address, "w") as f1:
                for line in f:
                    f1.write(line)