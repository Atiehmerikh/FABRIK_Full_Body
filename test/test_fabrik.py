import numpy as np
import sys
sys.path.append('..')
from fabrik import fabrik, input_reader



def main():
    reader = input_reader.InputReader()
    manipulator = fabrik.FABRIK(reader.joints(),
                         reader.initial_joints_position(), 
                         reader.orientation(), 
                         reader.target_position(),
                         reader.target_orientation(), 
                         reader.joints_constraints(),
                         reader.constraint_type(),
                         reader.bone_orientation_limit())

    print(manipulator.laterality())
    manipulator.solve()
    pass
    

if __name__ == "__main__":
    main()