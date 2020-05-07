import fabrik.angle_calculator.neck as neck
import fabrik.angle_calculator.trunck as trunk
import fabrik.angle_calculator.leg as leg
import fabrik.angle_calculator.upper_arm as upperArm
import fabrik.angle_calculator.lower_arm as lowerArm
import fabrik.angle_calculator.wrist as wrist


class AngleCalculator:
    def __init__(self,file,joints):
        self.file = file
        self.joints = joints

    def calculate(self):
        m_neck = neck.Neck(self.joints,self.file)
        m_neck.neck_flex_calculator()
        m_neck.neck_side_calculator()
        m_neck.neck_torsion_calculator()

        m_trunk = trunk.Trunk(self.joints,self.file)
        m_trunk.trunk_flex_calculator()
        m_trunk.trunk_side_calculator()
        m_trunk.trunk_torsion_calculator()

        m_leg = leg.Leg(self.joints, self.file)
        m_leg.leg_degree()

        m_UA = upperArm.UpperArm(self.joints, self.file)
        m_UA.upper_arm_flex()
        m_UA.upper_arm_side_bending()
        m_UA.shoulder_rise()
        m_UA.upper_arm_rotation()

        m_LA = lowerArm.LowerArm(self.joints, self.file)
        m_LA.lower_arm_degree()

        m_wrist = wrist.Wrist(self.joints,self.file)
        m_wrist.wrist_flex()
        m_wrist.wrist_side()
        m_wrist.wrist_torsion()