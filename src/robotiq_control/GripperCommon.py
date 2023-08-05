import numpy as np
from enum import Enum

class Robotiq2f85(object):
    min_stroke = 0.0
    max_stroke = 0.085

    def getPositionRequest(pos, stroke):
        return int(np.clip((3. - 230.)/stroke * pos + 230., 0, 255))
        

class RobotiqHandE(object):
    min_stroke = 0.0
    max_stroke = 0.055
    def getPositionRequest(pos, stroke):
        return int(np.clip((255. - (250 * pos /stroke)), 0, 255))


class RobotiqGripperType(Enum):
    Hand_E = 1
    TwoF_85 = 2

class Robotiq(Robotiq2f85, RobotiqHandE):
    def __init__(self, gripper_type):
        self.gripper_type = gripper_type
        if gripper_type == RobotiqGripperType.Hand_E:
            self.stroke = RobotiqHandE.max_stroke
            self.min_stroke = RobotiqHandE.min_stroke
            self.max_stroke = RobotiqHandE.max_stroke
            print("Initialized RobotiqHandE -max_stroke: {}, -stroke {}".format(self.max_stroke, self.stroke))
        elif gripper_type == RobotiqGripperType.TwoF_85:
            self.stroke = Robotiq2f85.max_stroke
            self.min_stroke = Robotiq2f85.min_stroke
            self.max_stroke = Robotiq2f85.max_stroke
            print("Initialized Robotiq2F85 -max_stroke: {}, -stroke {}".format(self.max_stroke, self.stroke))
    
    def setStroke(self, value):
        self.stroke = value
        
    def getPositionRequest(self, pos):
        if self.gripper_type == RobotiqGripperType.Hand_E:
            return RobotiqHandE.getPositionRequest(pos, self.stroke)
        elif self.gripper_type == RobotiqGripperType.TwoF_85:
            return Robotiq2f85.getPositionRequest(pos, self.stroke)


class RobotiqSocketCmds():
    cmd_activate = b'SET ACT 1\n'
    cmd_deactivate = b'SET ACT 0\n'

    cmd_EnableMove = b'SET GTO 1\n'
    cmd_DisableMove = b'SET GTO 0\n'

    cmd_full_close = b'SET POS 255\n'
    cmd_full_open = b'SET POS 0\n'
    
    cmd_set_pos = b'SET POS ' # aggiungere la posizione in byte desiderata + \n
    cmd_set_speed = b'SET SPE ' # aggiungere la velocità in byte desiderata + \n
    cmd_set_force = b'SET FOR ' # aggiungere la forza in byte desiderata + \n

    cmd_get_pos = b'GET POS\n '
    cmd_get_speed = b'GET SPE\n '
    cmd_get_force = b'GET FOR\n '

    cmd_get_status = b'GET OBJ\n'
    cmd_get_activation_status = b'GET STA\n' #check fine file

    cmd_get_fault = b'GET FLT\n'
    cmd_get_echo = b'GET PRE \n'

    cmd_get_current = b'GET COU\n'
    cmd_get_driver_state = b'GET DST\n'
    cmd_get_connection_state = b'GET PCO\n'