import numpy as np
from enum import Enum

class Robotiq2f85(object):
    min_stroke = 0.0
    max_stroke = 0.085
    max_grasp_force = 200 #N not sure
    
    def getPositionRequest(pos, stroke):
        return int(np.clip((3. - 230.)/stroke * pos + 230., 0, 255))
    
    def byteToPosition(pos, stroke):
        return np.clip(stroke/(3.-230.)*(pos-230.), 0, stroke)
    


class RobotiqHandE(object):
    min_stroke = 0.0
    max_stroke = 0.055
    max_grasp_force = 158 #N

    def getPositionRequest(pos, stroke):
        return int(np.clip((255. - (255 * pos /stroke)), 0, 255))
    
    def getForceRequest(force):
        return int(np.clip(force/100.*255., 0, 255))

    def byteToPosition(pos, stroke):
        return np.clip(stroke/(255.)*(255.-pos), 0, stroke)

class RobotiqGripperType(Enum):
    Hand_E = 1
    TwoF_85 = 2

class Robotiq(Robotiq2f85, RobotiqHandE):
    def __init__(self, gripper_type, stroke=None):
        self.gripper_type = gripper_type
        if gripper_type == RobotiqGripperType.Hand_E:
            self.stroke = stroke if stroke is not None else RobotiqHandE.max_stroke
            self.min_stroke = RobotiqHandE.min_stroke
            self.max_stroke = RobotiqHandE.max_stroke
            self.max_grasp_force = RobotiqHandE.max_grasp_force
            print("Initialized RobotiqHandE: \n -max stroke: {} \n -min stroke: {} \n -stroke {}".format(self.max_stroke, self.min_stroke, self.stroke))
        elif gripper_type == RobotiqGripperType.TwoF_85:
            self.stroke = stroke if stroke is not None else Robotiq2f85.max_stroke
            self.min_stroke = Robotiq2f85.min_stroke
            self.max_stroke = Robotiq2f85.max_stroke
            self.max_grasp_force = Robotiq2f85.max_grasp_force
            print("Initialized Robotiq2F85: \n -max stroke: {} \n -min stroke: {} \n -stroke {}".format(self.max_stroke, self.min_stroke, self.stroke))

    # --- Stroke Set Get Methods --- #
    def set_stroke(self, value):
        self.stroke = value
    
    def set_max_stroke(self, value):
        self.max_stroke = value
    
    def set_min_stroke(self, value):
        self.min_stroke = value
    
    def get_stroke(self):
        return self.stroke
    
    def get_max_stroke(self):
        return self.max_stroke

    def get_min_stroke(self):
        return self.min_stroke

    # --- Force Set Get Methods --- #
    def set_max_force(self, value):
        self.max_grasp_force = value
    
    def get_max_force(self):
        return self.max_grasp_force
        
    def getPositionRequest(self, pos):
        if self.gripper_type == RobotiqGripperType.Hand_E:
            return RobotiqHandE.getPositionRequest(pos, self.stroke)
        elif self.gripper_type == RobotiqGripperType.TwoF_85:
            return Robotiq2f85.getPositionRequest(pos, self.stroke)
    
    def getForceRequest(self, force): #in percentage
            return int(np.clip(force/100.*255., 0, 255))
    def getVelocityRequest(self, vel): #in percentage
        return int(np.clip(vel/100.*255., 0, 255))

    def byteToPosition(self, pos):
        if self.gripper_type == RobotiqGripperType.Hand_E:
            return RobotiqHandE.byteToPosition(pos, self.stroke)
        elif self.gripper_type == RobotiqGripperType.TwoF_85:
            return Robotiq2f85.byteToPosition(pos, self.stroke)

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

    cmd_object_detected = b'GET OBJ\n'
    cmd_get_activation_status = b'GET STA\n' #check fine file

    cmd_get_fault = b'GET FLT\n'
    cmd_get_echo = b'GET PRE \n'

    cmd_get_current = b'GET COU\n'
    cmd_get_driver_state = b'GET DST\n'
    cmd_get_connection_state = b'GET PCO\n'


'''
ACT: Activation bit
0 - Gripper not activated
1 - Gripper activated
GTO: 1 if the gripper is set to move to requested position 0 if gripper is set to stay at the same place
PRE: Position request eco. Should be same a the requested position if
the gripper successfully received the requested position.
POS: Current position of the gripper
SPE: Speed eco. Should be same as requested speed.
FOR: Force parameter of the gripper
OBJ: Object grippings status
0 - Fingers are inmotion towards requested position.No object detected.
1 - Fingers have stopped due to a contact while opening before requested position.Object detected opening.
2 - Fingers have stopped due to a contact while closing before requested position.Object detected closing.
3 - Fingers are at requested position.No object detected or object has been loss / dropped.
STA: Gripper status, returns the current status & motion of theGripper fingers.
0 -Gripper is in reset ( or automatic release )state. See Fault Status if Gripper is activated.
1 - Activation in progress.
2 - Not used.
3 - Activation is completed.
MOD: ...
FLT: Fault status returns general errormessages that are useful for troubleshooting. Fault LED (red) is present on theGripper chassis,
LED can be blue, red or both and be solid or blinking.
0 - No fault (LED is blue)
Priority faults (LED is blue)
5 - Action delayed, activation (reactivation)must be completed prior to performing the action.
7 - The activation bit must be set prior to action.
Minor faults (LED continuous red)
8 -Maximum operating temperature exceeded,wait for cool-down.
9 No communication during at least 1 second.
Major faults (LED blinking red/blue) - Reset is required (rising edge on activation bit rACT needed).
10 - Underminimum operating voltage.
11- Automatic release in progress.
12- Internal fault; contact support@robotiq.com.
13 - Activation fault, verify that no interference or other error occurred.
14-Overcurrent triggered.
15- Automatic release completed.
MSC: Gripper maximym current.
COU: Gripper current.
NCY: Number of cycles performed by the gripper
DST: Gripper driver state
0 - Gripper Driver State : RQ_STATE_INIT
1 - Gripper Driver State : RQ_STATE_LISTEN
2 - Gripper Driver State : Q_STATE_READ_INFO
3 - Gripper Driver State : RQ_STATE_ACTIVATION
Other - Gripper Driver State : RQ_STATE_RUN
PCO: Gripper connection state
0 - Gripper Connection State : No connection problem detected
Other - Gripper Connection State : Connection problem detected
ACT: Activation bit
0 - Gripper not activated
1 - Gripper activated
GTO: 1 if the gripper is set to move to requested position 0 if gripper is set to stay at the same place
PRE: Position request eco. Should be same a the requested position if
the gripper successfully received the requested position.
POS: Current position of the gripper
SPE: Speed eco. Should be same as requested speed.
FOR: Force parameter of the gripper
OBJ: Object grippings status
0 - Fingers are inmotion towards requested position.No object detected.
1 - Fingers have stopped due to a contact while opening before requested position.Object detected opening.
2 - Fingers have stopped due to a contact while closing before requested position.Object detected closing.
3 - Fingers are at requested position.No object detected or object has been loss / dropped.
STA: Gripper status, returns the current status & motion of theGripper fingers.
0 -Gripper is in reset ( or automatic release )state. See Fault Status if Gripper is activated.
1 - Activation in progress.
2 - Not used.
3 - Activation is completed.
MOD: ...
FLT: Fault status returns general errormessages that are useful for troubleshooting. Fault LED (red) is present on theGripper chassis,
LED can be blue, red or both and be solid or blinking.
0 - No fault (LED is blue)
Priority faults (LED is blue)
5 - Action delayed, activation (reactivation)must be completed prior to performing the action.
7 - The activation bit must be set prior to action.
Minor faults (LED continuous red)
8 -Maximum operating temperature exceeded,wait for cool-down.
9 No communication during at least 1 second.
Major faults (LED blinking red/blue) - Reset is required (rising edge on activation bit rACT needed).
10 - Underminimum operating voltage.
11- Automatic release in progress.
12- Internal fault; contact support@robotiq.com.
13 - Activation fault, verify that no interference or other error occurred.
14-Overcurrent triggered.
15- Automatic release completed.
MSC: Gripper maximym current.
COU: Gripper current.
NCY: Number of cycles performed by the gripper
DST: Gripper driver state
0 - Gripper Driver State : RQ_STATE_INIT
1 - Gripper Driver State : RQ_STATE_LISTEN
2 - Gripper Driver State : Q_STATE_READ_INFO
3 - Gripper Driver State : RQ_STATE_ACTIVATION
Other - Gripper Driver State : RQ_STATE_RUN
PCO: Gripper connection state
0 - Gripper Connection State : No connection problem detected
Other - Gripper Connection State : Connection problem detected
'''