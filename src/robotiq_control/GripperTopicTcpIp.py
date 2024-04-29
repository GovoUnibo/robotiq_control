import rospy

from src.robotiq_control.src.robotiq_control.include.GripperMdbsCommunicationControl import GripperCommand



class RobotiqControl(GripperCommand):
    def __init__(self, gripper_type, id=0, comPort='/dev/ttyUSB0',baud_rate=115200):
        GripperCommand.__init__(self, gripper_type=gripper_type, id=id, comPort=comPort, baud_rate=baud_rate)
   