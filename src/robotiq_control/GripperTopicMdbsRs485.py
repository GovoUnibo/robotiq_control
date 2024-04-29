import rospy
from robotiq_control.include.GripperMdbsCommunicationControl import GripperCommand
from robotiq_control.msg import GripperControl

import time

class RobotiqControl(GripperCommand):
    def __init__(self, gripper_type, id=0, comPort='/dev/ttyUSB0',baud_rate=115200, timeout=0.002):
        GripperCommand.__init__(self, gripper_type=gripper_type, id=id, comPort=comPort, baud_rate=baud_rate, timeout=timeout)
         
        init_done = False
        while not rospy.is_shutdown() and not init_done:
            rospy.logwarn_throttle(5, ": Waiting for gripper to be ready...")
            init_done = super().initialize()
            time.sleep(0.5)
        
        sub = rospy.Subscriber('robotiq_control', GripperControl, self.__gripper_command_callback, queue_size=1)
        #print in violet
        print(f'\033[95m Gripper is ready to recieve commands at hz: {1/timeout}\033[0m')
        self.last_time_called = rospy.get_time()
        self.min_time_between_messages = timeout
        # rospy.wait_for_message('robotiq_control', GripperControl)



    def __gripper_command_callback(self, msg):
        # if msg is not isinstance(GripperControl):
        #     rospy.logerr('GripperControl message is not valid')
        current_time = rospy.get_time()
        
        if current_time - self.last_time_called < self.min_time_between_messages:
            rospy.logwarn(f'Message frequency too high, message ignored, the frequency should be lower than {1/self.min_time_between_messages}')
            return
        print(f'Current time: {current_time}, Last time called: {self.last_time_called}, Difference: {current_time - self.last_time_called}')
        self.last_time_called = current_time
        
        super().goTo(msg.position, msg.speed, msg.force)


        

if __name__ == "__main__":
    rospy.init_node('robotiq_control')
    gripper = RobotiqControl('Hand_E',timeout=0.002)
    rospy.spin()
    print("Node is shutting down...")