#!/usr/bin/python3



from robotiq_control.msg import CommandRobotiqGripperAction
from robotiq_control.msg import CommandRobotiqGripperFeedback
from robotiq_control.msg import CommandRobotiqGripperResult

#from multinherit.multinherit import multi_super  #pip3 install multinherit
from robotiq_control.GripperCommon import RobotiqGripperType
from robotiq_control.GripperCmd import GripperCommand
from sensor_msgs.msg import JointState
import time
from threading import Thread
import rospy
import actionlib



GOAL_DETECTION_THRESHOLD = 0.01 # Max deviation from target goal to consider as goal "reached"
    

class RobotiqGripperActionServer(actionlib.SimpleActionServer, GripperCommand, rospy.Publisher):

    def __init__(self, action_server_name, gripper_type, slave_id = 0, usbComPort='/dev/ttyUSB0',baudRate=115200):
        self._action_name = action_server_name
        self.__feedback = CommandRobotiqGripperFeedback()
        self.__result = CommandRobotiqGripperResult()
        self._processing_goal = True
        self._seq = 0
        
        GripperCommand.__init__(self, gripper_type, id=slave_id, comPort=usbComPort ,baud_rate=baudRate)
        actionlib.SimpleActionServer.__init__(self, self._action_name, CommandRobotiqGripperAction, execute_cb=self.execute_callBack, auto_start=False)
        rospy.Publisher.__init__(self, 'joint_states', JointState, queue_size=10)

        whatchdog_connection = rospy.Timer(rospy.Duration(15.0), self.__connection_timeout, oneshot=True)
        while not rospy.is_shutdown() and not self.initialize():
            #rospy.sleep(1)
            rospy.logwarn_throttle(5, self._action_name + ": Waiting for gripper to be ready...")

        time.sleep(0.7)

        whatchdog_connection.shutdown()
        if self.is_ready():
            self.start()
            rospy.loginfo("Action server is Active")
        else:
            rospy.loginfo("Gripper Is Connected but Can't Activate ")
        
        self.__feedback = self.__getStatusFeedback()
        

    def __connection_timeout(self, event):
        rospy.logfatal("Gripper on port %s seems not to respond" % (self.com_port))
        rospy.signal_shutdown("Gripper on port %s seems not to respond" % (self.com_port))
        self._processing_goal = False

    def __movement_timeout(self, event):
        rospy.logerr("%s: Achieving goal is taking too long, dropping current goal")
    
    def __getStatusFeedback(self):
        status = CommandRobotiqGripperFeedback()
        status.header.stamp         = rospy.get_rostime()
        status.header.seq           = 0
        status.is_ready             = super().is_ready()
        status.is_reset             = super().is_reset()
        status.is_moving            = super().is_moving()
        status.obj_detected         = super().object_detected()
        status.fault_status         = super().get_fault_status()
        status.position             = super().get_pos()
        status.requested_position   = super().get_req_pos()
        status.current              = super().get_current()
        print(status)
        return status
    
    def __PosError(self):
        return abs(self.__feedback.requested_position - self.__feedback.position)

    def __abortingActionServer(self, abort_error):
        rospy.logerr("%s: Dropping current goal -> " + abort_error )
        self.set_aborted(self.__feedback , (self._action_name))

    def execute_callBack(self, goal):
        if not self.is_gripper_connected:
            self.__abortingActionServer("Connection Lost")
            
        
        self.__feedback = self.__getStatusFeedback()
        rospy.loginfo( (": New goal received Pos:%.3f Speed: %.3f Force: %.3f Force-Stop: %r") % (goal.position, goal.speed, goal.force, goal.stop) )

        success = False
        rate = rospy.Rate(1)

        
        if not goal.stop:
            is_modbus_msg_sent = self.goTo(goal.position, goal.speed, goal.force)
            self._processing_goal = True  
        else:
            rospy.logwarn_throttle(5, self._action_name + ": stop command is active")

        if not is_modbus_msg_sent:
            self.__abortingActionServer("Unable to Send Modbus MSG")
            
        watchdog_move = rospy.Timer(rospy.Duration(5.0), self.__movement_timeout, oneshot=True)

        while not rospy.is_shutdown() and self._processing_goal: 
            
            if not self.is_gripper_connected:
                self.__abortingActionServer("Connection Lost")
            
            self.__feedback = self.__getStatusFeedback()
            rospy.logdebug("Error = %.5f Requested position = %.3f Current position = %.3f" % (abs(self.__feedback.requested_position - self.__feedback.position), self.__feedback.requested_position, self.__feedback.position))
            
            if self.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self.set_preempted()
                break

            if self.__feedback.fault_status != 0:
                self.__abortingActionServer("Fault status (gFLT) is: %d" % self.__feedback.fault_status)
                self._processing_goal = False
                break
            if( self.__PosError() < GOAL_DETECTION_THRESHOLD or self.__feedback.obj_detected):
                self._processing_goal = False
                success = True
                print(success)
                break
            self.publish_feedback(self.__feedback)
        
            rate.sleep()
        

        self.__result = self.__feedback
        watchdog_move.shutdown()
        if success:
            rospy.logdebug(self._action_name + ": Goal reached or object detected Pos: %.3f PosRequested: %.3f ObjectDetected: %r" % (goal.position, self.__feedback.requested_position, self.__feedback.obj_detected) )
            self.set_succeeded(self.__result)
    
    def setGripperJointNames(self, joint_name1, joint_name2):
        self._joint_name = [joint_name1, joint_name2]
    
    def publish_joint_states(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            if self.is_gripper_connected:
                # __feedback = self.__getStatusFeedback()
                js = JointState()
                js.header.frame_id = ''
                js.header.stamp = rospy.Time.now()
                # js.header.seq = self._seq
                js.name = self._joint_name
                js.position = [self.__feedback.position/2-self._xacro_frame_error, self.__feedback.position-self._xacro_frame_error]
                super().publish(js)
                rate.sleep()
    def set_xacro_frame_error(self, error):
        self._xacro_frame_error = error
        

        

if __name__ == "__main__":

    rospy.init_node('robotiq_2f85_action_server')
    
    print('AS get Param')
    usb_port = rospy.get_param('~usb_port','/dev/ttyUSB0')
    name_finger1 = rospy.get_param('~name_finger1','ur5_bl_to_leftFinger')
    name_finger2 = rospy.get_param('~name_finger2','ur5_leftFinger_to_rightFinger')
    
    server_gripper = RobotiqGripperActionServer(action_server_name= "robotiq_hand_e", gripper_type = RobotiqGripperType.Hand_E, usbComPort = usb_port)
    server_gripper.setGripperJointNames(name_finger1, name_finger2)
    server_gripper.set_xacro_frame_error(0.008)
    thread_joint = Thread(target=server_gripper.publish_joint_states)
    thread_joint.start()
    
    # server_ur5e = RobotiqGripperActionServer(action_server_name= gripper_ur5e, gripper_type = RobotiqGripperType.Hand_E, usbComPort = "/dev/ttyUSB0")
    # server_ur5e.setGripperJointNames("ur5e_bl_to_leftFinger", "ur5e_leftFinger_to_rightFinger")
    # server_ur5 = RobotiqGripperActionServer(action_server_name= gripper_ur5, gripper_type = RobotiqGripperType.Hand_E, usbComPort = "/dev/ttyUSB1")
    # server_ur5.setGripperJointNames("ur5_bl_to_leftFinger", "ur5_leftFinger_to_rightFinger")
    # server_ur5.set_xacro_frame_error(0.008)
    # server_ur5e.set_xacro_frame_error(0.008)
    # thread_joint_states_ur5e= Thread(target=server_ur5e.publish_joint_states)
    # thread_joint_states_ur5 = Thread(target=server_ur5.publish_joint_states)
    # thread_joint_states_ur5.start()
    # thread_joint_states_ur5e.start()
    
    
    rospy.spin()
    