# !/usr/bin/env python 3.8.10
from robotiq_control.msg import CommandRobotiqGripperAction
from robotiq_control.msg import CommandRobotiqGripperFeedback
from robotiq_control.msg import CommandRobotiqGripperResult
from sensor_msgs.msg import JointState

from robotiq_control.GripperCommon import RobotiqGripperType
from robotiq_control.GripperSocket import GripperSocket

import rospy, actionlib

import time
GOAL_DETECTION_THRESHOLD = 0.001 # Max deviation from target goal to consider as goal "reached"

class GripperActSrvTcp_Ip(GripperSocket, actionlib.SimpleActionServer):
    def __init__(self, act_srv_name, robot_ip="127.0.0.1", port=63352, gripper_type = RobotiqGripperType.Hand_E):
        self.robot_ip = robot_ip
        self.port = port
        self._action_name = act_srv_name
        GripperSocket.__init__(self, robot_ip, port, gripper_type)
        actionlib.SimpleActionServer.__init__(self, self._action_name, CommandRobotiqGripperAction, execute_cb=self.__execute_callBack, auto_start=False)

        js_pub = rospy.Publisher('joint_states', JointState, queue_size=10)

        whatchdog_connection = rospy.Timer(rospy.Duration(15.0), self.__connection_timeout, oneshot=True)
        init_done = False
        while not rospy.is_shutdown() and not init_done:
            rospy.logwarn_throttle(5, self._action_name + ": Waiting for gripper to be ready...")
            init_done = super().initialize()
            time.sleep(1)

        whatchdog_connection.shutdown()
        if super().isReady():
            actionlib.SimpleActionServer.start(self)
            rospy.logdebug_throttle(5, "Action server {} is Active".format(self._action_name))
        else:
            rospy.loginfo("Gripper Is Connected but Can't Activate ")
        
        self._feedback = self.__buildFdbkMsg()
        print(self._feedback)
        

    def __connection_timeout(self, event):
        rospy.logfatal("Gripper on ip: {self.ip} seems not to respond")
        rospy.signal_shutdown("Gripper on port {self.ip} seems not to respond")

    def __buildFdbkMsg(self):
        status = CommandRobotiqGripperFeedback()
        status.header.stamp         = rospy.get_rostime()
        status.header.seq           = 0
        status.is_ready             = super().isReady()
        status.is_reset             = super().isSleeping()
        status.is_moving            = False #can't obtain directly
        status.obj_detected         = super().graspDetected()
        status.fault_status         = super().getFaultId()
        status.position             = super().getActualPos()
        status.requested_position   = super().getRequestedPosition()
        status.current              = super().getCurrent()
        # print(status)
        return status

    def __movement_timeout(self, event):
        rospy.logerr("%s: Achieving goal is taking too long, dropping current goal")

    def __abortingActionServer(self, abort_error):
        rospy.logerr("%s: Dropping current goal -> " + abort_error )
        actionlib.SimpleActionServer.set_aborted(self, self.__feedback , (self._action_name))

    def __execute_callBack(self, goal):
        self._processing_goal = False
        rospy.loginfo( (": New goal received Pos:%.3f Speed: %.3f Force: %.3f Force-Stop: %r") % (goal.position, goal.speed, goal.force, goal.stop) )
    
        success = False
        rate = rospy.Rate(1)

        if not goal.stop:
            cmd_sent = self.moveToPos( goal.position, goal.speed, goal.force)

        if not cmd_sent:
            self.__abortingActionServer("Unable to Send Tcp Command") 
        else:
            self._processing_goal = True 
        
        watchdog_move = rospy.Timer(rospy.Duration(5.0), self.__movement_timeout, oneshot=True)
        
        while not rospy.is_shutdown() and self._processing_goal:
            print("daje")
            self.__feedback = self.__buildFdbkMsg()
            rospy.logdebug("Error = %.5f Requested position = %.3f Current position = %.3f" % (self.__PosError(), self.__feedback.requested_position, self.__feedback.position))
            
            if self.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                actionlib.SimpleActionServer.set_preempted(self)
                break

            if self.__feedback.fault_status != 0:
                self.__abortingActionServer("Fault status (FLT) is: %d" % self.__feedback.fault_status)
                self._processing_goal = False
                break
            if( self.__PosError() < GOAL_DETECTION_THRESHOLD or self.__feedback.obj_detected):
                self._processing_goal = False
                success = True
                # print(success)
                break

            actionlib.SimpleActionServer.publish_feedback(self, self.__feedback)
        
            rate.sleep()
        

        self.__result = self.__feedback
        watchdog_move.shutdown()
        if success:
            rospy.logdebug(self._action_name + ": Goal reached or object detected Pos: %.3f PosRequested: %.3f ObjectDetected: %r" % (goal.position, self.__feedback.requested_position, self.__feedback.obj_detected) )
            self.set_succeeded(self.__result)

    def __PosError(self):
        return abs(self.__feedback.requested_position - self.__feedback.position)


if __name__ == "__main__":
    rospy.init_node('node_as', log_level=rospy.DEBUG)

    gripper_HandE = GripperActSrvTcp_Ip(act_srv_name = "robotiq_hand_e", robot_ip="192.168.0.102")

    rospy.spin()