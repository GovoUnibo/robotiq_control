#!/usr/bin/python3
import rospy
import actionlib

#from multinherit.multinherit import multi_super  #pip3 install multinherit

from robotiq_control.msg import CommandRobotiqGripperAction
from robotiq_control.msg import CommandRobotiqGripperGoal



def operate_gripper():

    action_name = 'robotiq_hand_e'
    robotiq_client = actionlib.SimpleActionClient(action_name, CommandRobotiqGripperAction)
    rospy.loginfo("Client: Waiting for Server")
    robotiq_client.wait_for_server()
    rospy.loginfo("Client: Connected to Server" + action_name)
    goal = CommandRobotiqGripperGoal()
    goal.emergency_release = False
    goal.stop = False
    goal.position = 0.001#float
    goal.speed = 0.02 #float
    goal.force = 10.1 #int

    robotiq_client.send_goal(goal)
    robotiq_client.wait_for_result()
    result = robotiq_client.get_result()
    print(result)

if __name__ == '__main__':
    # Initializes a rospy node so that the SimpleActionClient can
    # publish and subscribe over ROS.
    rospy.init_node('node_ac')
    result = operate_gripper()