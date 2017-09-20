#!/usr/bin/env python


import sys
import copy
import rospy
import moveit_commander
from yumi_demos import yumi_moveit_utils as yumi

from geometry_msgs.msg import Pose
import moveit_msgs.msg
import geometry_msgs.msg
from std_srvs.srv import Empty
import actionlib
from yumi_actions.msg import *


def close_grippers(arm):
    """Closes the grippers.

    Closes the grippers with an effort of 15 and then relaxes the effort to 0.

    :param arm: The side to be closed (moveit_utils LEFT or RIGHT)
    :type arm: int
    :returns: Nothing
    :rtype: None
    """
    yumi.gripper_effort(arm, 15.0)
   # yumi.gripper_effort(arm, 0.0)

def open_grippers(arm):
    """Opens the grippers.

    Opens the grippers with an effort of -15 and then relaxes the effort to 0.

    :param arm: The side to be opened (moveit_utils LEFT or RIGHT)
    :type arm: int
    :returns: Nothing
    :rtype: None
    """
    yumi.gripper_effort(arm, -10.0)
    yumi.gripper_effort(arm, 0.0)


class HomePositionServer:

    def __init__(self):
        self._feedback = PointActionFeedback()
        self._result = PointActionResult()
        self.server = actionlib.SimpleActionServer('yumi_home_position', HomeAction, self.execute, False)
        self.server.register_preempt_callback(self.preempt_callback)
        self.cancelled = False
	try:
           yumi.init_Moveit()
        except:
           rospy.logwarn("Cannot initialize move_it. Quitting.")
           rospy.signal_shutdown("Cannot initialize move_it")

        self.server.start()

    def preempt_callback(self):

        rospy.logwarn("Return to Home position action got cancelled!!")
        self.cancelled = True
        self._result.result = -1
        self.server.set_preempted(self._result)


    def execute(self, goal):

        self.cancelled = False

        hand_id = -1


        # Close the hand gripper
        grip_effort=0.0
        close_grippers(yumi.LEFT)
        yumi.gripper_effort(yumi.LEFT, 0.0)
        close_grippers(yumi.RIGHT)
        yumi.gripper_effort(yumi.RIGHT, 0.0)

        #Reset YuMi joints to "home" position
        yumi.reset_pose()

        if not self.cancelled:
            self._result.result = 0
            self.server.set_succeeded(self._result)



if __name__ == '__main__':
    rospy.init_node('home_position_server')
    server = HomePositionServer()
    rospy.spin()
