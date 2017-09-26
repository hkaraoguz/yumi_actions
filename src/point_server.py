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



def move_and_grasp(arm, pose_ee, grip_effort):

    #if (grip_effort <= 20 and grip_effort >= -20):
    #    yumi.gripper_effort(arm, grip_effort)
    #else:
    #    print("The gripper effort values should be in the range [-20, 20]")
   # try:
   #     yumi.traverse_path([pose_ee], arm, 10)
   # except Exception as ex:
   #     rospy.logerr(str(ex))
   #     return

    if (arm == yumi.LEFT):
        if not yumi.traverse_path([pose_ee], arm, 10):
            return False
        #yumi.plan_and_move(yumi.group_l,
        #                   yumi.create_pose_euler(pose_ee[0], pose_ee[1], pose_ee[2], pose_ee[3], pose_ee[4],
        #                                          pose_ee[5]))
    elif (arm == yumi.RIGHT):
        if not yumi.traverse_path([pose_ee], arm, 10):
            return False

    return True

        #yumi.plan_and_move(yumi.group_r,
        #                   yumi.create_pose_euler(pose_ee[0], pose_ee[1], pose_ee[2], pose_ee[3], pose_ee[4],
        #                                          pose_ee[5]))





class PointServer:

    def __init__(self):
        self._feedback = PointActionFeedback()
        self._result = PointActionResult()
        self.server = actionlib.SimpleActionServer('yumi_point', PointAction, self.execute, False)
        self.server.register_preempt_callback(self.preempt_callback)
        self.cancelled = False
	try:
           yumi.init_Moveit()
        except:
           rospy.logwarn("Cannot initialize move_it. Quitting.")
           rospy.signal_shutdown("Cannot initialize move_it")

        self.server.start()

    def preempt_callback(self):

        rospy.logwarn("Point action got cancelled!!")
        self.cancelled = True
        self._result.result = -1
        yumi.reset_pose()
        self.server.set_preempted(self._result)

    def abort(self):
        yumi.reset_pose()
        self._result.result = -2
        self.server.set_aborted(self._result)
        self.cancelled = True


    def execute(self, goal):

        self.cancelled = False

        hand_id = -1

        if goal.location.position.y > 0.0:
            hand_id = yumi.LEFT
        else:
            hand_id = yumi.RIGHT

        # Close the hand gripper
        grip_effort=0.0
        close_grippers(hand_id)
        yumi.gripper_effort(hand_id, 0.0)

        pose_ee_t = [goal.location.position.x, goal.location.position.y, 0.3, 0.0,3.14, 0.0]

        if not self.cancelled:
            if not move_and_grasp(hand_id, pose_ee_t, grip_effort):
                self.abort()
                return
            rospy.sleep(3.0)


        #Reset YuMi joints to "home" position
        #yumi.reset_pose()

        if not self.cancelled:
            self._result.result = 0
            self.server.set_succeeded(self._result)



if __name__ == '__main__':
    rospy.init_node('point_server')
    server = PointServer()
    rospy.spin()
