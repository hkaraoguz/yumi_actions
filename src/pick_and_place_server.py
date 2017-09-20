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
        yumi.plan_and_move(yumi.group_l,
                           yumi.create_pose_euler(pose_ee[0], pose_ee[1], pose_ee[2], pose_ee[3], pose_ee[4],
                                                  pose_ee[5]))
    elif (arm == yumi.RIGHT):
        yumi.plan_and_move(yumi.group_r,
                           yumi.create_pose_euler(pose_ee[0], pose_ee[1], pose_ee[2], pose_ee[3], pose_ee[4],
                                                  pose_ee[5]))


    



class PickPlaceServer:

    def __init__(self):
        self._feedback = PickPlaceActionFeedback()
        self._result = PickPlaceActionResult()
        self.server = actionlib.SimpleActionServer('pick_and_place', PickPlaceAction, self.execute, False)
        self.server.register_preempt_callback(self.preempt_callback)
        self.cancelled = False
	try:
           yumi.init_Moveit()
        except:
           rospy.logwarn("Cannot initialize move_it. Quitting.")
           rospy.signal_shutdown("Cannot initialize move_it")
        
        self.server.start()

    def preempt_callback(self):

        rospy.logwarn("Pick and Place action got cancelled!!")
        self.cancelled = True
        self._result.result = -1
        yumi.reset_pose()
        self.server.set_preempted(self._result)



    def execute(self, goal):

        self.cancelled = False

        hand_id = -1

        if goal.location.position.y > 0.0:
            hand_id = yumi.LEFT
        else:
            hand_id = yumi.RIGHT

        # Open the grippers initially
        grip_effort = -10.0
        open_grippers(yumi.LEFT)
        open_grippers(yumi.RIGHT)
        pose_ee_t = [goal.location.position.x, goal.location.position.y, 0.3, 0.0,3.14, goal.location.orientation.z]

        if not self.cancelled:
            move_and_grasp(hand_id, pose_ee_t, grip_effort)
            rospy.sleep(2.0)


        pose_ee_t[2] = 0.2

        if not self.cancelled:
            move_and_grasp(hand_id, pose_ee_t, grip_effort)
            rospy.sleep(2.0)
            close_grippers(hand_id)

        pose_ee_t[2] = 0.3
        if not self.cancelled:
            move_and_grasp(hand_id, pose_ee_t, grip_effort)
            rospy.sleep(2.0)


        if hand_id == yumi.LEFT:
            pose_ee = [0.1, 0.4, 0.3, 0.0, -2.0, 0.0]
            grip_effort = 10.0

            if not self.cancelled:
                move_and_grasp(hand_id, pose_ee, grip_effort)
                rospy.sleep(2.0)
                open_grippers(hand_id)
                #rospy.sleep(2.0)
                #pose_ee = [0.1, 0.4, 0.3, 0.0, 3.14, 0.0]
                #move_and_grasp(hand_id, pose_ee, grip_effort)


        else:
            pose_ee = [0.1, -0.4, 0.3, 0.0, -2.0, 0.0]
            if not self.cancelled:
                move_and_grasp(hand_id, pose_ee, grip_effort)
                rospy.sleep(2.0)
                open_grippers(hand_id)
                #rospy.sleep(2.0)
                #pose_ee = [0.1, -0.4, 0.3, 0.0, 3.14, 0.0]
                #move_and_grasp(hand_id, pose_ee, grip_effort)
        #Reset YuMi joints to "home" position
        yumi.reset_pose()
    
        if not self.cancelled:
            self._result.result = 0
            self.server.set_succeeded(self._result)



if __name__ == '__main__':
    rospy.init_node('pick_and_place_server')
    server = PickPlaceServer()
    rospy.spin()
