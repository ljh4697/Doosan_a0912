#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import os
import threading, time
import sys

sys.dont_write_bytecode = True
sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__),"../../../../common/imp")) ) # get import path : DSR_ROBOT.py 

# Python 2/3 compatibility imports
from six.moves import input
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import numpy as np

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))

# for single robot 
ROBOT_ID     = "dsr01"
ROBOT_MODEL  = "a0912"
import DR_init
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL
from DSR_ROBOT import *


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import Grasp, GripperTranslation, PlaceLocation
from trajectory_msgs.msg import JointTrajectoryPoint
from tf.transformations import quaternion_from_euler


def SET_ROBOT(id, model):
    ROBOT_ID = id; ROBOT_MODEL= model   

def shutdown():
    print("shutdown time!")
    print("shutdown time!")
    print("shutdown time!")

    pub_stop.publish(stop_mode=1) #STOP_TYPE_QUICK)
    return 0

# convert list to Float64MultiArray
def _ros_listToFloat64MultiArray(list_src):
    _res = []
    for i in list_src:
        item = Float64MultiArray()
        item.data = i
        _res.append(item)
    #print(_res)
    #print(len(_res))
    return _res

def gripper_control(value):
    srv_robotiq_2f_move(value)
    srv_robotiq_gripper_move(0.7-value)
    
def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    all_equal = True
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

    return True



if __name__ == "__main__":
    # robot default setting
    my_robot_id    = "dsr01"
    my_robot_model = "a0912"
    SET_ROBOT(my_robot_id, my_robot_model)
    rospy.init_node('move_group_pick_demo_py', anonymous=True)
    rospy.on_shutdown(shutdown)
    pub_stop = rospy.Publisher('/'+ROBOT_ID +ROBOT_MODEL+'/stop', RobotStop, queue_size=10)           
    srv_robotiq_2f_move = rospy.ServiceProxy('/' + ROBOT_ID + ROBOT_MODEL + '/gripper/robotiq_2f_move', Robotiq2FMove)
    srv_robotiq_gripper_move = rospy.ServiceProxy('/robotiq_control_move', Robotiq2FMove)
    
    ## declare moveit functions.
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    group_name = "arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    gripper_name = "robotiq_2f"
    gripper_group = moveit_commander.MoveGroupCommander(gripper_name)
    rospy.sleep(1)

    # clean the scene
    scene.remove_world_object("table")
    scene.remove_world_object("part")
    print("removed objects")
    raw_input()
    
    # publish a demo scene
    p = PoseStamped()
    p.header.frame_id = robot.get_planning_frame()

    # add a table
    p.pose.position.x = 0.62
    p.pose.position.y = -0.2
    p.pose.position.z = 0.3
    scene.add_box("table", p, (0.5, 1.5, 0.6))

    # add an object to be grasped
    p.pose.position.x = 0.405
    p.pose.position.y = -0.12
    p.pose.position.z = 0.7
    scene.add_box("part", p, (0.07, 0.01, 0.2))

    rospy.sleep(1)
    print("I added objects")
    
    print("grasp pose setting")
    raw_input()
    grasps = []


    g = Grasp()
    g.id = "test"
    grasp_pose = PoseStamped()
    grasp_pose.header.frame_id = "base_0"
    grasp_pose.pose.position.x = 0.18
    grasp_pose.pose.position.y = -0.116075
    grasp_pose.pose.position.z = 0.70493
    grasp_pose.pose.orientation.x = 0
    grasp_pose.pose.orientation.y = np.sqrt(2)/2
    grasp_pose.pose.orientation.z = 0
    grasp_pose.pose.orientation.w = np.sqrt(2)/2


    print(grasp_pose)
    print("go to grasp_pose")
    raw_input()
    move_group.set_pose_target(grasp_pose)
    move_group.go()

    print("continue")
    raw_input()
    rospy.sleep(2)

    # set the grasp pose
    g.grasp_pose = grasp_pose

    # define the pre-grasp approach
    g.pre_grasp_approach.direction.header.frame_id = "base_0"
    g.pre_grasp_approach.direction.vector.x = 1.0
    g.pre_grasp_approach.direction.vector.y = 0.0
    g.pre_grasp_approach.direction.vector.z = 0.0
    g.pre_grasp_approach.min_distance = 0.001
    g.pre_grasp_approach.desired_distance = 0.1

    g.pre_grasp_posture.header.frame_id = "robotiq_arg2f_base_link"
    g.pre_grasp_posture.joint_names = ["robotiq_85_left_knuckle_joint"]

    pos = JointTrajectoryPoint()
    pos.positions.append(0.0)
    
    g.pre_grasp_posture.points.append(pos)

    # set the grasp posture
    g.grasp_posture.header.frame_id = "robotiq_arg2f_base_link"
    g.grasp_posture.joint_names = ["robotiq_85_left_knuckle_joint"]

    pos = JointTrajectoryPoint()
    pos.positions.append(0.2)
    pos.effort.append(0.0)
   
    g.grasp_posture.points.append(pos)

    # set the post-grasp retreat
    g.post_grasp_retreat.direction.header.frame_id = "base_0"
    g.post_grasp_retreat.direction.vector.x = 0.0
    g.post_grasp_retreat.direction.vector.y = 0.0
    g.post_grasp_retreat.direction.vector.z = 1.0
    g.post_grasp_retreat.desired_distance = 0.25
    g.post_grasp_retreat.min_distance = 0.01

    g.allowed_touch_objects = ["table"]

    g.max_contact_force = 0
   
    # append the grasp to the list of grasps
    grasps.append(g)

    rospy.sleep(2)
    print("Let's pick")
    raw_input()
    # pick the object
    move_group.pick("part", grasps)
    print("finish")
    raw_input()

