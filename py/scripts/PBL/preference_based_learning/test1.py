#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import numpy as np
import sys
import os
import rospy
import moveit_commander
import copy
import moveit_msgs.msg
import trajectory_msgs.msg
from dsr_msgs.srv import OperatePytamp,Robotiq2FMove

sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))))

import control
import environment


sys.dont_write_bytecode = True
sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__),"../../../../../common/imp")) ) # get import path : DSR_ROBOT.py 

ROBOT_ID     = "dsr01"
ROBOT_MODEL  = "a0912"

import DR_init

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

from DSR_ROBOT import *

def SET_ROBOT(id, model):
    ROBOT_ID = id; ROBOT_MODEL= model   



setup = environment.environment()
planning = control.control()

operate_robot = rospy.ServiceProxy('/operate_robot_joint',OperatePytamp)
operate_gripper = rospy.ServiceProxy('/robotiq_control_move', Robotiq2FMove)

#table, box, laptop, visualhuman = setup.object_co()

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("move_group_python_test_", anonymous=True)

robot = moveit_commander.RobotCommander()
arm_move_group = moveit_commander.MoveGroupCommander("arm")
hand_move_group = moveit_commander.MoveGroupCommander("robotiq_2f")



def movesj_dsr(path , time = 5):
        # arg 
        #  path : joint path trajectory
        #  time : Time to execute all input trajectory
    path_ = copy.deepcopy(path)

    for i,joint_val in enumerate(path):
        path_[i] = posj(joint_val[0],joint_val[1],joint_val[2],joint_val[3],joint_val[4],joint_val[5])

    movesj(path_ , time = time)






if __name__ == "__main__":

    # Q1 = posj(0,0,90,0,90,0)
    # Q2 = posj(0,0,0,0,90,0)

    ######################### pick 


    initial_pose = posj(0,0,90,0,90,0)
    movej(initial_pose , time = 3)


    pick_trajectory = np.load("/home/kim/catkin_ws/src/doosan-robot/dsr_example/py/scripts/PBL/sampled_trajectories/pick_trajectory/pick_trajectory.npz", allow_pickle=True)['plan']

    for i in range(len(pick_trajectory)) :
        if i == 0 : 
            pick0 = list(pick_trajectory[0])
        if i == 1 :
            pick1 = list(pick_trajectory[1])
        if i == 2 :
            pick2 = list(pick_trajectory[2])

    pick0 = np.array(pick0).reshape(-1).tolist()
    pick1 = np.array(pick1).reshape(-1).tolist()
    pick2 = np.array(pick2).reshape(-1).tolist()

    # operate_robot(pick0, 5)
    operate_robot(pick1, 5)
    operate_robot(pick2, 5)

    input()
    ######################### close gripper

    operate_gripper(0)

    input()
    ######################### pick_up

    pick_up_trajectory = np.load("/home/kim/catkin_ws/src/doosan-robot/dsr_example/py/scripts/PBL/sampled_trajectories/pick_up_trajectory/pick_up_trajectory.npz", allow_pickle=True)['plan']
    pick_up_0 = np.array(pick_up_trajectory).reshape(-1).tolist()

    operate_robot(pick_up_0 , 5)

    ######################### mid_trajectory

    mid_trajectory = np.load("/home/kim/catkin_ws/src/doosan-robot/dsr_example/py/scripts/PBL/sampled_trajectories/mid_trajectory/mid_trajectory_1.npz", allow_pickle=True)['plan']


    for i in range(len(mid_trajectory)) :
        if i == 0 : 
            mid0 = list(mid_trajectory[0])
        if i == 1 :
            mid1 = list(mid_trajectory[1])
        if i == 2 :
            mid2 = list(mid_trajectory[2])
        if i == 3 :
            mid3 = list(mid_trajectory[3])


    mid0 = np.array(mid0).reshape(-1).tolist()
    mid1 = np.array(mid1).reshape(-1).tolist()
    mid2 = np.array(mid2).reshape(-1).tolist()
    mid3 = np.array(mid3).reshape(-1).tolist()

    operate_robot(mid0 , 5)
    operate_robot(mid1 , 5)
    operate_robot(mid2 , 5)
    operate_robot(mid3 , 5)


    ###################### place

    place_trajectory = np.load("/home/kim/catkin_ws/src/doosan-robot/dsr_example/py/scripts/PBL/sampled_trajectories/place_trajectory/place_trajectory.npz", allow_pickle=True)['plan']

    for i in range(len(place_trajectory)) :
        if i == 0 : 
            place0 = list(place_trajectory[0])
        if i == 1 :
            place1 = list(place_trajectory[1])

    place0 = np.array(place0).reshape(-1).tolist()
    place1 = np.array(place1).reshape(-1).tolist()

    operate_robot(place0 , 5)
    operate_robot(place1 , 5)

    ##################### open gripper

    operate_gripper(0.7)

    ###################### place_up

    place_up_trajectory = np.load("/home/kim/catkin_ws/src/doosan-robot/dsr_example/py/scripts/PBL/sampled_trajectories/place_up_trajectory/place_up_trajectory.npz", allow_pickle=True)['plan']

    place_up_0 = np.array(place_up_trajectory).reshape(-1).tolist()

    operate_robot(place_up_0 , 5)

    ######################## back to initial state

    back_to_initial_trajectory = np.load("/home/kim/catkin_ws/src/doosan-robot/dsr_example/py/scripts/PBL/sampled_trajectories/back_to_initial_trajectory/back_to_initial_trajectory.npz", allow_pickle=True)['plan']

    back_0 = np.array(back_to_initial_trajectory).reshape(-1).tolist()

    operate_robot(back_0 , 5)