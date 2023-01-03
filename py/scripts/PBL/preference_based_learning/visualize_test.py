#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import sys
import os
import rospy
import moveit_commander
import moveit_msgs.msg
import trajectory_msgs.msg
from dsr_msgs.srv import OperatePytamp

sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))))

import control
import environment

sys.dont_write_bytecode = True
sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__),"../../../../../common/imp")) ) # get import path : DSR_ROBOT.py 

# for single robot 
ROBOT_ID     = "dsr01"
ROBOT_MODEL  = "a0912"
import DR_init
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL
from DSR_ROBOT import *

import std_msgs.msg
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list, list_to_pose

def SET_ROBOT(id, model):
    ROBOT_ID = id; ROBOT_MODEL= model   




operate_robot = rospy.ServiceProxy('/operate_robot_joint',OperatePytamp)





def main():
    pass


def get_user_feedback(psi, idx):

    setup = environment.environment()
    planning = control.control()

    #table, box, laptop, visualhuman = setup.object_co()

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("move_group_python_test", anonymous=True)

    robot = moveit_commander.RobotCommander()
    arm_move_group = moveit_commander.MoveGroupCommander("arm")
    hand_move_group = moveit_commander.MoveGroupCommander("robotiq_2f")

    alpha = 0

    table,_ = setup.add_box(position_x = alpha + 1.025, position_z = -0.465, size = (1.2,1.4,0.72), box_name = 'table') 
    print("============== Table in the scene ===============")
    box,_ = setup.add_box(position_x = alpha + 0.7, position_y = -0.3, position_z = -0.055, size = (0.1,0.1,0.1), box_name = 'box')
    print("============== Box in the scene ===============")
    laptop,_ = setup.add_mesh(position_x = alpha + 1.025 , position_z = -0.055 , size = (0.1,0.1,0.1) , mesh_name = 'laptop')
    print("============== Laptop in the scene ===============")
    visualhuman,_ = setup.add_mesh(position_x = 1.8 , position_z = -0.25 , orientation_x = -0.5, orientation_y = 0.5, orientation_z = 0.5, orientation_w = -0.5, size = (0.5,0.5,0.5) , mesh_name = 'visualhuman')
    print("============== Visualhuman in the scene ===============")

    wall,_ = setup.add_box(position_y = 0.8, size = (5, 0.1, 3), box_name = 'wall')
    print("============== Wall in the scene ===============")


    

    s = 0


    while s == 0:


        selection = input('A/B to watch, 1/2 to vote: ').lower()

        if selection == 'a':
        
            planning.initial_joint_pose()
            input()

            planning.arm_cartesian_plan(object_pose = [0.7 , -0.3 , 0.3])
            input()

            planning.arm_cartesian_plan(object_pose = [0.7 , -0.3 , 0.2])
            input()

            planning.gripper_control(value = 0.7)
            input()
            
            planning.arm_cartesian_plan(object_pose = [0.7 , -0.3 , 0.4])
            input()

            mid_trajectory = np.load("/home/kim/catkin_ws/src/doosan-robot/dsr_example/py/scripts/PBL/sampled_trajectories/mid_trajectory/mid_trajectory_{num}.npz".format(num = idx[0]), allow_pickle=True)['plan']

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


            planning.arm_cartesian_plan(object_pose = [0.7, 0.3, 0.3])
            input()
            
            planning.arm_cartesian_plan(object_pose = [0.7, 0.3, 0.2])
            input()

            planning.gripper_control(value = 0)
            input()

            planning.arm_cartesian_plan(object_pose = [0.7 , 0.3 , 0.4])
            input()

            planning.arm_cartesian_plan(object_pose = [0.516021273967, 0.0390030718983, 0.630885729686], approach_direction = [-4.88940814326e-05, -0.999999963883, 0.000231584538729, 0.00012732903351])
            input()


        elif selection == 'b':

            planning.initial_joint_pose()
            input()

            planning.arm_cartesian_plan(object_pose = [0.7 , -0.3 , 0.3])
            input()

            planning.arm_cartesian_plan(object_pose = [0.7 , -0.3 , 0.2])
            input()

            planning.gripper_control(value = 0.7)
            input()
            
            planning.arm_cartesian_plan(object_pose = [0.7 , -0.3 , 0.4])
            input()

            mid_trajectory = np.load("/home/kim/catkin_ws/src/doosan-robot/dsr_example/py/scripts/PBL/sampled_trajectories/mid_trajectory/mid_trajectory_{num}.npz".format(num = idx[1]), allow_pickle=True)['plan']

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


            planning.arm_cartesian_plan(object_pose = [0.7, 0.3, 0.3])
            input()
            
            planning.arm_cartesian_plan(object_pose = [0.7, 0.3, 0.2])
            input()

            planning.gripper_control(value = 0)
            input()

            planning.arm_cartesian_plan(object_pose = [0.7 , 0.3 , 0.4])
            input()

            planning.arm_cartesian_plan(object_pose = [0.516021273967, 0.0390030718983, 0.630885729686], approach_direction = [-4.88940814326e-05, -0.999999963883, 0.000231584538729, 0.00012732903351])
            input()


        elif selection == '1':
            s = 1
        elif selection == '2':
            s = -1




    return psi, s







if __name__ == "__main__":
    print(get_user_feedback("dd", (0, 1)))


















