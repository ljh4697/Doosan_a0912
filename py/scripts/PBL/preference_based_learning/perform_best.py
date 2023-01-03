#!/usr/bin/env python3

import numpy as np
import scipy.optimize as opt
import algos
import a_algos
from models import  Avoid
import rospy
import moveit_commander
import os; import sys

sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))

import control



sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(os.path.dirname(__file__)))))


sys.dont_write_bytecode = True
sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__),"../../../../../common/imp")) ) # get import path : DSR_ROBOT.py 

# for single robot 
ROBOT_ID     = "dsr01"
ROBOT_MODEL  = "a0912"
import DR_init
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL
from DSR_ROBOT import *

def SET_ROBOT(id, model):
    ROBOT_ID = id; ROBOT_MODEL= model   


from dsr_msgs.srv import OperatePytamp,Robotiq2FMove




        
def main(DPB_current_w, BATCH_current_w):

    planning = control.control()

    operate_robot = rospy.ServiceProxy('/operate_robot_joint',OperatePytamp)
    operate_gripper = rospy.ServiceProxy('/robotiq_control_move', Robotiq2FMove)

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("perform_best", anonymous=True)

    robot = moveit_commander.RobotCommander()
    hand_move_group = moveit_commander.MoveGroupCommander("robotiq_2f")

    ############################################################  CHANGE ####################################################################
    feature_set = np.load("/home/joonhyeok/catkin_ws/src/doosan-robot/doosan-robot/dsr_example/py/scripts/PBL/sampled_trajectories/avoid_features.npz", allow_pickle=True)['features']
    #DPB_current_w = np.load("/home/joonhyeok/catkin_ws/src/doosan-robot/doosan-robot/dsr_example/py/scripts/PBL/preference_based_learning/simulation_results/avoid_DPB_iter20_seed1.npy", allow_pickle=True)['current_w']
    #BATCH_current_w = np.load("/home/joonhyeok/catkin_ws/src/doosan-robot/doosan-robot/dsr_example/py/scripts/PBL/preference_based_learning/simulation_results/avoid_batch_active_PBL_greedy_iter20_seed1.npy", allow_pickle=True)['current_w']
    ####################################################################################################################################################

    DPB_reward = np.matmul(feature_set , DPB_current_w)
    BATCH_reward = np.matmul(feature_set , BATCH_current_w)

    DPB_best = np.argmax(DPB_reward)
    BATCH_best = np.argmax(BATCH_reward)

    DPB_best_mid_trajectory = np.load("/home/joonhyeok/catkin_ws/src/doosan-robot/doosan-robot/dsr_example/py/scripts/PBL/sampled_trajectories/mid_trajectory/mid_trajectory_{num}.npz".format(num = DPB_best), allow_pickle=True)['plan']
    BATCH_best_mid_trajectory = np.load("/home/joonhyeok/catkin_ws/src/doosan-robot/doosan-robot/dsr_example/py/scripts/PBL/sampled_trajectories/mid_trajectory/mid_trajectory_{num}.npz".format(num = BATCH_best), allow_pickle=True)['plan']


    initial_pose = posj(0,0,90,0,90,0)
    movej(initial_pose , time = 3)
    a = 1
    while a == 1:

        selection = input('Query selection is finished ,1/2 to watch, q to quit: ').lower()

        if selection == '1':
            #DPB
            
            ###################### pick
            pick_trajectory = np.load("/home/joonhyeok/catkin_ws/src/doosan-robot/doosan-robot/dsr_example/py/scripts/PBL/sampled_trajectories/pick_trajectory/pick_trajectory.npz", allow_pickle=True)['plan']

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
            operate_robot(pick1, 3)
            operate_robot(pick2, 3)

            ######################### close gripper

            operate_gripper(0)

            ######################### pick_up

            pick_up_trajectory = np.load("/home/joonhyeok/catkin_ws/src/doosan-robot/doosan-robot/dsr_example/py/scripts/PBL/sampled_trajectories/pick_up_trajectory/pick_up_trajectory.npz", allow_pickle=True)['plan']
            pick_up_0 = np.array(pick_up_trajectory).reshape(-1).tolist()

            operate_robot(pick_up_0 , 3)

            ######################## mid_trajectory

            mid_trajectory = DPB_best_mid_trajectory

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
            
            operate_robot(mid0 , 1)
            operate_robot(mid1 , 1)
            operate_robot(mid2 , 1)
            operate_robot(mid3 , 1)


            ###################### place

            place_trajectory = np.load("/home/joonhyeok/catkin_ws/src/doosan-robot/doosan-robot/dsr_example/py/scripts/PBL/sampled_trajectories/place_trajectory/place_trajectory.npz", allow_pickle=True)['plan']

            for i in range(len(place_trajectory)) :
                if i == 0 : 
                    place0 = list(place_trajectory[0])
                if i == 1 :
                    place1 = list(place_trajectory[1])

            place0 = np.array(place0).reshape(-1).tolist()
            place1 = np.array(place1).reshape(-1).tolist()

            operate_robot(place0 , 1)
            operate_robot(place1 , 1)

            ##################### open gripper

            operate_gripper(0.7)

            ###################### place_up

            place_up_trajectory = np.load("/home/joonhyeok/catkin_ws/src/doosan-robot/doosan-robot/dsr_example/py/scripts/PBL/sampled_trajectories/place_up_trajectory/place_up_trajectory.npz", allow_pickle=True)['plan']

            place_up_0 = np.array(place_up_trajectory).reshape(-1).tolist()

            operate_robot(place_up_0 , 1)

            ######################## back to initial state

            back_to_initial_trajectory = np.load("/home/joonhyeok/catkin_ws/src/doosan-robot/doosan-robot/dsr_example/py/scripts/PBL/sampled_trajectories/back_to_initial_trajectory/back_to_initial_trajectory.npz", allow_pickle=True)['plan']

            back_0 = np.array(back_to_initial_trajectory).reshape(-1).tolist()

            operate_robot(back_0 , 1)

        elif selection == '2':
            #batch greedy
            ###################### pick

            pick_trajectory = np.load("/home/joonhyeok/catkin_ws/src/doosan-robot/doosan-robot/dsr_example/py/scripts/PBL/sampled_trajectories/pick_trajectory/pick_trajectory.npz", allow_pickle=True)['plan']

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
            operate_robot(pick1, 1)
            operate_robot(pick2, 1)

            ######################### close gripper

            operate_gripper(0)

            ######################### pick_up

            pick_up_trajectory = np.load("/home/joonhyeok/catkin_ws/src/doosan-robot/doosan-robot/dsr_example/py/scripts/PBL/sampled_trajectories/pick_up_trajectory/pick_up_trajectory.npz", allow_pickle=True)['plan']
            pick_up_0 = np.array(pick_up_trajectory).reshape(-1).tolist()

            operate_robot(pick_up_0 , 1)

            ######################## mid_trajectory

            mid_trajectory = BATCH_best_mid_trajectory

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
            
            operate_robot(mid0 , 1)
            operate_robot(mid1 , 1)
            operate_robot(mid2 , 1)
            operate_robot(mid3 , 1)


            ###################### place

            place_trajectory = np.load("/home/joonhyeok/catkin_ws/src/doosan-robot/doosan-robot/dsr_example/py/scripts/PBL/sampled_trajectories/place_trajectory/place_trajectory.npz", allow_pickle=True)['plan']

            for i in range(len(place_trajectory)) :
                if i == 0 : 
                    place0 = list(place_trajectory[0])
                if i == 1 :
                    place1 = list(place_trajectory[1])

            place0 = np.array(place0).reshape(-1).tolist()
            place1 = np.array(place1).reshape(-1).tolist()

            operate_robot(place0 , 1)
            operate_robot(place1 , 1)

            ##################### open gripper

            operate_gripper(0.7)

            ###################### place_up

            place_up_trajectory = np.load("/home/joonhyeok/catkin_ws/src/doosan-robot/doosan-robot/dsr_example/py/scripts/PBL/sampled_trajectories/place_up_trajectory/place_up_trajectory.npz", allow_pickle=True)['plan']

            place_up_0 = np.array(place_up_trajectory).reshape(-1).tolist()

            operate_robot(place_up_0 , 1)

            ######################## back to initial state

            back_to_initial_trajectory = np.load("/home/joonhyeok/catkin_ws/src/doosan-robot/doosan-robot/dsr_example/py/scripts/PBL/sampled_trajectories/back_to_initial_trajectory/back_to_initial_trajectory.npz", allow_pickle=True)['plan']

            back_0 = np.array(back_to_initial_trajectory).reshape(-1).tolist()

            operate_robot(back_0 , 1)

        else:
            a = 2