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
import environment2
import control

planning = control.control()

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

import std_msgs.msg
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list, list_to_pose

def SET_ROBOT(id, model):
    ROBOT_ID = id; ROBOT_MODEL= model   


from dsr_msgs.srv import OperatePytamp,Robotiq2FMove

operate_robot = rospy.ServiceProxy('/operate_robot_joint',OperatePytamp)
operate_gripper = rospy.ServiceProxy('/robotiq_control_move', Robotiq2FMove)

###########################################################

rospy.init_node("move_group_python_test", anonymous=True)

scene = moveit_commander.PlanningSceneInterface(synchronous=True)
scene.clear()

setup = environment2.environment()


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


###########################################################


def sampleBernoulli(mean):
    ''' function to obtain a sample from a Bernoulli distribution

    Input:
    mean -- mean of the Bernoulli
    
    Output:
    sample -- sample (0 or 1)
    '''


    if np.random.rand(1) < mean: return 1
    else: return 0



def mu(x, theta):
    return 1/(1+np.exp(-np.dot(x, theta)))




def get_feedback(algo, input_A, input_B, psi, w, m ="oracle", human='simulated'):
    

    s = 0

    
    if human=="simulated":
        while s==0:
            
            
            if m == "samling":
                # stochasitic samling model
                prefer_prob = mu(psi, w)
                s = sampleBernoulli(prefer_prob)
                if s == 0:
                    s=-1
            
            elif m == "oracle":
            
                # oracle model    
                if np.dot(psi, w)>0:
                    s = 1
                else:
                    s =-1
    
    elif human=="real":
        
        if algo.simulation_object.name =="avoid":
            
            
            idx = input_A
            #objects_co = algo.objects_co

            s = 0

            while s==0:
                initial_pose = posj(0,0,90,0,90,0)
                movej(initial_pose , time = 3)
                
                setup.new_box()
                selection = input('A/B to watch, 1/2 to vote, q to quit: ').lower()
                

                if selection == 'a':
        
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

                    # rviz
                    planning.hold_hand(object = 'box')

                    ######################### pick_up

                    pick_up_trajectory = np.load("/home/joonhyeok/catkin_ws/src/doosan-robot/doosan-robot/dsr_example/py/scripts/PBL/sampled_trajectories/pick_up_trajectory/pick_up_trajectory.npz", allow_pickle=True)['plan']
                    pick_up_0 = np.array(pick_up_trajectory).reshape(-1).tolist()

                    operate_robot(pick_up_0 , 1)

                    ######################## mid_trajectory

                    mid_trajectory = np.load("/home/joonhyeok/catkin_ws/src/doosan-robot/doosan-robot/dsr_example/py/scripts/PBL/sampled_trajectories/mid_trajectory/mid_trajectory_{num}.npz".format(num = idx), allow_pickle=True)['plan']

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
                    
                    # rviz
                    planning.release_hand(object = 'box')

                    ###################### place_up

                    place_up_trajectory = np.load("/home/joonhyeok/catkin_ws/src/doosan-robot/doosan-robot/dsr_example/py/scripts/PBL/sampled_trajectories/place_up_trajectory/place_up_trajectory.npz", allow_pickle=True)['plan']

                    place_up_0 = np.array(place_up_trajectory).reshape(-1).tolist()

                    operate_robot(place_up_0 , 1)

                    ######################## back to initial state

                    back_to_initial_trajectory = np.load("/home/joonhyeok/catkin_ws/src/doosan-robot/doosan-robot/dsr_example/py/scripts/PBL/sampled_trajectories/back_to_initial_trajectory/back_to_initial_trajectory.npz", allow_pickle=True)['plan']

                    back_0 = np.array(back_to_initial_trajectory).reshape(-1).tolist()

                    operate_robot(back_0 , 1)


                elif selection == 'b':

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
                    operate_robot(pick1, 5)
                    operate_robot(pick2, 5)

                    ######################### close gripper

                    planning.gripper_control(value = 0.7)

                    # rviz
                    planning.hold_hand(object = 'box')

                    ######################### pick_up

                    pick_up_trajectory = np.load("/home/joonhyeok/catkin_ws/src/doosan-robot/doosan-robot/dsr_example/py/scripts/PBL/sampled_trajectories/pick_up_trajectory/pick_up_trajectory.npz", allow_pickle=True)['plan']
                    pick_up_0 = np.array(pick_up_trajectory).reshape(-1).tolist()

                    operate_robot(pick_up_0 , 5)

                    ######################## mid_trajectory

                    mid_trajectory = np.load("/home/joonhyeok/catkin_ws/src/doosan-robot/doosan-robot/dsr_example/py/scripts/PBL/sampled_trajectories/mid_trajectory/mid_trajectory_{num}.npz".format(num = idx*2+1), allow_pickle=True)['plan']

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

                    place_trajectory = np.load("/home/joonhyeok/catkin_ws/src/doosan-robot/doosan-robot/dsr_example/py/scripts/PBL/sampled_trajectories/place_trajectory/place_trajectory.npz", allow_pickle=True)['plan']

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

                    planning.gripper_control(value = 0)

                    # rviz
                    planning.release_hand(object = 'box')

                    ###################### place_up

                    place_up_trajectory = np.load("/home/joonhyeok/catkin_ws/src/doosan-robot/doosan-robot/dsr_example/py/scripts/PBL/sampled_trajectories/place_up_trajectory/place_up_trajectory.npz", allow_pickle=True)['plan']

                    place_up_0 = np.array(place_up_trajectory).reshape(-1).tolist()

                    operate_robot(place_up_0 , 5)

                    ######################## back to initial state

                    back_to_initial_trajectory = np.load("/home/joonhyeok/catkin_ws/src/doosan-robot/doosan-robot/dsr_example/py/scripts/PBL/sampled_trajectories/back_to_initial_trajectory/back_to_initial_trajectory.npz", allow_pickle=True)['plan']

                    back_0 = np.array(back_to_initial_trajectory).reshape(-1).tolist()

                    operate_robot(back_0 , 5)

                elif selection == '1':
                    s = 1
                    
                    Q1 = input(' Is there any change in the own’s optimal trajectory he/she was thinking of while interacting with the robot? (7-Strongly agree, 1-Strongly disagree) ').lower()
                    Q3 = input(' Can you definitely choose the preferred trajectory in the comparison query? (7-Strongly agree, 1-Strongly disagree) ').lower()

                elif selection == '2':
                    s = -1
                    

                elif selection == 'q':
                    exit()


        
        else:
        
            while s==0:
                
                
                algo.simulation_object.feed(input_A)
                phi_A = algo.simulation_object.get_features()
                algo.simulation_object.feed(input_B)
                phi_B = algo.simulation_object.get_features()
                psi = np.array(phi_A) - np.array(phi_B)
                

                selection = input('A/B to watch, 1/2 to vote: ').lower()
                
                if selection == 'a':
                    algo.simulation_object.feed(input_A)
                    algo.simulation_object.watch(1)
                    
                    np.savez('./trajectory_ex/{}/tj1.npz'.format(algo.simulation_object.name),
                             human=np.array(algo.simulation_object.get_trajectory())[:, 0],
                             robot=np.array(algo.simulation_object.get_trajectory())[:, 1])
                elif selection == 'b':
                    algo.simulation_object.feed(input_B)
                    algo.simulation_object.watch(1)
                    np.savez('./trajectory_ex/{}/tj2.npz'.format(algo.simulation_object.name),
                             human=np.array(algo.simulation_object.get_trajectory())[:, 0],
                             robot=np.array(algo.simulation_object.get_trajectory())[:, 1])
                elif selection == '1':
                    s = 1
                elif selection == '2':
                    s = -1
                elif selection == 'q':
                    exit()
        
 
        

    return psi, s


def create_env(task):
    if task == 'avoid':
        return Avoid()
    else:
        print('There is no task called ' + task)
        exit(0)


def run_algo(method, simulation_object, w_samples, b=10, B=200):
    if simulation_object.name == "avoid":
        if method == 'nonbatch':
            return a_algos.nonbatch(simulation_object, w_samples)
        if method == 'greedy':
            return a_algos.greedy(simulation_object, w_samples, b)
        elif method == 'medoids':
            return a_algos.medoids(simulation_object, w_samples, b, B)
        elif method == 'boundary_medoids':
            return a_algos.boundary_medoids(simulation_object, w_samples, b, B)
        elif method == 'successive_elimination':
            return a_algos.successive_elimination(simulation_object, w_samples, b, B)
        elif method == 'random':
            return a_algos.random(simulation_object, w_samples, b)
        elif method == 'dpp':
            return a_algos.dpp(simulation_object, w_samples, b, B)
        else:
            print('There is no method called ' + method)
            exit(0)
        
    else:
        if method == 'nonbatch':
            return algos.nonbatch(simulation_object, w_samples)
        if method == 'greedy':
            return algos.greedy(simulation_object, w_samples, b)
        elif method == 'medoids':
            return algos.medoids(simulation_object, w_samples, b, B)
        elif method == 'boundary_medoids':
            return algos.boundary_medoids(simulation_object, w_samples, b, B)
        elif method == 'successive_elimination':
            return algos.successive_elimination(simulation_object, w_samples, b, B)
        elif method == 'random':
            return algos.random(simulation_object, w_samples, b)
        elif method == 'dpp':
            return algos.dpp(simulation_object, w_samples, b, B)
        else:
            print('There is no method called ' + method)
            exit(0)


def func(ctrl_array, *args):
    simulation_object = args[0]
    w = np.array(args[1])
    simulation_object.set_ctrl(ctrl_array)
    features = simulation_object.get_features()
    return -np.mean(np.array(features).dot(w))

def perform_best(simulation_object, w, iter_count=10):
    u = simulation_object.ctrl_size
    lower_ctrl_bound = [x[0] for x in simulation_object.ctrl_bounds]
    upper_ctrl_bound = [x[1] for x in simulation_object.ctrl_bounds]
    opt_val = np.inf
    for _ in range(iter_count):
        temp_res = opt.fmin_l_bfgs_b(func, x0=np.random.uniform(low=lower_ctrl_bound, high=upper_ctrl_bound, size=(u)),
                                    args=(simulation_object, w), bounds=simulation_object.ctrl_bounds, approx_grad=True)
        print(temp_res[1])
        if temp_res[1] < opt_val:
            optimal_ctrl = temp_res[0]
            opt_val = temp_res[1]
    simulation_object.set_ctrl(optimal_ctrl)
    keep_playing = 'y'
    while keep_playing == 'y':
        keep_playing = 'u'
        simulation_object.watch(1)
        while keep_playing != 'n' and keep_playing != 'y':
            keep_playing = input('Again? [y/n]: ').lower()
    return -opt_val

