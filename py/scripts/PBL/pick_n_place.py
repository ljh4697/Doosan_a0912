#!/usr/bin/env python3

import sys,os
import rospy
import moveit_commander
from tf.transformations import quaternion_from_euler
import numpy as np
import copy
from math import pi
import environment
import control
from moveit_commander.conversions import pose_to_list, list_to_pose


def main():
    
    
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('pick_n_place', anonymous=True)

    
    #robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface(synchronous=True)

    arm_move_group = moveit_commander.MoveGroupCommander('arm')
    hand_move_group = moveit_commander.MoveGroupCommander('robotiq_2f')

    # environment setting

    scene.clear()

    setup = environment.environment()


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



    # generate trajectory 

    pick_trajectory = list()
    pick_up_trajectory = list()
    
    place_trajectory = list()
    place_up_trajectory = list()

    back_to_initial_trajectory = list()



    # control setting

    planning = control.control()

    ## initial_pose

    p, plan = planning.initial_joint_pose()
    pick_trajectory.append(plan)

    ## pick

    p, plan = planning.arm_cartesian_plan(object_pose = [0.7 , -0.3 , 0.3])
    pick_trajectory.append(plan)

    p, plan = planning.arm_cartesian_plan(object_pose = [0.7 , -0.3 , 0.2])
    pick_trajectory.append(plan)


    ## close gripper

    #planning.attach_object(object = box)
    #planning.gripper_control(value = 0.7)
    #input()
    
    #planning.gripper_control(value = 0)
    #planning.hold_hand(object = 'box')

    ## up

    p, plan = planning.arm_cartesian_plan(object_pose = [0.75 , -0.3 , 0.4])
    pick_up_trajectory.append(plan)
    # input()


    # save pick trajectory

    pick_trajectory = np.array(pick_trajectory , dtype=object)

    # np.savez("/home/joonhyeok/catkin_ws/src/doosan-robot/doosan-robot/dsr_example/py/scripts/PBL/sampled_trajectories/pick_trajectory/pick_trajectory.npz" , plan = pick_trajectory)
    print("complete to save pick trajectory")

    # save pick_up trajectory

    pick_up_trajectory = np.array(pick_up_trajectory , dtype=object)  

    # np.savez("/home/joonhyeok/catkin_ws/src/doosan-robot/doosan-robot/dsr_example/py/scripts/PBL/sampled_trajectories/pick_up_trajectory/pick_up_trajectory.npz" , plan = pick_up_trajectory)
    print("complete to save pick_up trajectory")







    ## Random points

    ################################# making trajectories with random points #####################################
    ################################# initial pose position = [0.516021273967, 0.0390030718983, 0.630885729686] ###################################
    #################### initial pose orientation = [-4.88940814326e-05, -0.999999963883, 0.000231584538729, 0.00012732903351] ###################################   

    mid_point = [0.516021273967, 0.0390030718983, 0.630885729686]

    for i in range(1, 5001):

        mid_trajectory = list()
        feature_trajectory = list()

        random_pose = copy.deepcopy(mid_point)
        random_pose[0] += np.random.uniform(-0.2,0.2)
        random_pose[1] += np.random.uniform(-0.4,-0.2)
        random_pose[2] += np.random.uniform(-0.4,0.3)

        p, plan = planning.arm_cartesian_plan(object_pose = random_pose, approach_direction = 'vertical')
        mid_trajectory.append(plan)
        feature_trajectory = plan
        # input()


        random_pose = copy.deepcopy(mid_point)
        random_pose[0] += np.random.uniform(-0.3,0.5)
        random_pose[1] += np.random.uniform(-0.2,0.2)
        random_pose[2] += np.random.uniform(-0.4,0.3)

        p, plan = planning.arm_cartesian_plan(object_pose = mid_point , approach_direction = 'vertical')
        mid_trajectory.append(plan)
        feature_trajectory = feature_trajectory + plan
        # input()


        random_pose = copy.deepcopy(mid_point)
        random_pose[0] += np.random.uniform(-0.2,0.2)
        random_pose[1] += np.random.uniform(0.2,0.4)
        random_pose[2] += np.random.uniform(-0.4,0.3)

        p, plan = planning.arm_cartesian_plan(object_pose = random_pose, approach_direction = 'vertical')
        mid_trajectory.append(plan)
        feature_trajectory = feature_trajectory + plan

        # input()


        p, plan = planning.arm_cartesian_plan(object_pose = [0.75 , 0.3 , 0.4])
        mid_trajectory.append(plan)
        feature_trajectory = feature_trajectory + plan

        # input()


        # save mid_trajectory

        mid_trajectory = np.array(mid_trajectory , dtype=object)

        np.savez("/home/joonhyeok/catkin_ws/src/doosan-robot/doosan-robot/dsr_example/py/scripts/PBL/sampled_trajectories/mid_trajectory/mid_trajectory_{num}.npz".format(num = i), plan = mid_trajectory)
        print("complete to save mid trajectory_{num}.npz".format(num = i))

        # save feature_trajectory

        feature_trajectory = np.array(feature_trajectory , dtype=object)

        np.savez("/home/joonhyeok/catkin_ws/src/doosan-robot/doosan-robot/dsr_example/py/scripts/PBL/sampled_trajectories/feature_trajectory/feature_trajectory_{num}.npz".format(num = i), plan = feature_trajectory)
        print("complete to save feature trajectory_{num}.npz".format(num = i))
        # arm should start back at pick_up_position

        planning.arm_cartesian_plan(object_pose = [0.75 , -0.3 , 0.4])





    ## place

    p, plan = planning.arm_cartesian_plan(object_pose = [0.7, 0.3, 0.3])
    place_trajectory.append(plan)
    # input()

    p, plan = planning.arm_cartesian_plan(object_pose = [0.7, 0.3, 0.2])
    place_trajectory.append(plan)
    # input()

    pose = list_to_pose([0.7, 0.3, -0.055, 0, 0, 0, 1])
    box.primitive_poses = [pose]

    
    ## open gripper

    #planning.detach_object(object = box)
    #planning.gripper_control(value = 0)

    #planning.release_hand(object = 'box')
    #planning.gripper_control(value = 0.7)

    
    ## up

    p, plan = planning.arm_cartesian_plan(object_pose = [0.75 , 0.3 , 0.4])
    place_up_trajectory.append(plan)
    # input()


    # save place trajectory

    place_trajectory = np.array(place_trajectory , dtype=object)
    
    # np.savez("/home/joonhyeok/catkin_ws/src/doosan-robot/doosan-robot/dsr_example/py/scripts/PBL/sampled_trajectories/place_trajectory/place_trajectory.npz" , plan = place_trajectory)
    print("complete to save place trajectory")


    # save place_up trajectory

    place_up_trajectory = np.array(place_up_trajectory , dtype=object)

    # np.savez("/home/joonhyeok/catkin_ws/src/doosan-robot/doosan-robot/dsr_example/py/scripts/PBL/sampled_trajectories/place_up_trajectory/place_up_trajectory.npz" , plan = place_up_trajectory)
    print("complete to save place_up trajectory")





    
    ## initial_pose

    #planning.initial_joint_pose()
    #raw_input()

    p, plan = planning.arm_cartesian_plan(object_pose = [0.516021273967, 0.0390030718983, 0.630885729686], approach_direction = [-4.88940814326e-05, -0.999999963883, 0.000231584538729, 0.00012732903351])
    back_to_initial_trajectory.append(plan)


    # save back_to_initial trajectory

    back_to_initial_trajectory = np.array(back_to_initial_trajectory , dtype=object)

    # np.savez("/home/joonhyeok/catkin_ws/src/doosan-robot/doosan-robot/dsr_example/py/scripts/PBL/sampled_trajectories/back_to_initial_trajectory/back_to_initial_trajectory.npz" , plan = back_to_initial_trajectory)
    print("complete to save back_to_initial trajectory")




if __name__ == '__main__':
    main()
