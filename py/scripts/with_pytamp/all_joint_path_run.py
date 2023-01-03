#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#from typing import OrderedDict
import numpy as np
import rospy

import numpy as np
import argparse
from dsr_msgs.srv import OperatePytamp,Robotiq2FMove

from pykin.utils import plot_utils as p_utils
from pytamp.benchmark import Benchmark1
from pytamp.search.mcts import MCTS



rospy.init_node('all_joint_path_run_py', anonymous=True)

operate_robot = rospy.ServiceProxy('/operate_robot_joint',OperatePytamp)
# 0 : close , 0.7 : open 
operate_gripper = rospy.ServiceProxy('/robotiq_control_move', Robotiq2FMove)

def test():

    #? python3 benchmark1_test.py --budgets 1 --max_depth 1 --seed 3 --algo bai_ucb
    parser = argparse.ArgumentParser(description='Test Benchmark 1.')
    parser.add_argument('--budgets', metavar='T', type=int, default=300, help='Horizon')
    parser.add_argument('--max_depth', metavar='H', type=int, default=20, help='Max depth')
    parser.add_argument('--seed', metavar='i', type=int, default=1, help='A random seed')
    parser.add_argument('--algo', metavar='alg', type=str, default='uct', choices=['bai_perturb', 'bai_ucb', 'uct', 'random'], help='Sampler Name')
    parser.add_argument('--debug_mode', metavar='debug', type=bool, default=False, help='Debug mode')
    parser.add_argument('--benchmark', metavar='N', type=int, default=1, help='Benchmark Number')
    args = parser.parse_args()

    debug_mode = args.debug_mode
    budgets = args.budgets
    max_depth = args.max_depth
    algo = args.algo
    seed = args.seed
    np.random.seed(seed)

    benchmark1 = Benchmark1(robot_name="doosan", geom="collision", is_pyplot=True, box_num=1)

    c_list = 10**np.linspace(0., 3., 10)
    c_list = [10]
    for idx, c in enumerate(c_list):
        mcts = MCTS(benchmark1.scene_mngr)
        mcts.debug_mode = False
        mcts.only_optimize_1 = False

        # 최대부터
        mcts.budgets = 1
        mcts.max_depth = 14
        mcts.sampling_method = 'bai_perturb' 
        mcts.c = c
        print(c)
        for i in range(mcts.budgets):
            mcts.do_planning(i)

        # subtree = mcts.get_success_subtree(optimizer_level=1)
        # mcts.visualize_tree("MCTS", subtree)
        # best_nodes = mcts.get_best_node(subtree)

        level_1_max_value = mcts.values_for_level_1
        max_iter = np.argmax(level_1_max_value)

        level_1_max_values = mcts.values_for_level_1
        level_2_max_values = mcts.values_for_level_2
        # fig, ax = p_utils.init_2d_figure("test")
        # p_utils.plot_values(
        #     ax,
        #     level_1_max_values, 
        #     label=f"Sum of Values({mcts.sampling_method}, {mcts.budgets}, {mcts.c})", 
        #     title="Benchamrk1_Level_1_" + mcts.sampling_method + "-" + str(mcts.budgets) + "-" + str(mcts.c),
        #     save_dir_name='benchmark1_result', 
        #     is_save=True)
            
        # p_utils.plot_values(
        #     ax,
        #     level_2_max_values, 
        #     label="Optiaml Values", 
        #     title="Benchamrk1_Level_2_" + mcts.sampling_method,  
        #     save_dir_name='benchmark1_result', 
        #     is_save=True)
        # p_utils.show_figure()

        # # Do planning
        # # mcts.get_all_joint_path(mcts.optimal_nodes)
        pnp_all_joint_pathes, pick_all_objects, place_all_object_poses = mcts.get_all_joint_path(mcts.optimal_nodes)
    return pnp_all_joint_pathes

pnp_all_joint_pathes = test()
# for pnp_all_joint_path in pnp_all_joint_pathes:
#     for task, join_path in pnp_all_joint_path.items():
#         if task == "pre_grasp_pose":
#             pre_grasp_joint_path = np.array(pre_grasp_joint_path).reshape(-1).tolist()
#             operate_robot(pre_grasp_joint_path)
#         elif task == "grasp_pose":
#             grasp_joint_path = np.array(grasp_joint_path).reshape(-1).tolist()
#             operate_robot(grasp_joint_path)
#             operate_gripper(0)
#         elif task == "post_grasp_pose":
#             post_joint_path = np.array(post_joint_path).reshape(-1).tolist()
#             operate_robot(post_joint_path)
#             operate_gripper(0)
#         elif task == ""

global_move_time = 6
local_move_time = 3

init_pos = np.array([ 0, 0, np.pi/2, 0, np.pi/2, 0]).reshape(-1).tolist()
operate_robot(init_pos,global_move_time)


for pnp_all_joint_path in pnp_all_joint_pathes:
    for pnp_joint_path in pnp_all_joint_path:
        for task, value in pnp_joint_path.items():
            if task == "pre_grasp":
                pre_grasp_joint_path = np.array(value).reshape(-1).tolist()
                operate_robot(pre_grasp_joint_path,global_move_time)
            elif task == "grasp":
                grasp_joint_path = np.array(value).reshape(-1).tolist()
                operate_robot(grasp_joint_path,local_move_time)
                operate_gripper(0)
            elif task == "post_grasp":
                post_joint_path = np.array(value).reshape(-1).tolist()
                operate_robot(post_joint_path,local_move_time)
            elif task == "default_grasp":
                default_joint_path = np.array(value).reshape(-1).tolist()
                operate_robot(default_joint_path,global_move_time)
            elif task == "pre_release":
                pre_release_joint_path = np.array(value).reshape(-1).tolist()
                operate_robot(pre_release_joint_path,global_move_time)
            elif task == "release":
                release_joint_path = np.array(value).reshape(-1).tolist()
                operate_robot(release_joint_path,local_move_time)
                operate_gripper(0.7)
            elif task == "post_release":
                post_release_joint_path = np.array(value).reshape(-1).tolist()
                operate_robot(post_release_joint_path,local_move_time)
            elif task == "default_release":
                default_release_joint_path = np.array(value).reshape(-1).tolist()
                operate_robot(default_release_joint_path,global_move_time)
            
            
    
