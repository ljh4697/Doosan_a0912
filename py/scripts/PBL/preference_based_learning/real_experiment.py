#!/usr/bin/env python3


import numpy as np
from simulation_utils2 import get_feedback
import argparse

from run_algo.algo_utils import define_algo
from run_algo.evaluation_metrics import cosine_metric, simple_regret, regret


 
if __name__ == "__main__":

    
    ap = argparse.ArgumentParser()
    ap.add_argument("-a", "--algo", type=str, default="batch_active_PBL",
                    choices=['DPB', 'batch_active_PBL'], help="type of algorithm")
    ap.add_argument('-e', "--num-iteration", type=int, default=10,
                    help="# of iteration")
    ap.add_argument('-t', "--task-env", type=str, default="avoid",
                    help="type of simulation environment")
    ap.add_argument('-b', "--num-batch", type=int, default=10,
                    help="# of batch")
    ap.add_argument('-s' ,'--seed',  type=int, default=1, help='A random seed')
    ap.add_argument('-w' ,'--exploration-weight',  type=float, default=0.03, help='DPB hyperparameter exploration weight')
    ap.add_argument('-g' ,'--discounting-factor',  type=float, default=0.92, help='DPB hyperparameter discounting factor')
    ap.add_argument('-d' ,'--delta',  type=float, default=0.7, help='DPB hyperparameter delta')
    ap.add_argument('-l' ,'--regularized-lambda',  type=float, default=0.1, help='DPB regularized lambda')
    ap.add_argument('-bm' ,'--BA-method',  type=str, default='greedy', help='method of batch active')
    




    args = vars(ap.parse_args())

    # random seed
    seed = args['seed']
    np.random.seed(seed)
    
    
    algos_type = args['algo']
    b = args['num_batch']
    N = args['num_iteration']
    task = args['task_env']
    
    if N % b != 0:
        print('N must be divisible to b')
        exit(0)
    
    
    algo, true_w = define_algo(task, algos_type, args, 'real')
    
    
    t = 0
    t_th_w = 0
    
    turning_point = 100
    eval_cosine = [0]
    opt_simple_reward = [0]
    eval_simple_regret = [0]
    eval_cumulative_regret = [0]
    
    Q1_label = []
    Q3_label = []
    
    while t < N:
        print('Samples so far: ' + str(t))
        
        
        # time varying true theta
        if t!=0 and t%(turning_point)==0:
            t_th_w+=1
            
            
        algo.update_param(t)
        actions, inputA_set, inputB_set = algo.select_batch_actions(t, b)
        
        # evaluation 
        if t != 0:
            eval_cosine.append(cosine_metric(algo.hat_theta_D, true_w[t_th_w]))
            s_r, opt_reward = simple_regret(algo.predefined_features, algo.hat_theta_D, true_w[t_th_w])
            
            opt_simple_reward.append(opt_reward)
            eval_simple_regret.append(s_r)
            eval_cumulative_regret.append(eval_cumulative_regret[-1] + regret(algo.PSI , np.array(algo.action_s[-1:-b-1:-1]), true_w[t_th_w]))
            
            
        #  human feedback
        for i in range(b):

            A, R, Q1, Q3 = get_feedback(algo, inputA_set[i], inputB_set[i],
                                actions[i], true_w[t_th_w], m="samling", human='real')
            
            algo.action_s.append(A)
            algo.reward_s.append(R)
            
            if algos_type == "DPB":
                algo.compute_w_t(A)
            
            Q1_label.append(Q1)
            Q3_label.append(Q3)
            t+=1
        
    Q2_label = []
    Q2 = input(' Out of optimal trajectories A and B, which trajectory is closest to the recent preference? (7-a trajectory A is better than B, 1-a trajectory B is better than A) ').lower()
    Q2_label.append(int(Q2))
    
    
    # save result
    if algos_type == 'DPB':
        filename = '/home/kim/catkin_ws/src/doosan-robot/dsr_example/py/scripts/PBL/preference_based_learning/results/{}/{}/{}-iter{:d}-{:}-delta{:.2f}-alpha{:.4f}-gamma{:.2f}-lambda{:.4f}-seed{:d}.npy'.format(task, algos_type, task, N, algos_type, args["delta"], args["exploration_weight"], args["discounting_factor"], args["regularized_lambda"], seed)
        labelname = '/home/kim/catkin_ws/src/doosan-robot/dsr_example/py/scripts/PBL/preference_based_learning/results/{}/query_label/{}-iter{:d}-{:}-delta{:.2f}-alpha{:.4f}-gamma{:.2f}-lambda{:.4f}-seed{:d}.npy'.format(task, task, N, algos_type, args["delta"], args["exploration_weight"], args["discounting_factor"], args["regularized_lambda"], seed)
    elif algos_type == "batch_active_PBL":
        filename = '/home/kim/catkin_ws/src/doosan-robot/dsr_example/py/scripts/PBL/preference_based_learning/results/avoid/batch_active_PBL/{}-iter{:d}-{:}-method_{}-seed{:d}.npy'.format(task, N, algos_type, args["BA_method"], seed)
        labelname = '/home/kim/catkin_ws/src/doosan-robot/dsr_example/py/scripts/PBL/preference_based_learning/results/avoid/query_label/{}-iter{:d}-{:}-method_{}-seed{:d}.npy'.format(task, N, algos_type, args["BA_method"], seed)
    
    with open(filename, 'wb') as f:
        np.savez(f,
                eval_cosine=eval_cosine,
                eval_simple_regret=eval_simple_regret,
                opt_simple_reward=opt_simple_reward,
                eval_cumulative_regret=eval_cumulative_regret)

        print('data saved at {}'.format(filename))


    with open(labelname, 'wb') as f :
        np.savez(f,
                Q1_label = Q1_label,
                Q2_label = Q2_label,
                Q3_label = Q3_label)
        
        print('data saved at {}'.format(labelname))

        
        
    

    