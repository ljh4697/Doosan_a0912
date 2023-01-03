#!/usr/bin/env python3


import numpy as np
import os
import sys

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(os.path.dirname(__file__)))))



class PBL_model(object):
    def __init__(self, simulation_object, env='simulated'):
        
        self.simulation_object = simulation_object
        self.d = simulation_object.num_of_features

        ''' predefined data#####################################################'''
        
        data = np.load('/home/joonhyeok/catkin_ws/src/doosan-robot/doosan-robot/dsr_example/py/scripts/PBL/sampled_trajectories/psi_set.npz', allow_pickle=True)
        self.PSI = data['PSI_SET']
        # self.inputs_set = data['inputs_set']
        self.inputs_set = np.load("/home/joonhyeok/catkin_ws/src/doosan-robot/doosan-robot/dsr_example/py/scripts/PBL/preference_based_learning/ctrl_samples/avoid.npz" , allow_pickle = True)['inputs_set'][0:200]
        features_data = np.load('/home/joonhyeok/catkin_ws/src/doosan-robot/doosan-robot/dsr_example/py/scripts/PBL/sampled_trajectories/avoid_features.npz', allow_pickle=True)
        self.predefined_features = features_data['features']
        
        '''######################################################################'''
        
        self.action_s = []
        self.reward_s = []
        
            

            
            
    
    def update_param(self):
        raise NotImplementedError("must implement udate param method")
    def select_single_action(self):
        raise NotImplementedError("must implement select single action method")
    def select_batch_actions(self):
        raise NotImplementedError("must implement select single action method")
        
            
    def test(self):
        print("hello")
    
    
    
    
    
    
    
    