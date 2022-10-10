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
        
        data = np.load('/home/kim/catkin_ws/src/doosan-robot/dsr_launcher/scripts/preference_based_learning/ctrl_samples/' + self.simulation_object.name + '.npz', allow_pickle=True)
        self.PSI = data['psi_set']
        self.inputs_set = data['inputs_set']
        features_data = np.load('/home/kim/catkin_ws/src/doosan-robot/dsr_launcher/scripts/preference_based_learning/ctrl_samples/' + self.simulation_object.name + '_features'+'.npz', allow_pickle=True)
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
    
    
    
    
    
    
    
    