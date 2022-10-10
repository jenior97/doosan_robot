#!/usr/bin/env python3

import numpy as np
import os
import sys

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(os.path.dirname(__file__)))))


from simulation_utils import create_env, get_feedback
from algorithms.batch_active_PBL import batch_active_PBL
from algorithms.DPB import DPB

import numpy as np
import copy
##### hyper param ############
##############################
batch_active_params = {
    "method":"greedy",
    "samples_num":1000,
    "pre_greedy_nums":600
}

##############################
DPB_params = {
    "exploration_weight":0.03,
    "discounting_factor":0.92,
    "action_U":1.4,
    "param_U":1,
    "regularized_lambda":0.1,
    "reward_U":1,
    "delta":0.7,
    "c_mu":1/5,
    "k_mu":1/4
}

############################

def change_w_element(true_w):
    
    
    n_w = copy.deepcopy(true_w)
    
    max_id = np.argmax(n_w)
    min_id = np.argmin(n_w)
    
    max_v = n_w[max_id]
    min_v = n_w[min_id]
    
    n_w[max_id] = min_v
    n_w[min_id] = max_v
    
    
    return n_w



class NotDefinedAlgo(Exception):
    def __init__(self):
        super().__init__('it\'s not defined algorithm')
        
        


def define_algo(task, algo_type, args, env_type):
    
    task = task.lower()
    
    if algo_type not in ['DPB', 'batch_active_PBL']:
        raise NotDefinedAlgo
    
    
    simulation_object = create_env(task)
    
    d = simulation_object.num_of_features
    
    true_w = timevarying_true_w(task, d)
    
    if algo_type =="batch_active_PBL":
        batch_active_params["method"] = args["BA_method"]
        algo_params = batch_active_params
        algo = batch_active_PBL(simulation_object, algo_params, env_type)
        
        
    elif algo_type =="DPB":
        DPB_params["exploration_weight"] = args["exploration_weight"]
        DPB_params["delta"] = args["delta"]
        DPB_params['discounting_factor'] = args["discounting_factor"]
        DPB_params['regularized_lambda'] = args["regularized_lambda"]
        
        
        algo_params = DPB_params
        algo = DPB(simulation_object, algo_params, env_type)
        
    
    
    return algo, true_w


def timevarying_true_w(task, features_d):
    

        
    if task == "tosser":
        
        true_w = [np.random.rand(features_d)]
        true_w[0][2] = np.random.uniform(-0.99,-0.9)
        true_w[0][3] = np.random.uniform(-0.99,-0.9)
        true_w[0][0] = 0.3
        true_w[0][1] = 0.2
        true_w[0] = true_w[0]/np.linalg.norm(true_w[0])
        
        target_w = np.random.rand(features_d)
        target_w[3] = np.random.uniform(0.9,0.99)
        target_w[2] = np.random.uniform(0,0.1)
        target_w[0] = 0.3
        target_w[1] = 0.2
        target_w = target_w/np.linalg.norm(target_w)
        
        true_w.append(target_w)
        true_w.append(change_w_element(true_w[1]))
        
        
    elif task == "avoid":
        
            
        true_w = [np.random.rand(features_d)]
        true_w[0][0] = np.random.uniform(-0.99,-0.9)
        true_w[0][1] = np.random.uniform(0,0.1)
        true_w[0][2] = 0.3
        true_w[0][3] = 0.2
        true_w[0] = true_w[0]/np.linalg.norm(true_w[0])
        
        true_w.append(change_w_element(true_w[0]))
        
        target_w = np.random.rand(features_d)
        target_w[0] = np.random.uniform(-0.9,-0.99)
        target_w[1] = np.random.uniform(-0.9,-0.99)
        target_w[2] = 0.3
        target_w[3] = 0.2
        target_w = target_w/np.linalg.norm(target_w)
        
        true_w.append(target_w)
    

        

        
    elif task == "lunarlander":
            
        true_w = [np.random.rand(features_d)]
        true_w[0][0] = np.random.uniform(0.9,0.99)
        true_w[0][1] = np.random.uniform(0,0.1)
        true_w[0][2] = np.random.uniform(0.2 ,0.3)
        true_w[0][3] = np.random.uniform(0.2 ,0.3)
        true_w[0][4] = np.random.uniform(0.2 ,0.3)
        true_w[0][5] = np.random.uniform(0.2 ,0.3)
        
        true_w[0] = true_w[0]/np.linalg.norm(true_w[0])
        
        true_w.append(change_w_element(true_w[0]))
        
        target_w = np.random.rand(features_d)
        target_w[0] = np.random.uniform(0,0.1)
        target_w[1] = np.random.uniform(0,0.1)
        target_w[2] = np.random.uniform(0.2 ,0.3)
        target_w[3] = np.random.uniform(0.2 ,0.3)
        target_w[4] = np.random.uniform(0.2 ,0.3)
        target_w[5] = np.random.uniform(0.2 ,0.3)
        
        target_w = target_w/np.linalg.norm(target_w)
        print(target_w)
        
        true_w.append(target_w)
    
        
    else:
            
        true_w = [np.random.rand(features_d)]
        true_w[0][0] = np.random.uniform(0.9,0.99)
        true_w[0][1] = np.random.uniform(0,0.1)
        true_w[0][2] = 0.3
        true_w[0][3] = 0.2
        true_w[0] = true_w[0]/np.linalg.norm(true_w[0])
        
        true_w.append(change_w_element(true_w[0]))
        
        target_w = np.random.rand(features_d)
        target_w[0] = np.random.uniform(-0.9,-0.99)
        target_w[1] = np.random.uniform(-0.9,-0.99)
        target_w[2] = 0.3
        target_w[3] = 0.2
        target_w = target_w/np.linalg.norm(target_w)
        
        true_w.append(target_w)
    
    
    return true_w
    
    
    