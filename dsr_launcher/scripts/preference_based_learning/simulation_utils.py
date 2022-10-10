#!/usr/bin/env python2.7
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

planning = control.control()

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(os.path.dirname(__file__)))))


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


            rospy.init_node("move_group_python_test", anonymous=True)

            robot = moveit_commander.RobotCommander()
            hand_move_group = moveit_commander.MoveGroupCommander("robotiq_2f")

            s = 0

            while s==0:

                selection = input('A/B to watch, 1/2 to vote: ').lower()


                if selection == 'a':
        
                    planning.initial_joint_pose()
                    raw_input()

                    planning.arm_cartesian_plan(object_pose = [0.7 , -0.3 , 0.3])
                    raw_input()

                    planning.arm_cartesian_plan(object_pose = [0.7 , -0.3 , 0.2])
                    raw_input()

                    planning.gripper_control(value = 0.7)
                    raw_input()

                    planning.arm_cartesian_plan(object_pose = [0.7 , -0.3 , 0.4])
                    raw_input()

                    mid_trajectory_data = np.load("/home/kim/catkin_ws/src/doosan-robot/dsr_launcher/scripts/sampled_trajectories/mid_trajectory/mid_trajectory_{num}.npz".format(num = idx[0]), allow_pickle=True)['plan']    

                    for j in range(len(mid_trajectory_data)):
                        planning.arm_cartesian_plan(object_pose = mid_trajectory_data[j])

                    planning.arm_cartesian_plan(object_pose = [0.7, 0.3, 0.3])
                    raw_input()
            
                    planning.arm_cartesian_plan(object_pose = [0.7, 0.3, 0.2])
                    raw_input()

                    planning.gripper_control(value = 0)
                    raw_input()

                    planning.arm_cartesian_plan(object_pose = [0.7 , 0.3 , 0.4])
                    raw_input()

                    planning.initial_joint_pose()
                    raw_input()


                elif selection == 'b':

                    planning.initial_joint_pose()
                    raw_input()

                    planning.arm_cartesian_plan(object_pose = [0.7 , -0.3 , 0.3])
                    raw_input()

                    planning.arm_cartesian_plan(object_pose = [0.7 , -0.3 , 0.2])
                    raw_input()

                    planning.gripper_control(value = 0.7)
                    raw_input()

                    planning.arm_cartesian_plan(object_pose = [0.7 , -0.3 , 0.4])
                    raw_input()

                    mid_trajectory_data = np.load("/home/kim/catkin_ws/src/doosan-robot/dsr_launcher/scripts/sampled_trajectories/mid_trajectory/mid_trajectory_{num}.npz".format(num = idx[1]), allow_pickle=True)['plan']    

                    for j in range(len(mid_trajectory_data)):
                        planning.arm_cartesian_plan(object_pose = mid_trajectory_data[j])

                    planning.arm_cartesian_plan(object_pose = [0.7, 0.3, 0.3])
                    raw_input()
            
                    planning.arm_cartesian_plan(object_pose = [0.7, 0.3, 0.2])
                    raw_input()

                    planning.gripper_control(value = 0)
                    raw_input()

                    planning.arm_cartesian_plan(object_pose = [0.7 , 0.3 , 0.4])
                    raw_input()

                    planning.initial_joint_pose()
                    raw_input()

                elif selection == '1':
                    s = 1
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
