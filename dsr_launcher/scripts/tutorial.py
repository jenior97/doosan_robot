#!/usr/bin/env python

import sys, os
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler
import numpy as np
from math import pi
from moveit_commander.conversions import pose_to_list, list_to_pose
import copy

from moveit_msgs.srv import GetPlanningScene, GetPlanningSceneRequest
from moveit_msgs.srv import ApplyPlanningScene, ApplyPlanningSceneRequest
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.msg import AttachedCollisionObject
from shape_msgs.msg import SolidPrimitive, Plane, Mesh, MeshTriangle

from moveit_msgs.msg import RobotState
from dsr_msgs.srv import Robotiq2FMove,SerialSendData

import control
import feature
import environment

sys.dont_write_bytecode = True
sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__),"../../common/imp")) ) # get import path : DSR_ROBOT.py 

ROBOT_ID     = "dsr01"
ROBOT_MODEL  = "a0912"

import DR_init

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

from DSR_ROBOT import *

def SET_ROBOT(id, model):
    ROBOT_ID = id; ROBOT_MODEL= model   



def main():


    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('tutorial', anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface(synchronous=True)

    arm_move_group = moveit_commander.MoveGroupCommander('arm')
    hand_move_group = moveit_commander.MoveGroupCommander('robotiq_2f')

    planning = control.control()
    get_feature_map = feature.feature()
    

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




    # pick_n_place

    for i in range(1,201):

        # pick    

        pick_planning_trajectory = np.load("/home/kim/catkin_ws/src/doosan-robot/dsr_launcher/scripts/sampled_trajectories/pick_trajectory/pick_trajectory.npz", allow_pickle=True)['plan']

        for j in range(len(pick_planning_trajectory)) :
            planning.joint_move(pick_planning_trajectory[j])
        
        raw_input()


        # close gripper

        #planning.attach_object(object = box)
        #planning.gripper_control(value = 0)

        #planning.gripper_control(value = 0)
        #planning.hold_hand(object = 'box')

        #raw_input()


        # pick_up

        pick_up_planning_trajectory = np.load("/home/kim/catkin_ws/src/doosan-robot/dsr_launcher/scripts/sampled_trajectories/pick_up_trajectory/pick_up_trajectory.npz", allow_pickle=True)['plan']

        for j in range(len(pick_up_planning_trajectory)) :
            planning.joint_move(pick_up_planning_trajectory[j])

        raw_input()


        # trajectory

        mid_planning_trajectory = np.load("/home/kim/catkin_ws/src/doosan-robot/dsr_launcher/scripts/sampled_trajectories/mid_trajectory/mid_trajectory_{num}.npz".format(num = i), allow_pickle=True)['plan']

        for j in range(len(mid_planning_trajectory)) :
            planning.joint_move(mid_planning_trajectory[j])

        raw_input()


        # place

        place_planning_trajectory = np.load("/home/kim/catkin_ws/src/doosan-robot/dsr_launcher/scripts/sampled_trajectories/place_trajectory/place_trajectory.npz", allow_pickle=True)['plan']

        for j in range(len(place_planning_trajectory)) :
            planning.joint_move(place_planning_trajectory[j])
        
        raw_input()


        # open gripper

        #planning.detach_object(object = box)
        #planning.gripper_control(value = 0.7)

        #planning.release_hand(object = 'box')
        #planning.gripper_control(value = 0.7)

        #raw_input()


        # place_up

        place_up_planning_trajectory = np.load("/home/kim/catkin_ws/src/doosan-robot/dsr_launcher/scripts/sampled_trajectories/place_up_trajectory/place_up_trajectory.npz", allow_pickle=True)['plan']

        for j in range(len(place_up_planning_trajectory)) :
            planning.joint_move(place_up_planning_trajectory[j])

        raw_input()


        # back_to_initial_state

        back_to_initial_planning_trajectory = np.load("/home/kim/catkin_ws/src/doosan-robot/dsr_launcher/scripts/sampled_trajectories/back_to_initial_trajectory/back_to_initial_trajectory.npz", allow_pickle=True)['plan']

        for j in range(len(back_to_initial_planning_trajectory)) :
            planning.joint_move(back_to_initial_planning_trajectory[j])
        
        raw_input()




if __name__ == '__main__':
    main()