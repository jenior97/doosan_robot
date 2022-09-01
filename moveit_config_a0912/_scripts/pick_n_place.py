#!/usr/bin/env python

import sys
import rospy
import moveit_commander
from tf.transformations import quaternion_from_euler
import numpy as np
from math import pi
import environment
import control



def main():
    
    
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('pick_n_place', anonymous=True)


    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface(synchronous=True)
    group_name = "arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    # environment setting

    scene.clear()

    setup = environment.environment()

    alpha = 0.01

    setup.add_box(position_x = alpha + 1.025, position_z = -0.465, size = (1.2,1.4,0.72), box_name = 'table') 
    print("============== Table in the scene ===============")
    setup.add_box(position_x = alpha + 1.025, position_y = -0.3, position_z = -0.1, size = (0.1,0.1,0.1), box_name = 'box')
    print("============== Box in the scene ===============")
    setup.add_mesh(position_x = alpha + 1.025 , position_z = -0.055 , size = (0.1,0.1,0.1) , mesh_name = 'laptop')
    print("============== Laptop in the scene ===============")
    setup.add_mesh(position_x = 1.8 , position_z = -0.25 , orientation_x = -0.5, orientation_y = 0.5, orientation_z = 0.5, orientation_w = -0.5, size = (0.5,0.5,0.5) , mesh_name = 'visualhuman')
    print("============== Visualhuman in the scene ===============")



    # control setting

    planning = control.control()

    ## approaching to box
    planning.cartesian_plan(object_pose = [0.5 , 0 , -1] , approach_direction = 'horizon')





    #box_pose = planning.approach_pose('box')
    #laptop_pose = planning.approach_pose('laptop')
    #print(box_pose)
    
    #approach w/ cartesian path?
    #close gripper
    #pick
    #generate trajectory
    #place
    #open gripper
    #back to original state








if __name__ == '__main__':
    main()