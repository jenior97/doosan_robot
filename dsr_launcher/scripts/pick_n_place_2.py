#!/usr/bin/env python

import sys
import rospy
import moveit_commander
from tf.transformations import quaternion_from_euler
import numpy as np
from math import pi
import environment
import control
import a0912_plan
import control_planning_scene

def main():
    
    
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('pick_n_place', anonymous=True)


    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface(synchronous=True)

    arm_move_group = moveit_commander.MoveGroupCommander('arm')
    hand_move_group = moveit_commander.MoveGroupCommander('robotiq_2f')
    cpl = control_planning_scene.control_planning_scene()

    # environment setting

    scene.clear()

    setup = environment.environment()

    alpha = 0.01
    box_x ,box_y, box_z = alpha + .8, -0.45, -0.055

    table = cpl._make_box(name='table', pos=[alpha + 1.025, 0, -0.465], size =(1.2,1.4,0.72))
    object1= cpl._make_box(name='object1', pos=[box_x ,box_y, box_z], size =(0.1,0.1,0.1))
    cpl._update_planning_scene(cpl.get_planning_scene)
        
    #setup.add_box(position_x = alpha + 1.025, position_z = -0.465, size = (1.2,1.4,0.72), box_name = 'table') 
    print("============== Table in the scene ===============")
    #setup.add_box(position_x = box_x, position_y = box_y, position_z = box_z, size = (0.1,0.1,0.1), box_name = 'box')
    print("============== Box in the scene ===============")
    #setup.add_mesh(position_x = alpha + 1.025 , position_z = -0.055 , size = (0.1,0.1,0.1) , mesh_name = 'laptop')
    print("============== Laptop in the scene ===============")
    #setup.add_mesh(position_x = 1.8 , position_z = -0.25 , orientation_x = -0.5, orientation_y = 0.5, orientation_z = 0.5, orientation_w = -0.5, size = (0.5,0.5,0.5) , mesh_name = 'visualhuman')
    print("============== Visualhuman in the scene ===============")

    #setup.add_box(position_x = 1 , size = (1 , -0.3 , 0), box_name = 'box')

    # control setting

    planning = control.control()
    plan = a0912_plan.a0912_plan()

    ## approaching to box

    neutral_joint = np.array([0 , 0 , pi/2 , 0 , pi/2 , 0])

    
    planning.set_joint_state_to_neutral_pose(neutral_pose = neutral_joint)
    planning._update_planning_scene(planning.get_planning_scene)

    
    # +0.2 grasp
    last_position, pose_goal, plan1, _  = plan.pose_plan_path(object_pose = [box_x ,box_y, box_z+0.35])
    planning.set_joint_state_to_neutral_pose(neutral_pose = last_position)
    planning._update_planning_scene(planning.get_planning_scene)
    planning.close_gripper()

    pose_goal.position.z -=0.15
    last_position, plan3, _ =plan.plan_cartesian_path(wpose=pose_goal)
    planning.set_joint_state_to_neutral_pose(neutral_pose = last_position)
    planning._update_planning_scene(planning.get_planning_scene)
    
    input()
    cpl.attach_object(object1)
    cpl._update_planning_scene(cpl.get_planning_scene)
    input()
    
    cpl.detach_object(object1)
    cpl._update_planning_scene(cpl.get_planning_scene)
    
    
    

    #plan = planning.arm_cartesian_plan(object_pose1 = [1 , 0 , 0] , approach_direction = 'vertical')

    #planning.arm_pose_plan(object_pose1 = [1 , 0 , 0] , approach_direction = 'vertical')
    
    



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