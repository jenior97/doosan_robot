#!/usr/bin/env python

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


    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface(synchronous=True)

    arm_move_group = moveit_commander.MoveGroupCommander('arm')
    hand_move_group = moveit_commander.MoveGroupCommander('robotiq_2f')

    # environment setting

    scene.clear()

    setup = environment.environment()


    alpha = 0.01

    table,_ = setup.add_box(position_x = alpha + 1.025, position_z = -0.465, size = (1.2,1.4,0.72), box_name = 'table') 
    print("============== Table in the scene ===============")
    box,_ = setup.add_box(position_x = alpha + 1.025, position_y = -0.3, position_z = -0.055, size = (0.1,0.1,0.1), box_name = 'box')
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

    plan = planning.initial_joint_pose()
    pick_trajectory = plan
    raw_input()

    ## pick

    plan = planning.arm_cartesian_plan(object_pose = [1.035 , -0.3 , 0.3])
    pick_trajectory = pick_trajectory + plan
    raw_input()

    plan = planning.arm_cartesian_plan(object_pose = [1.035 , -0.3 , 0.1])
    pick_trajectory = pick_trajectory + plan
    raw_input()


    ## close gripper

    planning.attach_object(object = box)
    raw_input()

    #planning.gripper_control(0)

    ## up

    plan = planning.arm_cartesian_plan(object_pose = [0.75 , -0.3 , 0.4])
    pick_up_trajectory = plan
    raw_input()


    # save pick trajectory

    pick_trajectory = np.array(pick_trajectory , dtype=object)
    
    np.savez("/home/kim/catkin_ws/src/doosan-robot/dsr_launcher/scripts/sampled_trajectories/pick_trajectory/pick_trajectory.npz" , plan = pick_trajectory)
    print("complete to save pick trajectory")

    # save pick_up trajectory

    pick_up_trajectory = np.array(pick_up_trajectory , dtype=object)
    
    np.savez("/home/kim/catkin_ws/src/doosan-robot/dsr_launcher/scripts/sampled_trajectories/pick_up_trajectory/pick_up_trajectory.npz" , plan = pick_up_trajectory)
    print("complete to save pick_up trajectory")







    ## Random points

    ################################# making trajectories with random points #####################################
    ################################# initial pose position = [0.516021273967, 0.0390030718983, 0.630885729686] ###################################
    #################### initial pose orientation = [-4.88940814326e-05, -0.999999963883, 0.000231584538729, 0.00012732903351] ###################################   

    mid_point = [0.516021273967, 0.0390030718983, 0.630885729686]

    for i in range(1, 11):

        mid_trajectory = list()

        random_pose = copy.deepcopy(mid_point)
        random_pose[0] += np.random.uniform(-0.3,0.5)
        random_pose[1] += np.random.uniform(-0.3,-0.1)
        random_pose[2] += np.random.uniform(-0.3,0.3)

        plan = planning.arm_cartesian_plan(object_pose = random_pose, approach_direction = 'vertical')
        mid_trajectory = plan
        raw_input()


        random_pose = copy.deepcopy(mid_point)
        random_pose[0] += np.random.uniform(-0.1,0.1)
        random_pose[1] += np.random.uniform(-0.1,0.1)
        random_pose[2] += np.random.uniform(-0.1,0.1)

        plan = planning.arm_cartesian_plan(object_pose = mid_point , approach_direction = 'vertical')
        mid_trajectory = mid_trajectory + plan
        raw_input()


        random_pose = copy.deepcopy(mid_point)
        random_pose[0] += np.random.uniform(-0.3,0.5)
        random_pose[1] += np.random.uniform(0.1,0.3)
        random_pose[2] += np.random.uniform(-0.3,0.3)

        plan = planning.arm_cartesian_plan(object_pose = random_pose, approach_direction = 'vertical')
        mid_trajectory = mid_trajectory + plan
        raw_input()


        plan = planning.arm_cartesian_plan(object_pose = [0.75 , 0.3 , 0.4])
        mid_trajectory = mid_trajectory + plan
        raw_input()


        # save mid_trajectory

        mid_trajectory = np.array(mid_trajectory , dtype=object)
    
        np.savez("/home/kim/catkin_ws/src/doosan-robot/dsr_launcher/scripts/sampled_trajectories/mid_trajectory/mid_trajectory_{num}.npz".format(num = i), plan=mid_trajectory)
        print("complete to save mid trajectory_{num}.npz".format(num = i))


        # arm should start back at pick_up_position

        planning.arm_cartesian_plan(object_pose = [0.75 , -0.3 , 0.4])





    ## place

    plan = planning.arm_cartesian_plan(object_pose = [1.035, 0.3, 0.3])
    place_trajectory = plan
    raw_input()

    plan = planning.arm_cartesian_plan(object_pose = [1.035, 0.3, 0.1])
    place_trajectory = place_trajectory + plan    
    raw_input()

    pose = list_to_pose([1.035, 0.3, -0.055, 0, 0, 0, 1])
    box.primitive_poses = [pose]

    
    ## open gripper

    planning.detach_object(object = box)
    raw_input()

    #planning.gripper_control(0.7)

    
    ## up

    plan = planning.arm_cartesian_plan(object_pose = [0.75 , 0.3 , 0.4])
    place_up_trajectory = plan
    raw_input()


    # save place trajectory

    place_trajectory = np.array(place_trajectory , dtype=object)
    
    np.savez("/home/kim/catkin_ws/src/doosan-robot/dsr_launcher/scripts/sampled_trajectories/place_trajectory/place_trajectory.npz" , plan = place_trajectory)
    print("complete to save place trajectory")


    # save place_up trajectory

    place_up_trajectory = np.array(place_up_trajectory , dtype=object)

    np.savez("/home/kim/catkin_ws/src/doosan-robot/dsr_launcher/scripts/sampled_trajectories/place_up_trajectory/place_up_trajectory.npz" , plan = place_up_trajectory)
    print("complete to save place_up trajectory")





    
    ## initial_pose

    #planning.initial_joint_pose()
    #raw_input()

    plan = planning.arm_cartesian_plan(object_pose = [0.516021273967, 0.0390030718983, 0.630885729686], approach_direction = [-4.88940814326e-05, -0.999999963883, 0.000231584538729, 0.00012732903351])
    back_to_initial_trajectory = plan


    # save back_to_initial trajectory

    back_to_initial_trajectory = np.array(back_to_initial_trajectory , dtype=object)
    
    np.savez("/home/kim/catkin_ws/src/doosan-robot/dsr_launcher/scripts/sampled_trajectories/back_to_initial_trajectory/back_to_initial_trajectory.npz" , plan = back_to_initial_trajectory)
    print("complete to save back_to_initial trajectory")




if __name__ == '__main__':
    main()