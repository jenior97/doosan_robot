#!/usr/bin/env python


import numpy as np
import sys
import os
import rospy
import moveit_commander
import moveit_msgs.msg
import trajectory_msgs.msg

sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))

import control
import environment



def main():
    pass



def get_user_feedback(psi, idx):

    setup = environment.environment()
    planning = control.control()

    #table, box, laptop, visualhuman = setup.object_co()

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("move_group_python_test", anonymous=True)

    robot = moveit_commander.RobotCommander()
    arm_move_group = moveit_commander.MoveGroupCommander("arm")
    hand_move_group = moveit_commander.MoveGroupCommander("robotiq_2f")


    

    s = 0


    while s == 0:



        selection = input('A/B to watch, 1/2 to vote: ').lower()

        robot_state = moveit_msgs.msg.RobotTrajectory()

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
                a = trajectory_msgs.msg.JointTrajectoryPoint()
                a.positions = [mid_trajectory_data[j]]
                robot_state.joint_trajectory = a
                arm_move_group.execute(a, wait = True)

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

            p,a = planning.arm_cartesian_plan(object_pose = [0.7 , -0.3 , 0.3])
            print(p)
            raw_input()

            planning.arm_cartesian_plan(object_pose = [0.7 , -0.3 , 0.2])
            raw_input()

            planning.gripper_control(value = 0.7)
            raw_input()

            planning.arm_cartesian_plan(object_pose = [0.7 , -0.3 , 0.4])
            raw_input()

            mid_trajectory_data = np.load("/home/kim/catkin_ws/src/doosan-robot/dsr_launcher/scripts/sampled_trajectories/mid_trajectory/mid_trajectory_{num}.npz".format(num = idx[1]), allow_pickle=True)['plan']    
            for j in range(len(mid_trajectory_data)):
                arm_move_group.set_joint_value_target(mid_trajectory_data[j])
                arm_move_group.execute(plan, wait = True)


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




    return psi, s








if __name__ == "__main__":
    print(get_user_feedback("dd", (0, 1)))



