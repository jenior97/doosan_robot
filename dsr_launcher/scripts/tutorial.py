#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import os
import threading, time
import sys

sys.dont_write_bytecode = True
sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__),"../../../../common/imp")) ) # get import path : DSR_ROBOT.py 

# Python 2/3 compatibility imports
from six.moves import input


import copy
import moveit_commander
import moveit_msgs.msg
from moveit_msgs.msg import MoveGroupActionResult
import geometry_msgs.msg
import shape_msgs.msg
import numpy as np

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))

# for single robot 
ROBOT_ID     = "dsr01"
ROBOT_MODEL  = "a0912"
#import DR_init
#DR_init.__dsr__id = ROBOT_ID
#DR_init.__dsr__model = ROBOT_MODEL
#from DSR_ROBOT import *

import std_msgs.msg
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list, list_to_pose

def SET_ROBOT(id, model):
    ROBOT_ID = id; ROBOT_MODEL= model   

def shutdown():
    print("shutdown time!")
    print("shutdown time!")
    print("shutdown time!")

    #pub_stop.publish(stop_mode=1) #STOP_TYPE_QUICK)
    return 0

# convert list to Float64MultiArray
def _ros_listToFloat64MultiArray(list_src):
    _res = []
    for i in list_src:
        item = Float64MultiArray()
        item.data = i
        _res.append(item)
    #print(_res)
    #print(len(_res))
    return _res
    
def all_close(goal, actual, tolerance):

    """
    Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    all_equal = True
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

    return True




class PickNPlaceTutorial():
    """PickNPlaceTutorial"""
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        self.scene = moveit_commander.PlanningSceneInterface()
        self.robot_group = moveit_commander.MoveGroupCommander('arm')
        self.hand_group = moveit_commander.MoveGroupCommander('robotiq_2f')

        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                        moveit_msgs.msg.DisplayTrajectory,
                                                        queue_size=20)
        
        self.plan_result_pub = rospy.Publisher("/move_group/result", MoveGroupActionResult, queue_size=1)

        # Misc variables
        self.box_name = ''

        self.place_ = []
        self.object_info = {}
        self.attached_object_name = ""
        self.tray_object_count = 0
    def close_gripper(self,value):
        gripper_joint_goal = self.hand_group.get_current_joint_values()

        print("gripper joint_values : " ,joint_goal)
        joint_goal[0] = 0.7
        self.hand_group.plan(joint_goal, wait=True)

    def hold_hand(self, target_name):
        touch_links = ['left_inner_finger_pad', 'right_inner_finger_pad']
        self.hand_group.attach_object(object_name = target_name,link_name = 'robotiq_arg2f_base_link', touch_links=touch_links)
        self.attached_object_name = target_name

    def release_hand(self, target_name):
        self.hand_group.detach_object(target_name)

    def jmove_to_pose_goal(self, pose_goal):
        # self.robot_group.set_pose_target(pose_goal)
        plan = self.robot_group.plan(pose_goal)
        # print("wait for raw_input")
        # raw_input()
        self.robot_group.execute(plan, wait=True)
        # self.robot_group.go(wait=True)


    def jmove_to_joint_goal(self, joint_goal):
        # self.robot_group.go(joint_goal, wait=True)
        plan = self.robot_group.plan(joint_goal)
        # print("wait for raw_input")
        # raw_input()
        self.robot_group.execute(plan, wait=True)


    def tmove_to_pose_goal(self ,pose_goal):
        waypoints = []

        #waypoints.append(current_pose)
        #waypoints.append(pose_goal)
        wpose = self.robot_group.get_current_pose().pose
        print("current pose : ",wpose)
        print("pose_goal : ",pose_goal)
        wpose.position.z += 0.15
        wpose.orientation = pose_goal.orientation
        print("goal_pose")
        waypoints.append(copy.deepcopy(wpose))

        plan, _ = self.robot_group.compute_cartesian_path(
                                   waypoints,   # waypoints to follow
                                   0.01,        # eef_step
                                   0.0)         # jump_threshold

        msg = MoveGroupActionResult()
        msg.result.planned_trajectory = plan
        self.plan_result_pub.publish(msg)

        # print("wait for raw_input")
        # raw_input()
        self.robot_group.execute(plan, wait=True)


    def wait_for_state_update(self, box_name, box_is_known=False, box_is_attached=False, timeout=4):
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = self.scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0

            # Note that attaching the box will remove it from known_objects
            is_known = box_name in self.scene.get_known_object_names()

            # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            rospy.sleep(0.1)
            seconds = rospy.get_time()
        return False

    def add_box(self, name, pose_stamp, size=(0.05, 0.05, 0.05)):
        self.scene.add_box(name, pose_stamp, size=size)
        self.wait_for_state_update(box_name=name, box_is_known=True)

    def make_tray(self,tray_center_pose,tray_size):
        # Tray is rectangular.
        # tray_center_pose : (x,y,z) 
        # tray_size : (width, length, height)
        tray_wall_upper = geometry_msgs.msg.PoseStamped()
        tray_wall_upper.header.frame_id = self.robot_group.get_planning_frame()
        tray_wall_upper.pose.position.x = tray_center_pose[0] + tray_size[1]/2.0
        tray_wall_upper.pose.position.y = tray_center_pose[1]
        tray_wall_upper.pose.position.z = tray_center_pose[2] + tray_size[2]/2.0
        tray_wall_upper.pose.orientation.w = 1.0      
        self.add_box(name='tray_wall_upper', pose_stamp=tray_wall_upper, size=(0.01, tray_size[0], tray_size[2]))
        self.object_info['tray_wall_upper'] = tray_wall_upper
        
        tray_wall_lower = geometry_msgs.msg.PoseStamped()
        tray_wall_lower.header.frame_id = self.robot_group.get_planning_frame()
        tray_wall_lower.pose.position.x = tray_center_pose[0] - tray_size[1]/2.0
        tray_wall_lower.pose.position.y = tray_center_pose[1]
        tray_wall_lower.pose.position.z = tray_center_pose[2] + tray_size[2]/2.0
        tray_wall_lower.pose.orientation.w = 1.0      
        self.add_box(name='tray_wall_lower', pose_stamp=tray_wall_lower, size=(0.01, tray_size[0], tray_size[2]))
        self.object_info['tray_wall_lower'] = tray_wall_lower

        tray_wall_left = geometry_msgs.msg.PoseStamped()
        tray_wall_left.header.frame_id = self.robot_group.get_planning_frame()
        tray_wall_left.pose.position.x = tray_center_pose[0] 
        tray_wall_left.pose.position.y = tray_center_pose[1] + tray_size[0]/2.0
        tray_wall_left.pose.position.z = tray_center_pose[2] + tray_size[2]/2.0
        tray_wall_left.pose.orientation.w = 1.0      
        self.add_box(name='tray_wall_left', pose_stamp=tray_wall_left, size=(tray_size[1], 0.01, tray_size[2]))
        self.object_info['tray_wall_left'] = tray_wall_left
       
        tray_wall_right = geometry_msgs.msg.PoseStamped()
        tray_wall_right.header.frame_id = self.robot_group.get_planning_frame()
        tray_wall_right.pose.position.x = tray_center_pose[0] 
        tray_wall_right.pose.position.y = tray_center_pose[1] - tray_size[0]/2.0
        tray_wall_right.pose.position.z = tray_center_pose[2] + tray_size[2]/2.0
        tray_wall_right.pose.orientation.w = 1.0      
        self.add_box(name='tray_wall_right', pose_stamp=tray_wall_right, size=(tray_size[1], 0.01, tray_size[2]))
        self.object_info['tray_wall_right'] = tray_wall_right
        
        tray_center = geometry_msgs.msg.PoseStamped()
        tray_center.header.frame_id = self.robot_group.get_planning_frame()
        tray_center.pose.position.x =tray_center_pose[0]
        tray_center.pose.position.y =tray_center_pose[1]
        tray_center.pose.position.z =tray_center_pose[2]
        tray_center.pose.orientation.w = 1.0  

        self.object_info['tray_center'] = tray_center

    def add_objects(self):
        table0 = geometry_msgs.msg.PoseStamped()
        table0.header.frame_id = self.robot_group.get_planning_frame()
        table0.pose.position.x =0.0
        table0.pose.position.y =0.0
        table0.pose.position.z =-0.25
        table0.pose.orientation.w = 1.0      
        self.add_box(name='table0', pose_stamp=table0, size=(0.4, 0.8, 0.4))
        self.object_info['table0'] = table0
    


        table = geometry_msgs.msg.PoseStamped()
        table.header.frame_id = self.robot_group.get_planning_frame()
        table.pose.position.x =0.7
        table.pose.position.y =0
        table.pose.position.z =-0.10
        table.pose.orientation.w = 1.0      
        self.add_box(name='table', pose_stamp=table, size=(0.7, 1.5, 0.1))
        self.object_info['table'] = table
    
        wall2 = geometry_msgs.msg.PoseStamped()
        wall2.header.frame_id = self.robot_group.get_planning_frame()
        wall2.pose.position.x =0.0
        wall2.pose.position.y =1.0
        wall2.pose.position.z =-0.15
        wall2.pose.orientation.w = 1.0                            

        self.add_box(name='wall2', pose_stamp=wall2, size=(2, 0.1, 2))
        self.object_info['wall2'] = wall2

        cube_x = 0.05
        cube_y = 0.1
        cube_0 = geometry_msgs.msg.PoseStamped()
        cube_0.header.frame_id = self.robot_group.get_planning_frame()
        cube_0.pose.position.x =0.55 + cube_x
        cube_0.pose.position.y =0.1 + cube_y
        cube_0.pose.position.z =-0.01
        cube_0.pose.orientation.w = 1.0                            

        self.add_box(name='cube_0', pose_stamp=cube_0, size=(0.1, 0.1, 0.1))
        self.object_info['cube_0'] = cube_0

        cube_1 = geometry_msgs.msg.PoseStamped()
        cube_1.header.frame_id = self.robot_group.get_planning_frame()
        cube_1.pose.position.x =0.55+ cube_x
        cube_1.pose.position.y =0.30+ cube_y
        cube_1.pose.position.z =-0.01
        cube_1.pose.orientation.w = 1.0     

        self.add_box(name='cube_1', pose_stamp=cube_1, size=(0.1, 0.1, 0.1))
        self.object_info['cube_1'] = cube_1

        cube_2 = geometry_msgs.msg.PoseStamped()
        cube_2.header.frame_id = self.robot_group.get_planning_frame()
        cube_2.pose.position.x =0.75+ cube_x
        cube_2.pose.position.y =0.10+ cube_y
        cube_2.pose.position.z =-0.01
        cube_2.pose.orientation.w = 1.0     

        self.add_box(name='cube_2', pose_stamp=cube_2, size=(0.1, 0.1, 0.1))
        self.object_info['cube_2'] = cube_2

        cube_3 = geometry_msgs.msg.PoseStamped()
        cube_3.header.frame_id = self.robot_group.get_planning_frame()
        cube_3.pose.position.x =0.75+ cube_x
        cube_3.pose.position.y =0.30+ cube_y
        cube_3.pose.position.z =-0.01
        cube_3.pose.orientation.w = 1.0     

        self.add_box(name='cube_3', pose_stamp=cube_3, size=(0.1, 0.1, 0.1))
        self.object_info['cube_3'] = cube_3


        tray_center = [0.55, -0.2, -0.05]
        tray_size = [0.3, 0.3, 0.05]
        self.make_tray(tray_center,tray_size)



    def pick(self,object_name):
        # pose : position to go 
        # objecto_name : object_name for pick
        # ex) pose : list_to_pose([0.55, -0.10, 0.5, 0, -pi, -pi/2])
        pose = list_to_pose([0, 0, 0, 0, -pi, 0])

        object_pose = self.object_info[object_name]

        pre_pose = copy.deepcopy(pose)
        pre_pose.position.x = object_pose.pose.position.x
        pre_pose.position.y = object_pose.pose.position.y
        pre_pose.position.z = object_pose.pose.position.z + 0.30
        self.jmove_to_pose_goal(pre_pose)
        print("wait for move to pre_position")
        raw_input()

        pick_pose = copy.deepcopy(pre_pose)
        pick_pose.position.z -= 0.1
        self.jmove_to_pose_goal(pick_pose)
        print("wait for move to pick_position")
        raw_input()

        gripper_control(0.7)
        self.hold_hand(object_name)

        
        print("wait for close gripper")
        raw_input()

        post_pose = copy.deepcopy(pick_pose)
        post_pose.position.z += 0.15
        self.tmove_to_pose_goal(post_pose)

        print("wait for move to post_position")
        raw_input()

    def place(self,pose):

        self.jmove_to_pose_goal(pose)
        print("wait for move to pre_position")
        raw_input()

        place_pose = copy.deepcopy(pose)
        place_pose.position.z -= 0.1
        self.jmove_to_pose_goal(place_pose)
        print("wait for move to place_position")
        raw_input()

        self.release_hand(self.attached_object_name)
        gripper_control(0.0)
        print("detached object")
        raw_input() 

        post_pose = copy.deepcopy(place_pose)
        post_pose.position.z += 0.15
        self.tmove_to_pose_goal(post_pose)
        print("wait for move to post_position")
        raw_input()
        if (self.object_info['tray_wall_upper'].pose.position.x > pose.position.x) and (self.object_info['tray_wall_lower'].pose.position.x < pose.position.x) and (self.object_info['tray_wall_left'].pose.position.y > pose.position.y) and (self.object_info['tray_wall_right'].pose.position.y < pose.position.y):
            self.tray_object_count += 1
        print("tray_object count : ", self.tray_object_count)

    def pnp(self,object_name):
        
        print("pick ",object_name)
        raw_input()

        self.pick(object_name)
        print("Let's place")
        raw_input()

        place_pose =list_to_pose([self.object_info['tray_center'].pose.position.x, self.object_info['tray_center'].pose.position.y, 0.25 + self.tray_object_count*0.1, 0, -pi, pi/2])
        self.place(place_pose)

def gripper_control(value):
    # 0.7 : close / 0 : open 
    #srv_robotiq_2f_move(value)
    srv_robotiq_gripper_move(0.7-value)


def main():
    try:
        pnp = PickNPlaceTutorial()

        gripper_control(0.0)
        srv_robotiq_2f_move(0.0)

        print("tutorial start & add object")
        raw_input()
        pnp.scene.remove_world_object("table")
        pnp.scene.remove_world_object("part")
        pnp.scene.remove_world_object("wall")
        pnp.scene.remove_world_object("wall2")
        pnp.scene.remove_world_object("cube_1")
        pnp.scene.remove_world_object("cube_0")
        pnp.scene.remove_world_object("cube_2")
        pnp.scene.remove_world_object("cube_3")

        pnp.add_objects()
        

        print("move_robot")
        raw_input()

        pnp.jmove_to_joint_goal([0, 0, pi/2, 0, pi/2, 0])
        print("wait for move")
        raw_input()


        pnp.pnp('cube_0')
        pnp.pnp('cube_1')
        pnp.pnp('cube_2')
        pnp.pnp('cube_3')

        """
        ## pick object function test        
        pnp.pick( "cube_0")
        print("I attached cube_0 and Let's move place_space")
        raw_input()
        place_pose =list_to_pose([pnp.object_info['tray_center'].pose.position.x, pnp.object_info['tray_center'].pose.position.y, 0.4 + pnp.tray_object_count*0.1, 0, -pi, 0])
        pnp.place(place_pose)
        print("pick cube_1")
        raw_input()
        pnp.pick("cube_1")
        print("Let's place")
        raw_input()
        place_pose =list_to_pose([pnp.object_info['tray_center'].pose.position.x, pnp.object_info['tray_center'].pose.position.y, 0.4 + pnp.tray_object_count*0.1, 0, -pi, 0])
        pnp.place(place_pose)
        print("pick cube_2")
        raw_input()
        pnp.pick("cube_2")
        print("Let's place")
        raw_input()
        place_pose =list_to_pose([pnp.object_info['tray_center'].pose.position.x, pnp.object_info['tray_center'].pose.position.y, 0.4 + pnp.tray_object_count*0.1, 0, -pi, 0])
        pnp.place(place_pose)
        """

        print("Finish!!")
        raw_input()         
        
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return
    else:
        print("============ Python pick place demo complete!")


if __name__ == "__main__":
    #----- set target robot --------------- 
    my_robot_id    = "dsr01"
    my_robot_model = "a0912"
    SET_ROBOT(my_robot_id, my_robot_model)
    rospy.init_node('benchmark1_tutorial_py', anonymous=True)

    rospy.on_shutdown(shutdown)

    #pub_stop = rospy.Publisher('/'+ROBOT_ID +ROBOT_MODEL+'/stop', RobotStop, queue_size=10)           
    
    srv_robotiq_2f_move = rospy.ServiceProxy('/' + ROBOT_ID + ROBOT_MODEL + '/gripper/robotiq_2f_move', Robotiq2FMove)
    srv_robotiq_gripper_move = rospy.ServiceProxy('/robotiq_control_move', Robotiq2FMove)
    
    main()
    print('good bye!')