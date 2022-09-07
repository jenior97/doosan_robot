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



class control(object):

    def __init__(self):
        super(control, self).__init__()


        #moveit_commander.roscpp_initialize(sys.argv)
        #rospy.init_node('control_environment', anonymous=True)


        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface(synchronous=True)
        arm_move_group = moveit_commander.MoveGroupCommander('arm')
        hand_move_group = moveit_commander.MoveGroupCommander('robotiq_2f')

        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

        planning_frame = arm_move_group.get_planning_frame()
        eef_link = arm_move_group.get_end_effector_link()
        group_names = robot.get_group_names()

        self.robot = robot
        self.scene = scene
        self.arm_move_group = arm_move_group
        self.hand_move_group = hand_move_group
        self.display_trajectory_publisher = display_trajectory_publisher

        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names



        self.get_ps_srv = self._get_planning_response()
        self.get_planning_scene = self._get_planning_response_call(self.get_ps_srv).scene
        
        self.apply_ps_srv = rospy.ServiceProxy('apply_planning_scene', ApplyPlanningScene)


        #self.srv_robotiq_2f_move = rospy.ServiceProxy('/dsr01a0912/gripper/robotiq_2f_move', Robotiq2FMove)
        #self.srv_robotiq_gripper_move = rospy.ServiceProxy('/robotiq_control_move', Robotiq2FMove)
    

#############################################################################################

    #def gripper_control(self, value):

        # if value = 0.7 : open / value = 0 : close 
        
        #self.srv_robotiq_2f_move(0.7-value)
        #self.srv_robotiq_gripper_move(0.7-value)



##############################################################################################


    def _get_planning_response(self):

        get_ps_srv = rospy.ServiceProxy('/dsr01a0912/get_planning_scene', GetPlanningScene())
        return get_ps_srv


    def _get_planning_response_call(self, get_ps_srv):

        get_req_ = GetPlanningSceneRequest()
        get_ps_srv.wait_for_service(0.5)
        return get_ps_srv.call(get_req_)


    def set_joint_state_to_neutral_pose(self, neutral_pose=[0]):

        current_position = np.array(self.get_planning_scene.robot_state.joint_state.position)
        current_position[:6] = neutral_pose
        self.get_planning_scene.robot_state.joint_state.position = current_position


    def _update_planning_scene(self, ps = PlanningScene):
        ps.is_diff = True   
        ps.robot_state.is_diff = True

        apply_req = ApplyPlanningSceneRequest()
        apply_req.scene = ps

        self.apply_ps_srv.call(apply_req)

        return


###########################################################################################



    def joint_move(self, plan):

        self.arm_move_group.plan(plan)
        #self.arm_move_group.stop()

        self.set_joint_state_to_neutral_pose(plan)
        self._update_planning_scene(self.get_planning_scene)




    def initial_joint_pose(self):

        #self.arm_move_group.set_start_state_to_current_state()
        plan = self.arm_move_group.plan([0 , 0 , pi/2 , 0 , pi/2 , 0])
        #self.arm_move_group.stop()
        self.set_joint_state_to_neutral_pose([0 , 0 , pi/2 , 0 , pi/2 , 0])
        self._update_planning_scene(self.get_planning_scene)

        self.arm_move_group.execute(plan, wait = True)

        plan_positions = []
        for i in range(len(plan.joint_trajectory.points)):
            plan_positions.append(plan.joint_trajectory.points[i].positions)

        return plan_positions
        


    def arm_pose_plan(self, object_pose, approach_direction = 'vertical'):

        self.arm_move_group.set_planner_id('RRTstar')

        # set start state
        #self.arm_move_group.set_start_state_to_current_state()

        # set goal state
        self.pose_goal = geometry_msgs.msg.PoseStamped()
        self.arm_move_group.set_goal_position_tolerance(0.0001)

        # approach direction
        if approach_direction == "horizon":
            q = quaternion_from_euler(0, pi, -pi/2)
        elif approach_direction == "vertical":
            q = quaternion_from_euler(0, pi, -pi/2)
        else :
            q = approach_direction

        self.pose_goal.header.frame_id = 'world'

        self.pose_goal.pose.orientation.x = q[0]
        self.pose_goal.pose.orientation.y = q[1]
        self.pose_goal.pose.orientation.z = q[2]
        self.pose_goal.pose.orientation.w = q[3]

        self.pose_goal.pose.position.x = object_pose[0]
        self.pose_goal.pose.position.y = object_pose[1]
        self.pose_goal.pose.position.z = object_pose[2]

        # set target
        self.arm_move_group.set_pose_target(self.pose_goal)

        # plan
        plan = self.arm_move_group.plan()

        self.arm_move_group.stop()
        self.arm_move_group.clear_pose_targets()

        current_position = plan.joint_trajectory.points[-1].positions

        self.set_joint_state_to_neutral_pose(current_position)
        self._update_planning_scene(self.get_planning_scene)

        plan_positions = []
        for i in range(len(plan.joint_trajectory.points)):
            plan_positions.append(plan.joint_trajectory.points[i].positions)

        return plan, plan_positions




    def arm_cartesian_plan(self, object_pose , approach_direction = 'vertical'):
        

        # set start state
        #self.arm_move_group.set_start_state_to_current_state()
        
        # set goal state
        self.pose_goal = geometry_msgs.msg.Pose()
        self.arm_move_group.set_goal_position_tolerance(0.0001)

        # approach direction
        if approach_direction == "horizon":
            q = quaternion_from_euler(0, pi, -pi/2)
        elif approach_direction == "vertical":
            q = quaternion_from_euler(0, pi, -pi/2)
        else :
            q = approach_direction

        self.pose_goal.orientation.x = q[0]
        self.pose_goal.orientation.y = q[1]
        self.pose_goal.orientation.z = q[2]
        self.pose_goal.orientation.w = q[3]

        self.pose_goal.position.x = object_pose[0]
        self.pose_goal.position.y = object_pose[1]
        self.pose_goal.position.z = object_pose[2]

        # waypoints
        # waypoints should be in the format of Pose
        waypoints = []
        waypoints.append(self.pose_goal)

        # computing cartesian path
        plan, _ = self.arm_move_group.compute_cartesian_path(waypoints, 0.01, 0.0)
        self.arm_move_group.execute(plan , wait = True)

        current_position = plan.joint_trajectory.points[-1].positions
        
        self.set_joint_state_to_neutral_pose(current_position)
        self._update_planning_scene(self.get_planning_scene)


        plan_positions = []
        for i in range(len(plan.joint_trajectory.points)):
            plan_positions.append(plan.joint_trajectory.points[i].positions)

        return plan_positions




##########################################################################################



    def open_gripper(self):

        gripper_joint_goal = self.hand_move_group.get_current_joint_values()

        gripper_joint_goal[0] = 0
        self.hand_move_group.plan(gripper_joint_goal)

        self._update_planning_scene(self.get_planning_scene)


    def close_gripper(self):

        gripper_joint_goal = self.hand_move_group.get_current_joint_values()

        gripper_joint_goal[0] = 0.7
        self.hand_move_group.plan(gripper_joint_goal)

        self._update_planning_scene(self.get_planning_scene)


    def hold_hand(self, object):

        touch_links = ['left_inner_finger_pad', 'right_inner_finger_pad']
        self.hand_move_group.attach_object(object_name = object,link_name = 'robotiq_arg2f_base_link', touch_links=touch_links)

    def release_hand(self, object):

        self.hand_move_group.detach_object(object)


    def attach_object(self, object):

        eef_link = self.arm_move_group.get_end_effector_link()
        touch_links = self.robot.get_link_names(group = 'robotiq_2f')

        remove_object = CollisionObject()
        remove_object.id = object.id
        remove_object.header.frame_id = object.header.frame_id
        remove_object.operation = remove_object.REMOVE

        aco = AttachedCollisionObject()

        aco.object.id = object.id
        aco.object.header.frame_id = eef_link
        aco.link_name = eef_link
        aco.touch_links = touch_links
        aco.object.operation = aco.object.ADD

        #self.get_planning_scene.robot_state.attached_collision_objects.clear()
        #self.get_planning_scene.world.collision_objects.clear()
        
        self.get_planning_scene.world.collision_objects.append(remove_object)
        self.get_planning_scene.robot_state.attached_collision_objects.append(aco)

        self._update_planning_scene(self.get_planning_scene)


    def detach_object(self, object):

        eef_link = self.arm_move_group.get_end_effector_link()
        touch_links = self.robot.get_link_names(group = 'robotiq_2f')

        detach_object = AttachedCollisionObject()

        detach_object.object.id = object.id
        detach_object.object.header.frame_id = eef_link
        detach_object.link_name = eef_link
        detach_object.touch_links = touch_links
        detach_object.object.operation = detach_object.object.REMOVE

        re_introduce_object = CollisionObject()
        re_introduce_object = object

        #self.get_planning_scene.robot_state.attached_collision_objects.clear()
        #self.get_planning_scene.world.collision_objects.clear()

        self.get_planning_scene.robot_state.attached_collision_objects.append(detach_object)
        self.get_planning_scene.world.collision_objects.append(re_introduce_object)





################################################################################################



def main():
    try:

        planning = control()

        planning.gripper_control(0)

        #plan = planning.arm_cartesian_plan(object_pose = [0.4,0.4,0.4])

        #print(plan.joint_trajectory.points[-1].positions)

        #planning.arm_cartesian_plan(object_pose = [0.4,0.4,0,4])

        #planning.arm_go()

        #planning.close_gripper()
        
        #planning.hold_hand('box')


    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return






if __name__ == '__main__':
    main()