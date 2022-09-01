#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler
import numpy as np
from math import pi






class control(object):

    def __init__(self):
        super(control, self).__init__()


        #moveit_commander.roscpp_initialize(sys.argv)
        #rospy.init_node('control_environment', anonymous=True)


        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface(synchronous=True)
        group_name = "arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

        planning_frame = move_group.get_planning_frame()
        eef_link = move_group.get_end_effector_link()
        group_names = robot.get_group_names()

        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher

        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names


    def pose_plan(self, object_pose, approach_direction = 'vertical'):

        self.move_group.set_planner_id('RRTstar')

        # set start state
        self.move_group.set_start_state_to_current_state()

        # set goal state
        self.pose_goal = geometry_msgs.msg.PoseStamped()
        self.move_group.set_goal_position_tolerance(0.0001)

        # approach direction
        if approach_direction == "horizon":
            q = quaternion_from_euler(pi/2, 0, 0)
        elif approach_direction == "vertical":
            q = quaternion_from_euler(pi, pi/2, -pi/2)

        self.pose_goal.header.frame_id = 'world'

        self.pose_goal.pose.orientation.x = q[0]
        self.pose_goal.pose.orientation.y = q[1]
        self.pose_goal.pose.orientation.z = q[2]
        self.pose_goal.pose.orientation.w = q[3]

        self.pose_goal.pose.position.x = object_pose[0]
        self.pose_goal.pose.position.y = object_pose[1]
        self.pose_goal.pose.position.z = object_pose[2]

        # set target
        self.move_group.set_pose_target(self.pose_goal)

        # plan
        plan = self.move_group.plan()

        self.move_group.stop()
        self.move_group.clear_pose_targets()


        #last_position = np.array(plan.joint_trajectory.points[-1].positions)

        #plan_positions = []
        #for i in range(len(plan.joint_trajectory.points)):
        #    plan_positions.append(plan.joint_trajectory.points[i].positions)




        #return last_position, self.pose_goal, plan_positions, plan




    def display_trajectory(self, plan) :
        
        self.display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        self.display_trajectory.trajectory_start = self.robot.get_current_state()
        self.display_trajectory.trajectory.append(plan)
        self.display_trajectory_publisher.publish(display_trajectory);



    def cartesian_plan(self, object_pose , approach_direction = 'horizon'):
        

        # set start state
        self.move_group.set_start_state_to_current_state()
        
        # set goal state
        self.pose_goal = geometry_msgs.msg.Pose()
        self.move_group.set_goal_position_tolerance(0.0001)

        # approach direction
        if approach_direction == "horizon":
            q = quaternion_from_euler(pi/2, 0, 0)
        elif approach_direction == "vertical":
            q = quaternion_from_euler(pi, pi/2, -pi/2)

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
        waypoints.append(self.move_group.get_current_pose().pose)
        waypoints.append(self.pose_goal)

        # computing cartesian path
        plan, _ = self.move_group.compute_cartesian_path(waypoints, 0.01, 0.0)



    def go(self , object_pose):
        
        self.pose_goal = geometry_msgs.msg.PoseStamped()

        self.pose_goal.header.frame_id = 'world'
        
        self.pose_goal.pose.orientation.w = 1.0
        
        self.pose_goal.pose.position.x = object_pose[0]
        self.pose_goal.pose.position.y = object_pose[1]
        self.pose_goal.pose.position.z = object_pose[2]

        self.move_group.set_goal_position_tolerance(0.0001)
        self.move_group.set_pose_target(self.pose_goal)

        success = self.move_group.go(wait=True)

        self.move_group.stop()
        self.move_group.clear_pose_targets()

        current_pose = self.move_group.get_current_pose().pose




    def approach_pose(self, object_name) :
        
        object_poses = self.scene.get_object_poses(object_name)
        print(object_poses)
        
        
        return object_poses





    #def open_gripper(self) :






    #def close_gripper(self) :
    


    

























def main():
    try:

        planning = control()

        #planning.pose_plan(object_pose = [0.4,0.4,0.4])

        #planning.cartesian_plan(object_pose = [0.4,0.4,0,4])

        #planning.go()


    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return






if __name__ == '__main__':
    main()