#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from shape_msgs.msg import SolidPrimitive





class environment(object):

    def __init__(self):
        super(environment, self).__init__()


        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('setup_environment', anonymous=True)


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


        self.box_name = ''
        self.mesh_name = ''
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names




    def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):

        start = rospy.get_time()
        seconds = rospy.get_time()

        while (seconds - start < timeout) and not rospy.is_shutdown():
            attached_objects = self.scene.get_attached_objects([self.box_name])
            is_attached = len(attached_objects.keys()) > 0
            is_known = self.box_name in self.scene.get_known_object_names()
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True
            rospy.sleep(0.1)
            seconds = rospy.get_time()

        return False





    def add_box(self, frame = 'world', orientation_x = 0 , orientation_y = 0, orientation_z = 0, orientation_w = 1, position_x = 0, position_y = 0, position_z = 0, box_name = 'box', size = (0.1,0.1,0.1), timeout=4 ):
    
        
        ##### world frame reference x : front, y: right, z : up (same with size)
        ##### position is always in the midpoint of the size

        
        rospy.sleep(2)

        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = frame  

        box_pose.pose.orientation.x = orientation_x
        box_pose.pose.orientation.y = orientation_y
        box_pose.pose.orientation.z = orientation_z
        box_pose.pose.orientation.w = orientation_w
        box_pose.pose.position.x = position_x 
        box_pose.pose.position.y = position_y         
        box_pose.pose.position.z = position_z 

        self.box_name = box_name

        self.scene.add_box(name = self.box_name, pose = box_pose, size = size)
        #self.scene._make_box(name = self.box_name , pose = box_pose, size = size)

        #return self.wait_for_state_update(box_is_known=True, timeout=timeout)




    def add_mesh(self, frame = 'world', orientation_x = 0 , orientation_y = 0, orientation_z = 0, orientation_w = 1, position_x = 0, position_y = 0, position_z = 0, mesh_name = 'box', size = (0.1,0.1,0.1), timeout=4 ):
    
        
        ##### world frame reference x : front, y: right, z : up (same with size)
        ##### position is always in the midpoint of the size


        rospy.sleep(2)

        co = moveit_msgs.msg.CollisionObject()

        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = frame  

        box_pose.pose.orientation.x = orientation_x
        box_pose.pose.orientation.y = orientation_y
        box_pose.pose.orientation.z = orientation_z
        box_pose.pose.orientation.w = orientation_w
        box_pose.pose.position.x = position_x 
        box_pose.pose.position.y = position_y         
        box_pose.pose.position.z = position_z 

        mesh_path = '/home/kim/catkin_ws/src/doosan-robot/moveit_config_a0912/config/' + mesh_name + '.stl'

        self.scene.add_mesh(name = mesh_name, filename = mesh_path, pose = box_pose, size = size)

        co.id = mesh_name
        co.operation = co.ADD

        #return self.wait_for_state_update(box_is_known=True, timeout=timeout)



    def clear(self):

        self.scene.clear()






def main():
    try:

        setup = environment()

        #### size = (length , width , height) starting at the midpoint of position

        setup.clear()

        alpha = 0.01

        setup.add_box(position_x = alpha + 1.025, position_z = -0.465, size = (1.2,1.4,0.72), box_name = 'table') 
        print("============== Table in the scene ===============")

        #setup.add_box(position_x = alpha + 1.025, position_y = -0.3, position_z = -0.1, size = (0.1,0.1,0.1), box_name = 'box')
        print("============== Box in the scene ===============")

        #setup.add_mesh(position_x = alpha + 1.025 , position_z = -0.055 , size = (0.1,0.1,0.1) , mesh_name = 'laptop')
        print("============== Laptop in the scene ===============")

        #setup.add_mesh(position_x = 1.8 , position_z = -0.25 , orientation_x = -0.5, orientation_y = 0.5, orientation_z = 0.5, orientation_w = -0.5, size = (0.5,0.5,0.5) , mesh_name = 'visualhuman')
        print("============== Visualhuman in the scene ===============")


    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return






if __name__ == '__main__':
    main()