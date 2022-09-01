#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import trimesh

from moveit_msgs.srv import GetPlanningScene, GetPlanningSceneRequest
from moveit_msgs.srv import ApplyPlanningScene, ApplyPlanningSceneRequest
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.msg import AttachedCollisionObject
from shape_msgs.msg import SolidPrimitive, Plane, Mesh, MeshTriangle
from geometry_msgs.msg import Point




class environment(object):

    def __init__(self):
        super(environment, self).__init__()


        #moveit_commander.roscpp_initialize(sys.argv)
        #rospy.init_node('setup_environment', anonymous=True)


        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface(synchronous=True)
        move_group = moveit_commander.MoveGroupCommander("arm")


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

        self.objects = {}


        self.get_ps_srv = self._get_planning_response()
        self.get_planning_scene = self._get_planning_response_call(self.get_ps_srv).scene
        
        self.apply_ps_srv = rospy.ServiceProxy('apply_planning_scene', ApplyPlanningScene)



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



###############################################################################



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

        co = CollisionObject()
        co.id = self.box_name
        co.operation = CollisionObject.ADD
        co.header = box_pose.header

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = list(size)

        co.primitives = [box]
        co.primitive_poses = [box_pose.pose]

        self.get_planning_scene.world.collision_objects.append(co)

        self.objects[self.box_name] = box_pose

        self._update_planning_scene(self.get_planning_scene)

        return co, self.wait_for_state_update(box_is_known=True, timeout=timeout)




    def add_mesh(self, mesh_name, position_x = 0, position_y = 0, position_z = 0, orientation_x = 0 , orientation_y = 0, orientation_z = 0, orientation_w = 1, size=(1, 1, 1), frame = "world", timeout=4):

        co = CollisionObject()
        co.operation = CollisionObject.ADD
        co.id = mesh_name
        co.header.frame_id = frame

        #make mesh

        mesh_path = '/home/kim/catkin_ws/src/doosan-robot/moveit_config_a0912/config/' + mesh_name + '.stl'
        mesh = trimesh.load(mesh_path, force='mesh')

        mesh_01 = Mesh()
        for face in mesh.faces:
            triangle = MeshTriangle()
            triangle.vertex_indices = face
            mesh_01.triangles.append(triangle)

        for vertex in mesh.vertices:
            point = Point()
            point.x = vertex[0] * size[0]
            point.y = vertex[1] * size[1]
            point.z = vertex[2] * size[2]
            mesh_01.vertices.append(point)
        
        object_pose = geometry_msgs.msg.Pose()

        object_pose.position.x = position_x
        object_pose.position.y = position_y
        object_pose.position.z = position_z
        object_pose.orientation.x = orientation_x
        object_pose.orientation.y = orientation_y
        object_pose.orientation.z = orientation_z
        object_pose.orientation.w = orientation_w
        
        
        co.meshes = [mesh_01]
        co.mesh_poses = [object_pose]

        self.get_planning_scene.world.collision_objects.append(co)

        self._update_planning_scene(self.get_planning_scene)

        return co, self.wait_for_state_update(box_is_known=True, timeout=timeout)




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

        setup.add_box(position_x = alpha + 1.025, position_y = -0.3, position_z = -0.1, size = (0.1,0.1,0.1), box_name = 'box')
        print("============== Box in the scene ===============")

        setup.add_mesh(position_x = alpha + 1.025 , position_z = -0.055 , size = (0.1,0.1,0.1) , mesh_name = 'laptop')
        print("============== Laptop in the scene ===============")

        setup.add_mesh(position_x = 1.8 , position_z = -0.25 , orientation_x = -0.5, orientation_y = 0.5, orientation_z = 0.5, orientation_w = -0.5, size = (0.5,0.5,0.5) , mesh_name = 'visualhuman')
        print("============== Visualhuman in the scene ===============")






    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return






if __name__ == '__main__':
    main()