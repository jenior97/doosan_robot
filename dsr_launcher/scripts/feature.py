#!/usr/bin/env python


import sys
import moveit_commander
import rospy
import environment
import control
import fk
import numpy as np
from moveit_msgs.srv import GetPlanningScene, GetPlanningSceneRequest
from moveit_msgs.srv import ApplyPlanningScene, ApplyPlanningSceneRequest
from moveit_msgs.msg import CollisionObject, PlanningScene


class featuremapping(object):

    def __init__(self):
        
        super(featuremapping, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)        
        rospy.init_node("test_fk", anonymous=False)
        
        self.robot = moveit_commander.robot.RobotCommander()

        self.get_ps_srv = self._get_planning_response()
        self.get_planning_scene = self._get_planning_response_call(self.get_ps_srv).scene
        
        self.apply_ps_srv = rospy.ServiceProxy('apply_planning_scene', ApplyPlanningScene)

        self.environment = environment.environment()
        self.getfk = fk.GetFK('link6', 'world')
        self.joint_state = self.get_planning_scene.robot_state.joint_state




#############################################################################################

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



    def get_feature(self, planning_trajectory):


        table, box, laptop, visualhuman = self.environment.object_co()
        
        self.object_position = np.zeros(3)
        self.object_position[0] = laptop.mesh_poses[0].position.x
        self.object_position[1] = laptop.mesh_poses[0].position.y
        self.object_position[2] = laptop.mesh_poses[0].position.z
        
        self.user_position = np.zeros(3)
        self.user_position[0] = visualhuman.mesh_poses[0].position.x
        self.user_position[1] = visualhuman.mesh_poses[0].position.y
        self.user_position[2] = visualhuman.mesh_poses[0].position.z + 0.1 ####################### 0.1 meaning?


        feature_mapping = np.zeros(4)

        '''
        1. end_effector's height
        2. distance between eef and laptop
        3. moving distance
        4. distance between eef and user
        '''       

        TABLE_HEIGHT = 0.72 ########################################### initial condition setting?
        eef_height = 0
        t_distance_to_laptop = 0
        moving_distance = 0
        t_distance_to_user = 0 
        previous_position = np.zeros(3)
            
        for i in range(len(planning_trajectory)):
            
            joint_position = np.zeros(12)
            joint_position[:6] = planning_trajectory[i]

            self.get_planning_scene.robot_state.joint_state.position = joint_position
            eef_position = self.getfk.get_fk(self.joint_state)
            
            distance_to_laptop = np.linalg.norm(eef_position - self.object_position)
            distance_to_user = np.linalg.norm(eef_position - self.user_position)
            
            t_distance_to_laptop += distance_to_laptop
            t_distance_to_user += distance_to_user

            eef_height += eef_position[2] - TABLE_HEIGHT ######################## eef_height = np.max(eef_position) - table_height
            
            if np.sum(previous_position) != 0:
                moving_distance += np.linalg.norm(eef_position - previous_position) 
                    
            previous_position = eef_position
            
            
        eef_height /= len(planning_trajectory) 
        t_distance_to_laptop /= len(planning_trajectory)
        t_distance_to_user /= len(planning_trajectory)

        feature_mapping[0] = eef_height
        feature_mapping[1] = t_distance_to_laptop
        feature_mapping[2] = moving_distance
        feature_mapping[3] = t_distance_to_user
        

        return feature_mapping
        


    
    
def main():

    # choose one sample trajectory

    feature = []

    for i in range(1,11):
       
        planning_trajectory = np.load("/home/kim/catkin_ws/src/doosan-robot/dsr_launcher/scripts/sampled_trajectories/mid_trajectory/mid_trajectory_{num}.npz".format(num = i), allow_pickle=True)['plan']

        get_feature_map = featuremapping()
        features = get_feature_map.get_feature(planning_trajectory = planning_trajectory)   
    
        print('end_effectors height : {}, distance between eef and laptop : {} moving distance : {} distance between eef and user : {} '.format(features[0], features[1], features[2], features[3]))

        feature.append(features)

    # feature sum

        features_sum = np.sum(features)

        print('sum of featuremap : {}'.format(features_sum))

    print(feature)
    
    


if __name__ == "__main__":
    main()