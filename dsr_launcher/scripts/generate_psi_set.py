#!/usr/bin/env python

import numpy as np
import rospy
import feature


def main():

    rospy.init_node("create_queue", anonymous=True)

    featuremap = feature.feature()

    PHI_A = list()
    PHI_B = list()


    for i in range(1,201):

        planning_trajectory = np.load("/home/kim/catkin_ws/src/doosan-robot/dsr_launcher/scripts/sampled_trajectories/mid_trajectory/mid_trajectory_{num}.npz".format(num = i), allow_pickle=True)['plan']

        feature_map = featuremap.get_feature(planning_trajectory = planning_trajectory)

        if i % 2 == 0:
            PHI_A.append(feature_map)
        else:
            PHI_B.append(feature_map)

        print("{} phi is generated".format(i))


    PHI_A = np.array(PHI_A)
    PHI_B = np.array(PHI_B)
    PSI_SET = PHI_A - PHI_B

    print(len(PHI_B))
    print(len(PHI_A))
    print(len(PSI_SET))

    #print(PHI_A[0])
    #print(PHI_B[0])
    #print(PSI_SET[0])
    
    np.savez("/home/kim/catkin_ws/src/doosan-robot/dsr_launcher/scripts/sampled_trajectories/psi_set.npz", PHI_A=PHI_A, PHI_B=PHI_B, PSI_SET=PSI_SET)



if __name__ == "__main__":
    main()