#!/usr/bin/env python3

import numpy as np
import rospy
import feature
from tqdm import trange

def main():

    rospy.init_node("create_queue", anonymous=True)

    featuremap = feature.feature()

    PHI_A = list()
    PHI_B = list()
    features = list()

    for i in trange(1,5001):

        planning_trajectory = np.load("/home/joonhyeok/catkin_ws/src/doosan-robot/doosan-robot/dsr_example/py/scripts/PBL/sampled_trajectories/feature_trajectory/feature_trajectory_{num}.npz".format(num = i), allow_pickle=True)['plan']

        feature_map = featuremap.get_2d_feature(planning_trajectory = planning_trajectory)
        features.append(feature_map)
        
        if i%2 == 0:
            PHI_A.append(feature_map)
        else:
            PHI_B.append(feature_map)

        print("{} phi is generated".format(i))


    PHI_A = np.array(PHI_A)
    PHI_B = np.array(PHI_B)
    features = np.array(features)
    
    PSI_SET = PHI_A - PHI_B
    
    #features = np.concatenate((PHI_A , PHI_B) , axis = 0)

    print(len(PHI_B))
    print(len(PHI_A))
    print(len(PSI_SET))

    #print(PHI_A[0])
    #print(PHI_B[0])
    #print(PSI_SET[0])
    
    np.savez("/home/joonhyeok/catkin_ws/src/doosan-robot/doosan-robot/dsr_example/py/scripts/PBL/sampled_trajectories/2f_psi_set.npz", PHI_A=PHI_A, PHI_B=PHI_B, PSI_SET=PSI_SET)
    np.savez("/home/joonhyeok/catkin_ws/src/doosan-robot/doosan-robot/dsr_example/py/scripts/PBL/sampled_trajectories/2f_avoid_features.npz", features = features)


if __name__ == "__main__":
    main()