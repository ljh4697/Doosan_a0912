import numpy as np


aa = np.load('/home/joonhyeok/catkin_ws/src/doosan-robot/doosan-robot/dsr_example/py/scripts/PBL/sampled_trajectories/avoid_features.npz')
psiset = np.load('/home/joonhyeok/catkin_ws/src/doosan-robot/doosan-robot/dsr_example/py/scripts/PBL/sampled_trajectories/psi_set.npz')


print(aa.files)
print(len(aa['features']))