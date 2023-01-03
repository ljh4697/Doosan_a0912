#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import os,threading, time, sys
import numpy as np
from dsr_msgs.srv import OperatePytamp

sys.dont_write_bytecode = True
sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__),"../../../../common/imp")) ) # get import path : DSR_ROBOT.py 

import std_msgs.msg
from std_msgs.msg import String

import copy

import geometry_msgs.msg
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
import DR_init
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL
from DSR_ROBOT import *


def SET_ROBOT(id, model):
    ROBOT_ID = id; ROBOT_MODEL= model   

def shutdown():
    print("shutdown time!")
    print("shutdown time!")
    print("shutdown time!")

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



##########
##########  must be revised
##########
class operate_position_trajectory():
    def __init__(self):
        self.srv_operate_robot = rospy.Service("/operate_robot_position", OperatePytamp, self.operate_robot_cb)

        # self.srv_robotiq_2f_move = rospy.ServiceProxy('/' + ROBOT_ID + ROBOT_MODEL + '/gripper/robotiq_2f_move', Robotiq2FMove)
        # self.srv_robotiq_gripper_move = rospy.ServiceProxy('/robotiq_control_move', Robotiq2FMove)

    def change_path_to_degree(self,path):
        return np.rad2deg(np.array(path).reshape(-1,6)).tolist()

    def operate_robot_cb(self,req):
        # req.path : 1-D list ( joint value is radian-type)
        path = self.change_path_to_degree(req.path)
        # path is 2-D list degree type
        self.operate_path(path,req.time)
        return True

    def operate_path(self,path,time):
        # path is degree type, and list type is (N x 6)
        # self.movej_dsr(path)
        self.movesj_dsr(path,time)
    
    # Moving the path as the robot motion is interrupted
    def movej_dsr(self,path):
        for joint_val in path:
            # operate slowly
            movej(joint_val, vel = 100, acc = 100)

    # The robot moves smoothly. 
    # movesj method makes motion smoothly. 
    # Doosan robot controller automatically calculates the time parameterization of the input trajectory.
    def movesj_dsr(self,path,time):
        # arg 
        #  path : joint path trajectory
        #  time : Time to execute all input trajectory
        path_ = copy.deepcopy(path)
        # print(path)
        for i,joint_val in enumerate(path):
            path_[i] = posj(joint_val[0],joint_val[1],joint_val[2],joint_val[3],joint_val[4],joint_val[5])

        movesj(path_,t = time)

    # def gripper_control(self,value):
    #     # 0.7 : close / 0 : open 
    #     self.srv_robotiq_2f_move(value)
    #     self.srv_robotiq_gripper_move(0.7-value)


def main():
    rospy.sleep(5)
    my_robot_id    = "dsr01"
    my_robot_model = "a0912"
    SET_ROBOT(my_robot_id, my_robot_model)
    rospy.init_node('operate_robot_position_trajectory_py', anonymous=True)
    
    rospy.on_shutdown(shutdown)

    try:
        ojt = operate_position_trajectory()
        rate = rospy.Rate(10)

        rospy.spin()
        
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return
    else:
        print("====== operate_robot_position_trajectory is closed! ======")

if __name__ == "__main__":
    main()
    print('good bye!')