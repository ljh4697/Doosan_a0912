#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import os,threading, time, sys
import numpy as np
from dsr_msgs.srv import PickPytamp,PlacePytamp,PickPytamp_demo,OperatePytamp

sys.dont_write_bytecode = True
sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__),"../../../../common/imp")) ) # get import path : DSR_ROBOT.py 

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
import DR_init
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL
from DSR_ROBOT import *

import std_msgs.msg
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list, list_to_pose

def SET_ROBOT(id, model):
    ROBOT_ID = id; ROBOT_MODEL= model   

def shutdown():
    print("shutdown time!")
    print("shutdown time!")
    print("shutdown time!")

    pub_stop.publish(stop_mode=1) #STOP_TYPE_QUICK)
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

class moveit_pytamp():
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        self.scene = moveit_commander.PlanningSceneInterface()
        self.robot_group = moveit_commander.MoveGroupCommander('arm')
        self.hand_group = moveit_commander.MoveGroupCommander('robotiq_2f')

        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                        moveit_msgs.msg.DisplayTrajectory,
                                                        queue_size=20)
        
        self.plan_result_pub = rospy.Publisher("/move_group/result", MoveGroupActionResult, queue_size=1)

        ## drive robot on the given path!!
        self.srv_operate_robot = rospy.Service("/operate_robot", OperatePytamp, self.operate_robot_cb)

        # Misc variables
        self.box_name = ''

        self.place_ = []
        self.object_info = {}
        self.attached_object_name = ""
        self.tray_object_count = 0
        
        self.srv_robotiq_2f_move = rospy.ServiceProxy('/' + ROBOT_ID + ROBOT_MODEL + '/gripper/robotiq_2f_move', Robotiq2FMove)
        self.srv_robotiq_gripper_move = rospy.ServiceProxy('/robotiq_control_move', Robotiq2FMove)

        # declared at pick_srv.py
        self.srv_get_pick_path = rospy.ServiceProxy('/get_pick_path',PickPytamp)
        self.srv_get_place_path = rospy.ServiceProxy('/get_place_path',PlacePytamp)

        # delcared at pick_demo_srv.py
        self.srv_get_path_demo = rospy.ServiceProxy('/get_path_demos',PickPytamp_demo)

    
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

    def hold_hand(self, target_name):
        touch_links = ['left_inner_finger_pad', 'right_inner_finger_pad']
        self.hand_group.attach_object(object_name = target_name,link_name = 'robotiq_arg2f_base_link', touch_links=touch_links)
        self.attached_object_name = target_name
    
    def release_hand(self, target_name):
        self.hand_group.detach_object(target_name)

    # def jmove_to_joint_goal(self, joint_goal):
    #     self.robot_group.go(joint_goal, wait=True)
    #     plan = self.robot_group.plan(joint_goal)
    #     self.robot_group.execute(plan, wait=True)


    def get_current_joint_values(self):
        ## You can receive status information directly from dsr without receiving joint value through Moveit.
        current_joint = self.robot_group.get_current_joint_values()
        return current_joint

    def get_demo_path(self):
        self.demo_path = self.srv_get_path_demo("A_box")

        print(self.demo_path)
        self._demo_path = self.change_path_to_degree(self.demo_path.demo_grasp)
        print(self._demo_path)

    def demo_execution(self):
        demo_path = np.rad2deg(self._demo_path).tolist()
        self.move_dsr(demo_path)

    def get_pick_path(self,object_name):
        # object_name : str
        current_joint = self.get_current_joint_values()
        planned_path = self.srv_get_pick_path(current_joint,object_name) 
        
        self.pre_grasp = np.array(planned_path.pre_grasp).reshape(-1,6)
        if len(self.pre_grasp)==1:
            self.pre_grasp = np.array(self.get_current_joint_values()).reshape(-1,6)
        self.grasp = np.array(planned_path.grasp).reshape(-1,6)
        self.post_grasp = np.array(planned_path.post_grasp).reshape(-1,6)

    def get_place_path(self,place_obj_name):
        # object_name : str
        print(self.post_grasp[-1])
        planned_path = self.srv_get_place_path(self.post_grasp[-1], place_obj_name)
        self.pre_release = np.array(planned_path.pre_release).reshape(-1,6)
        self.release = np.array(planned_path.release).reshape(-1,6)
        self.post_release = np.array(planned_path.post_release).reshape(-1,6)
    
    def movej_dsr(self,path):
        for joint_val in path:
            # operate slowly
            movej(joint_val, vel = 100, acc = 100)

    def movesj_dsr(self,path,time):
        path_ = copy.deepcopy(path)
        # print(path)
        for i,joint_val in enumerate(path):
            path_[i] = posj(joint_val[0],joint_val[1],joint_val[2],joint_val[3],joint_val[4],joint_val[5])

        movesj(path_,t = time)

    def pick_execution(self):
        pre_grasp = np.rad2deg(self.pre_grasp).tolist()
        grasp = np.rad2deg(self.grasp).tolist()
        post_grasp = np.rad2deg(self.post_grasp).tolist()
        # move dsr
        self.move_dsr(pre_grasp)
        self.move_dsr(grasp)
        # close gripper
        self.gripper_control(0.65)
        # move dsr
        self.move_dsr(post_grasp)

    def place_execution(self):
        pre_release = np.rad2deg(self.pre_release).tolist()
        release = np.rad2deg(self.release).tolist()
        post_release = np.rad2deg(self.post_release).tolist()
        # move dsr
        self.move_dsr(pre_release)
        self.move_dsr(release)
        # open gripper
        self.gripper_control(0)
        # move dsr
        self.move_dsr(post_release)

    def gripper_control(self,value):
        # 0.7 : close / 0 : open 
        #srv_robotiq_2f_move(value)
        self.srv_robotiq_gripper_move(0.7-value)

def main():
    try:
        MP = moveit_pytamp()
        rate = rospy.Rate(10)

        #MP.get_pick_path("F_box")
        #MP.pick_execution()
        #MP.get_place_path('table')
        #MP.place_execution()
        #MP.get_demo_path()
        #MP.demo_execution()
        
        #MP.get_current_joint_values()
        rospy.spin()
        
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return
    else:
        print("============ moveit_pytamp communicate!")



# def thread_subscriber():
#     rospy.Subscriber('/'+ROBOT_ID +ROBOT_MODEL+'/state', RobotState, msgRobotState_cb)
#     rospy.spin()
#     # rospy.spinner(2)  
if __name__ == "__main__":
    #----- set target robot --------------- 
    my_robot_id    = "dsr01"
    my_robot_model = "a0912"
    SET_ROBOT(my_robot_id, my_robot_model)
    rospy.init_node('moveit_pytamp_py', anonymous=True)

    rospy.on_shutdown(shutdown)
    # set_robot_mode  = rospy.ServiceProxy('/'+ROBOT_ID +ROBOT_MODEL+'/system/set_robot_mode', SetRobotMode)
    # t1 = threading.Thread(target=thread_subscriber)
    # t1.daemon = True 
    # t1.start()

    pub_stop = rospy.Publisher('/'+ROBOT_ID +ROBOT_MODEL+'/stop', RobotStop, queue_size=10)         

    main()
    print('good bye!')







