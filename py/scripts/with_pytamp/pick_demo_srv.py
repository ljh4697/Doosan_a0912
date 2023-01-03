#!/usr/bin/env python3
#-*- coding:utf-8 -*-

import rospy	
import numpy as np
from dsr_msgs.srv import PickPytamp, PlacePytamp, PickPytamp_demo, PickPytamp_demoResponse,OperatePytamp

from pytamp.benchmark import Benchmark1
from pytamp.action.pick import PickAction
from pytamp.action.place import PlaceAction


import numpy as np
import copy
from pytamp.benchmark import Benchmark
from pytamp.action.pick import PickAction
from pytamp.utils.heuristic_utils import get_custom_tcp_pose

from pykin.robots.single_arm import SingleArm
from pykin.kinematics.transform import Transform
from pykin.utils.mesh_utils import get_object_mesh

class pnp_pytamp():
    def __init__(self):
        self.custom_benchmark = Benchmark(robot_name="doosan", geom="visual", is_pyplot=True)
        self.custom_benchmark.robot = SingleArm(f_name=self.custom_benchmark.urdf_file,
                               offset=Transform(rot=[0.0, 0.0, 0.0], pos=[0, 0, 0.913]),
                               has_gripper=True,
                               gripper_name=self.custom_benchmark.gripper_name)        
        self.custom_benchmark.robot.setup_link_name("base_0", "right_hand")
        self.custom_benchmark.robot.init_qpos = np.array([ 0, 0, np.pi/1.5, 0, np.pi/3, 0])
        self.custom_benchmark.scene_mngr.add_robot(self.custom_benchmark.robot, self.custom_benchmark.robot.init_qpos)
        
        self.table_mesh = get_object_mesh('ben_table.stl')
        self.tray_red_mesh = get_object_mesh('ben_tray_red.stl')
        self.box_mesh = get_object_mesh('ben_cube.stl', 0.06)

        self.table_pose = Transform(pos=np.array([1.1, -0.4, -0.03]))
        self.tray_red_pose = Transform(pos=np.array([0.6, -0.5-0.3, 0.8]))
        self.A_box_pose = Transform(pos=np.array([0.6, 0, self.table_mesh.bounds[1][2]]))

        self.custom_benchmark.scene_mngr.add_object(name="table", gtype="mesh", gparam=self.table_mesh, h_mat=self.table_pose.h_mat, color=[0.39, 0.263, 0.129])
        self.custom_benchmark.scene_mngr.add_object(name="A_box", gtype="mesh", gparam=self.box_mesh, h_mat=self.A_box_pose.h_mat, color=[1.0, 1.0, 0.])

    


        self.pick = PickAction(self.custom_benchmark.scene_mngr)

        self.srv_demo = rospy.Service("/get_path_demos", PickPytamp_demo, self.get_pick_demo_cb)
        self.srv_operate_robot = rospy.ServiceProxy('/operate_robot',OperatePytamp)

    def move_robot(self, obj_name):
        self.tcp_pose = get_custom_tcp_pose(self.pick.scene_mngr, obj_name)

        grasp_poses = {}
        grasp_poses[self.pick.move_data.MOVE_grasp] = self.pick.scene_mngr.scene.robot.gripper.compute_eef_pose_from_tcp_pose(self.tcp_pose)
        grasp_poses[self.pick.move_data.MOVE_pre_grasp] = self.pick.get_pre_grasp_pose(grasp_poses[self.pick.move_data.MOVE_grasp])
        grasp_poses[self.pick.move_data.MOVE_post_grasp] = self.pick.get_post_grasp_pose(grasp_poses[self.pick.move_data.MOVE_grasp])

        pre_grasp_pose = grasp_poses[self.pick.move_data.MOVE_pre_grasp]
        grasp_pose = grasp_poses[self.pick.move_data.MOVE_grasp]

        pre_grasp_thetas = self.pick.scene_mngr.scene.robot.inverse_kin(self.pick.scene_mngr.scene.robot.init_qpos, pre_grasp_pose)
        demo_grasp = self.pick.get_cartesian_path(pre_grasp_thetas, grasp_pose)        
        #print(demo_grasp[0][0])
        demo_grasp = np.round(np.array(demo_grasp),6).reshape(-1).tolist()
        #print(len(demo_grasp))
        #print(demo_grasp)
        self.srv_operate_robot(demo_grasp)

    def get_pick_demo_cb(self, req):
        
        self.tcp_pose = get_custom_tcp_pose(self.pick.scene_mngr, req.object_name)

        grasp_poses = {}
        grasp_poses[self.pick.move_data.MOVE_grasp] = self.pick.scene_mngr.scene.robot.gripper.compute_eef_pose_from_tcp_pose(self.tcp_pose)
        grasp_poses[self.pick.move_data.MOVE_pre_grasp] = self.pick.get_pre_grasp_pose(grasp_poses[self.pick.move_data.MOVE_grasp])
        grasp_poses[self.pick.move_data.MOVE_post_grasp] = self.pick.get_post_grasp_pose(grasp_poses[self.pick.move_data.MOVE_grasp])

        pre_grasp_pose = grasp_poses[self.pick.move_data.MOVE_pre_grasp]
        grasp_pose = grasp_poses[self.pick.move_data.MOVE_grasp]

        pre_grasp_thetas = self.pick.scene_mngr.scene.robot.inverse_kin(self.pick.scene_mngr.scene.robot.init_qpos, pre_grasp_pose)
        demo_grasp = self.pick.get_cartesian_path(pre_grasp_thetas, grasp_pose)        
        #print(demo_grasp[0][0])
        demo_grasp = np.round(np.array(demo_grasp),6).reshape(-1).tolist()
        #print(len(demo_grasp))
        #print(demo_grasp)
        
        

        return PickPytamp_demoResponse(demo_grasp)


    def get_pick_object_cb(self, req):
        self.actions = list(self.pick.get_possible_actions_level_1())

        self.pick_path = []
        self.pick_objects = []
        self.pick_object_poses = []
        self.success_joint_path = False

        print(req.current_joint)
        if req.current_joint == (0.0, 0.0, 0.0, 0.0, 0.0, 0.0):
            thetas = None
            print("theta is None")
        else:
            thetas = req.current_joint
            print("init_thetas : ",thetasdeom_grasp)
        for i in range(len(self.actions)):
            if req.object_name == self.actions[i]['pick_obj_name']:
                for idx, pick_scene in enumerate(self.pick.get_possible_transitions(self.pick.scene_mngr.scene, action=self.actions[i])):
                    self.pick_scene = pick_scene
                    pick_joint_path = self.pick.get_possible_joint_path_level_2(scene=self.pick_scene, grasp_poses=self.pick_scene.grasp_poses, init_thetas=thetas)
                    
                    if pick_joint_path:
                        self.success_joint_path = True
                        
                        self.pick_pre_grasp = np.array(pick_joint_path[0]['pre_grasp'],dtype="float32").reshape(-1).tolist()
                        self.pick_grasp = np.array(pick_joint_path[0]['grasp'],dtype="float32").reshape(-1).tolist()
                        self.pick_post_grasp = np.array(pick_joint_path[0]['post_grasp'],dtype="float32").reshape(-1).tolist()
                        self.pick_objects.append(self.pick.scene_mngr.attached_obj_name)
                        self.pick_object_poses.append(self.pick.scene_mngr.scene.robot.gripper.pick_obj_pose)
                        break
                if self.success_joint_path: 
                    break
            if self.success_joint_path: 
                break

        if not self.pick_pre_grasp:
            self.pick_pre_grasp = [0,0,0,0,0,0]
        return self.pick_pre_grasp ,self.pick_grasp, self.pick_post_grasp

    def get_place_object_cb(self, req):

        if req.current_joint == (0.0, 0.0, 0.0, 0.0, 0.0, 0.0):
            thetas = self.pick_post_grasp[-6:]
            print("theta is ",thetas)
        else:
            thetas = req.current_joint
            print("init_thetas : ",thetas)
        self.success_joint_path = False
        self.place_actions = list(self.place.get_possible_actions_level_1(self.pick_scene)) 
        print(self.place_actions)
        for action in self.place_actions:
            if req.place_object_name == action['place_obj_name']:
                for place_scene in self.place.get_possible_transitions(scene=self.pick_scene, action=action):
                    place_joint_path = self.place.get_possible_joint_path_level_2(
                        scene=place_scene, release_poses=place_scene.release_poses, init_thetas=self.pick_post_grasp[-6:])
                    if place_joint_path:
                        self.success_joint_path = True
                        self.pre_release = np.array(place_joint_path[0]['pre_release'],dtype='float32').reshape(-1).tolist()
                        self.release = np.array(place_joint_path[0]['release'],dtype='float32').reshape(-1).tolist()
                        self.post_release = np.array(place_joint_path[0]['post_release'],dtype='float32').reshape(-1).tolist()
                        break
                if self.success_joint_path: 
                    break    
            
            if self.success_joint_path: 
                break          
        
        return self.pre_release, self.release, self.post_release  

    
def main():
    rospy.init_node("pytamp_pick_srv_demo_node",anonymous=False)
    rate = rospy.Rate(10)

    Pick_Pytamp = pnp_pytamp()
    Pick_Pytamp.move_robot("A_box")
    print("Ready to start")
    rospy.spin()
      
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass