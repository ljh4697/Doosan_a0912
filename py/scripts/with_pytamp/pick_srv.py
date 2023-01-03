#!/usr/bin/env python3
#-*- coding:utf-8 -*-

import rospy	
import numpy as np
from dsr_msgs.srv import PickPytamp,PlacePytamp

from pytamp.benchmark import Benchmark1
from pytamp.action.pick import PickAction
from pytamp.action.place import PlaceAction

class pnp_pytamp():
    def __init__(self):
        self.benchmark1 = Benchmark1(robot_name="doosan", geom="visual", is_pyplot=True)
        self.pick = PickAction(self.benchmark1.scene_mngr, n_contacts=0, n_directions=0)
        self.place = PlaceAction(self.benchmark1.scene_mngr, n_samples_held_obj=0, n_samples_support_obj=0)

        self.srv_pick = rospy.Service("/get_pick_path", PickPytamp, self.get_pick_object_cb)
        self.srv_place = rospy.Service("/get_place_path", PlacePytamp, self.get_place_object_cb)

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
            print("init_thetas : ",thetas)
        for i in range(len(self.actions)):
            if req.object_name == self.actions[i]['pick_obj_name']:
                for idx, pick_scene in enumerate(self.pick.get_possible_transitions(self.pick.scene_mngr.scene, action=self.actions[i])):
                    self.pick_scene = pick_scene
                    pick_joint_path = self.pick.get_possible_joint_path_level_2(scene=self.pick_scene, grasp_poses=self.pick_scene.grasp_poses, init_thetas=thetas)
                    
                    if pick_joint_path:
                        self.success_joint_path = True
                        
                        self.pick_pre_grasp = np.array(pick_joint_path[0]['pre_grasp'],dtype="float32").reshape(-1).tolist()
                        print(pick_joint_path[0]['pre_grasp'])
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
    rospy.init_node("pytamp_pick_srv_node",anonymous=False)
    rate = rospy.Rate(10)

    Pick_Pytamp = pnp_pytamp()
    print("Ready to start")
    rospy.spin()
      
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass