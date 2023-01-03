#!/usr/bin/env python

import rospy, roslib
import numpy as np
import tf,math
from geometry_msgs.msg import Transform
import tf2_ros
import time
class cam_to_world():
    def __init__(self):

        self.tfbuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfbuffer)
        self.pub = rospy.Publisher('/world_to_cam',Transform,queue_size=5)

    def get_cam_config(self):

        trans = self.tfbuffer.lookup_transform('dsr01/world','dsr01/camera_color_optical_frame',rospy.Time(0),rospy.Duration(3.0))
        self.trans = trans.transform

        self.pub.publish(self.trans)

    def get_cam_transpose_invrse(self):
        euler = tf.transformations.quaternion_matrix([self.trans.rotation.w,self.trans.rotation.x,self.trans.rotation.y,self.trans.rotation.z])

        euler[0,3] = self.trans.translation.x
        euler[1,3] = self.trans.translation.y
        euler[2,3] = self.trans.translation.z

        np.linalg.inv(euler)


if __name__=="__main__":
    rospy.init_node("world_to_cam_py")
    rate = rospy.Rate(10)

    ctw = cam_to_world()
    rospy.sleep(5)
    try:
        while not rospy.is_shutdown():
            ctw.get_cam_config()
            # ctw.get_cam_transpose_invrse()
            rate.sleep()
    except rospy.ROSInternalException:
        pass
