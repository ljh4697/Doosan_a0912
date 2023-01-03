#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
import cv2,os,threading,time
import numpy as np
from sensor_msgs.msg import Image,PointCloud2,PointField
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String

class img_test():
    def __init__(self):    
        self.bridge = CvBridge()
        self.subscribe_img = rospy.Subscriber('/camera/color/image_raw',Image, self.img_stereo_callback)
        # self.subscribe_img = rospy.Subscriber('/camera/depth/image_raw',Image, self.img_depth_callback)
        # self.subscribe_img = rospy.Subscriber('/camera/infra1/image_raw',Image, self.img_infra_callback)
        # self.subscribe_pcl = rospy.Subscriber('/camera/depth/color/points',PointCloud2,self.pcl_callback)

        self.pftype_sizes = {PointField.INT8: 1, PointField.UINT8: 1, PointField.INT16: 2, PointField.UINT16: 2,
                PointField.INT32: 4, PointField.UINT32: 4, PointField.FLOAT32: 4, PointField.FLOAT64: 8}
        self.DUMMY_FIELD_PREFIX = '__'  
        self.type_mappings = [(PointField.INT8, np.dtype('int8')), (PointField.UINT8, np.dtype('uint8')), (PointField.INT16, np.dtype('int16')),
                 (PointField.UINT16, np.dtype('uint16')), (PointField.INT32, np.dtype('int32')), (PointField.UINT32, np.dtype('uint32')),
                 (PointField.FLOAT32, np.dtype('float32')), (PointField.FLOAT64, np.dtype('float64'))]

        self.pftype_to_nptype = dict(self.type_mappings)

        self.publisher_img = rospy.Publisher('/camera/color/resized_image_raw',Image,queue_size=10)

        self.pcl = np.array([])
        self.desired_shape = ()

    def img_infra_callback(self,data):
        ''' convert ros imgmsg to cv-array type '''
        # for infra1,2
        cv_infra = self.bridge.imgmsg_to_cv2(data,"8UC1")
        print(type(cv_infra))
        
        # (rows,cols,channels) = cv_infra.shape
        # if cols > 60 and rows > 60:

        cv2.circle(cv_infra, (int(320),int(240)), 10, 255)

        print("infra_size : ",cv_infra.shape)
        cv2.imshow("infra window", cv_infra)
        cv2.waitKey(3)
 
    def img_stereo_callback(self,data):
        cv_image = self.bridge.imgmsg_to_cv2(data,"bgr8") 

        # cv2.circle(cv_image, (100,100), 5, 255)
        # print("image_size : ",cv_image.shape, cv_image[100,100])
        
        cv_resized = cv2.resize(cv_image,(1280,720))
        # print("image_size : ",cv_resized.shape, cv_resized[100,100])

        # cv2.imshow("Image window", cv_resized)
        # cv2.waitKey(3)

        image_message = self.bridge.cv2_to_imgmsg(cv_resized)
        self.publisher_img.publish(image_message)
    
    def img_depth_callback(self,data):
        cv_depth = self.bridge.imgmsg_to_cv2(data,"32FC1") 
        cv_image_array = np.array(cv_depth, dtype = np.dtype('f8'))
        cv_image_norm = cv2.normalize(cv_image_array, cv_image_array, 0, 1, cv2.NORM_MINMAX)

        # cv_image_resized = cv2.resize(cv_image_norm, self.desired_shape, interpolation = cv2.INTER_CUBIC)
        cv2.circle(cv_depth, (int(320),int(240)), 10, 255)
        print("image_size : ",cv_depth.shape, cv_depth[100,100])
        
        cv2.imshow("Image window", cv_image_norm)
        cv2.waitKey(3)

    def pcl_callback(self,data):
        # Converts a rospy PointCloud2 message to a numpy recordarray
        # construct a numpy record type equivalent to the point type of this cloud
        dtype_list = self.fields_to_dtype(data.fields, data.point_step)

        # parse the cloud into an array
        cloud_arr = np.fromstring(data.data, dtype_list)

        # remove the dummy fields that were added
        cloud_arr = cloud_arr[
            [fname for fname, _type in dtype_list if not (fname[:len(self.DUMMY_FIELD_PREFIX)] == self.DUMMY_FIELD_PREFIX)]]

        if data.height == 1:
           self.pcl = np.reshape(cloud_arr, (data.width,))
        else:
           self.pcl = np.reshape(cloud_arr, (data.height, data.width)) 
        
        print("pcl_shape : ", self.pcl.shape, self.pcl[100,100])
        print( self.pcl[350,600])
        print( self.pcl[650,900])

    def fields_to_dtype(self,fields, point_step):
        '''Convert a list of PointFields to a numpy record datatype.
        '''
        offset = 0
        np_dtype_list = []
        for f in fields:
            while offset < f.offset:
                # might be extra padding between fields
                np_dtype_list.append(('%s%d' % (self.DUMMY_FIELD_PREFIX, offset), np.uint8))
                offset += 1

            dtype = self.pftype_to_nptype[f.datatype]
            if f.count != 1:
                dtype = np.dtype((dtype, f.count))

            np_dtype_list.append((f.name, dtype))
            offset += self.pftype_sizes[f.datatype] * f.count

        # might be extra padding between points
        while offset < point_step:
            np_dtype_list.append(('%s%d' % (self.DUMMY_FIELD_PREFIX, offset), np.uint8))
            offset += 1
            

        return np_dtype_list

def main():
    rospy.sleep(5)
    it = img_test()
    rospy.init_node("dsr_camera_test_py")
    rate = rospy.Rate(5)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
   
if __name__=="__main__":
    main()