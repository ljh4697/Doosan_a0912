#!/usr/bin/env python2
# -*- coding: utf-8 -*-

from copy import deepcopy
from xml.dom.minidom import Attr
import rospy,tf
import cv2,os,threading,time
import numpy as np
from sensor_msgs.msg import Image,PointCloud2,PointField
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
from geometry_msgs.msg import Transform


class img_test():
    def __init__(self):   
        self.bridge = CvBridge()

        # camera matrix
        self.mtx = np.array([[601.73815588,   0.        , 640.35016164],[  0.        , 610.19885083, 360.579869  ],[0,0,1]])
        self.mtx_h = np.array([[601.73815588,   0.        , 640.35016164,0],[  0.        , 610.19885083, 360.579869 ,0 ],[0,0,1,0],[0,0,0,1]])
        self.mtx_inv = np.linalg.inv(self.mtx)
        self.mtx_inv = np.hstack((self.mtx_inv,[[0],[0],[0]]))
        self.mtx_inv = np.vstack((self.mtx_inv,[0,0,0,1]))
        # print(self.mtx_inv)
        self.dist = np.array([[ 0.09222483,  0.00764416,  0.00203016, -0.00808426, -0.50662685]])
        # various for aruco 
        self.arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_7X7_1000)
        self.arucoParms = cv2.aruco.DetectorParameters_create()
        self.fontFace = cv2.FONT_HERSHEY_SIMPLEX
        self.fontScale = 0.5
        self.clr = (255,0,0)

        self.subscribe_img = rospy.Subscriber('/camera/color/resized_image_raw',Image, self.img_callback)
        self.subscribe_pcl = rospy.Subscriber('/camera/depth/color/points',PointCloud2,self.pcl_callback)
        self.subscribe_world_to_cam = rospy.Subscriber('/world_to_cam',Transform,self.world_to_cam_callback)
        self.img = []
        
        self.pftype_sizes = {PointField.INT8: 1, PointField.UINT8: 1, PointField.INT16: 2, PointField.UINT16: 2,
                PointField.INT32: 4, PointField.UINT32: 4, PointField.FLOAT32: 4, PointField.FLOAT64: 8}
        self.DUMMY_FIELD_PREFIX = '__'  
        self.type_mappings = [(PointField.INT8, np.dtype('int8')), (PointField.UINT8, np.dtype('uint8')), (PointField.INT16, np.dtype('int16')),
                (PointField.UINT16, np.dtype('uint16')), (PointField.INT32, np.dtype('int32')), (PointField.UINT32, np.dtype('uint32')),
                (PointField.FLOAT32, np.dtype('float32')), (PointField.FLOAT64, np.dtype('float64'))]

        self.pftype_to_nptype = dict(self.type_mappings)

    def pcl_callback(self,data):

        try:
            # Converts a rospy PointCloud2 message to a numpy recordarray
            # construct a numpy record type equivalent to the point type of this cloud
            if self.point_x != None:
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

            # print(self.pcl[self.point_y,self.point_x])
            self.marker_tvec = self.pcl[self.point_y,self.point_x]
        except AttributeError:
            pass

    def img_callback(self,data):
        self.img = self.bridge.imgmsg_to_cv2(data,"8UC3") 
        # print(type(self.img))

        self.detect_aruco_corners(self.img)
        self.get_cam_transpose_invrse()

    def detect_aruco_corners(self,img):
        corners,ids,rejected = cv2.aruco.detectMarkers(img,self.arucoDict,parameters = self.arucoParms)   #detect

        cv2.aruco.drawDetectedMarkers(img, corners) 
        try:
            corner = corners[0][0]
            # print(corner)
            self.point_x = int((max(corner[:,0]) + min(corner[:,0]))/2)
            self.point_y = int((max(corner[:,1]) + min(corner[:,1]))/2)
            print(self.point_x, self.point_y)

            # cv2.circle(img, (self.point_x,self.point_y), 5, 255)
        except IndexError:
            pass

        if len(corners):
            # If you do not enter the actual marker size, tvec will not going well.
            rvec, tvec  = cv2.aruco.estimatePoseSingleMarkers(corners[0], 0.3, cameraMatrix = self.mtx, distCoeffs = self.dist)
            print("rvec : ",rvec)
            self.marker_quat_vec =tf.transformations.quaternion_from_euler(tvec[0][0][0],tvec[0][0][1],tvec[0][0][2])
            # print( "rvec : ", rvec, " tvec : ", tvec )
            for _,i in enumerate(corners):
                cv2.putText(img, str('id:{}'.format(int(ids[_]))), (int(i[0][2][0]),int(i[0][2][1])), self.fontFace, self.fontScale, self.clr)
            
            for i in range(0,ids.size):
                cv2.aruco.drawAxis(img, self.mtx, self.dist, rvec[i], tvec[i], 0.02)
                # img ,pts = self.draw_cube(img, rvec[i], tvec[i], self.mtx, self.dist, 0.025)

        cv2.imshow("Image window", img)
        cv2.waitKey(3)

        return img,corners,ids
    def draw_cube(self,img,rvecs,tvecs,mtx,dist,l):
        axis = np.float32([[-l,l, 0], [l, l, 0], [l, -l, 0], [-l, -l, 0],
                            [-l,l, l], [l, l, l], [l, -l, l], [-l, -l, l]])
        imgpts, jac = cv2.projectPoints(axis, rvecs, tvecs, mtx, dist)
        imgpts = np.int32(imgpts).reshape(-1, 2)
        # img = cv2.drawContours(img, [imgpts[:4]], -1, (255, 0, 0), -2)
        # draw pillars in blue color
        for i,j in zip(range(4),range(4,8)):
            img = cv2.line(img, tuple(imgpts[i]), tuple(imgpts[j]),(255),2)
        # draw top layer in red color
        img = cv2.drawContours(img, [imgpts[4:]],-1,(0,0,255),1)    
        return img, imgpts

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
    
    def get_cam_transpose_invrse(self):
        try:
            # position aquired from point cloud. 
            xyz_c = np.array([self.marker_tvec[0],self.marker_tvec[1],self.marker_tvec[2],1]) 

            print("cam to marker : ",xyz_c)

            # Translation (World_to_cam) * (cam_to_point)
            self.XYZ = self.cam_mtx.dot(xyz_c)
            print("world to point", self.XYZ)

        except AttributeError:
            pass

    def world_to_cam_callback(self,data):
        euler = tf.transformations.quaternion_matrix([data.rotation.x,data.rotation.y,data.rotation.z,data.rotation.w])

        euler[0,3] = data.translation.x
        euler[1,3] = data.translation.y
        euler[2,3] = data.translation.z
        
        self.cam_mtx = euler

        euler_inv = np.linalg.inv(euler)
        self.euler_inv = euler_inv


def main():
    it = img_test()
    rospy.init_node("camera_aruco_detection_py")
    rospy.sleep(3)
    rate = rospy.Rate(5)
    try:
        while not rospy.is_shutdown():
            # it.detect_aruco_corners(it.img)

            # ctw.get_cam_transpose_invrse()
            rate.sleep()
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
   
if __name__=="__main__":
    main()