#!/usr/bin/env python3
import numpy as np
import os
import math
import cv2
from renderClass import Renderer
from dt_apriltags import Detector

import rospy
import yaml
import sys
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage,CameraInfo
from cv_bridge import CvBridge, CvBridgeError

import rospkg 


"""

This is a template that can be used as a starting point for the CRA1 exercise.
You need to project the model file in the 'models' directory on an AprilTag.
To help you with that, we have provided you with the Renderer class that render the obj file.

"""

class ARNode(DTROS):

    def __init__(self, node_name):

        # Initialize the DTROS parent class
        super(ARNode, self).__init__(node_name=node_name,node_type=NodeType.GENERIC)
        self.veh = rospy.get_namespace().strip("/")

        rospack = rospkg.RosPack()
        # Initialize an instance of Renderer giving the model in input.
        self.renderer = Renderer(rospack.get_path('augmented_reality_apriltag') + '/src/models/duckie.obj')

        self.at_detector = Detector(families='tag36h11',
                       nthreads=5,
                       quad_decimate=2.0,
                       quad_sigma=0.0,
                       refine_edges=1,
                       decode_sharpening=0.25,
                       debug=0)
        
        self.bridge=CvBridge()
        
        self.pub_tagimg=rospy.Publisher('at_detect/image/compressed',CompressedImage, queue_size=1)
        self.sub_img = rospy.Subscriber('camera_node/image/compressed', CompressedImage, self.cb_at_detect)
    
    def cb_at_detect(self,msg):
        img=self.readImage(msg)
        img_gray=cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        camerainfo=self.load_intrinsics()
        cameraMatrix = np.array(camerainfo.K).reshape((3,3))
        camera_params = (cameraMatrix[0,0],cameraMatrix[1,1],cameraMatrix[0,2],cameraMatrix[1,2])

        tags = self.at_detector.detect(img_gray, False, camera_params, 0.065)
        
        for tag in tags:
#            R=tag.pose_R
#            t=(tag.pose_t).reshape((3,))
#            extrinsic=np.eye(4)
#            extrinsic[0:3,0:3]=R
#            extrinsic[0:3,3]=t
#            aux=np.eye(3)
#            aux=np.c_[aux,[0,0,0]]
#            projection=np.linalg.multi_dot([cameraMatrix,aux,extrinsic])
#            rospy.loginfo(extrinsic)
            projection1=self.projection_matrix(cameraMatrix,tag.homography)
            #rospy.loginfo(projection1)
            img=self.renderer.render(img,projection1)
#            for idx in range(len(tag.corners)):                
#                cv2.line(img, tuple(tag.corners[idx-1, :].astype(int)), tuple(tag.corners[idx, :].astype(int)), (0, 255, 0))
        
        tag_msg=self.bridge.cv2_to_compressed_imgmsg(img)
        self.pub_tagimg.publish(tag_msg)
        
    def projection_matrix(self, camera_matrix, homography):
        """
            Write here the compuatation for the projection matrix, namely the matrix
            that maps the camera reference frame to the AprilTag reference frame.
        """
        # Compute rotation along the x and y axis as well as the translation
        #homography = homography * (-1)
        rot_and_transl = np.dot(np.linalg.inv(camera_matrix), homography)
        col_1 = rot_and_transl[:, 0]
        col_2 = rot_and_transl[:, 1]
        col_3 = rot_and_transl[:, 2]
        # normalise vectors
        l = math.sqrt(np.linalg.norm(col_1, 2) * np.linalg.norm(col_2, 2))
        rot_1 = col_1 / l
        rot_2 = col_2 / l
        translation = col_3 / l
        # compute the orthonormal basis
        c = rot_1 + rot_2
        p = np.cross(rot_1, rot_2)
        d = np.cross(c, p)
        rot_1 = np.dot(c / np.linalg.norm(c, 2) + d / np.linalg.norm(d, 2), 1 / math.sqrt(2))
        rot_2 = np.dot(c / np.linalg.norm(c, 2) - d / np.linalg.norm(d, 2), 1 / math.sqrt(2))
        rot_3 = np.cross(rot_1, rot_2)
        # finally, compute the 3D projection matrix from the model to the current frame
        projection = np.stack((rot_1, rot_2, rot_3, translation)).T
        #rospy.loginfo(projection)
        return np.dot(camera_matrix, projection)

        

    def readImage(self, msg_image):
        """
            Convert images to OpenCV images
            Args:
                msg_image (:obj:`CompressedImage`) the image from the camera node
            Returns:
                OpenCV image
        """
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg_image)
            return cv_image
        except CvBridgeError as e:
            self.log(e)
            return []

    def readYamlFile(self,fname):
        """
            Reads the 'fname' yaml file and returns a dictionary with its input.

            You will find the calibration files you need in:
            `/data/config/calibrations/`
        """
        with open(fname, 'r') as in_file:
            try:
                yaml_dict = yaml.load(in_file)
                return yaml_dict
            except yaml.YAMLError as exc:
                self.log("YAML syntax error. File: %s fname. Exc: %s"
                         %(fname, exc), type='fatal')
                rospy.signal_shutdown()
                return

    def load_intrinsics(self):
        # load intrinsic calibration
        cali_file_folder = '/data/config/calibrations/camera_intrinsic/'
        cali_file = cali_file_folder + rospy.get_namespace().strip("/") + ".yaml"
        calib_data=self.readYamlFile(cali_file)
        cam_info = CameraInfo()
        cam_info.width = calib_data['image_width']
        cam_info.height = calib_data['image_height']
        cam_info.K = calib_data['camera_matrix']['data']
        cam_info.D = calib_data['distortion_coefficients']['data']
        cam_info.R = calib_data['rectification_matrix']['data']
        cam_info.P = calib_data['projection_matrix']['data']
        cam_info.distortion_model = calib_data['distortion_model']
        return cam_info
        
    def onShutdown(self):
        super(ARNode, self).onShutdown()


if __name__ == '__main__':
    # Initialize the node
    camera_node = ARNode(node_name='augmented_reality_apriltag_node')
    # Keep it spinning to keep the node alive
    rospy.spin()