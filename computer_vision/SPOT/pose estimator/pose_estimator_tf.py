#!/usr/bin/env python3

from itertools import count
import rospy

import numpy as np
from pathlib import Path
import os
import sys
from rostopic import get_topic_type
import tf
from math import atan2, cos, sin, sqrt, pi,acos
import cv2
import struct
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from detection_msgs.msg import BoundingBox, BoundingBoxes
from std_msgs.msg import Int32, Float32,String


class ObjDetector:
    def __init__(self):

        self.br = tf.TransformBroadcaster()
        self.bridge = CvBridge()
        self.angle=0

        self.calibration_K =  np.reshape(np.array(rospy.wait_for_message("/zed2i/zed_node/left/camera_info", CameraInfo, timeout=5).K), (-1, 3))
        rospy.Subscriber("pose", String, self.callback_pose,queue_size=1)
        #print(self.calibration_K)



    def callback_pose(self, data):
        print(data.data)
        pose_array = data.data.split(",")
        x_center=float(pose_array[0])
        y_center=float(pose_array[1])
        orientation_deg=float(pose_array[2])

        orientation_rad=orientation_deg*pi/180
        
        robot_angle=-(orientation_rad)+pi/2
        print(robot_angle)

        listener = tf.TransformListener()
        depth = rospy.wait_for_message("/zed2i/zed_node/depth/depth_registered", Image, timeout=5)
        depth_image = self.bridge.imgmsg_to_cv2(depth, "passthrough")
        depth_image = depth_image[int(y_center)-50 : int(y_center)+50, int(x_center-50) : int(x_center+50)]
        depth_array = depth_image[~np.isnan(depth_image)].flatten()

        floor_distance = self.find_floor(depth_array)
        
        #print(xCenter, yCenter)
        #print(floor_distance)
        
        [X,Y,Z] = (np.dot(np.linalg.pinv(self.calibration_K), np.array([x_center*floor_distance, y_center*floor_distance, floor_distance])))
        #print([X,Y,Z])


        self.br.sendTransform((X, Y, floor_distance),
                tf.transformations.quaternion_from_euler(0, 0, robot_angle),
                rospy.Time.now(),
                "obj_detected",
                "zed2i_left_camera_optical_frame")



    def find_floor(self, data, m = 2.):
        # We are making the assumption that the camera is overhead
        # perform outlier rejection and median of values for floor distance estimation

        d = np.abs(data - np.median(data))
        mdev = np.median(d)
        s = d/mdev if mdev else 0.
        return np.median(data[s<m])



if __name__ == "__main__":
    rospy.init_node("obj_detector", anonymous=True)
    
    find_bottle = ObjDetector()
    rospy.spin()
