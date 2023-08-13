#!/usr/bin/env python3

from itertools import count
import rospy

import numpy as np
from pathlib import Path
import os
import sys
from rostopic import get_topic_type
import tf
from math import atan2, cos, sin, sqrt, pi,acos,atan
import cv2
import struct
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from detection_msgs.msg import BoundingBox, BoundingBoxes
from std_msgs.msg import Int32, Float32

indoor_test = 0

global angle
angle=0
def callback_angle(data):
    print("--------------------------------hello_angle")
    global angle
    angle=data.data



class ObjDetector:
    def __init__(self):

        self.br = tf.TransformBroadcaster()
        self.bridge = CvBridge()
        self.angle=0

        # self.image_sub = rospy.Subscriber(
        #         rospy.get_param("~output_topic"), Image, self.callback, queue_size=1
        #     )
        self.calibration_K =  np.reshape(np.array(rospy.wait_for_message("/zed2i/zed_node/left/camera_info", CameraInfo, timeout=5).K), (-1, 3))
        #print(self.calibration_K)

        self.image_sub = rospy.Subscriber(
            "/yolov5/detections", BoundingBoxes, self.callback, queue_size=1
        )


        self.pub=rospy.Publisher("area_talker",Int32, queue_size=1)
        self.pub_angle=rospy.Publisher("angle_talker",Float32, queue_size=1)

        # input_image_type, input_image_topic, _ = get_topic_type(rospy.get_param("~input_image_topic"), blocking = True)
        # self.compressed_input = input_image_type == "sensor_msgs/CompressedImage"




    def callback(self, data):
        print("hello")
        listener = tf.TransformListener()
        #print(data)
        if data.bounding_boxes:
            depth = rospy.wait_for_message("/zed2i/zed_node/depth/depth_registered", Image, timeout=5)
            # print(struct.unpack('f', depth.data[:10]))
            depth_image = self.bridge.imgmsg_to_cv2(depth, "passthrough")
            depth_image = depth_image[data.bounding_boxes[0].ymin -30 :data.bounding_boxes[0].ymax+30, data.bounding_boxes[0].xmin-30:data.bounding_boxes[0].xmax +30]
            depth_array = depth_image[~np.isnan(depth_image)].flatten()

            floor_distance = self.find_floor(depth_array)
            #print(floor_distance)

            if indoor_test:
                try:
                    # now= rospy.Time.now()
                    # listener.waitForTransform('/obj_detected', '/base_link', now, rospy.Duration(3.0));
                    now = listener.getLatestCommonTime('/base_link', '/zed2i_left_camera_optical_frame')
                    (trans,rot) = listener.lookupTransform('/base_link', '/zed2i_left_camera_optical_frame', now)
                    # print("tf from base_footprint to zed2i" + str((trans,rot)))
                    floor_distance = trans[2] - 0.05 + 0.715
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as ex:
                    print(ex)
            #print("Floor distance: "+ str(floor_distance))

            # find object most probable to be waste

            max_value = None

            for i, item in enumerate(data.bounding_boxes):
                itemCenterY =(item.ymin + item.ymax) / 2
                area = (item.ymax-item.ymin)*(item.xmax-item.xmin)
                height=item.ymax-item.ymin
                width=item.xmax-item.xmin
                # if itemCenterY > 240 or area < 500:
                #     continue

                if (max_value is None or item.probability > max_value):
                    print(item.Class)
                    if item.Class=="Plastic":
                        max_value = item.probability
                        max_index = i
                        angle_est=0
                        #ip=sqrt(pow(altezza,2)+pow(lunghezza,2))
                        #angle_est=acos(lunghezza/ip)
                        angle_est=atan(height/width)
                        angle_deg=angle_est*180/pi
                        #print(angle_est)
                        print(angle_deg)
                        #print("l'angolo in gradi è ", str(angle*180/pi))
                        self.pub_angle.publish(angle_est)
                        #print(height)
                        #print(width)
                        if height>width:
                            self.pub.publish(area)
                            print(area)
                        else:
                            self.pub.publish(100000)



            if max_value is not None:
                xCenter = (data.bounding_boxes[max_index].xmin + data.bounding_boxes[max_index].xmax) / 2
                yCenter = (data.bounding_boxes[max_index].ymin + data.bounding_boxes[max_index].ymax) / 2
                print(xCenter, yCenter)
                # floor_distance = 1 # Cambiato per test indoor - 27/1/23
                [X,Y,Z] = (np.dot(np.linalg.pinv(self.calibration_K), np.array([xCenter*floor_distance, yCenter*floor_distance, floor_distance])))
                #print([X,Y,Z])

                global angle
                print("angolo -------------------------------------------",str(angle))

                self.br.sendTransform((X, Y, floor_distance),
                         tf.transformations.quaternion_from_euler(0, 0, angle),
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
    rospy.Subscriber("/angle_pose", Float32, callback_angle)
    find_bottle = ObjDetector()
    rospy.spin()
