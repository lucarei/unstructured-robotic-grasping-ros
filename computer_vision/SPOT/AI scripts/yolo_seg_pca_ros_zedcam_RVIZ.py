#!/usr/bin/env python

"""
IMPORTANT
- code used just to test YOLO segmentation in ROS
- activate webcam with --> roslaunch usb_cam usb_cam-test.launch
- change source in launch file (search it using rospy)
- segmentation and axis are always published in a topic (use RVIZ)
"""

import rospy
import cv2
import torch
import torch.backends.cudnn as cudnn
import numpy as np
from cv_bridge import CvBridge
from pathlib import Path
import os
import sys
from rostopic import get_topic_type
from std_msgs.msg import Int32,String
from sensor_msgs.msg import Image, CompressedImage

import sys
from ultralytics import YOLO
from math import atan2, cos, sin, sqrt, pi


def drawAxis(img, p_, q_, color, scale):
  p = list(p_)
  q = list(q_)
 
  ## [visualization1]
  angle = atan2(p[1] - q[1], p[0] - q[0]) # angle in radians
  hypotenuse = sqrt((p[1] - q[1]) * (p[1] - q[1]) + (p[0] - q[0]) * (p[0] - q[0]))
 
  # Here we lengthen the arrow by a factor of scale
  q[0] = p[0] - scale * hypotenuse * cos(angle)
  q[1] = p[1] - scale * hypotenuse * sin(angle)
  cv2.line(img, (int(p[0]), int(p[1])), (int(q[0]), int(q[1])), color, 3, cv2.LINE_AA)
 
  # create the arrow hooks
  p[0] = q[0] + 9 * cos(angle + pi / 4)
  p[1] = q[1] + 9 * sin(angle + pi / 4)
  cv2.line(img, (int(p[0]), int(p[1])), (int(q[0]), int(q[1])), color, 3, cv2.LINE_AA)
 
  p[0] = q[0] + 9 * cos(angle - pi / 4)
  p[1] = q[1] + 9 * sin(angle - pi / 4)
  cv2.line(img, (int(p[0]), int(p[1])), (int(q[0]), int(q[1])), color, 3, cv2.LINE_AA)
  ## [visualization1]
 

def getOrientation(pts, img):
  ## [pca]
  # Construct a buffer used by the pca analysis
  sz = len(pts)
  data_pts = np.empty((sz, 2), dtype=np.float64)
  for i in range(data_pts.shape[0]):
    data_pts[i,0] = pts[i,0,0]
    data_pts[i,1] = pts[i,0,1]
 
  # Perform PCA analysis
  mean = np.empty((0))
  mean, eigenvectors, eigenvalues = cv2.PCACompute2(data_pts, mean)
 
  # Store the center of the object
  cntr = (int(mean[0,0]), int(mean[0,1]))
  #print(cntr)
  ## [pca]
 
  ## [visualization]
  # Draw the principal components
  cv2.circle(img, cntr, 3, (255, 0, 255), 2)
  p1 = (cntr[0] + 0.02 * eigenvectors[0,0] * eigenvalues[0,0], cntr[1] + 0.02 * eigenvectors[0,1] * eigenvalues[0,0])
  p2 = (cntr[0] - 0.02 * eigenvectors[1,0] * eigenvalues[1,0], cntr[1] - 0.02 * eigenvectors[1,1] * eigenvalues[1,0])
  drawAxis(img, cntr, p1, (255, 255, 0), 1)
  drawAxis(img, cntr, p2, (0, 0, 255), 5)
 
  angle = atan2(eigenvectors[0,1], eigenvectors[0,0]) # orientation in radians
  
  #optimized angles for grasp application
  angle_deg = -(int(np.rad2deg(angle))-180) % 180
 
  # Label with the rotation angle
  label = str(angle_deg) + " degrees"
  #textbox = cv2.rectangle(img, (cntr[0], cntr[1]), (cntr[0] + 250, cntr[1] + 10), (255,255,255), -1)
  cv2.putText(img, label, (cntr[0], cntr[1]-25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0), 1, cv2.LINE_AA)
 
  return angle_deg, cntr


class image_receiver(object):
    def __init__(self):
        self.br = CvBridge()
        self.sub=rospy.Subscriber("/zed2i/zed_node/left/image_rect_color/compressed",CompressedImage,self.callback,queue_size=1)
        self.pub=rospy.Publisher("/yolo/segmented/oriented", Image, queue_size=1)
        self.pub_data=rospy.Publisher("pose", String, queue_size=1)
        self.model=YOLO("best_taco_yolo8_segmentation.pt")


    def callback(self, msg):
        rospy.loginfo('Image received...')
        self.image = self.br.compressed_imgmsg_to_cv2(msg)
        self.image=cv2.cvtColor(self.image, cv2.COLOR_RGBA2RGB) #used for zedcam
        img=self.image
        print("shape immagine in",img.shape)
        img = cv2.resize(img, None, fx= 2, fy= 2, interpolation= cv2.INTER_LINEAR)
        height=img.shape[0]
        width=img.shape[1]
        dim=(width,height)

        prediction=self.model.predict(img,save=False, save_txt=False,device=0)

        try:
            #take class id
            class_id=prediction[0].boxes[0].cls[0].item()
            #take string from list of objects through class id
            print("predicted object: ",prediction[0].names[class_id])

            #bw image
            bw=(prediction[0].masks.data[0].cpu().numpy() * 255).astype("uint8")
            #cv2.imshow("Input image BN",bw)
            bw=cv2.resize(bw, dim, interpolation = cv2.INTER_AREA)
            print("shape immagine bw",bw.shape)

            #contours from bw images
            contours, _ = cv2.findContours(bw, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
            #print(contours)
            for i, c in enumerate(contours):

                # Calculate the area of each contour
                area = cv2.contourArea(c)
                    
                # Ignore contours that are too small or too large
                if area < 3700 or 100000 < area:
                        continue
                    
                # Draw each contour only for visualisation purposes
                cv2.drawContours(img, contours, i, (0, 0, 255), 2)
                    
                # Find the orientation of each shape
                angle_deg, cntr=getOrientation(c, img)
                print(cntr)
                print(angle_deg)
                #print(getOrientation(c, img))
                data=str(cntr[0]/2)+","+str(cntr[1]/2)+","+str(angle_deg)
                print(data)
                self.pub_data.publish(data)

                #publish result after conversion
            #self.pub.publish(self.br.cv2_to_imgmsg(img))
            

        except (AttributeError, IndexError):
            print("Prediction Error YOLO")
        #rospy.spin(0.5)
        

def main():
    # create a subscriber instance
    sub = image_receiver()
    # follow it up with a no-brainer sequence check
    print('Currently in the main function...')
    # initializing the subscriber node
    rospy.init_node('listener', anonymous=True)
    rospy.spin()
  
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
