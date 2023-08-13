#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32, Float32
from math import atan2, cos, sin, sqrt, pi,acos

def callback_rotation(data):
    start=-10
    end=170
    increment=10
    store=[]
    # Iterazione con il ciclo for
    for a in range(start, end, increment):
        pub.publish(a)
        #rospy.sleep(1)
        try:
            area=rospy.wait_for_message('/area_talker', Int32,timeout=5)
            store.append(area.data)
        except rospy.exceptions.ROSException:
            area=100000
            store.append(area)
        print(area)
        # if a>0:
        #     if store[-1]>store[-2]:
        #         break

    min_value = min(store)  # Trova il valore minimo nell'array
    min_index = store.index(min_value)  # Trova l'indice del valore minimo
    print(min_index*10)
    ang_rad=(min_index*10)*pi/180
    pub.publish(0)
    pub_pose.publish(ang_rad)

rospy.init_node('integer_publisher', anonymous=True)
pub = rospy.Publisher('angle_image', Int32, queue_size=10)
pub_pose = rospy.Publisher('angle_pose', Float32, queue_size=10)
pub.publish(0)
pub_pose.publish(0)

rospy.Subscriber('/flag', Int32, callback_rotation)
rospy.spin()





