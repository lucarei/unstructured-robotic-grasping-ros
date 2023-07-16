#!/usr/bin/env python3
 
# Author: Automatic Addison https://automaticaddison.com
# Description: An example of a basic finite state machine for a turnstile at 
#   a stadium or metro station.
 
# Import the necessary libraries
import rospy # Python client library
from smach import State, StateMachine # State machine library
import smach_ros # Extensions for SMACH library to integrate it with ROS
from time import sleep # Handle time
import roslaunch
import subprocess
import os

 

class ZED(State):
  def __init__(self):
    State.__init__(self, outcomes=['to_ai'])
 
  def execute(self, userdata):
    rospy.loginfo('Executing state ZED')
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch_file = ['zed_wrapper', 'zed_no_tf.launch']
    roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(launch_file)
    parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
    parent.start() 
    rospy.sleep(15)
    return 'to_ai'
         

 

class AI(State):
  def __init__(self):
    State.__init__(self, outcomes=['to_pe'])
 
  def execute(self, userdata):
    rospy.loginfo('Executing state AI and Computer Vision')
    try:
        print("before")
        #os.system("conda run -n yolov8_env python /home/user/yolo_seg_pca_ros_zedcam_RVIZ_multiple_predictions_tracking.py")
        process = subprocess.Popen(['conda', 'run', '-n', 'yolov8_env', 'python', '/home/user/yolo_seg_pca_ros_zedcam_RVIZ_multiple_predictions_tracking.py'])
        print("after")
        rospy.sleep(60)
        return 'to_pe'
    except subprocess.CalledProcessError:
        rospy.logerr("Failed to run the Conda environment")
        return 'failure'
  
    rospy.sleep(5)
    return 'end'


class POSE_ESTIMATOR(State):
  def __init__(self):
    State.__init__(self, outcomes=['end'])
 
  def execute(self, userdata):
    rospy.loginfo('Executing state AI and Computer Vision')
    try:
        print("before pe")
        #os.system("conda run -n yolov8_env python /home/user/yolo_seg_pca_ros_zedcam_RVIZ_multiple_predictions_tracking.py")
        process = subprocess.Popen([ 'python', '/home/user/catkin_ws/src/yolov5_ros/src/pose_estimator_tf_multiple_track.py'])
        print("after pe")
        rospy.spin()
        return 'end'
    except subprocess.CalledProcessError:
        rospy.logerr("Failed to run the Conda environment")
        return 'failure'
  
    rospy.sleep(5)
    return 'end'

 
# Main method
def main():
 
  # Initialize the node
  rospy.init_node('fsm_turnstile_py')
 
  # Create a SMACH state machine container
  sm = StateMachine(outcomes=['succeeded','failed'])
 
  # Set user data for the finite state machine
  sm.userdata.sm_input = 0
  #sm.userdata.sm_input = input("Enter 1 to Push or 0 to Insert a Coin: ")
     
  # Open the state machine container. A state machine container holds a number of states.
  with sm:
     
    # Add states to the container, and specify the transitions between states
    # For example, if the outcome of state LOCKED is 'coin', then we transition to state UNLOCKED.
    StateMachine.add('ZED_ON', ZED(), transitions={'to_ai':'CV_AI'})
    StateMachine.add('CV_AI', AI(), transitions={'to_pe':'PE'})
    StateMachine.add('PE', POSE_ESTIMATOR(), transitions={'end':'succeeded'})
 
  # View our state transitions using ROS by creating and starting the instrospection server
  sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
  sis.start()
   
  # Execute the state machine 
  outcome = sm.execute()
 
  # Wait for ctrl-c to stop the application
  rospy.spin()
  sis.stop()
 
if __name__ == '__main__':
  main()
