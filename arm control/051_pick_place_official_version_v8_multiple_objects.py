# Python 2/3 compatibility imports
from __future__ import print_function
from cgi import test
from six.moves import input

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
from sensor_msgs.msg import Imu

from geometry_msgs.msg import PoseStamped

import actionlib
from control_msgs.msg import GripperCommandAction, GripperCommandActionFeedback, GripperCommandResult, GripperCommandGoal
import tf
from std_msgs.msg import Int32, Float32

try:
    from math import pi, tau, dist, fabs, cos, radians
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt, radians

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

pub_angle=rospy.Publisher("angle_pose",Float32, queue_size=1)
pub_request_angle=rospy.Publisher("activate",Int32, queue_size=1)


moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("move_group_BIOBLU_picknPLACE", anonymous=True)
listener = tf.TransformListener()
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "manipulator"
move_group = moveit_commander.MoveGroupCommander(group_name)
display_trajectory_publisher = rospy.Publisher(
    "/move_group/display_planned_path",
    moveit_msgs.msg.DisplayTrajectory,
    queue_size=20,
)

# Initialize gripper actionserver
gripper = actionlib.SimpleActionClient('onrobot_gripper_action', GripperCommandAction)
print('connection to gripper')
gripper.wait_for_server()

def openGripper(dimension):
    gripper_goal = GripperCommandGoal()
    gripper_goal.command.position = dimension
    gripper_goal.command.max_effort = 40.0
    gripper.send_goal(gripper_goal)
    gripper.wait_for_result()
    print(gripper.get_result())



# We can get the name of the reference frame for this robot:
planning_frame = move_group.get_planning_frame()
print("============ Planning frame: %s" % planning_frame)

# We can also print the name of the end-effector link for this group:
eef_link = move_group.get_end_effector_link()
print("============ End effector link: %s" % eef_link)

# We can get a list of all the groups in the robot:
group_names = robot.get_group_names()
print("============ Available Planning Groups:", robot.get_group_names())

# Sometimes for debugging it is useful to print the entire state of the
# robot:
print("============ Printing robot state")
print(robot.get_current_state())
print("")


p = PoseStamped()
p.header.frame_id = planning_frame
p.pose.position.z = -0.76/2
scene.add_box("turi_chassis", p, (1.0, 1.0, 0.76))

top_of_AOI = move_group.get_current_joint_values()


top_of_AOI = [0, -0.7797770661166687, 0.5476763884173792, -1.3394437295249482, -1.5682809988604944, -0.07637578645815069]

taoi2=[-0.25449926057924444, -1.1534841519645234, 1.5105436483966272, -1.937953611413473, -1.566495720540182, -0.3282888571368616]

home =[0.03176402300596237, -2.5033604107298792, 2.773778740559713, -3.4929057560362757, -1.6299508253680628, -0.028481308613912404]
# home = [0, -2.1378003559508265, 2.681972328816549, -3.0, -1.5927050749408167, -0.026705090199605763]

top_of_box = [-3.0792575518237513, -2.248068471948141, 2.5582273642169397, -3.532614847222799, -1.6746795813189905, -0.0324633757220667]

while True: # CHANGES FOR MULTIPLE GRASP

    move_group.go(home, wait=True)
    # exit()


    rospy.sleep(2.0)

    openGripper(75.0)

    move_group.go(top_of_AOI, wait=True)


    move_group.go(taoi2, wait=True)


    print("---> gripper in top AOI")

    # exit()
    rospy.sleep(10)

    try:
        now = listener.getLatestCommonTime('/obj_detected', '/base_link')
        (trans,rot) = listener.lookupTransform('/base_link', '/obj_detected', now)
        print("obtained from first transform")
        print((trans,rot))
    except:
        print("TRANSFORMATION NOT AVAILABLE: maybe there isn't any object")
        break

    
    print("*********************************************")

    if(trans[1]<-0.4 or trans[1]>0.4):
            print("out of y-workspace, exit from p&p!")
            break
    
    



    waypoints = []
    wpose = move_group.get_current_pose().pose
    #print(move_group.get_current_pose())
    wpose.position.x =  trans[0]-0.2
    wpose.position.y =  trans[1]
    wpose.position.z = +0.25

    waypoints.append(copy.deepcopy(wpose))

    print("first movement")
    print(waypoints)



    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
    (plan, fraction) = move_group.compute_cartesian_path(
                                    waypoints,   # waypoints to follow
                                    0.01,        # eef_step
                                    0.0)         # jump_threshold

    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    # Publish
    display_trajectory_publisher.publish(display_trajectory)

    move_group.execute(plan, wait=True)
    move_group.stop()

    # exit()

    rospy.sleep(5)

    print("*********************************************")

    try:
        now = listener.getLatestCommonTime('/obj_detected', '/base_link')
        (trans,rot) = listener.lookupTransform('/base_link', '/obj_detected', now)
        print("obtained from first transform")
        print((trans,rot))
    except:
        print("TRANSFORMATION NOT AVAILABLE: maybe there isn't any object")
        break

    print("*********************************************")

    if(trans[1]<-0.4 or trans[1]>0.4):
            print("out of y-workspace, exit from p&p!")
            break

    #ROTATION
    waypoints = []

    wpose = move_group.get_current_pose().pose
    wpose.position.x =  trans[0]-0.05
    wpose.position.y =  trans[1]
    #wpose.position.z = 0.0
    wpose.orientation.x =  rot[0]
    wpose.orientation.y =  rot[1]
    wpose.orientation.z = rot[2]
    wpose.orientation.w = rot[3]
    waypoints.append(copy.deepcopy(wpose))

    print("---waypoint rotation")
    print(waypoints)


    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
    (plan, fraction) = move_group.compute_cartesian_path(
                                    waypoints,   # waypoints to follow
                                    0.01,        # eef_step
                                    0.0)         # jump_threshold

    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    # Publish
    display_trajectory_publisher.publish(display_trajectory)

    move_group.execute(plan, wait=True)
    move_group.stop()

    rospy.sleep(1)

    #GO DOWN
    waypoints = []
    wpose = move_group.get_current_pose().pose
    wpose.position.z = max(trans[2],-0.06)
    waypoints.append(copy.deepcopy(wpose))  

    print("---waypoint go down")
    (plan, fraction) = move_group.compute_cartesian_path(
                                    waypoints,   # waypoints to follow
                                    0.01,        # eef_step
                                    0.0)         # jump_threshold

    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    # Publish
    display_trajectory_publisher.publish(display_trajectory)

    move_group.execute(plan, wait=True)
    move_group.stop()

    rospy.sleep(1)

    openGripper(22.0)

    move_group.go(top_of_AOI, wait=True)

    move_group.go(home, wait=True)

    move_group.go(top_of_box, wait=True)

    openGripper(75.0)

    move_group.go(home, wait=True)


    move_group.stop()
    listener.clear()

# CHANGES FOR MULTIPLE GRASP
move_group.go(home, wait=True)
print("CODE FINISHED")
exit()

