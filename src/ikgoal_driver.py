#!/usr/bin/python3

from turtle import pos
import readchar
import rospy
from geometry_msgs.msg import PoseStamped, Vector3Stamped, QuaternionStamped, Pose
from std_msgs.msg import Bool
from relaxed_ik_ros1.msg import EEPoseGoals
import transformations as T
from teleop import user_cmd_to_marker

_user_cmd_to_marker = user_cmd_to_marker(user_interface = "joy")
if(_user_cmd_to_marker.active is True):
    _user_cmd_to_marker.start_sub_user_cmd()


rospy.init_node('ikgoal_driver')

ee_pose_goals_pub = rospy.Publisher('/relaxed_ik/ee_pose_goals', EEPoseGoals, queue_size=5)
quit_pub = rospy.Publisher('/relaxed_ik/quit',Bool,queue_size=5)

position = [0,0,0]
rotation = [1,0,0,0]

seq = 1
rate = rospy.Rate(1000)
pose_by_marker = Pose()
pose_by_marker.orientation.w = 1
while not rospy.is_shutdown():

    pose_goal = _user_cmd_to_marker.add_user_cmd(pose_by_marker, _user_cmd_to_marker.del_pose_by_user) # Adding del pose input via additional user interface  
    
    position[0] = pose_goal.position.x
    position[1] = pose_goal.position.y
    position[2] = pose_goal.position.z

    rotation[0] = pose_goal.orientation.w
    rotation[1] = pose_goal.orientation.x
    rotation[2] = pose_goal.orientation.y
    rotation[3] = pose_goal.orientation.z
    # print("Pose: {}, {}".format(position, rotation))

    ee_pose_goals = EEPoseGoals()

    ee_pose_goals.ee_poses.append(pose_goal)

    ee_pose_goals.header.seq = seq
    seq += 1
    ee_pose_goals_pub.publish(ee_pose_goals)

    if _user_cmd_to_marker.active == False:
        print("Exit")
        q = Bool()
        q.data = False
        quit_pub.publish(q)     # nothing subscribes to this yet
        rospy.signal_shutdown()
