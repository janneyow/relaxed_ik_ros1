#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose
import transformations as T

from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped, Twist
import copy

class user_cmd_to_marker:
    def __init__(self, user_interface):
        self.del_pose_by_user = Pose()
        self.del_pose_by_user.orientation.w = 1.0
        self.active = True     
        if user_interface == "joy":
            self.user_cmd_topic_name = "/joy"
            self.user_cmd_msg_type = Joy
            self.cb_user_cmd = self.cb_user_cmd_joy
            self.add_user_cmd = self.add_user_cmd_ros
            rospy.loginfo("User interface: Joystick with Joy command")
        elif user_interface == "twist":
            self.user_cmd_topic_name = "/joy"
            self.user_cmd_msg_type = TwistStamped
            self.cb_user_cmd = self.cb_user_cmd_twist
            self.add_user_cmd = self.add_user_cmd_ros
            rospy.loginfo("User interface: Joystick with Twist command")
        else:
            self.active = False
            self.user_cmd_topic_name = ""
            self.user_cmd_msg_type = TwistStamped
            rospy.logwarn("No user interface defined")
            exit()

    ## (1) For Joystick (Joy)
    # Takes in Joy message, returns Pose()
    def cb_user_cmd_joy(self, data):     
        linear_scale = 0.000005
        pose = Pose()
        pose.orientation.w = 1.0

        if data.buttons[0] == 0:
            pose.position.x = data.axes[1]*linear_scale
            pose.position.y = data.axes[0]*linear_scale
            pose.position.z = data.axes[4]*linear_scale
        
        # TODO: implement mode switch button, currently need to hold
        else:
            angular_scale = 0.00001
            roll = data.axes[1]*angular_scale
            pitch = data.axes[0]*angular_scale
            yaw = data.axes[4]*angular_scale
            quat = T.quaternion_from_euler(roll,pitch,yaw)
            pose.orientation.w = quat[0]
            pose.orientation.x = quat[1]
            pose.orientation.y = quat[2]
            pose.orientation.z = quat[3]          
 
        self.del_pose_by_user = pose
    
        if data.buttons[7] == 1:
            self.active = False
            rospy.loginfo("Quitting")
        

    ## (3) For Absolute Twist input from AdaTeleop
    # TODO: currently taking Twist message as absolute, should process Twist msg -> See how moveit servo does this
    # position.x = linear.x * time
    def cb_user_cmd_twist(self, data):    
        pose = Pose()
        pose.orientation.w = 1.0

        pose.position.x = data.linear.x
        pose.position.y = data.linear.y
        pose.position.z = data.linear.z

        roll = data.angular.x
        pitch = data.angular.y
        yaw = data.angular.z
        quat = T.quaternion_from_euler(roll,pitch,yaw)
        pose.orientation.w = quat[0]
        pose.orientation.x = quat[1]
        pose.orientation.y = quat[2]
        pose.orientation.z = quat[3]        

        self.del_pose_by_user = pose

    # To adjust the marker's position by adding a user command
    def add_user_cmd_ros(self, pose_by_marker, del_pose_by_user):   
        pose = pose_by_marker # Note: Rviz marker position also changes according to user input
        pose.position.x += del_pose_by_user.position.x
        pose.position.y += del_pose_by_user.position.y
        pose.position.z += del_pose_by_user.position.z
      
        quat_existing = [pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z]
        quat_user = [del_pose_by_user.orientation.w, del_pose_by_user.orientation.x, del_pose_by_user.orientation.y, del_pose_by_user.orientation.z]
        
        new_quat = T.quaternion_multiply(quat_user, quat_existing)
        pose.orientation.w = new_quat[0]
        pose.orientation.x = new_quat[1]
        pose.orientation.y = new_quat[2]
        pose.orientation.z = new_quat[3]

        return pose

    # To adjust the marker's position by adding a user command with a Twist message
    # TODO: check why even need this?
    def add_user_cmd_unity(self, pose_by_marker, del_pose_by_user):     
        if(del_pose_by_user.position.x**2 + del_pose_by_user.position.y**2 + del_pose_by_user.position.z**2 + (del_pose_by_user.orientation.w - 1.0)**2 + del_pose_by_user.orientation.x**2 + del_pose_by_user.orientation.y**2 + del_pose_by_user.orientation.z**2 == 0.0):
            self.ini_pose_by_marker = copy.deepcopy(pose_by_marker)            
   
        pose = copy.deepcopy(self.ini_pose_by_marker)
        pose.position.x += del_pose_by_user.position.x
        pose.position.y += del_pose_by_user.position.y
        pose.position.z += del_pose_by_user.position.z
      
        quat_existing = [pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z]
        quat_user = [del_pose_by_user.orientation.w, del_pose_by_user.orientation.x, del_pose_by_user.orientation.y, del_pose_by_user.orientation.z]
        
        new_quat = T.quaternion_multiply(quat_user, quat_existing)
        pose.orientation.w = new_quat[0]
        pose.orientation.x = new_quat[1]
        pose.orientation.y = new_quat[2]
        pose.orientation.z = new_quat[3]

        # Update "pose_by_marker"
        pose_by_marker_update = pose_by_marker      # Note: pose_by_marker is the position where the user last dropped the Rviz marker
        pose_by_marker_update.position.x = pose.position.x
        pose_by_marker_update.position.y = pose.position.y
        pose_by_marker_update.position.z = pose.position.z
      
        pose_by_marker_update.orientation.w = new_quat[0]
        pose_by_marker_update.orientation.x = new_quat[1]
        pose_by_marker_update.orientation.y = new_quat[2]
        pose_by_marker_update.orientation.z = new_quat[3]        

        return pose



    def start_sub_user_cmd(self): 
        rospy.Subscriber(self.user_cmd_topic_name, self.user_cmd_msg_type, self.cb_user_cmd)