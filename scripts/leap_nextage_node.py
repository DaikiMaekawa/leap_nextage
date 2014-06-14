#!/usr/bin/env python

#############################################################################################
 # Copyright (c) 2014 Daiki Maekawa and ROS JAPAN Users Group All Rights Reserved.         #
 #                                                                                         #
 # @file moveit_command_sender.py                                                          #
 # @brief This program will run you through using python interface to the move_group node. #
 # @author Daiki Maekawa                                                                   #
 # @date 2014-06-10                                                                        #
#############################################################################################

import moveit_commander
import rospy
import geometry_msgs.msg
import copy
import tf
import math
import threading

from moveit_commander.exception import MoveItCommanderException
from leap_motion2.msg import Hand

import Leap

class LeapReceiver(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        rospy.Subscriber("leapmotion2/data", Hand, self.__callback_hands_data)
        self.__hand_data = None

    def __callback_hands_data(self, msg):
        rospy.loginfo(rospy.get_name() + ": Leap ROS Data %s" % msg)
        self.__hand_data = msg

    @property
    def hand_data(self):
        return self.__hand_data

class LeapNextage(object):
    R_HAND_HOME_X = 0
    R_HAND_HOME_Y = 1.0
    R_HAND_HOME_Z = 0
            
    def __init__(self):
        self.__robot = moveit_commander.RobotCommander()
        self.__rarm = moveit_commander.MoveGroupCommander("right_arm")
        self.__larm = moveit_commander.MoveGroupCommander("left_arm")
        self.__waypoints = []

        self.__rarm_start_pose = self.__larm_start_pose = geometry_msgs.msg.Pose()
        
        target_pose_r = target_pose_l = geometry_msgs.msg.Pose()
        target_pose_r.position.x = 0.2035
        target_pose_r.position.y = -0.45
        target_pose_r.position.z = 0.0709
        
        target_pose_r.orientation.x = -0.003
        target_pose_r.orientation.y = -0.708
        target_pose_r.orientation.z = -0.003
        target_pose_r.orientation.w = 0.706
                
        self.__rarm.set_pose_target(target_pose_r)
        self.__rarm.go()

        target_pose_l.position.y = -target_pose_r.position.y
        self.__larm.set_pose_target(target_pose_l)
        self.__larm.go()

        self.__leap_receiver = LeapReceiver()
        self.__leap_receiver.setDaemon(True)
        self.__leap_receiver.start()

    """
    def __callback_hands_data(self, msg):
        rospy.loginfo(rospy.get_name() + ": Leap ROS Data %s" % msg)
        self.__hand_data = msg
    """

    def run(self):
        while not rospy.is_shutdown():
            if self.__leap_receiver.hand_data:
                hand_data = self.__leap_receiver.hand_data
                pose = self.__rarm.get_current_pose().pose

                print "diff.x = ", hand_data.ypr.y * 0.001
                print "diff.y = ", hand_data.ypr.x * 0.001

                pose.position.x += -hand_data.ypr.x * 0.001
                pose.position.y += hand_data.ypr.y * 0.001

                print "ypr: "
                print hand_data.ypr
                self.__rarm.set_pose_target(pose)
                self.__rarm.go(wait = False)
                rospy.sleep(2)

        
        """
        waypoints = []
        future_pose = None
        pose_count = 0
        while not rospy.is_shutdown():
            if self.__leap_receiver.hand_data:
                hand_data = self.__leap_receiver.hand_data
                
                if not future_pose:
                    pose = self.__rarm.get_current_pose().pose
                else:
                    pose = future_pose
                
                #pose.position.y += 0.01

                q = tf.transformations.quaternion_from_euler(hand_data.ypr.z, hand_data.ypr.y, hand_data.ypr.x)
                #q = tf.transformations.quaternion_from_euler(3.14, -1.57, -3.14)
                pose.orientation.x = q[0]
                pose.orientation.y = q[1]
                pose.orientation.z = q[2]
                pose.orientation.w = q[3]

                pose_count += 1
                
                if pose_count >= 1:
                    self.__rarm.set_pose_target(pose)
                    self.__rarm.go(wait = False)
                    rospy.sleep(1)
                    pose_count = 0
                    future_pose = None
                else:
                    future_pose = pose

            rospy.sleep(0.3)
        """

if __name__ == '__main__':
    try:
        rospy.init_node("moveit_command_sender")
        leap_nextage = LeapNextage()
        leap_nextage.run()
    except MoveItCommanderException:
        pass

    except rospy.ROSInterruptException:
        pass

