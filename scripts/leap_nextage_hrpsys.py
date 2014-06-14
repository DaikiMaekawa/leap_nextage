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

import numpy
from nextage_ros_bridge import nextage_client
from hrpsys_ros_bridge.srv import *
from tf.transformations import euler_from_matrix

class NextageUtil(object):
    def __init__(self):
        self.init_pose = nextage_client.NextageClient.InitialPose
        self.off_pose = nextage_client.NextageClient.OffPose

        # degrees to radians
        for angles in self.init_pose:
            for i, a in enumerate(angles):
                angles[i] = math.radians(a)
        for angles in self.off_pose:
            for i, a in enumerate(angles):
                angles[i] = math.radians(a)
        rospy.loginfo('start setup services')
        rospy.wait_for_service('/SequencePlayerServiceROSBridge/setTargetPose')
        rospy.loginfo('/SequencePlayerServiceROSBridge/setTargetPose is ready')
        self.set_target_pose = rospy.ServiceProxy(
            '/SequencePlayerServiceROSBridge/setTargetPose',
            OpenHRP_SequencePlayerService_setTargetPose)
        self.set_joint_angles_of_group = rospy.ServiceProxy(
            '/SequencePlayerServiceROSBridge/setJointAnglesOfGroup',
            OpenHRP_SequencePlayerService_setJointAnglesOfGroup)

        rospy.wait_for_service('/ForwardKinematicsServiceROSBridge/getCurrentPose')
        rospy.loginfo('/SequencePlayerServiceROSBridge/getCurrentPose is ready')
        self.get_current_pose = rospy.ServiceProxy(
            '/ForwardKinematicsServiceROSBridge/getCurrentPose',
            OpenHRP_ForwardKinematicsService_getCurrentPose)

    def set_target_pose_relative(self, name, delta_xyz, delta_rpy, tm):
        if name.lower() == 'rarm':
            joint = 'RARM_JOINT5'
        elif name.lower() == 'larm':
            joint = 'LARM_JOINT5'
        else:
            raise rospy.ServiceException()

        matrix = self.get_current_pose(joint).pose.data
        xyz = numpy.array([matrix[3], matrix[7], matrix[11]])
        rpy = numpy.array(euler_from_matrix([matrix[0:3], matrix[4:7], matrix[8:11]], 'sxyz'))
        xyz += [delta_xyz[0], delta_xyz[1], delta_xyz[2]]
        rpy += [delta_rpy[0], delta_rpy[1], delta_rpy[2]]
        return self.set_target_pose(name, list(xyz), list(rpy), tm)

    def go_pose(self, pose, tm):
        self.set_joint_angles_of_group('torso', pose[0], tm)
        self.set_joint_angles_of_group('head', pose[1], tm)
        self.set_joint_angles_of_group('rarm', pose[2], tm)
        return self.set_joint_angles_of_group('larm', pose[3], tm)

    def go_initial(self, tm):
        return self.go_pose(self.init_pose, tm)

    def go_off_pose(self, tm):
        return self.go_pose(self.off_pose, tm)

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
        self.__leap_receiver = LeapReceiver()
        self.__leap_receiver.setDaemon(True)
        self.__leap_receiver.start()
        self.__util = NextageUtil()

    def run(self):
        self.__util.go_initial(2.0)
        rospy.sleep(2.0)
        
        while not rospy.is_shutdown():
            if self.__leap_receiver.hand_data:
                hand_data = self.__leap_receiver.hand_data
                if hand_data.is_left.data:
                    hand_name = "larm"
                else:
                    hand_name = "rarm"
                
                print "diff.x = ", hand_data.ypr.y * 0.001
                print "diff.y = ", hand_data.ypr.x * 0.001
                diff_x = -hand_data.ypr.x * 0.001
                diff_y = hand_data.ypr.y * 0.001
                
                self.__util.set_target_pose_relative(hand_name, [diff_x, diff_y, 0], [0, 0, 0], 1.0)
                rospy.sleep(1.0)

                #pose.position.x += -hand_data.ypr.x * 0.001
                #pose.position.y += hand_data.ypr.y * 0.001

                #print "ypr: "
                #print hand_data.ypr
                #self.__rarm.set_pose_target(pose)
                #self.__rarm.go(wait = False)
                #rospy.sleep(2)
        
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
        #util = NextageUtil()
        
        #util.go_initial(2.0)
        #rospy.sleep(2.0)
        #while True:
        #    util.set_target_pose_relative("rarm", [0, 0, 0.1], [0, 0, 0], 2.0)
        #    rospy.sleep(2.0)

        leap_nextage = LeapNextage()
        leap_nextage.run()
    except MoveItCommanderException:
        pass

    except rospy.ROSInterruptException:
        pass

