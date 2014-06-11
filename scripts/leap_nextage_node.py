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

from moveit_commander.exception import MoveItCommanderException
from leap_motion2.msg import Hand

class LeapNextage(object):
    MAX_ARM_POS_X = 0.4
    MIN_ARM_POS_X = 0.1
    MAX_ARM_POS_Y = -0.1
    MIN_ARM_POS_Y = -0.3
    MAX_ARM_POS_Z = 0.4
    MIN_ARM_POS_Z = -0.1
    
    MAX_LEAP_POS_X = 1.0
    MIN_LEAP_POS_X = -1.0
    MAX_LEAP_POS_Y = 2.0
    MIN_LEAP_POS_Y = 0.3
    MAX_LEAP_POS_Z = 1.0
    MIN_LEAP_POS_Z = -1.0
    
    def __init__(self):
        rospy.init_node("moveit_command_sender")
        rospy.Subscriber("leapmotion2/data", Hand, self.__callback_hands_data)
        self.__robot = moveit_commander.RobotCommander()

        self.__rarm = moveit_commander.MoveGroupCommander("right_arm")
        self.__larm = moveit_commander.MoveGroupCommander("left_arm")
        self.__waypoints = []

        self.__rarm_start_pose = self.__larm_start_pose = geometry_msgs.msg.Pose()
        
        target_pose_r = target_pose_l = geometry_msgs.msg.Pose()
        target_pose_r.position.x = 0.2035
        target_pose_r.position.y = -0.25
        target_pose_r.position.z = 0.0709
        target_pose_r.orientation.x = 0.000427
        target_pose_r.orientation.y = 0.000317
        target_pose_r.orientation.z = -0.000384
        target_pose_r.orientation.w = 0.999999
        self.__rarm.set_pose_target(target_pose_r)
        self.__rarm.go()

        target_pose_l.position.y = -target_pose_r.position.y
        self.__larm.set_pose_target(target_pose_l)
        self.__larm.go()

        self.__old_hand_data = None
        self.__hand_data = None
    
    def __callback_hands_data(self, msg):
        rospy.loginfo(rospy.get_name() + ": Leap ROS Data %s" % msg)
        self.__hand_data = msg

        """
        if not self.__old_hand_data:
            self.__old_hand_data = msg
            return

        wpose = geometry_msgs.msg.Pose()
        pos = self.__rarm.get_current_pose().pose.position

        wpose.position.x = pos.x + (msg.palmpos.z - self.__old_hand_data.palmpos.z) * 0.01
        wpose.position.y = pos.y + (msg.palmpos.x - self.__old_hand_data.palmpos.x) * 0.01
        wpose.position.z = pos.z + (msg.palmpos.y - self.__old_hand_data.palmpos.y) * 0.01
        wpose.orientation.x = -0.003
        wpose.orientation.y = -0.708
        wpose.orientation.z = -0.003
        wpose.orientation.w = 0.706
        
        wpose.position.x = wpose.position.x if self.MIN_ARM_POS_X < wpose.position.x < self.MAX_ARM_POS_X else pos.x
        wpose.position.y = wpose.position.y if self.MIN_ARM_POS_Y < wpose.position.y < self.MAX_ARM_POS_Y else pos.y
        wpose.position.z = wpose.position.z if self.MIN_ARM_POS_Z < wpose.position.z < self.MAX_ARM_POS_Z else pos.z
        
        self.__rarm.clear_pose_targets()
        self.__rarm.set_pose_target(wpose)
        self.__rarm.go()
        """

        """
        self.__waypoints.append(copy.deepcopy(wpose))
        
        if len(self.__waypoints) > 5: self.__rarm.clear_pose_targets()
            (plan, fraction) = self.__rarm.compute_cartesian_path(self.__waypoints, 0.05, 0.0)
            self.__rarm.execute(plan)
            print "=" * 10, " plan..."
            self.__waypoints = []

        self.__old_hands_data = msg
        """

    def run(self):
        rate = rospy.Rate(20)
        old_hand_data = None
        waypoints = []

        while not rospy.is_shutdown():
            rate.sleep()
            if self.__hand_data:
                if not old_hand_data:
                    old_hand_data = self.__hand_data
                    continue
                    
                hand_data = self.__hand_data
                wpose = geometry_msgs.msg.Pose()
                pos = self.__rarm.get_current_pose().pose.position

                wpose.position.x = pos.x + (hand_data.palmpos.z - old_hand_data.palmpos.z) * 0.01
                wpose.position.y = pos.y + (hand_data.palmpos.x - old_hand_data.palmpos.x) * 0.01
                wpose.position.z = pos.z + (hand_data.palmpos.y - old_hand_data.palmpos.y) * 0.01
                wpose.orientation.x = -0.003
                wpose.orientation.y = -0.708
                wpose.orientation.z = -0.003
                wpose.orientation.w = 0.706
                
                #wpose.position.x = wpose.position.x if self.MIN_ARM_POS_X < wpose.position.x < self.MAX_ARM_POS_X else pos.x
                #wpose.position.y = wpose.position.y if self.MIN_ARM_POS_Y < wpose.position.y < self.MAX_ARM_POS_Y else pos.y
                #wpose.position.z = wpose.position.z if self.MIN_ARM_POS_Z < wpose.position.z < self.MAX_ARM_POS_Z else pos.z
                
                if len(waypoints) < 10:
                    waypoints.append(wpose)
                else:
                    self.__rarm.clear_pose_targets()
                    (plan, fraction) = self.__rarm.compute_cartesian_path(waypoints, 0.05, 0.0)
                    self.__rarm.execute(plan)
                    del waypoints[:]
                    
                old_hand_data = hand_data
        
        #rospy.spin()

if __name__ == '__main__':
    try:
        leap_nextage = LeapNextage()
        leap_nextage.run()
    except MoveItCommanderException:
        pass

    except rospy.ROSInterruptException:
        pass

