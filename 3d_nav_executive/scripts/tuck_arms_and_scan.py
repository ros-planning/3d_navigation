#! /usr/bin/env python
# -*- coding: utf-8 -*-

import roslib; roslib.load_manifest('pr2_teleop_general')
import rospy

import trajectory_msgs
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


import actionlib
from pr2_controllers_msgs.msg import PointHeadGoal,PointHeadAction
from geometry_msgs.msg import PointStamped

from pr2_common_action_msgs.msg import TuckArmsGoal,TuckArmsAction

if __name__ == '__main__':

	rospy.init_node('tuck_arms_and_scan')

	rospy.loginfo("Tucking arms...")
	tuck_arm_client = actionlib.SimpleActionClient("tuck_arms", TuckArmsAction)
	goal = TuckArmsGoal()
	goal.tuck_left = True
	goal.tuck_right = True
	tuck_arm_client.wait_for_server(rospy.Duration(5.0))
	tuck_arm_client.send_goal(goal)
	tuck_arm_client.wait_for_result(rospy.Duration.from_sec(30.0))

	#tuck_arm_client.send_goal_and_wait(goal, rospy.Duration(30.0), rospy.Duration(5.0))
	
	rospy.loginfo("Head scanning... ")
	goalH = PointHeadGoal()
	goalH.target.header.frame_id = "base_link";
	goalH.target.point.x = 1.7; 
	goalH.target.point.y = -3.0; 
	goalH.target.point.z = 0.0;
	goalH.pointing_frame = "high_def_frame";
	goalH.pointing_axis.x = 1;
	goalH.pointing_axis.y = 0;
	goalH.pointing_axis.z = 0;
	
	point_head_client = actionlib.SimpleActionClient("/head_traj_controller/point_head_action", PointHeadAction)
	point_head_client.wait_for_server()
	point_head_client.send_goal(goalH)
	point_head_client.wait_for_result(rospy.Duration.from_sec(5.0))
	rospy.sleep(1.0)
	
	goalH.target.point.y = -1.0
	point_head_client.send_goal(goalH)
	point_head_client.wait_for_result(rospy.Duration.from_sec(5.0))
	rospy.sleep(1.0)
	
	goalH.target.point.y = 1.0
	point_head_client.send_goal(goalH)
	point_head_client.wait_for_result(rospy.Duration.from_sec(5.0))
	rospy.sleep(1.0)
	
	goalH.target.point.y = 3.0
	point_head_client.send_goal(goalH)
	point_head_client.wait_for_result(rospy.Duration.from_sec(5.0))
	rospy.sleep(1.0)
	
	goalH.target.point.y = 0.0
	point_head_client.send_goal(goalH)
	point_head_client.wait_for_result(rospy.Duration.from_sec(5.0))
	rospy.sleep(1.0)
	
	rospy.loginfo("Untucking arms...")
	goal.tuck_left = False
	goal.tuck_right = False
	tuck_arm_client.send_goal(goal)
	tuck_arm_client.wait_for_result(rospy.Duration.from_sec(30.0))

