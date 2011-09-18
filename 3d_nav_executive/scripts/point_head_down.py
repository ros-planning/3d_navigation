#! /usr/bin/env python
# -*- coding: utf-8 -*-

import roslib; roslib.load_manifest('pr2_teleop_general')
import rospy

import trajectory_msgs
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


import actionlib
from pr2_controllers_msgs.msg import PointHeadGoal,PointHeadAction
from geometry_msgs.msg import PointStamped

if __name__ == '__main__':

	rospy.init_node('point_head_down')
  	pub = rospy.Publisher("/head_traj_controller/command", JointTrajectory)
  	
  	goal = PointHeadGoal()
  	goal.target.header.frame_id = "base_link";
	goal.target.point.x = 1.5; 
	goal.target.point.y = 0; 
	goal.target.point.z = 0.0;
	goal.pointing_frame = "high_def_frame";
	goal.pointing_axis.x = 1;
	goal.pointing_axis.y = 0;
	goal.pointing_axis.z = 0;
	
	point_head_client = actionlib.SimpleActionClient("/head_traj_controller/point_head_action", PointHeadAction)
	point_head_client.wait_for_server()
	point_head_client.send_goal(goal)
  	
  	
