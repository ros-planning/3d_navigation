#! /usr/bin/env python
# -*- coding: utf-8 -*-

import roslib; roslib.load_manifest('pr2_teleop_general')
import rospy

import trajectory_msgs
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


import actionlib
from pr2_controllers_msgs.msg import PointHeadGoal,PointHeadAction
from geometry_msgs.msg import PointStamped, Twist
from math import fabs, copysign, cos, sin, atan2


class PointHeadDriving:
  def __init__(self):
    rospy.init_node('point_head_driving')
    
    
    self.rad = 1.5
    self.k_theta = 5
    self.thres_xy = 0.03
    self.thres_th = 0.03
    self.lastAngle = 0.0
    
    self.goal = PointHeadGoal()
    self.goal.target.header.frame_id = "base_link";
    self.goal.target.point.x = self.rad; 
    self.goal.target.point.y = 0; 
    self.goal.target.point.z = 0.0;
    self.goal.pointing_frame = "high_def_frame";
    self.goal.pointing_axis.x = 1;
    self.goal.pointing_axis.y = 0;
    self.goal.pointing_axis.z = 0;
    self.goal.max_velocity=1.0;
    
    self.point_head_client = actionlib.SimpleActionClient("/head_traj_controller/point_head_action", PointHeadAction)
    self.point_head_client.wait_for_server()
    self.twistSub = rospy.Subscriber("/base_controller/command", Twist, self.twistCallback,queue_size=1)
    
    self.point_head_client.send_goal(self.goal)
    rospy.loginfo("point_head_driving.py initialized")

    

  def twistCallback(self,data):
    if fabs(data.linear.x) < self.thres_xy:
      data.linear.x = 0.0
    if fabs(data.linear.y) < self.thres_xy:
      data.linear.y = 0.0  
    if fabs(data.angular.z) < self.thres_th:
      data.angular.z = 0.0
      
    if fabs(data.linear.x) < self.thres_xy and fabs(data.linear.y) < self.thres_xy and fabs(data.angular.z) < self.thres_th:
      return

    theta = atan2(data.linear.y, data.linear.x)
    
    theta = theta + data.angular.z * self.k_theta
    
    if (data.linear.x >= -1.0*self.thres_xy):
      theta = max(-1.6, theta)
      theta = min(1.6, theta)
      
    if (fabs(theta) > 2.9):
      theta = copysign(theta, self.lastAngle)
    
    rospy.loginfo("Pointing angle: %f", theta)
    
    self.goal.target.point.x = self.rad * cos(theta)
    self.goal.target.point.y = self.rad * sin(theta)
    self.point_head_client.send_goal(self.goal)
    self.lastAngle = theta
    rospy.loginfo("Pointing to %f %f", self.goal.target.point.x, self.goal.target.point.y)
    


if __name__ == '__main__':
  node = PointHeadDriving()
  rospy.spin()
  
  exit(0)

