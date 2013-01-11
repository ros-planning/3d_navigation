/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2011, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Armin Hornung, based on the pose_follower
*********************************************************************/
#include <pose_follower_3d/pose_follower_3d.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_DECLARE_CLASS(pose_follower_3d, PoseFollower3D, pose_follower_3d::PoseFollower3D, nav_core::BaseLocalPlanner)

namespace pose_follower_3d
{

PoseFollower3D::PoseFollower3D(): tf_(NULL),
                                  costmap_ros_(NULL),
                                  collision_model_3d_("robot_description"),
                                  kinematic_state_(NULL),
                                  collisions_received_(false)
{}

void PoseFollower3D::initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros)
{
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  ROS_INFO("Local planner name: %s", name.c_str());
  current_waypoint_ = 0;
  goal_reached_time_ = ros::Time::now();
  ros::NodeHandle node_private("~/" + name);

  kinematic_state_ = new planning_models::KinematicState(collision_model_3d_.getKinematicModel());
  collision_planner_.initialize(name, tf_, costmap_ros_);

  node_private.param("k_trans", K_trans_, 1.5);
  node_private.param("k_rot", K_rot_, 1.25);

  node_private.param("tolerance_trans", tolerance_trans_, 0.05);
  node_private.param("tolerance_rot", tolerance_rot_, 0.1);
  node_private.param("tolerance_timeout", tolerance_timeout_, 0.5);

  node_private.param("holonomic", holonomic_, true);

  node_private.param("samples", samples_, 5);

  node_private.param("max_vel_lin", max_vel_lin_, 0.5);
  node_private.param("max_vel_th", max_vel_th_, 0.7);

  node_private.param("min_vel_lin", min_vel_lin_, 0.1);
  node_private.param("min_vel_th", min_vel_th_, 0.0);
  node_private.param("min_in_place_vel_th", min_in_place_vel_th_, 0.0);
  node_private.param("in_place_trans_vel", in_place_trans_vel_, 0.0);

  node_private.param("trans_stopped_velocity", trans_stopped_velocity_, 1e-4);
  node_private.param("rot_stopped_velocity", rot_stopped_velocity_, 1e-4);
  node_private.param("sim_time", sim_time_, 1.0);
  node_private.param("sim_granularity", sim_granularity_, 0.1);

  ros::NodeHandle node;
  odom_sub_ = node.subscribe<nav_msgs::Odometry>("odom", 1, boost::bind(&PoseFollower3D::odomCallback, this, _1));
  collisions_sub_ = node.subscribe<mapping_msgs::CollisionObject>("octomap_collision_object", 1,
                                                                  boost::bind(&PoseFollower3D::collisionsCallback, this, _1));
  vel_pub_ = node.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  robot_state_client_ = node.serviceClient<planning_environment_msgs::GetRobotState>("/environment_server/get_robot_state");

  ROS_DEBUG("Initialized");
}

void PoseFollower3D::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  //we assume that the odometry is published in the frame of the base
  boost::mutex::scoped_lock lock(odom_lock_);
  base_odom_.twist.twist.linear.x = msg->twist.twist.linear.x;
  base_odom_.twist.twist.linear.y = msg->twist.twist.linear.y;
  base_odom_.twist.twist.angular.z = msg->twist.twist.angular.z;
  ROS_DEBUG("In the odometry callback with velocity values: (%.2f, %.2f, %.2f)",
            base_odom_.twist.twist.linear.x, base_odom_.twist.twist.linear.y, base_odom_.twist.twist.angular.z);
}

void PoseFollower3D::attachedCallback(const mapping_msgs::AttachedCollisionObjectConstPtr& msg)
{
  ROS_DEBUG("AttachedCollisionObjects received");
  boost::mutex::scoped_lock lock(collisions_lock_);

  // kinematic state needs to be freed because of mutex lock
  if( kinematic_state_ )
  {
    delete kinematic_state_;
    kinematic_state_ = NULL;
  }
  collision_model_3d_.addAttachedObject(*msg);
  kinematic_state_ = new planning_models::KinematicState(collision_model_3d_.getKinematicModel());
}

void PoseFollower3D::collisionsCallback(const mapping_msgs::CollisionObjectConstPtr& msg)
{
  ROS_DEBUG("CollisionObject in PoseFollower3D received, storing locally");

  if(msg->header.frame_id != costmap_ros_->getGlobalFrameID()){
    //ROS_WARN("poser_follower_3d: Collision map and costmap in different frames (%s / %s)", msg->header.frame_id.c_str(), costmap_ros_->getGlobalFrameID().c_str());
  }
  boost::mutex::scoped_lock lock(collisions_lock_);
  // TODO: only copy here, addStaticObject later?
  //collision_object_ = *msg;

  collision_model_3d_.addStaticObject(*msg);

  // ignore collisions by the wheels:
  collision_space::EnvironmentModel::AllowedCollisionMatrix acm = collision_model_3d_.getCurrentAllowedCollisionMatrix();
  acm.changeEntry("fl_caster_l_wheel_link", true);
  acm.changeEntry("fl_caster_r_wheel_link", true);
  acm.changeEntry("fl_caster_rotation_link", true);
  acm.changeEntry("bl_caster_l_wheel_link", true);
  acm.changeEntry("bl_caster_r_wheel_link", true);
  acm.changeEntry("bl_caster_rotation_link", true);
  acm.changeEntry("fr_caster_l_wheel_link", true);
  acm.changeEntry("fr_caster_r_wheel_link", true);
  acm.changeEntry("fr_caster_rotation_link", true);
  acm.changeEntry("br_caster_l_wheel_link", true);
  acm.changeEntry("br_caster_r_wheel_link", true);
  acm.changeEntry("br_caster_rotation_link", true);

  collision_model_3d_.setAlteredAllowedCollisionMatrix(acm);

  collisions_received_ = true;
  collision_object_time_ = msg->header.stamp;
}

double PoseFollower3D::headingDiff(double x, double y, double pt_x, double pt_y, double heading)
{
  double v1_x = x - pt_x;
  double v1_y = y - pt_y;
  double v2_x = cos(heading);
  double v2_y = sin(heading);

  double perp_dot = v1_x * v2_y - v1_y * v2_x;
  double dot = v1_x * v2_x + v1_y * v2_y;

  //get the signed angle
  double vector_angle = atan2(perp_dot, dot);

  return -1.0 * vector_angle;
}

bool PoseFollower3D::checkTrajectory3D(double x, double y, double theta, double vx, double vy, double vtheta)
{
  int num_steps = int(sim_time_ / sim_granularity_ + 0.5);
  //we at least want to take one step... even if we won't move, we want to score our current position
  if(num_steps == 0)
  {
    num_steps = 1;
  }

  for(int i = 0; i <= num_steps; ++i)
  {
    double dt = sim_time_ / num_steps * i;
    double x_i = x + (vx * cos(theta) + vy * cos(M_PI_2 + theta)) * dt;
    double y_i = y + (vx * sin(theta) + vy * sin(M_PI_2 + theta)) * dt;
    double theta_i = theta + vtheta * dt;

    if (isIn3DCollision(x_i, y_i, theta_i))
    {
      return false;
    }
  }

  return true;
}

/// 3D collision check at x,y,theta in global map coordinates
bool PoseFollower3D::isIn3DCollision(double x, double y, double theta)
{
  ROS_DEBUG("PoseFollower3D: 3D collision check at %f %f %f", x, y, theta);
  if (!collisions_received_){
    ROS_ERROR("pose_follower_3d did not receive any 3D CollisionObject, no 3D collision check possible");
    return true;
  }
  //m_num3DCollChecks++;

  // move robot to planned base configuration:
  btTransform cur(tf::createQuaternionFromYaw(theta), btVector3(x, y, 0.0));
  kinematic_state_->getJointStateVector()[0]->setJointStateValues(cur);
  kinematic_state_->updateKinematicLinks();

  // this is the collision check:
  boost::mutex::scoped_lock lock(collisions_lock_);
  return collision_model_3d_.isKinematicStateInEnvironmentCollision(*kinematic_state_);
}

bool PoseFollower3D::stopped()
{
  //copy over the odometry information
  nav_msgs::Odometry base_odom;
  {
    boost::mutex::scoped_lock lock(odom_lock_);
    base_odom = base_odom_;
  }

  return fabs(base_odom.twist.twist.angular.z) <= rot_stopped_velocity_
    && fabs(base_odom.twist.twist.linear.x) <= trans_stopped_velocity_
    && fabs(base_odom.twist.twist.linear.y) <= trans_stopped_velocity_;
}

bool PoseFollower3D::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
  //get the current pose of the robot in the fixed frame
  tf::Stamped<tf::Pose> robot_pose;
  if( !costmap_ros_->getRobotPose( robot_pose ))
  {
    ROS_ERROR("Can't get robot pose");
    geometry_msgs::Twist empty_twist;
    cmd_vel = empty_twist;
    return false;
  }

  //we want to compute a velocity command based on our current waypoint
  tf::Stamped<tf::Pose> target_pose;
  tf::poseStampedMsgToTF(global_plan_[current_waypoint_], target_pose);

  ROS_DEBUG("PoseFollower3D: current robot pose %f %f ==> %f",
            robot_pose.getOrigin().x(), robot_pose.getOrigin().y(), tf::getYaw(robot_pose.getRotation()));
  ROS_DEBUG("PoseFollower3D: target robot pose %f %f ==> %f",
            target_pose.getOrigin().x(), target_pose.getOrigin().y(), tf::getYaw(target_pose.getRotation()));

  //get the difference between the two poses
  geometry_msgs::Twist diff = diff2D(target_pose, robot_pose);
  ROS_DEBUG("PoseFollower3D: diff %f %f ==> %f", diff.linear.x, diff.linear.y, diff.angular.z);

  geometry_msgs::Twist limit_vel = limitTwist(diff);

  geometry_msgs::Twist test_vel = limit_vel;
  // TODO: fix footprint: update costmap one / own collcheck here?
  double s = 1.2;
  bool legal_traj = collision_planner_.checkTrajectory(s*test_vel.linear.x, s*test_vel.linear.y, s*test_vel.angular.z, true);

  double collision_age = (ros::Time::now() - collision_object_time_).toSec();
  if (collision_age > 1.5)
  {
    ROS_WARN("Warning: pose_follower_3d's collision objects might be outdated (%f s old)", collision_age);
  }

  if( !legal_traj )
  {
    ROS_DEBUG("Local 2D plan not legal, checking 3D collisions");

    // assuming that costmap frame is the same as collision map frame (=map)
    legal_traj = checkTrajectory3D( robot_pose.getOrigin().x(),
                                    robot_pose.getOrigin().y(),
                                    tf::getYaw(robot_pose.getRotation()),
                                    s*test_vel.linear.x,
                                    s*test_vel.linear.y,
                                    s*test_vel.angular.z );
  }

  /*
    double scaling_factor = 1.0;
    double ds = scaling_factor / samples_;

    //let's make sure that the velocity command is legal... and if not, scale down
    if(!legal_traj){
    ROS_INFO("Local 3D check failed, scaling down");
    for(int i = 0; i < samples_; ++i){
    test_vel.linear.x = limit_vel.linear.x * scaling_factor;
    test_vel.linear.y = limit_vel.linear.y * scaling_factor;
    test_vel.angular.z = limit_vel.angular.z * scaling_factor;
    test_vel = limitTwist(test_vel);
    // only check 3D now (probably 2D won't clear before 3D)
    if(checkTrajectory3D(robot_pose.getOrigin().x(), robot_pose.getOrigin().y(), tf::getYaw(robot_pose.getRotation()), test_vel.linear.x, test_vel.linear.y, test_vel.angular.z)){
    legal_traj = true;
    break;
    }
    scaling_factor -= ds;
    }
    }
  */

  if( !legal_traj )
  {
    ROS_WARN("scaling down velocity on collision check...");
    s = 1.0;
    legal_traj = collision_planner_.checkTrajectory(s*test_vel.linear.x, s*test_vel.linear.y, s*test_vel.angular.z, true);
    if( !legal_traj )
    {
      legal_traj = checkTrajectory3D( robot_pose.getOrigin().x(),
                                      robot_pose.getOrigin().y(),
                                      tf::getYaw(robot_pose.getRotation()),
                                      s*test_vel.linear.x,
                                      s*test_vel.linear.y,
                                      s*test_vel.angular.z );
    }
  }

  if(!legal_traj)
  {
    ROS_ERROR("Local plan not legal (%.2f, %.2f, %.2f)", limit_vel.linear.x, limit_vel.linear.y, limit_vel.angular.z);
    geometry_msgs::Twist empty_twist;
    cmd_vel = empty_twist;
    return false;
  }

  //if it is legal... we'll pass it on
  cmd_vel = test_vel;

  bool in_goal_position = false;
  while(fabs(diff.linear.x) <= tolerance_trans_ &&
        fabs(diff.linear.y) <= tolerance_trans_ &&
        fabs(diff.angular.z) <= tolerance_rot_)
  {
    if(current_waypoint_ < global_plan_.size() - 1)
    {
      current_waypoint_++;
      tf::poseStampedMsgToTF(global_plan_[current_waypoint_], target_pose);
      diff = diff2D(target_pose, robot_pose);
    }
    else
    {
      ROS_DEBUG("Reached goal: %d", current_waypoint_);
      in_goal_position = true;
      break;
    }
  }

  //if we're not in the goal position, we need to update time
  if(!in_goal_position)
  {
    goal_reached_time_ = ros::Time::now();
  }
  //check if we've reached our goal for long enough to succeed
  if(goal_reached_time_ + ros::Duration(tolerance_timeout_) < ros::Time::now())
  {
    geometry_msgs::Twist empty_twist;
    cmd_vel = empty_twist;
  }

  return true;
}

bool PoseFollower3D::setPlan(const std::vector<geometry_msgs::PoseStamped>& global_plan)
{
  current_waypoint_ = 0;
  goal_reached_time_ = ros::Time::now();
  if(!transformGlobalPlan(*tf_, global_plan, *costmap_ros_, costmap_ros_->getGlobalFrameID(), global_plan_))
  {
    ROS_ERROR("Could not transform the global plan to the frame of the controller");
    return false;
  }

  // update kinematic state of robot:
  planning_environment_msgs::GetRobotStateRequest req;
  planning_environment_msgs::GetRobotStateResponse resp;
  if (robot_state_client_.call(req, resp))
  {
    if (!planning_environment::setRobotStateAndComputeTransforms(resp.robot_state, *kinematic_state_))
    {
      ROS_WARN("Robot State to kinematic model update incomplete");
    } else{
      ROS_INFO("Kinematic model updated from robot state");
    }
  }
  else
  {
    ROS_ERROR("Error calling robot state service");
    return false;
  }
  collision_planner_.updateFootprint();

  return true;
}

bool PoseFollower3D::isGoalReached()
{
  if(goal_reached_time_ + ros::Duration(tolerance_timeout_) < ros::Time::now() && stopped()){
    return true;
  }
  return false;
}

geometry_msgs::Twist PoseFollower3D::diff2D(const tf::Pose& pose1, const tf::Pose& pose2)
{
  geometry_msgs::Twist res;
  tf::Pose diff = pose2.inverse() * pose1;
  res.linear.x = diff.getOrigin().x();
  res.linear.y = diff.getOrigin().y();
  res.angular.z = tf::getYaw(diff.getRotation());

  if(holonomic_ || (fabs(res.linear.x) <= tolerance_trans_ && fabs(res.linear.y) <= tolerance_trans_))
    return res;

  //in the case that we're not rotating to our goal position and we have a non-holonomic robot
  //we'll need to command a rotational velocity that will help us reach our desired heading
    
  //we want to compute a goal based on the heading difference between our pose and the target
  double yaw_diff = headingDiff(pose1.getOrigin().x(), pose1.getOrigin().y(), 
                                pose2.getOrigin().x(), pose2.getOrigin().y(), tf::getYaw(pose2.getRotation()));

  //we'll also check if we can move more effectively backwards
  double neg_yaw_diff = headingDiff(pose1.getOrigin().x(), pose1.getOrigin().y(), 
                                    pose2.getOrigin().x(), pose2.getOrigin().y(), M_PI + tf::getYaw(pose2.getRotation()));

  //check if its faster to just back up
  if(fabs(neg_yaw_diff) < fabs(yaw_diff)){
    ROS_DEBUG("Negative is better: %.2f", neg_yaw_diff);
    yaw_diff = neg_yaw_diff;
  }

  //compute the desired quaterion
  tf::Quaternion rot_diff = tf::createQuaternionFromYaw(yaw_diff);
  tf::Quaternion rot = pose2.getRotation() * rot_diff;
  tf::Pose new_pose = pose1;
  new_pose.setRotation(rot);

  diff = pose2.inverse() * new_pose;
  res.linear.x = diff.getOrigin().x();
  res.linear.y = diff.getOrigin().y();
  res.angular.z = tf::getYaw(diff.getRotation());
  return res;
}

geometry_msgs::Twist PoseFollower3D::limitTwist(const geometry_msgs::Twist& twist)
{
  geometry_msgs::Twist res = twist;
  res.linear.x *= K_trans_;
  if(!holonomic_)
    res.linear.y = 0.0;
  else    
    res.linear.y *= K_trans_;
  res.angular.z *= K_rot_;

  //make sure to bound things by our velocity limits
  double lin_overshoot = sqrt(res.linear.x * res.linear.x + res.linear.y * res.linear.y) / max_vel_lin_;
  double lin_undershoot = min_vel_lin_ / sqrt(res.linear.x * res.linear.x + res.linear.y * res.linear.y);
  if (lin_overshoot > 1.0) 
  {
    res.linear.x /= lin_overshoot;
    res.linear.y /= lin_overshoot;
  }

  //we only want to enforce a minimum velocity if we're not rotating in place
  if(lin_undershoot > 1.0)
  {
    res.linear.x *= lin_undershoot;
    res.linear.y *= lin_undershoot;
  }

  if (fabs(res.angular.z) > max_vel_th_) res.angular.z = max_vel_th_ * sign(res.angular.z);
  if (fabs(res.angular.z) < min_vel_th_) res.angular.z = 0.0;// min_vel_th_ * sign(res.angular.z);

  //we want to check for whether or not we're desired to rotate in place
  if(sqrt(twist.linear.x * twist.linear.x + twist.linear.y * twist.linear.y) < in_place_trans_vel_)
  {
    if (fabs(res.angular.z) < min_in_place_vel_th_)
    {
      res.angular.z = min_in_place_vel_th_ * sign(res.angular.z);
    }
    res.linear.x = 0.0;
    res.linear.y = 0.0;
  }

  ROS_DEBUG("Angular command %f", res.angular.z);
  return res;
}

bool PoseFollower3D::transformGlobalPlan(const tf::TransformListener& tf, const std::vector<geometry_msgs::PoseStamped>& global_plan, 
                                         const costmap_2d::Costmap2DROS& costmap, const std::string& global_frame,
                                         std::vector<geometry_msgs::PoseStamped>& transformed_plan)
{
  const geometry_msgs::PoseStamped& plan_pose = global_plan[0];

  transformed_plan.clear();

  try
  {
    if (!global_plan.size() > 0)
    {
      ROS_ERROR("Recieved plan with zero length");
      return false;
    }

    tf::StampedTransform transform;
    tf.lookupTransform(global_frame, ros::Time(), 
                       plan_pose.header.frame_id, plan_pose.header.stamp, 
                       plan_pose.header.frame_id, transform);

    tf::Stamped<tf::Pose> tf_pose;
    geometry_msgs::PoseStamped newer_pose;
    //now we'll transform until points are outside of our distance threshold
    for(unsigned int i = 0; i < global_plan.size(); ++i)
    {
      const geometry_msgs::PoseStamped& pose = global_plan[i];
      poseStampedMsgToTF(pose, tf_pose);
      tf_pose.setData(transform * tf_pose);
      tf_pose.stamp_ = transform.stamp_;
      tf_pose.frame_id_ = global_frame;
      poseStampedTFToMsg(tf_pose, newer_pose);

      transformed_plan.push_back(newer_pose);
    }
  }
  catch(tf::LookupException& ex)
  {
    ROS_ERROR("No Transform available Error: %s\n", ex.what());
    return false;
  }
  catch(tf::ConnectivityException& ex)
  {
    ROS_ERROR("Connectivity Error: %s\n", ex.what());
    return false;
  }
  catch(tf::ExtrapolationException& ex)
  {
    ROS_ERROR("Extrapolation Error: %s\n", ex.what());
    if (global_plan.size() > 0)
    {
      ROS_ERROR("Global Frame: %s Plan Frame size %d: %s\n",
                global_frame.c_str(), (unsigned int)global_plan.size(), global_plan[0].header.frame_id.c_str());
    }
    return false;
  }

  return true;
}

} // end namespace pose_follower_3d
