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
#ifndef LOCAL_PLANNER_3D_H_
#define LOCAL_PLANNER_3D_H_
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <nav_core/base_local_planner.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <mapping_msgs/CollisionObject.h>
#include <base_local_planner/trajectory_planner_ros.h>
#include <planning_environment/models/collision_models.h>
#include <planning_models/kinematic_state.h>
#include <planning_environment_msgs/GetRobotState.h>
#include <planning_environment/models/model_utils.h>


namespace pose_follower_3d {
  class PoseFollower3D : public nav_core::BaseLocalPlanner {
    public:
      PoseFollower3D();
      void initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros);
      bool isGoalReached();
      bool setPlan(const std::vector<geometry_msgs::PoseStamped>& global_plan);
      bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

    private:
      inline double sign(double n){
        return n < 0.0 ? -1.0 : 1.0;
      }

      geometry_msgs::Twist diff2D(const tf::Pose& pose1, const tf::Pose&  pose2);
      geometry_msgs::Twist limitTwist(const geometry_msgs::Twist& twist);
      double headingDiff(double pt_x, double pt_y, double x, double y, double heading);

      bool transformGlobalPlan(const tf::TransformListener& tf, const std::vector<geometry_msgs::PoseStamped>& global_plan, 
          const costmap_2d::Costmap2DROS& costmap, const std::string& global_frame,
          std::vector<geometry_msgs::PoseStamped>& transformed_plan);

      void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
      void collisionsCallback(const mapping_msgs::CollisionObjectConstPtr& msg);
      void attachedCallback(const mapping_msgs::AttachedCollisionObjectConstPtr& msg);
      bool stopped();
      void updateRobotPosition(double x, double y, double theta);
      bool checkTrajectory3D(double x, double y, double theta, double vx, double vy, double vtheta);

      /// Check for 3D collision of the robot's kinematic state.
      /// x,y,theta are in continuous costmap coords, will be transformed
      /// into the 3D coll. map coords.
      bool isIn3DCollision(double x, double y, double theta);

      tf::TransformListener* tf_;
      costmap_2d::Costmap2DROS* costmap_ros_;
      ros::Publisher vel_pub_;
      double K_trans_, K_rot_, tolerance_trans_, tolerance_rot_;
      double tolerance_timeout_;
      double max_vel_lin_, max_vel_th_;
      double min_vel_lin_, min_vel_th_;
      double min_in_place_vel_th_, in_place_trans_vel_;
      double sim_time_, sim_granularity_;
      bool holonomic_;
      boost::mutex odom_lock_, collisions_lock_, attached_lock_;
      ros::Subscriber odom_sub_, collisions_sub_;
      ros::ServiceClient robot_state_client_;
      nav_msgs::Odometry base_odom_;
      double trans_stopped_velocity_, rot_stopped_velocity_;
      ros::Time goal_reached_time_;
      unsigned int current_waypoint_; 
      std::vector<geometry_msgs::PoseStamped> global_plan_;
      base_local_planner::TrajectoryPlannerROS collision_planner_;
      planning_environment::CollisionModels collision_model_3d_;
      planning_models::KinematicState* kinematic_state_;
      int samples_;
      bool collisions_received_;
//      mapping_msgs::CollisionObject collision_object_;
      ros::Time collision_object_time_;
  };
};
#endif
