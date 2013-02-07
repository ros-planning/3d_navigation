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
*   * Neither the name of the Willow Garage nor the names of its
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
* Author: Armin Hornung, based on sbpl_lattice_planner
*********************************************************************/

#ifndef SBPL_LATTICE_PLANNER_LAYER_3D_H
#define SBPL_LATTICE_PLANNER_LAYER_3D_H

#include <iostream>
#include <vector>

using namespace std;

/** ROS **/
#include <ros/ros.h>


// sbpl headers
#include <sbpl/headers.h>
#include <sbpl_lattice_planner_layer_3d/environment_navxythetamlevlat.h>

//global representation
#include <nav_core/base_global_planner.h>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

namespace sbpl_lattice_planner_layer_3d{

class SBPLLatticePlannerLayer3D : public nav_core::BaseGlobalPlanner{
public:
  
  /** @brief Return the (latest) instance of this class created.
   *
   * This is a hack to implement a connection between
   * SBPLLatticePlannerLayer3D and PoseFollower3D which is not
   * otherwise possible in the groovy release of move_base.
   *
   * @return The most-recently create instance of
   * SBPLLatticePlannerLayer3D, or NULL if none has been created yet.
   */
  static SBPLLatticePlannerLayer3D* getInstance() { return instance_; }

  /**
   * @brief  Default constructor for the NavFnROS object
   */
  SBPLLatticePlannerLayer3D();

  
  /**
   * @brief  Constructor for the SBPLLatticePlanner object
   * @param  name The name of this planner
   * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use
   */
  SBPLLatticePlannerLayer3D(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

  virtual ~SBPLLatticePlannerLayer3D();

  /**
   * @brief  Initialization function for the SBPLLatticePlanner object
   * @param  name The name of this planner
   * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use
   */
  virtual void initialize(std::string name, 
                          costmap_2d::Costmap2DROS* costmap_ros);
  
  /**
   * @brief Given a goal pose in the world, compute a plan
   * @param start The start pose 
   * @param goal The goal pose 
   * @param plan The plan... filled by the planner
   * @return True if a valid plan was found, false otherwise
   */
  virtual bool makePlan(const geometry_msgs::PoseStamped& start, 
                        const geometry_msgs::PoseStamped& goal, 
                        std::vector<geometry_msgs::PoseStamped>& plan);

  void collisionsCallback(const mapping_msgs::CollisionObjectConstPtr& msg);
  void attachedCallback(const mapping_msgs::AttachedCollisionObjectConstPtr& msg);

  void useMultiLayer(bool use);
  void close_files();
  void drawPose(double x, double y, double theta, double i){env_->drawPose(x,y,theta,i);};

  void setControllerCostmap( costmap_2d::Costmap2DROS* controller_costmap ) { controller_costmap_ = controller_costmap; }
  costmap_2d::Costmap2DROS* getControllerCostmap() const { return controller_costmap_; }

  planning_scene_monitor::PlanningSceneMonitorPtr getPlanningSceneMonitor() { return planning_scene_monitor_; }

private:
  bool use_multi_layer;
  unsigned char costMapCostToSBPLCost(unsigned char newcost);
  void publishStats(int solution_cost, int solution_size, 
                    const geometry_msgs::PoseStamped& start, 
                    const geometry_msgs::PoseStamped& goal);

  void writeStats(FILE* fout, bool ret, int solution_cost);

  bool initialized_;

  SBPLPlanner* planner_;
  EnvironmentNAVXYTHETAMLEVLAT3D* env_;
  
  std::string planner_type_; /**< sbpl method to use for planning.  choices are ARAPlanner and ADPlanner */

  double allocated_time_; /**< amount of time allowed for search */
  double initial_epsilon_; /**< initial epsilon for beginning the anytime search */

  std::string environment_type_; /** what type of environment in which to plan.  choices are 2D and XYThetaLattice. */ 
  std::string cost_map_topic_; /** what topic is being used for the costmap topic */

  bool forward_search_; /** whether to use forward or backward search */
  std::string primitive_filename_; /** where to find the motion primitives for the current robot */
  int force_scratch_limit_; /** the number of cells that have to be changed in the costmap to force the planner to plan from scratch even if its an incremental planner */

  unsigned char lethal_obstacle_;
  unsigned char inscribed_inflated_obstacle_;
  unsigned char sbpl_cost_multiplier_;

  tf::TransformListener* tf_;
  costmap_2d::Costmap2DROS* costmap_ros_; /**< manages the cost map for us */
  costmap_2d::Costmap2DROS* base_costmap_ros_; /**< manages the cost map for us */
  costmap_2d::Costmap2DROS* spine_costmap_ros_; /**< manages the cost map for us */
  costmap_2d::Costmap2DROS* arm_costmap_ros_; /**< manages the cost map for us */
  costmap_2d::Costmap2D cost_map_;        /**< local copy of the costmap underlying cost_map_ros_ */
  costmap_2d::Costmap2DROS* controller_costmap_;
  bool footprint_computed_;

  ros::Publisher plan_pub_;
  ros::Publisher stats_publisher_;
  ros::Subscriber collision_objects_sub_, attached_objects_sub_;

  std::vector<geometry_msgs::Point> footprint_;

  bool collisionObjectsReceived;
  bool attachedObjectsReceived;

  FILE* file1Layer;
  FILE* fileMLayer;

  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

  static SBPLLatticePlannerLayer3D* instance_;
};

} // end namespace sbpl_lattice_planner_layer_3d

#endif
