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

#include <sbpl_lattice_planner_layer_3d/sbpl_lattice_planner_3d.h>
#include <pluginlib/class_list_macros.h>
#include <nav_msgs/Path.h>
#include <sbpl_lattice_planner_layer_3d/SBPLLatticePlanner3DStats.h>
#include <planning_environment_msgs/GetRobotState.h>

using namespace std;
using namespace ros;


PLUGINLIB_REGISTER_CLASS(SBPLLatticePlannerLayer3D, sbpl_lattice_planner_layer_3d::SBPLLatticePlannerLayer3D, nav_core::BaseGlobalPlanner);

namespace sbpl_lattice_planner_layer_3d{

class LatticeSCQ : public StateChangeQuery{
  public:
    LatticeSCQ(EnvironmentNAVXYTHETALAT* env, std::vector<nav2dcell_t> const & changedcellsV)
      : env_(env), changedcellsV_(changedcellsV) {
    }

    // lazy init, because we do not always end up calling this method
    virtual std::vector<int> const * getPredecessors() const{
      if(predsOfChangedCells_.empty() && !changedcellsV_.empty())
        env_->GetPredsofChangedEdges(&changedcellsV_, &predsOfChangedCells_);
      return &predsOfChangedCells_;
    }

    // lazy init, because we do not always end up calling this method
    virtual std::vector<int> const * getSuccessors() const{
      if(succsOfChangedCells_.empty() && !changedcellsV_.empty())
        env_->GetSuccsofChangedEdges(&changedcellsV_, &succsOfChangedCells_);
      return &succsOfChangedCells_;
    }

    EnvironmentNAVXYTHETALAT * env_;
    std::vector<nav2dcell_t> const & changedcellsV_;
    mutable std::vector<int> predsOfChangedCells_;
    mutable std::vector<int> succsOfChangedCells_;
};

SBPLLatticePlannerLayer3D::SBPLLatticePlannerLayer3D()
  : initialized_(false), costmap_ros_(NULL), controller_costmap_(NULL), controller_costmap_initialized(false){
}

SBPLLatticePlannerLayer3D::SBPLLatticePlannerLayer3D(std::string name, costmap_2d::Costmap2DROS* costmap_ros) 
  : initialized_(false), costmap_ros_(NULL), controller_costmap_(NULL), controller_costmap_initialized(false){
  initialize(name, costmap_ros);
}

SBPLLatticePlannerLayer3D::~SBPLLatticePlannerLayer3D(){
	if (planner_){
		delete planner_;
	}
}
    
void SBPLLatticePlannerLayer3D::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
  if(name.compare("") == 0){
    controller_costmap_ = costmap_ros;
    return;
  }
  if(!initialized_){
    ros::NodeHandle private_nh("~/"+name);
    ros::NodeHandle nh;
    
    ROS_INFO("Name is %s", name.c_str());

    private_nh.param("planner_type", planner_type_, string("ARAPlanner"));
    private_nh.param("allocated_time", allocated_time_, 10.0);
    private_nh.param("initial_epsilon",initial_epsilon_,3.0);
    private_nh.param("environment_type", environment_type_, string("XYThetaLattice"));
    private_nh.param("forward_search", forward_search_, bool(false));
    private_nh.param("primitive_filename",primitive_filename_,string(""));
    private_nh.param("force_scratch_limit",force_scratch_limit_,500);

    double nominalvel_mpersecs, timetoturn45degsinplace_secs;
    private_nh.param("nominalvel_mpersecs", nominalvel_mpersecs, 0.4);
    private_nh.param("timetoturn45degsinplace_secs", timetoturn45degsinplace_secs, 0.6);

    int lethal_obstacle;
    private_nh.param("lethal_obstacle",lethal_obstacle,20);
    lethal_obstacle_ = (unsigned char) lethal_obstacle;
    inscribed_inflated_obstacle_ = lethal_obstacle_-1;
    sbpl_cost_multiplier_ = (unsigned char) (costmap_2d::INSCRIBED_INFLATED_OBSTACLE/inscribed_inflated_obstacle_ + 1);
    ROS_ERROR("SBPL: lethal: %u, inscribed inflated: %u, multiplier: %u",lethal_obstacle,inscribed_inflated_obstacle_,sbpl_cost_multiplier_);

    std::vector<std::string> footprintLinks;
    // get list of links for footprint computation:
    if (!private_nh.hasParam ("footprint_links")){
    	ROS_WARN ("No links specified for footprint computation (parameter ~footprint_links).");
    } else{
    	XmlRpc::XmlRpcValue xmlrpc_vals;;

    	private_nh.getParam ("footprint_links", xmlrpc_vals);
    	if (xmlrpc_vals.getType () != XmlRpc::XmlRpcValue::TypeArray){
    		ROS_WARN ("footprint_links need to be an array");
    	}
    	else {
    		if (xmlrpc_vals.size () == 0){
    			ROS_WARN ("No values in footprint_links array");
    		} else {
    			for (int i = 0; i < xmlrpc_vals.size (); ++i){

    				if (xmlrpc_vals[i].getType() != XmlRpc::XmlRpcValue::TypeStruct)
    				{
    					ROS_WARN ("Self see links entry %d is not a structure.  Stopping processing of self see links", i);
    					break;
    				}
    				if (!xmlrpc_vals[i].hasMember ("name"))
    				{
    					ROS_WARN ("Self see links entry %d has no name.  Stopping processing of self see links", i);
    					break;
    				}
    				std::string name = std::string (xmlrpc_vals[i]["name"]);
    				footprintLinks.push_back(name);
    			}
    		}
    	}
    }

    tf_ = new tf::TransformListener(ros::Duration(10));
    base_costmap_ros_ = new costmap_2d::Costmap2DROS("base_costmap", *tf_);
    base_costmap_ros_->pause();
    spine_costmap_ros_ = new costmap_2d::Costmap2DROS("spine_costmap", *tf_);
    spine_costmap_ros_->pause();
    arm_costmap_ros_ = new costmap_2d::Costmap2DROS("arm_costmap", *tf_);
    arm_costmap_ros_->pause();

    
    costmap_ros_=costmap_ros;
    //base_costmap_ros_->clearRobotFootprint();
    //spine_costmap_ros_->clearRobotFootprint();
    //arm_costmap_ros_->clearRobotFootprint();
    base_costmap_ros_->getCostmapCopy(cost_map_);


    if ("XYThetaLattice" == environment_type_){
      ROS_DEBUG("Using a 3D costmap for theta lattice\n");
      env_ = new EnvironmentNAVXYTHETAMLEVLAT3D();
    }
    else{
      ROS_ERROR("XYThetaLattice is currently the only supported environment!\n");
      exit(1);
    }

    if(!env_->SetEnvParameter("cost_inscribed_thresh",costMapCostToSBPLCost(costmap_2d::INSCRIBED_INFLATED_OBSTACLE))){
      ROS_ERROR("Failed to set cost_inscribed_thresh parameter");
      exit(1);
    }
    if(!env_->SetEnvParameter("cost_possibly_circumscribed_thresh", costMapCostToSBPLCost(cost_map_.getCircumscribedCost()))){
      ROS_ERROR("Failed to set cost_possibly_circumscribed_thresh parameter");
      exit(1);
    }
    int obst_cost_thresh = costMapCostToSBPLCost(costmap_2d::LETHAL_OBSTACLE);

    collisionObjectsReceived = false;
    attachedObjectsReceived = false;

    robot_state_client_ = nh.serviceClient<planning_environment_msgs::GetRobotState>("/environment_server/get_robot_state");
    // get initial robot configuration for footprint:
    // update kinematic state of the robot:
    planning_environment_msgs::GetRobotStateRequest req;
    planning_environment_msgs::GetRobotStateResponse resp;
    if (!robot_state_client_.call(req, resp)){
  	  ROS_ERROR("Error calling robot state service");
    }

    std::vector<geometry_msgs::Point> footprint = base_costmap_ros_->getRobotFootprint();
    vector<sbpl_2Dpt_t> fp;
    fp.reserve(footprint.size());
    for (unsigned int i=0; i<footprint.size(); i++){
      sbpl_2Dpt_t pt;
      pt.x = footprint[i].x;
      pt.y = footprint[i].y;
      fp.push_back(pt);
    }

    bool ret;
    vector<sbpl_2Dpt_t> new_fp;
    try{
      ret = env_->InitializeEnv(base_costmap_ros_->getSizeInCellsX(), // width
              base_costmap_ros_->getSizeInCellsY(), // height
              cost_map_.getOriginX(), cost_map_.getOriginY(),
              cost_map_.getResolution(),
              0, // mapdata
              footprintLinks, resp.robot_state,
              nominalvel_mpersecs,
              timetoturn45degsinplace_secs, obst_cost_thresh,
              primitive_filename_.c_str(),fp,&new_fp);
    }
    catch(SBPL_Exception e){
      ROS_ERROR("SBPL encountered a fatal exception!");
      ret = false;
    }
    if(!ret){
      ROS_ERROR("SBPL initialization failed!");
      exit(1);
    }

    vector<sbpl_2Dpt_t> additional_footprints[2];
    additional_footprints[0] = new_fp;
    /*
    footprint = arm_costmap_ros_->getRobotFootprint();
    additional_footprints[0].reserve(footprint.size());
    for (unsigned int i=0; i<footprint.size(); i++){
      sbpl_2Dpt_t pt;
      pt.x = footprint[i].x;
      pt.y = footprint[i].y;
      additional_footprints[0].push_back(pt);
    }
    */

    footprint = spine_costmap_ros_->getRobotFootprint();
    additional_footprints[1].reserve(footprint.size());
    for (unsigned int i=0; i<footprint.size(); i++){
      sbpl_2Dpt_t pt;
      pt.x = footprint[i].x;
      pt.y = footprint[i].y;
      additional_footprints[1].push_back(pt);
    }

    unsigned char inscribed_thresh[2] = {costMapCostToSBPLCost(costmap_2d::INSCRIBED_INFLATED_OBSTACLE),costMapCostToSBPLCost(costmap_2d::INSCRIBED_INFLATED_OBSTACLE)};
    unsigned char circumscribed_thresh[2] = {costMapCostToSBPLCost(cost_map_.getCircumscribedCost()),costMapCostToSBPLCost(cost_map_.getCircumscribedCost())};
    env_->InitializeAdditionalLevels(2, additional_footprints, inscribed_thresh, circumscribed_thresh);
    env_->visualizeFootprints();

    vector<geometry_msgs::Point> foot_pts;
    for(unsigned int i=0; i<new_fp.size(); i++){
      geometry_msgs::Point p;
      p.x = new_fp[i].x;
      p.y = new_fp[i].y;
      p.z = 0;
      foot_pts.push_back(p);
    }
    arm_costmap_ros_->setFootprint(foot_pts,0.15);

    // fill environment model according to costmap:
    for (ssize_t ix(0); ix < base_costmap_ros_->getSizeInCellsX(); ++ix)
      for (ssize_t iy(0); iy < base_costmap_ros_->getSizeInCellsY(); ++iy)
        env_->UpdateCost(ix, iy, costMapCostToSBPLCost(cost_map_.getCost(ix,iy)));
    spine_costmap_ros_->getCostmapCopy(cost_map_);
    for (ssize_t ix(0); ix < spine_costmap_ros_->getSizeInCellsX(); ++ix)
      for (ssize_t iy(0); iy < spine_costmap_ros_->getSizeInCellsY(); ++iy)
        env_->UpdateCostinAddLev(ix, iy, costMapCostToSBPLCost(cost_map_.getCost(ix,iy)), 1);
    arm_costmap_ros_->getCostmapCopy(cost_map_);
    for (ssize_t ix(0); ix < arm_costmap_ros_->getSizeInCellsX(); ++ix)
      for (ssize_t iy(0); iy < arm_costmap_ros_->getSizeInCellsY(); ++iy)
        env_->UpdateCostinAddLev(ix, iy, costMapCostToSBPLCost(cost_map_.getCost(ix,iy)), 0);

    if ("ARAPlanner" == planner_type_){
      ROS_INFO("Planning with ARA*");
      planner_ = new ARAPlanner(env_, forward_search_);
    }
    else if ("ADPlanner" == planner_type_){
      ROS_INFO("Planning with AD*");
      planner_ = new ADPlanner(env_, forward_search_);
    }
    else{
      ROS_ERROR("ARAPlanner and ADPlanner are currently the only supported planners!\n");
      exit(1);
    }

    ROS_INFO("[sbpl_lattice_planner_3d] Initialized successfully");
    plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);
    stats_publisher_ = private_nh.advertise<sbpl_lattice_planner_layer_3d::SBPLLatticePlanner3DStats>("sbpl_lattice_planner_stats", 1);
    collision_objects_sub_ = nh.subscribe("octomap_collision_object", 1, &SBPLLatticePlannerLayer3D::collisionsCallback, this);
    attached_objects_sub_ = nh.subscribe("attached_collision_object", 1, &SBPLLatticePlannerLayer3D::attachedCallback, this);
    
    initialized_ = true;
    base_costmap_ros_->start();
    spine_costmap_ros_->start();
    arm_costmap_ros_->start();
    useMultiLayer(true);
  }
}
  
//Taken from Sachin's sbpl_cart_planner
//This rescales the costmap according to a rosparam which sets the obstacle cost
unsigned char SBPLLatticePlannerLayer3D::costMapCostToSBPLCost(unsigned char newcost){
  if(newcost == costmap_2d::TALL_OBSTACLE || newcost == costmap_2d::NO_INFORMATION)
    return lethal_obstacle_+1;
  else if(newcost == costmap_2d::LETHAL_OBSTACLE)
    return lethal_obstacle_;
  else if(newcost == costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
    return inscribed_inflated_obstacle_;
  else
    return (unsigned char) (newcost/sbpl_cost_multiplier_ + 0.5);
}

void SBPLLatticePlannerLayer3D::publishStats(int solution_cost, int solution_size, 
                                      const geometry_msgs::PoseStamped& start, 
                                      const geometry_msgs::PoseStamped& goal){
  // Fill up statistics and publish
  sbpl_lattice_planner_layer_3d::SBPLLatticePlanner3DStats stats;
  stats.initial_epsilon = initial_epsilon_;
  stats.plan_to_first_solution = false;
  stats.final_number_of_expands = planner_->get_n_expands();
  stats.allocated_time = allocated_time_;
  stats.number_of_3d_coll_checks = env_->getNum3DChecks();
  stats.number_of_2d_coll_checks = env_->getNum2DChecks();

  stats.time_to_first_solution = planner_->get_initial_eps_planning_time();
  stats.actual_time = planner_->get_final_eps_planning_time();
  stats.number_of_expands_initial_solution = planner_->get_n_expands_init_solution();
  stats.final_epsilon = planner_->get_final_epsilon();

  stats.solution_cost = solution_cost;
  stats.path_size = solution_size;
  stats.start = start;
  stats.goal = goal;
  stats_publisher_.publish(stats);
}


void SBPLLatticePlannerLayer3D::writeStats(FILE* fout, bool ret, int solution_cost){
  if(ret){
    fprintf(fout,"%f %d %f %d %f %d %d %d %d %d %f\n",
            planner_->get_initial_eps_planning_time(),planner_->get_n_expands_init_solution(),
            planner_->get_final_eps_planning_time(),planner_->get_n_expands(),planner_->get_final_epsilon(),
            solution_cost,env_->getNum2DChecks(),env_->getNum3DChecks(),
            planner_->get_initial_2D_checks(), planner_->get_initial_3D_checks(), planner_->get_5s_eps());
    printf("%f %d %f %d %f %d %d %d %d %d %f\n",
            planner_->get_initial_eps_planning_time(),planner_->get_n_expands_init_solution(),
            planner_->get_final_eps_planning_time(),planner_->get_n_expands(),planner_->get_final_epsilon(),
            solution_cost,env_->getNum2DChecks(),env_->getNum3DChecks(),
            planner_->get_initial_2D_checks(), planner_->get_initial_3D_checks(), planner_->get_5s_eps());
  }
  else{
    fprintf(fout,"-1 -1 -1 %d -1 -1 %d %d -1 -1 -1\n",planner_->get_n_expands(),env_->getNum2DChecks(),env_->getNum3DChecks());
    printf("-1 -1 -1 %d -1 -1 %d %d -1 -1 -1\n",planner_->get_n_expands(),env_->getNum2DChecks(),env_->getNum3DChecks());
  }
}


void SBPLLatticePlannerLayer3D::collisionsCallback(const mapping_msgs::CollisionObjectConstPtr& msg){
	ROS_DEBUG("CollisionObjects (%d/%d) received, storing locally", int(msg->poses.size()), int(msg->poses.size()));
	collisionObjectsReceived = true;
	boost::mutex::scoped_lock lock(collision_object_lock_);
	collision_object_ = *msg;
}

void SBPLLatticePlannerLayer3D::attachedCallback(const mapping_msgs::AttachedCollisionObjectConstPtr& msg){
	ROS_DEBUG("AttachedCollisionObjects received");
	attachedObjectsReceived = true;
	boost::mutex::scoped_lock lock(attached_object_lock_);
	attached_object_ = *msg;
}

bool SBPLLatticePlannerLayer3D::makePlan(const geometry_msgs::PoseStamped& start,
                                 const geometry_msgs::PoseStamped& goal,
                                 std::vector<geometry_msgs::PoseStamped>& plan){
  if(!initialized_){
    ROS_ERROR("Global planner is not initialized");
    return false;
  }

  plan.clear();

  if (base_costmap_ros_->getSizeInCellsX() != cost_map_.getSizeInCellsX()
		  	  || base_costmap_ros_->getSizeInCellsY() != cost_map_.getSizeInCellsY()){
	  ROS_WARN("Costmap size changed, this might lead to strange results!");
    //TODO: update the environments map size or it will crash here!!!!
  }

  // update attached objects:
  {
	  boost::mutex::scoped_lock lock(attached_object_lock_);
	  if (attachedObjectsReceived)
		  env_->updateAttachedObjects(attached_object_);

  }

  // update collision object from local copy:
  {
		boost::mutex::scoped_lock lock(collision_object_lock_);
		double collision_age = (ros::Time::now() - collision_object_.header.stamp).toSec();
	    if (collision_age > 1.0){
	    	ROS_WARN("Warning: sbpl_lattice_planner's collision objects might be outdated (%f s old)", collision_age);
	    }
		env_->updateCollisionObjects(collision_object_);
  }



  // update kinematic state of the robot:
  planning_environment_msgs::GetRobotStateRequest req;
  planning_environment_msgs::GetRobotStateResponse resp;
  if (robot_state_client_.call(req, resp)){
    vector<sbpl_2Dpt_t> new_fp;
	  bool footprintUpdated = env_->updateKinematicState(resp.robot_state, &new_fp);
    if(footprintUpdated || !controller_costmap_initialized){
      env_->setFootprint(0,new_fp);
      vector<geometry_msgs::Point> foot_pts;
      for(unsigned int i=0; i<new_fp.size(); i++){
        geometry_msgs::Point p;
        p.x = new_fp[i].x;
        p.y = new_fp[i].y;
        p.z = 0;
        foot_pts.push_back(p);
      }
      arm_costmap_ros_->setFootprint(foot_pts,0.15);
      costmap_ros_->setFootprint(foot_pts,0.15);
      controller_costmap_->setFootprint(foot_pts,0.15);
      controller_costmap_initialized = true;
    }
    env_->visualizeFootprints();
  }
  else{
	  ROS_ERROR("Error calling robot state service");
  }

  //base_costmap_ros_->clearRobotFootprint();
  //spine_costmap_ros_->clearRobotFootprint();
  //arm_costmap_ros_->clearRobotFootprint();
  ROS_DEBUG("[sbpl_lattice_planner_3d] robot footprint cleared");

  ROS_INFO("[sbpl_lattice_planner_3d] getting start point (%g,%g) goal point (%g,%g)",
           start.pose.position.x, start.pose.position.y,goal.pose.position.x, goal.pose.position.y);
  double theta_start = 2 * atan2(start.pose.orientation.z, start.pose.orientation.w);
  double theta_goal = 2 * atan2(goal.pose.orientation.z, goal.pose.orientation.w);

  try{
    int ret = env_->SetStart(start.pose.position.x - cost_map_.getOriginX(), start.pose.position.y- cost_map_.getOriginY(), theta_start);
    if(ret < 0 || planner_->set_start(ret) == 0){
      ROS_ERROR("ERROR: failed to set start state\n");
      return false;
    }
  }
  catch(SBPL_Exception e){
    ROS_ERROR("SBPL encountered a fatal exception while setting the start state");
    return false;
  }

  try{
    int ret = env_->SetGoal(goal.pose.position.x - cost_map_.getOriginX(), goal.pose.position.y- cost_map_.getOriginY() , theta_goal);
    if(ret < 0 || planner_->set_goal(ret) == 0){
      ROS_ERROR("ERROR: failed to set goal state\n");
      return false;
    }
  }
  catch(SBPL_Exception e){
    ROS_ERROR("SBPL encountered a fatal exception while setting the goal state");
    return false;
  }

  //update cell costs
  base_costmap_ros_->getCostmapCopy(cost_map_);
  for (ssize_t ix(0); ix < cost_map_.getSizeInCellsX(); ++ix)
    for (ssize_t iy(0); iy < cost_map_.getSizeInCellsY(); ++iy)
      env_->UpdateCost(ix, iy, costMapCostToSBPLCost(cost_map_.getCost(ix,iy)));
  //FILE* fout = fopen("spine_map.csv","w");
  //FILE* fout2 = fopen("spine_map_raw.csv","w");
  spine_costmap_ros_->getCostmapCopy(cost_map_);
  for (ssize_t ix(0); ix < cost_map_.getSizeInCellsX(); ++ix){
    for (ssize_t iy(0); iy < cost_map_.getSizeInCellsY(); ++iy){
      env_->UpdateCostinAddLev(ix, iy, costMapCostToSBPLCost(cost_map_.getCost(ix,iy)), 1);
      //fprintf(fout,"%d ",costMapCostToSBPLCost(cost_map_.getCost(ix,iy)));
      //fprintf(fout2,"%d ",cost_map_.getCost(ix,iy));
    }
    //fprintf(fout,"\n");
    //fprintf(fout2,"\n");
  }
  //fclose(fout);
  //fclose(fout2);



  vector<int> solution_stateIDs;
  int solution_cost;
  //for(int i=1; i<2; i++){
    env_->resetCollisionCount();
    solution_stateIDs.clear();
    /*
    if(i==0){
      env_->use_multi_layer = false;
      costmap_ros_->getCostmapCopy(cost_map_);
    }
    else{
      env_->use_multi_layer = true;
      arm_costmap_ros_->getCostmapCopy(cost_map_);
    }
		env_->updateCollisionObjects(collision_object_);
    */
    if(use_multi_layer)
      arm_costmap_ros_->getCostmapCopy(cost_map_);
    else
      costmap_ros_->getCostmapCopy(cost_map_);

    for (ssize_t ix(0); ix < cost_map_.getSizeInCellsX(); ++ix)
      for (ssize_t iy(0); iy < cost_map_.getSizeInCellsY(); ++iy)
        env_->UpdateCostinAddLev(ix, iy, costMapCostToSBPLCost(cost_map_.getCost(ix,iy)), 0);
    printf("inner:%f outer:%f\n",arm_costmap_ros_->getInscribedRadius(),arm_costmap_ros_->getCircumscribedRadius());

    planner_->force_planning_from_scratch();

    //setting planner parameters
    ROS_DEBUG("allocated:%f, init eps:%f\n",allocated_time_,initial_epsilon_);
    planner_->set_initialsolution_eps(initial_epsilon_);
    //TODO: MIKE: return the first solution!!!!!
    planner_->set_search_mode(false);

    ROS_DEBUG("[sbpl_lattice_planner_3d] run planner");
    try{
      clock_t t0 = clock();
      int ret = planner_->replan(allocated_time_, &solution_stateIDs, &solution_cost);
      clock_t t1 = clock();
      printf("planning took %f\n", ((double)(t1-t0))/CLOCKS_PER_SEC);
      env_->printCollisionStats();
      if(use_multi_layer)
        writeStats(fileMLayer,ret,solution_cost);
      else
        writeStats(file1Layer,ret,solution_cost);

      if(ret)
        ROS_DEBUG("Solution is found\n");
      else{
        ROS_WARN("Solution not found\n");
        publishStats(solution_cost, 0, start, goal);
        //if(i==1)
          return false;
      }
    }
    catch(SBPL_Exception e){
      ROS_ERROR("SBPL encountered a fatal exception while planning");
      return false;
    }
  //}

  printf("size of solution=%d", (int)solution_stateIDs.size());

  vector<EnvNAVXYTHETALAT3Dpt_t> sbpl_path;
  try{
    env_->ConvertStateIDPathintoXYThetaPath(&solution_stateIDs, &sbpl_path);
  }
  catch(SBPL_Exception e){
    ROS_ERROR("SBPL encountered a fatal exception while reconstructing the path");
    return false;
  }
  printf("origin %f %f\n",cost_map_.getOriginX(),cost_map_.getOriginY());
  ROS_DEBUG("Plan has %d points.\n", (int)sbpl_path.size());
  ros::Time plan_time = ros::Time::now();

  //create a message for the plan 
  nav_msgs::Path gui_path;
  gui_path.poses.resize(sbpl_path.size());
  std::string frame_id = "/map"; // TODO get from collision map
  gui_path.header.frame_id = frame_id;
  gui_path.header.stamp = plan_time;
  //printf("path:\n");
  if(sbpl_path.size() > 1){
    for(unsigned int i=0; i<sbpl_path.size(); i++){
      geometry_msgs::PoseStamped pose;
      pose.header.stamp = plan_time;
      pose.header.frame_id = base_costmap_ros_->getGlobalFrameID();

      pose.pose.position.x = sbpl_path[i].x + cost_map_.getOriginX();
      pose.pose.position.y = sbpl_path[i].y + cost_map_.getOriginY();
      pose.pose.position.z = start.pose.position.z;


      btQuaternion temp;
      temp.setEulerZYX(sbpl_path[i].theta,0,0);
      pose.pose.orientation.x = temp.getX();
      pose.pose.orientation.y = temp.getY();
      pose.pose.orientation.z = temp.getZ();
      pose.pose.orientation.w = temp.getW();

      //printf("%f %f %f\n",pose.pose.position.x,pose.pose.position.y,sbpl_path[i].theta);

      plan.push_back(pose);

      gui_path.poses[i].pose.position.x = plan[i].pose.position.x;
      gui_path.poses[i].pose.position.y = plan[i].pose.position.y;
      gui_path.poses[i].pose.position.z = plan[i].pose.position.z;
    }
  }
  else{
    plan.push_back(start);
    plan.push_back(goal);
    gui_path.poses.push_back(start);
    gui_path.poses.push_back(goal);
  }
  plan_pub_.publish(gui_path);
  publishStats(solution_cost, sbpl_path.size(), start, goal);

  return true;
}

void SBPLLatticePlannerLayer3D::useMultiLayer(bool use){
  use_multi_layer = use;
  env_->use_multi_layer = use;
  if(use)
    fileMLayer = fopen("statsMLayer.csv","w");
  else
    file1Layer = fopen("stats1Layer.csv","w");
}

void SBPLLatticePlannerLayer3D::close_files(){
  if(use_multi_layer)
    fclose(fileMLayer);
  else
    fclose(file1Layer);
}

};
