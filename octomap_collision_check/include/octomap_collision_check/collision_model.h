/*
 * collision_model.h
 *
 *  Created on: Mar 28, 2011
 *      Author: ahornung
 */

#ifndef COLLISION_MODEL_H_
#define COLLISION_MODEL_H_

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <planning_environment/models/collision_models.h>
#include <planning_models/kinematic_state.h>
#include <mapping_msgs/CollisionMap.h>
#include <planning_environment_msgs/ContactInformation.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <pcl/ros/conversions.h>
#include <octomap_ros/conversions.h>

#include <octomap_collision_check/octomap_object.h>

namespace octomap {
	class CollisionModel {
	public:
		CollisionModel();
		virtual ~CollisionModel();
		void setRoot(OctomapObject* root);
		OctomapObject* getRoot() const {return mapRoot; };
		void updateVis();

	protected:
		void ocTreeToCollisionMap(const OcTree& octree, const octomap::pose6d& origin, std::vector<shapes::Shape*>& shapes, std::vector<btTransform>& poses) const;
		void publishRobotStampedTransforms();
		void publishMarkerVis(const OctomapObject& tree, unsigned id, ros::Publisher pub);
		void publishPointcloudVis(const OctomapObject& tree, unsigned id, ros::Publisher pub);
		ros::NodeHandle m_nh;
		ros::Publisher m_occupiedCellsPub, m_occupiedPointcloudPub, m_occupiedObjPointcloudPub,
			m_collisionMarkerPub;
		// map hierarchy
		OctomapObject* mapRoot;
		planning_environment::CollisionModels m_planningCollisionModel;
		planning_models::KinematicState m_kinematicState;
		tf::TransformBroadcaster m_tfBroadcaster;

	};
}

#endif /* COLLISION_MODEL_H_ */
