/*
 * collision_model.cpp
 *
 *  Created on: Mar 28, 2011
 *      Author: ahornung
 */

#include <octomap_collision_check/collision_model.h>

namespace octomap{

	CollisionModel::CollisionModel()
	 : mapRoot(NULL), m_planningCollisionModel("robot_description"),
	   m_kinematicState(m_planningCollisionModel.getKinematicModel())
	{
		m_occupiedCellsPub = m_nh.advertise<visualization_msgs::Marker>("occupied_cells", 10);
		m_occupiedPointcloudPub = m_nh.advertise<sensor_msgs::PointCloud2>("occupied_points", 10);
		m_occupiedObjPointcloudPub = m_nh.advertise<sensor_msgs::PointCloud2>("occupied_points_obj", 10);
		m_collisionMarkerPub = m_nh.advertise<visualization_msgs::MarkerArray>("planning_scene_visualizer_array", 10);

		//m_kinematicState = new planning_models::KinematicState();
		m_kinematicState.setKinematicStateToDefault();
	}

	CollisionModel::~CollisionModel() {
		if (mapRoot)
			delete mapRoot;

	}

	void CollisionModel::ocTreeToCollisionMap(const OcTree& octree, const octomap::pose6d& origin, std::vector<shapes::Shape*>& shapes, std::vector<btTransform>& poses) const{
		tf::Transform tf(tf::createIdentityQuaternion());

		for(OcTree::iterator it =  octree.begin(), end =  octree.end();
				it != end; ++it)
		{
			if(octree.isNodeOccupied(*it)){
				double size = it.getSize();
				shapes.push_back(new shapes::Box(size,size,size));

				poses.push_back(octomap::poseOctomapToTf(octomap::pose6d(it.getCoordinate(), octomath::Quaternion())*origin));
			}
		}
	}

	void CollisionModel::setRoot(OctomapObject* root){
		mapRoot = root;

		// convert to shape vector for collision model:
		std::vector<shapes::Shape*> shapes;
		std::vector<btTransform> poses;
		ocTreeToCollisionMap(*mapRoot->getOcTree(), mapRoot->getOrigin(), shapes, poses);
		m_planningCollisionModel.setCollisionMap(shapes, poses);
	}

	void CollisionModel::updateVis(){

		unsigned id = 0;
		OctomapObject* tree = mapRoot;
		// global map only as pointcloud:
		publishPointcloudVis(*(tree), id, m_occupiedPointcloudPub);


		// update objects (move and vis)
		for (OctomapObject::iterator it = tree->begin(); it !=tree->end(); ++it){
			id++;
			std::vector<double> d(3, 0.0);
			d[0] = 0.005;
			(*it)->moveDiff(d);
//			(*it)->getOrigin().trans() += point3d(0.1,0.0,0.0);
			// objects as markers (cubes):
			publishMarkerVis(**it, id, m_occupiedCellsPub);

			std::stringstream ss;
			ss << id;
			std::vector<shapes::Shape*> shapes;
			std::vector<btTransform> poses;
			ocTreeToCollisionMap(*(*it)->getOcTree(), (*it)->getOrigin(), shapes, poses);
			// this will also delete a previous object with the same name
			m_planningCollisionModel.addStaticObject(ss.str(), shapes, poses, 0.0); //TODO: param for padding?
		}

		// move robot:
		 btTransform cur = m_kinematicState.getRootTransform();
		 cur.setOrigin(btVector3(cur.getOrigin().x()+0.1, cur.getOrigin().y(), cur.getOrigin().z()));
		 m_kinematicState.getJointStateVector()[0]->setJointStateValues(cur);
		 m_kinematicState.updateKinematicLinks();
		 publishRobotStampedTransforms();

		// this is the collision check:
		bool coll = m_planningCollisionModel.isKinematicStateInCollision(m_kinematicState);

		visualization_msgs::MarkerArray arr;
		std_msgs::ColorRGBA col;
		col.r = 0.0;
		col.g = 1.0;
		col.b = 1.0;
		col.a = 0.9;
		m_planningCollisionModel.getAllCollisionPointMarkers(m_kinematicState, arr, col, ros::Duration(0.1));
		//planningCollisionModel.getStaticCollisionObjectMarkers(m_kinematicState, arr, col, ros::Duration(0.1));
		m_collisionMarkerPub.publish(arr);
//    	std::vector<planning_environment_msgs::ContactInformation> contacts;
//		m_planningCollisionModel.getAllCollisionsForState(state, contacts,1);


	}

	void CollisionModel::publishMarkerVis(const OctomapObject& obj, unsigned id, ros::Publisher pub){
		if(pub.getNumSubscribers() > 0){
			size_t numLeafs = obj.getOcTree()->getNumLeafNodes();
			std_msgs::ColorRGBA color;
			color.r = 0;
			color.g = 0;
			color.b = 1.0;
			color.a = 0.5;

			visualization_msgs::Marker occupiedCellsVis;
			occupiedCellsVis.header.stamp=ros::Time::now();
			occupiedCellsVis.header.frame_id = m_planningCollisionModel.getWorldFrameId();
			std::stringstream ss;
			ss << id;
			occupiedCellsVis.ns = ss.str();
			occupiedCellsVis.id = id;
			occupiedCellsVis.type = visualization_msgs::Marker::CUBE_LIST;
			occupiedCellsVis.scale.x =  obj.getOcTree()->getResolution();
			occupiedCellsVis.scale.y =  obj.getOcTree()->getResolution();
			occupiedCellsVis.scale.z =  obj.getOcTree()->getResolution();
			occupiedCellsVis.color = color;
			occupiedCellsVis.points.reserve(numLeafs/4.0);

			for (OcTree::iterator it =  obj.getOcTree()->begin(), end =  obj.getOcTree()->end();
					it != end; ++it){
				if ( obj.getOcTree()->isNodeOccupied(*it)){
					point3d transformed = obj.getOrigin().transform(it.getCoordinate());
					occupiedCellsVis.points.push_back(pointOctomapToMsg(transformed));
				}
			}

			pub.publish(occupiedCellsVis);
		}
	}


	void CollisionModel::publishPointcloudVis(const OctomapObject& obj, unsigned id, ros::Publisher pub){
		if (pub.getNumSubscribers() > 0){
			size_t numLeafs = obj.getOcTree()->getNumLeafNodes();
			pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
			pcl_cloud.points.reserve (numLeafs/4);

			for (OcTree::iterator it = obj.getOcTree()->begin(), end = obj.getOcTree()->end();
					it != end; ++it){
				if (obj.getOcTree()->isNodeOccupied(*it)){
					point3d transformed = obj.getOrigin().transform(it.getCoordinate());
					pcl_cloud.points.push_back(pointOctomapToPCL<pcl::PointXYZ>(transformed));
				}
			}

			sensor_msgs::PointCloud2 cloud;
			pcl::toROSMsg (pcl_cloud, cloud);
			cloud.header.stamp = ros::Time::now();
			cloud.header.frame_id = m_planningCollisionModel.getWorldFrameId();
			pub.publish (cloud);

		}
	}

	void CollisionModel::publishRobotStampedTransforms(){
		ros::Time stamp = ros::Time::now();
		std::vector<geometry_msgs::TransformStamped> trans_vector;
	    const std::map<std::string, geometry_msgs::TransformStamped>& transforms = m_planningCollisionModel.getSceneTransformMap();
	    geometry_msgs::TransformStamped transvec;
	    for(std::map<std::string, geometry_msgs::TransformStamped>::const_iterator it = transforms.begin();
	        it != transforms.end();
	        it++) {
	      trans_vector.push_back(it->second);
	    }
	    for(unsigned int i = 0; i < m_kinematicState.getLinkStateVector().size(); i++) {
	      const planning_models::KinematicState::LinkState* ls = m_kinematicState.getLinkStateVector()[i];
	      geometry_msgs::TransformStamped ts;
	      ts.header.stamp = stamp;
	      ts.header.frame_id = m_planningCollisionModel.getWorldFrameId();
	      ts.child_frame_id = ls->getName();
	      tf::transformTFToMsg(ls->getGlobalLinkTransform(),ts.transform);
	      trans_vector.push_back(ts);
	    }

	    m_tfBroadcaster.sendTransform(trans_vector);
	  }

}
