/*
 * Copyright (c) 2010-2011, A. Hornung, University of Freiburg
 * Copyright (c) 2013, Willow Garage
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Freiburg nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/ColorRGBA.h>
#include <mapping_msgs/CollisionObject.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>


#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <octomap_ros/OctomapBinary.h>
#include <octomap_ros/GetOctomap.h>
#include <octomap_ros/ClearBBXRegion.h>
#include <octomap_ros/OctomapROS.h>
#include <octomap/OcTreeKey.h>

#include <mapping_msgs/AttachedCollisionObject.h>


namespace sbpl_lattice_planner_layer_3d
{

/** @brief This class projects 3 slices of an OcTree down to 3 2D
 *  occupancy grids.
 *
 *  Currently this code is very specific to the needs of 3D navigation.
 *
 *  This is a port of OctomapServerCombined. */
class OctomapLayerProjector
{
public:
  OctomapLayerProjector( const std::string& filename= "" );
  virtual ~OctomapLayerProjector();

  bool serviceCallback( octomap_ros::GetOctomap::Request &req,
                        octomap_ros::GetOctomap::Response &res );

  bool clearBBXSrv( octomap_ros::ClearBBXRegionRequest& req,
                    octomap_ros::ClearBBXRegionRequest& resp );

  void insertCloudCallback( const sensor_msgs::PointCloud2::ConstPtr& cloud );

  void attachedCallback( const mapping_msgs::AttachedCollisionObjectConstPtr& msg );

protected:
  std_msgs::ColorRGBA heightMapColor( double h ) const;

  void publishMap( const ros::Time& rostime = ros::Time::now() );

  void publishAll( const ros::Time& rostime = ros::Time::now() );

  ros::NodeHandle m_nh;

  ros::Publisher m_markerPub,
    m_binaryMapPub,
    m_pointCloudPub,
    m_collisionObjectPub,
    m_mapPub,
    base_mapPub,
    spine_mapPub,
    arm_mapPub,
    boundPub;

  ros::ServiceServer m_octomapService, m_clearBBXService;

  tf::TransformListener m_tfListener;

  ros::Subscriber attachedObjectsSub;

  std::string attachedFrame;

  double attachedMaxOffset;
  double attachedMinOffset;

  bool haveAttachedObject;

  KeyRay m_keyRay;  // temp storage for ray casting
  double m_maxRange;
  std::string m_worldFrameId; // the map frame
  std::string m_baseFrameId; // base of the robot for ground plane filtering
  bool m_useHeightMap;
  std_msgs::ColorRGBA m_color;
  double m_colorFactor;

  double m_pointcloudMinZ;
  double m_pointcloudMaxZ;
  double m_occupancyMinZ;
  double m_occupancyMaxZ;
  double m_minSizeX;
  double m_minSizeY;
  bool m_filterSpeckles;

  bool m_filterGroundPlane;
  double m_groundFilterDistance;
  double m_groundFilterAngle;
  double m_groundFilterPlaneDistance;

  /** @brief Pointer to the octree instance managed by the planning scene monitor.
   *
   * Will likely become a boost::shared_ptr I guess. */
  OcTree* octree_;

  // All projected grids have same dimensions.
  int grid_size_x_;
  int grid_size_y_;
  char* full_grid_; ///< I'm not sure what this is for exactly.
  char* base_grid_;
  char* spine_grid_;
  char* arm_grid_;
};

} // end namespace sbpl_lattice_planner_layer_3d


