#ifndef OBJECT_MAPPING_WORLD_MODEL_H
#define OBJECT_MAPPING_WORLD_MODEL_H

#include "omapping_types.h"
#include "bounding_box.h"
#include "map_node.h"
#include "pcl/features/feature.h"
#include "pcl/io/pcd_io.h"
#include "pcl/common/common.h"


namespace omapping {
  
  
  class WorldModel {
    
  public:
    
    WorldModel(double object_res, double table_res, double world_res);
    WorldModel(); // resolutions set to default values
    ~WorldModel();

    void integrateMeasurement(const PointCloud& cloud, const Point& sensor_origin, std::string class_label);

    std::vector<MapNode*> getNodes(std::string class_label) const;

    // access nodes by ID or by query point and label
    MapNode* getNodeById(int id) const;
    // access nodes by query point and label
    MapNode* queryNode(octomap::point3d p, std::string class_label) const;

    PointCloud getPointcloud(std::string class_label) const;
    PointCloud getWorldPointcloud() const;
    PointCloud generatePointCloudUnknown() const;

    // write debug files
    void writePCDMaps() const;
    void writeOcTrees() const;
    void writePCDUnknown() const;
    unsigned long long int memoryUsage();

    void removeSmallNodes(std::string class_label, float min_vol);

    // used e.g. by viewer updaters
    std::vector<MapNode*> getMovedNodes() const;
    std::vector<MapNode*> getUpdatedNodes() const;
    void resetUpdatedStates();

    // nodes that probably were removed in the scene
    std::vector<MapNode*> getStaleNodes() const; 

  protected:  

    void init();

    // DA methods, these may return NULL
    MapNode* associateNode(const PointCloud& cloud, std::string class_label) const ; 
    MapNode* getEnclosingNode(const PointCloud& cloud, const std::vector<MapNode*>& node_list) const;
    MapNode* getOverlappingNode(const PointCloud& cloud, const std::vector<MapNode*>& node_list) const;
    MapNode* getNodeNearCentroid(const PointCloud& cloud, const std::vector<MapNode*>& node_list, double& min_dist) const;
    MapNode* findTable(const PointCloud& cloud) const;
    

    // recursive calls
    void getNodesRecurs(MapNode* node, const std::string& class_label, std::vector<MapNode*>& node_list) const;
    void writePCDMapsRecurs(MapNode* node) const;
    void writeOcTreesRecurs(MapNode* node) const;
    void getWorldPointcloudRecurs(MapNode* node, PointCloud& pc) const;
    PointCloud generatePointCloudUnknown(MapNode* object_node) const;
    void getPointcloudRecurs(MapNode* node, PointCloud& pc, const std::string& class_label) const;
    MapNode* getNodeByIdRecurs(MapNode* parent, int id) const;
    void getMovedNodesRecurs(MapNode* node, std::vector<MapNode*>& node_list) const;
    void getUpdatedNodesRecurs(MapNode* node, std::vector<MapNode*>& node_list) const;
    void getStaleNodesRecurs(MapNode* node, std::vector<MapNode*>& node_list) const;



  protected:

    // map hierarchy
    MapNode* map_root;
    int id_counter; // id of next node

    // params
    double background_resolution;
    double table_resolution;
    double object_resolution;

  };


} // namespace 

#endif 
