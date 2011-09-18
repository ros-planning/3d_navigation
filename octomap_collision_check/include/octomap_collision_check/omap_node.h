#ifndef OBJECT_MAPPING_MAP_NODE_H
#define OBJECT_MAPPING_MAP_NODE_H

#include <octomap/OcTree.h>
#include <octomap_ros/conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "bounding_box.h"

namespace omapping {

	typedef octomap::OcTree OcTreeMap;
	typedef pcl::PointXYZ Point;
	typedef pcl::PointCloud<Point> PointCloud;
    
  class MapNode {
    
  public:

    MapNode(int node_id, std::string label, double map_resolution, MapNode* parent_);
    ~MapNode();

    // update map of this node and bounding_box
    // cloud and origin are assumed to be in GLOBAL coordinate
    // frame (will be transformed)
    void updateMap(const PointCloud& cloud, Point sensor_origin);

    // returns cloud of voxel centers in global reference frame
    PointCloud generatePointCloud();

    void writePCD();
    void writeOcTree();

    // tree management

    // create new child
    MapNode* addNode(int node_id, std::string label, double map_resolution);
    // remove child
    void deleteNode(int node_id);
    inline std::vector<MapNode*>& getChildren() { return children; }
    MapNode* getChild(int node_id); // may return NULL
    inline MapNode* getParent() { return parent; }
    
    // accessor
    inline int& getId() { return id; }
    inline std::string& getLabel() { return class_label; }
    inline OcTreeMap& getMap() { return *node_map; }

    // origin and bbx 
    inline void setOrigin(octomap::pose6d o) { origin = o; }
    inline octomap::pose6d&  getOrigin() { return origin; }
    octomap::pose6d  getOriginGlobal(); // get node origin in global frame

    octomap::point3d getBBXCenter(); // get ABB local center 
    octomap::point3d getBBXCenterGlobal(); // get global BBX center 
    octomap::point3d getBBXBounds(); // get ABB bounds
    /* bool inBBX(octomap::point3d p);  */
        

    // move nodes in space

    octomap::pose6d& moveNode(octomap::pose6d transform);

    inline bool moved() const { return node_moved; } 
    inline void setMovedState()   { node_moved = true; }
    inline void resetMovedState() { node_moved = false; }

    inline bool mapUpdated() const { return map_updated; } 
    inline void setMapUpdatedState()   { map_updated = true; }
    inline void resetMapUpdatedState() { map_updated = false; }


    typedef std::vector<MapNode*>::iterator iterator;
    typedef std::vector<MapNode*>::const_iterator const_iterator;
    inline iterator begin () { return (children.begin ()); }
    inline iterator end ()   { return (children.end ()); }
    inline const_iterator begin () const { return children.begin (); }
    inline const_iterator end () const  { return children.end (); }
    inline size_t size () const { return children.size (); }

    unsigned long long int memoryUsage() const; // usage of this node
    unsigned long long int memoryUsageRecursive() const; // usage of this node

    double getExProb() const; // returns existence probability [0..1]
    void integrateHit();  // node existence confirmed
    void integrateMiss(); // node existence is in question

  protected:

    // initialize map origin and orientation
    void initializeMap(const PointCloud& cloud, bool adapt_map_orientation);
    void getOriginGlobalRecurs(octomap::pose6d& transform);


  protected:

    int             id;

    bool            initialized;
    bool            node_moved;          // this node has been moved in space
    bool            map_updated;    // node's map has been updated

    std::string     class_label;
    OcTreeMap*      node_map;  // occupancy grid map
    octomap::pose6d origin;    // origin and orientation relative to parent

    MapNode*              parent;
    std::vector<MapNode*> children;

    double   p_existence_log_odds;

  };


} // namespace 

#endif 
