/*
 * collision_model_node.cpp
 *
 *  Created on: Mar 29, 2011
 *      Author: ahornung
 */
#include <ros/ros.h>
#include <octomap/OcTree.h>
#include <octomap_collision_check/collision_model.h>
#include <octomap_collision_check/octomap_object.h>

using namespace octomap;

int main (int argc, char** argv) {


  if (argc < 2) {
    printf("usage: %s environment.bt [obj.bt]\n", argv[0]);
    exit(-1);
  }

  ros::init (argc, argv, "collision_model_test");

  std::string filename (argv[1]);
  OcTree* staticTree = new OcTree(filename);
  OctomapObject* staticObject = new StaticObject(staticTree);


  if (argc > 2){
	  filename = std::string(argv[2]);
	  OcTree* objectTree = new OcTree(filename);
	  OctomapObject* movableObject = new PlanarObject(objectTree);
	  movableObject->setOrigin(pose6d(2.0, 0.0, 0.3, 0.0, 0.0, 0.0));
	  movableObject->init();
	  movableObject->setRange(0, 2.0, 8.0);
	  staticObject->addChild(movableObject);
  }

  CollisionModel model;
  model.setRoot(staticObject);

  ros::NodeHandle nh;
  ros::Rate rate(10);
  while (nh.ok()){
	  ros::spinOnce();
	  model.updateVis();

	  rate.sleep();
  }



}
