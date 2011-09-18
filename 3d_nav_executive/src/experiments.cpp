#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <3d_nav_executive/arm.h>
#include <mapping_msgs/AttachedCollisionObject.h>
#include <octomap_ros/ClearBBXRegion.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h> 
#include <sbpl_lattice_planner_layer_3d/sbpl_lattice_planner_3d.h>


using namespace std;

class Demo3DNav{
  public:

Demo3DNav(){
  larm = new Arm(std::string("left"));
  rarm = new Arm(std::string("right"));
  std::string poses_filename;
  std::string goal_filename;
  ros::NodeHandle ph("~");
  ph.param<std::string>("arm_positions_filename", poses_filename, " ");
  ph.param<std::string>("goal_positions_filename", goal_filename, " ");
  goalPub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
  collisionPub = nh.advertise<mapping_msgs::CollisionObject>("collision_object", 1);
  attachPub = nh.advertise<mapping_msgs::AttachedCollisionObject>("attached_collision_object", 1);
  moveBaseStatus_sub = nh.subscribe("/move_base/status", 1, &Demo3DNav::moveBaseStatusCallback, this);
  clearClient = nh.serviceClient<octomap_ros::ClearBBXRegionRequest>("/octomap_server_combined/clear_bbx");
  markerPub = nh.advertise<visualization_msgs::Marker>("basket", 1);
  marker_pub = nh.advertise<visualization_msgs::Marker>("path",1);
  motionSleep = 3.0;
  basket_radius = 0.24;
  basket_height = 0.34;
  basket_x = 0.72;
  basket_y = 0;
  basket_z = -0.1;
  goals.resize(6);

  sleep(2);

  if(!parseArmPositionFile(poses_filename))
    ROS_ERROR("Failed to parse positions file");
  else
    ROS_INFO("Parsed the arm positions file.");
  if(!parseGoalFile(goal_filename))
    ROS_ERROR("Failed to parse positions file");
  else
    ROS_INFO("Parsed the arm positions file.");

  global_costmap_ros_ = new costmap_2d::Costmap2DROS("global_costmap", tfListener);
  global_costmap_ros_->pause();
  controller_costmap_ros_ = new costmap_2d::Costmap2DROS("local_costmap", tfListener);
  controller_costmap_ros_->pause();

  planner1Layer.initialize("SBPLLatticePlannerLayer3D", global_costmap_ros_);
  planner1Layer.initialize("", controller_costmap_ros_);
  planner1Layer.useMultiLayer(false);
  plannerMultiLayer.initialize("SBPLLatticePlannerLayer3D", global_costmap_ros_);
  plannerMultiLayer.initialize("", controller_costmap_ros_);
  plannerMultiLayer.useMultiLayer(true);

  global_costmap_ros_->start();
  controller_costmap_ros_->start();
}

bool parseArmPositionFile(std::string filename){
  FILE* file = fopen(filename.c_str(), "r");
  char sTemp[1024];
  std::vector<float> v(7,0);

  if(file == NULL){
    ROS_ERROR("ERROR: unable to open the file %s. Exiting.",filename.c_str());
    return false;
  }

  if(fscanf(file,"%s",sTemp) < 1)
    ROS_WARN("Parsed string has length < 1.");

  while(!feof(file) && strlen(sTemp) != 0){
    if(strcmp(sTemp, "l_prep") == 0){
      if(fscanf(file,"%f %f %f %f %f %f %f",&v[0],&v[1],&v[2],&v[3],&v[4],&v[5],&v[6]) < 1)
        ROS_ERROR("Error parsing l_prep pose.");
      l_prep_pose = v;
    }
    else if(strcmp(sTemp, "l_pinch") == 0){
      if(fscanf(file,"%f %f %f %f %f %f %f",&v[0],&v[1],&v[2],&v[3],&v[4],&v[5],&v[6]) < 1)
        ROS_ERROR("Error parsing l_pinch pose.");
      l_pinch_pose = v;
    }
    else if(strcmp(sTemp, "l_lift") == 0){
      if(fscanf(file,"%f %f %f %f %f %f %f",&v[0],&v[1],&v[2],&v[3],&v[4],&v[5],&v[6]) < 1)
        ROS_ERROR("Error parsing l_lift pose.");
      l_lift_pose = v;
    }
    else if(strcmp(sTemp, "r_prep") == 0){
      if(fscanf(file,"%f %f %f %f %f %f %f",&v[0],&v[1],&v[2],&v[3],&v[4],&v[5],&v[6]) < 1)
        ROS_ERROR("Error parsing r_prep pose.");
      r_prep_pose = v;
    }
    else if(strcmp(sTemp, "r_pinch") == 0){
      if(fscanf(file,"%f %f %f %f %f %f %f",&v[0],&v[1],&v[2],&v[3],&v[4],&v[5],&v[6]) < 1)
        ROS_ERROR("Error parsing r_pinch pose.");
      r_pinch_pose = v;
    }
    else if(strcmp(sTemp, "r_lift") == 0){
      if(fscanf(file,"%f %f %f %f %f %f %f",&v[0],&v[1],&v[2],&v[3],&v[4],&v[5],&v[6]) < 1)
        ROS_ERROR("Error parsing r_lift pose.");
      r_lift_pose = v;
    }
    else if(strcmp(sTemp, "l_init") == 0){
      if(fscanf(file,"%f %f %f %f %f %f %f",&v[0],&v[1],&v[2],&v[3],&v[4],&v[5],&v[6]) < 1)
        ROS_ERROR("Error parsing l_init pose.");
      l_init_pose = v;
    }
    else if(strcmp(sTemp, "r_init") == 0){
      if(fscanf(file,"%f %f %f %f %f %f %f",&v[0],&v[1],&v[2],&v[3],&v[4],&v[5],&v[6]) < 1)
        ROS_ERROR("Error parsing r_init pose.");
      r_init_pose = v;
    }
    else
      ROS_ERROR("Can't parse: %s", sTemp);
    if(fscanf(file,"%s",sTemp) < 1)
      ROS_WARN("Parsed string has length < 1. (%s)", sTemp);
  }
  fclose(file);
  return true;
}


bool parseGoalFile(std::string filename){
  FILE* file = fopen(filename.c_str(), "r");
  char sTemp[1024];
  std::vector<float> v(3,0);

  if(file == NULL){
    ROS_ERROR("ERROR: unable to open the file %s. Exiting.",filename.c_str());
    return false;
  }

  if(fscanf(file,"%s",sTemp) < 1)
    ROS_WARN("Parsed string has length < 1.");

  while(!feof(file) && strlen(sTemp) != 0){
    if(strcmp(sTemp, "table1") == 0){
      if(fscanf(file,"%f %f %f",&v[0],&v[1],&v[2]) < 1)
        ROS_ERROR("Error parsing table1 goal.");
      //table1_goal = v;
      goals[0] = v;
    }
    else if(strcmp(sTemp, "table2") == 0){
      if(fscanf(file,"%f %f %f",&v[0],&v[1],&v[2]) < 1)
        ROS_ERROR("Error parsing table2 goal.");
      //table2_goal = v;
      goals[1] = v;
    }
    else if(strcmp(sTemp, "table3") == 0){
      if(fscanf(file,"%f %f %f",&v[0],&v[1],&v[2]) < 1)
        ROS_ERROR("Error parsing table3 goal.");
      goals[2] = v;
      //table3_goal = v;
    }
    else if(strcmp(sTemp, "table4") == 0){
      if(fscanf(file,"%f %f %f",&v[0],&v[1],&v[2]) < 1)
        ROS_ERROR("Error parsing table3 goal.");
      goals[3] = v;
    }
    else if(strcmp(sTemp, "table5") == 0){
      if(fscanf(file,"%f %f %f",&v[0],&v[1],&v[2]) < 1)
        ROS_ERROR("Error parsing table3 goal.");
      goals[4] = v;
    }
    else if(strcmp(sTemp, "table6") == 0){
      if(fscanf(file,"%f %f %f",&v[0],&v[1],&v[2]) < 1)
        ROS_ERROR("Error parsing table3 goal.");
      goals[5] = v;
    }
    else
      ROS_ERROR("Can't parse: %s", sTemp);
    if(fscanf(file,"%s",sTemp) < 1)
      ROS_WARN("Parsed string has length < 1. (%s)", sTemp);
  }
  fclose(file);
  return true;
}

bool parsePathPositionFile(std::string filename){
  FILE* file = fopen(filename.c_str(), "r");
  char sTemp[1024];
  std::vector<int> v(3,0);

  if(file == NULL){
    ROS_ERROR("ERROR: unable to open the file %s. Exiting.",filename.c_str());
    return false;
  }

  int i=0;
  visualization_msgs::Marker marker;
  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = "/map";
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 0.025;
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;
  marker.color.a = 1.0;
  marker.ns = "path";
  marker.id = 0;
  marker.lifetime = ros::Duration();
  while(!feof(file)){
    if(fscanf(file,"%d %d %d",&v[0],&v[1],&v[2]) < 1)
      ROS_ERROR("Error parsing path pose");
    if(i==0 || i==17 || i==40 || i==60 || i==70 || i==80)
      drawPose(v[0]*.025+.0125-12.487500,v[1]*.025+.0125-12.487500,v[2]*2*M_PI/16,i);
    i++;
    geometry_msgs::Point p;
    p.x = v[0]*.025+.0125-12.487500;
    p.y = v[1]*.025+.0125-12.487500;
    p.z = 0;
    marker.points.push_back(p);
  }
  marker_pub.publish(marker);
  sleep(5.0);
  fclose(file);
  return true;
}

void moveBaseStatusCallback(const actionlib_msgs::GoalStatusArrayConstPtr& msg){
  if(!msg->status_list.empty()){
    if(msg->status_list.back().status != msg->status_list.back().SUCCEEDED){
      ROS_INFO("goal received..\n");
      goalReceived = true;
    }
    if(goalReceived && msg->status_list.back().status == msg->status_list.back().SUCCEEDED){
      ROS_INFO("we are at the goal!\n");
      atGoal = true;
    }
  }
}

bool clearLinks(int num_links, char** links, double* xy_offsets, double* z_offsets){
  geometry_msgs::PointStamped vin;
  vin.point.x = 0;
  vin.point.y = 0;
  vin.point.z = 0;
  vin.header.stamp = ros::Time(0);

  double max_x = -100000;
  double max_y = -100000;
  double max_z = -100000;
  double min_x = 100000;
  double min_y = 100000;
  double min_z = 100000;
  for(int i=0; i<num_links; i++){
    vin.header.frame_id = links[i];
    geometry_msgs::PointStamped vout;
    tfListener.transformPoint("map",vin,vout);
    max_x = std::max(max_x,vout.point.x + xy_offsets[i]);
    min_x = std::min(min_x,vout.point.x - xy_offsets[i]);
    max_y = std::max(max_y,vout.point.y + xy_offsets[i]);
    min_y = std::min(min_y,vout.point.y - xy_offsets[i]);
    max_z = std::max(max_z,vout.point.z + z_offsets[i]);
    min_z = std::min(min_z,vout.point.z - z_offsets[i]);
  }

  octomap_ros::ClearBBXRegionRequest req;
  octomap_ros::ClearBBXRegionResponse res;
  req.max.x = max_x;
  req.max.y = max_y;
  req.max.z = max_z;
  req.min.x = min_x;
  req.min.y = min_y;
  req.min.z = min_z;
  return clearClient.call(req,res);
}

void goToPose(vector<float> v1, vector<float> v2){
  printf("%f %f %f -> %f %f %f\n",v1[0],v1[1],v1[2],v2[0],v2[1],v2[2]);
  geometry_msgs::PoseStamped start;
  start.pose.position.x = v1[0];
  start.pose.position.y = v1[1];
  start.pose.position.z = 0;
  btQuaternion temp;
  temp.setEulerZYX(v1[2],0,0);
  start.pose.orientation.x = temp.getX();
  start.pose.orientation.y = temp.getY();
  start.pose.orientation.z = temp.getZ();
  start.pose.orientation.w = temp.getW();
  start.header.stamp = ros::Time::now();
  start.header.frame_id = "/map";

  geometry_msgs::PoseStamped goal;
  goal.pose.position.x = v2[0];
  goal.pose.position.y = v2[1];
  goal.pose.position.z = 0;
  temp.setEulerZYX(v2[2],0,0);
  goal.pose.orientation.x = temp.getX();
  goal.pose.orientation.y = temp.getY();
  goal.pose.orientation.z = temp.getZ();
  goal.pose.orientation.w = temp.getW();
  goal.header.stamp = ros::Time::now();
  goal.header.frame_id = "/map";

  std::vector<geometry_msgs::PoseStamped> plan;
  planner1Layer.makePlan(start,goal,plan);
  plan.clear();
  plannerMultiLayer.makePlan(start,goal,plan);
}

void initPose(){
  rarm->sendArmToConfiguration(r_init_pose, 2.0);
  larm->sendArmToConfiguration(l_init_pose, 2.0);
  sleep(motionSleep);
}

void pickup(){
  rarm->sendArmToConfiguration(r_prep_pose, 2.0);
  larm->sendArmToConfiguration(l_prep_pose, 2.0);
  sleep(motionSleep);
  rarm->sendArmToConfiguration(r_pinch_pose, 2.0);
  larm->sendArmToConfiguration(l_pinch_pose, 2.0);
  sleep(motionSleep);
  rarm->sendArmToConfiguration(r_lift_pose, 2.0);
  larm->sendArmToConfiguration(l_lift_pose, 2.0);
  sleep(motionSleep);

  mapping_msgs::CollisionObject c;
  c.header.stamp = ros::Time(0);
  c.header.frame_id = "torso_lift_link";
  c.id = "basket";
  c.operation.operation = mapping_msgs::CollisionObjectOperation::ADD;
  geometric_shapes_msgs::Shape s;
  s.type = geometric_shapes_msgs::Shape::CYLINDER;
  s.dimensions.push_back(basket_radius);
  s.dimensions.push_back(basket_height);
  c.shapes.push_back(s);
  geometry_msgs::Pose p;
  p.position.x = basket_x;
  p.position.y = basket_y;
  p.position.z = basket_z;
  p.orientation.x = 0;
  p.orientation.y = 0;
  p.orientation.z = 0;
  p.orientation.w = 1;
  c.poses.push_back(p);

  collisionPub.publish(c);
  sleep(1.0);

  mapping_msgs::AttachedCollisionObject m;
  m.link_name = "torso_lift_link";
  string touch_links[30] = {"r_gripper_palm_link", "r_gripper_r_finger_link", "r_gripper_l_finger_link",
                            "r_gripper_r_finger_tip_link", "r_gripper_l_finger_tip_link", "r_wrist_roll_link", "r_wrist_flex_link", "r_forearm_link", "r_gripper_motor_accelerometer_link",
                            "r_elbow_flex_link","r_forearm_roll_link","r_shoulder_lift_link","r_shoulder_pan_link","r_upper_arm_link","r_upper_arm_roll_link",
                            "l_gripper_palm_link", "l_gripper_r_finger_link", "l_gripper_l_finger_link",
                            "l_gripper_r_finger_tip_link", "l_gripper_l_finger_tip_link", "l_wrist_roll_link", "l_wrist_flex_link", "l_forearm_link", "l_gripper_motor_accelerometer_link",
                            "l_elbow_flex_link","l_forearm_roll_link","l_shoulder_lift_link","l_shoulder_pan_link","l_upper_arm_link","l_upper_arm_roll_link"};
  for(int i=0; i<30; i++)
    m.touch_links.push_back(touch_links[i]);
  m.object.header.stamp = ros::Time(0);
  m.object.header.frame_id = "torso_lift_link";
  m.object.id = "basket";
  m.object.operation.operation = mapping_msgs::CollisionObjectOperation::ATTACH_AND_REMOVE_AS_OBJECT;
  m.object.shapes.push_back(s);
  m.object.poses.push_back(p);

  attachPub.publish(m);
  sleep(1.0);

  //get the right gripper in the map frame
  geometry_msgs::PointStamped vin;
  vin.point.x = basket_x;
  vin.point.y = basket_y;
  vin.point.z = basket_z;
  vin.header.stamp = ros::Time(0);
  vin.header.frame_id = "torso_lift_link";
  geometry_msgs::PointStamped vout;
  tfListener.transformPoint("map",vin,vout);
  double basket_padding = 0.08;

  octomap_ros::ClearBBXRegionRequest req;
  octomap_ros::ClearBBXRegionResponse res;
  req.max.x = vout.point.x + (basket_radius + basket_padding);
  req.max.y = vout.point.y + (basket_radius + basket_padding);
  req.max.z = vout.point.z + (basket_height/2 + basket_padding);
  req.min.x = vout.point.x - (basket_radius + basket_padding);
  req.min.y = vout.point.y - (basket_radius + basket_padding);
  req.min.z = vout.point.z - (basket_height/2 + basket_padding);
  bool ret = clearClient.call(req,res);
  printf("clear success? %d\n",ret);

  visualization_msgs::Marker marker;
  marker.header.stamp = ros::Time(0);
  marker.header.frame_id = "map";
  marker.type = visualization_msgs::Marker::CYLINDER;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position = vout.point;
  marker.pose.orientation.w = 1;
  marker.scale.x = basket_radius*2;
  marker.scale.y = basket_radius*2;
  marker.scale.z = basket_height;
  marker.color.r = 0.0f;
  marker.color.g = 0.0f;
  marker.color.b = 1.0f;
  marker.color.a = 1.0;
  marker.ns = "basket";
  marker.id = 0;
  marker.lifetime = ros::Duration(120.0);
  markerPub.publish(marker);
}

void putdown(){
  rarm->sendArmToConfiguration(r_lift_pose, 2.0);
  larm->sendArmToConfiguration(l_lift_pose, 2.0);
  sleep(motionSleep);
  rarm->sendArmToConfiguration(r_pinch_pose, 2.0);
  larm->sendArmToConfiguration(l_pinch_pose, 2.0);
  sleep(motionSleep);
  rarm->sendArmToConfiguration(r_prep_pose, 2.0);
  larm->sendArmToConfiguration(l_prep_pose, 2.0);
  sleep(motionSleep);

  mapping_msgs::AttachedCollisionObject m;
  m.link_name = "torso_lift_link";
  string touch_links[30] = {"r_gripper_palm_link", "r_gripper_r_finger_link", "r_gripper_l_finger_link",
                            "r_gripper_r_finger_tip_link", "r_gripper_l_finger_tip_link", "r_wrist_roll_link", "r_wrist_flex_link", "r_forearm_link", "r_gripper_motor_accelerometer_link",
                            "r_elbow_flex_link","r_forearm_roll_link","r_shoulder_lift_link","r_shoulder_pan_link","r_upper_arm_link","r_upper_arm_roll_link",
                            "l_gripper_palm_link", "l_gripper_r_finger_link", "l_gripper_l_finger_link",
                            "l_gripper_r_finger_tip_link", "l_gripper_l_finger_tip_link", "l_wrist_roll_link", "l_wrist_flex_link", "l_forearm_link", "l_gripper_motor_accelerometer_link",
                            "l_elbow_flex_link","l_forearm_roll_link","l_shoulder_lift_link","l_shoulder_pan_link","l_upper_arm_link","l_upper_arm_roll_link"};
  for(int i=0; i<30; i++)
    m.touch_links.push_back(touch_links[i]);
  m.object.header.stamp = ros::Time::now();
  m.object.header.frame_id = "torso_lift_link";
  m.object.id = "basket";
  m.object.operation.operation = mapping_msgs::CollisionObjectOperation::DETACH_AND_ADD_AS_OBJECT;
  geometric_shapes_msgs::Shape s;
  s.type = geometric_shapes_msgs::Shape::CYLINDER;
  s.dimensions.push_back(basket_radius);
  s.dimensions.push_back(basket_height);
  m.object.shapes.push_back(s);
  geometry_msgs::Pose p;
  p.position.x = basket_x;
  p.position.y = basket_y;
  p.position.z = basket_z;
  p.orientation.x = 0;
  p.orientation.y = 0;
  p.orientation.z = 0;
  p.orientation.w = 1;
  m.object.poses.push_back(p);

  attachPub.publish(m);
}

void finish(){
  planner1Layer.close_files();
  plannerMultiLayer.close_files();
}

void drawPose(double x, double y, double th, int i){
  vector<float> v;
  v.push_back(x);
  v.push_back(y);
  v.push_back(th);
  drawPose(v,i);
}

void drawPose(vector<float> v,int i){
  sleep(5);
  printf("publishing...\n");

  geometry_msgs::PoseStamped vin;
  vin.pose.position.x = v[0];
  vin.pose.position.y = v[1];
  vin.pose.position.z = 0;

  btQuaternion temp;
  temp.setEulerZYX(v[2],0,0);
  vin.pose.orientation.x = temp.getX();
  vin.pose.orientation.y = temp.getY();
  vin.pose.orientation.z = temp.getZ();
  vin.pose.orientation.w = temp.getW();
  vin.header.stamp = ros::Time(0);

  vin.header.frame_id = "map";
  geometry_msgs::PoseStamped vout;
  tfListener.transformPose("map",vin,vout);

  plannerMultiLayer.drawPose(vout.pose.position.x+12.487500,vout.pose.position.y+12.487500, 2*atan2(vout.pose.orientation.z, vout.pose.orientation.w),i);
}

vector<float> r_prep_pose;
vector<float> r_pinch_pose;
vector<float> r_lift_pose;
vector<float> l_prep_pose;
vector<float> l_pinch_pose;
vector<float> l_lift_pose;
vector<float> l_init_pose;
vector<float> r_init_pose;
vector<float> table1_goal;
vector<float> table2_goal;
vector<float> table3_goal;
Arm* larm;
Arm* rarm;

bool atGoal;
bool goalReceived;
double motionSleep;
double basket_radius;
double basket_height;
double basket_x;
double basket_y;
double basket_z;

ros::Publisher attachPub;
ros::Publisher collisionPub;
ros::Publisher goalPub;
ros::Publisher markerPub;
ros::Publisher marker_pub;
ros::Subscriber moveBaseStatus_sub;
ros::ServiceClient clearClient;
ros::NodeHandle nh;

sbpl_lattice_planner_layer_3d::SBPLLatticePlannerLayer3D planner1Layer;
sbpl_lattice_planner_layer_3d::SBPLLatticePlannerLayer3D plannerMultiLayer;
costmap_2d::Costmap2DROS* global_costmap_ros_;
costmap_2d::Costmap2DROS* controller_costmap_ros_;

vector< vector<float> > goals;

tf::TransformListener tfListener;
};

int main(int argc, char** argv){
  ros::init(argc, argv, "demo_3d_nav");
  Demo3DNav demo;

  /*
  for(int i=0; i<6; i++)
    demo.drawPose(demo.goals[i],i);
  */

  //demo.parsePathPositionFile("/u/mphillips/ros/3dnav/stacks/3d_navigation/3d_nav_executive/config/path.txt");

  for(int i=0; i<6; i++){
    for(int j=0; j<6; j++){
      if(i==j)
        continue;
      printf("plan from %d to %d\n",i,j);
      demo.goToPose(demo.goals[i],demo.goals[j]);
    }
  }
  demo.finish();

  exit(0);
}

