#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <3d_nav_executive/arm.h>
#include <mapping_msgs/AttachedCollisionObject.h>
#include <octomap_ros/ClearBBXRegion.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h> 


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
  motionSleep = 3.0;
  basket_radius = 0.24;
  basket_height = 0.34;
  basket_x = 0.72;
  basket_y = 0;
  basket_z = -0.1;

  sleep(2);

  if(!parseArmPositionFile(poses_filename))
    ROS_ERROR("Failed to parse positions file");
  else
    ROS_INFO("Parsed the arm positions file.");
  if(!parseGoalFile(goal_filename))
    ROS_ERROR("Failed to parse positions file");
  else
    ROS_INFO("Parsed the arm positions file.");
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
      table1_goal = v;
    }
    else if(strcmp(sTemp, "table2") == 0){
      if(fscanf(file,"%f %f %f",&v[0],&v[1],&v[2]) < 1)
        ROS_ERROR("Error parsing table2 goal.");
      table2_goal = v;
    }
    else if(strcmp(sTemp, "table3") == 0){
      if(fscanf(file,"%f %f %f",&v[0],&v[1],&v[2]) < 1)
        ROS_ERROR("Error parsing table3 goal.");
      table3_goal = v;
    }
    else
      ROS_ERROR("Can't parse: %s", sTemp);
    if(fscanf(file,"%s",sTemp) < 1)
      ROS_WARN("Parsed string has length < 1. (%s)", sTemp);
  }
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

void goToPose(vector<float> v){
  //clear each layer of the start state
  const int num_base_links = 1;
  char* base_links[num_base_links] = {"base_footprint"};
  double base_xy_offsets[num_base_links] = {0.5};
  double base_z_offsets[num_base_links] = {0.125};
  clearLinks(num_base_links, base_links, base_xy_offsets, base_z_offsets);

  const int num_spine_links = 1;
  char* spine_links[num_spine_links] = {"torso_lift_link"};
  double spine_xy_offsets[num_spine_links] = {0.42};
  double spine_z_offsets[num_spine_links] = {1.0};
  clearLinks(num_spine_links, spine_links, spine_xy_offsets, spine_z_offsets);

  const int num_arm_links = 10;
  char* arm_links[num_arm_links] = {"l_elbow_flex_link","l_gripper_l_finger_tip_link",
    "l_gripper_r_finger_tip_link","l_upper_arm_roll_link","l_wrist_flex_link",
    "r_elbow_flex_link","r_gripper_l_finger_tip_link","r_gripper_r_finger_tip_link",
    "r_upper_arm_roll_link","r_wrist_flex_link"};
  double arm_offsets[num_arm_links] = {0.10, 0.03, 0.03, 0.16, 0.05, 0.10, 0.03, 0.03, 0.16, 0.05};
  clearLinks(num_arm_links, arm_links, arm_offsets, arm_offsets);

  geometry_msgs::PoseStamped goal;
  goal.pose.position.x = v[0];
  goal.pose.position.y = v[1];
  goal.pose.position.z = 0;
  btQuaternion temp;
  temp.setEulerZYX(v[2],0,0);
  goal.pose.orientation.x = temp.getX();
  goal.pose.orientation.y = temp.getY();
  goal.pose.orientation.z = temp.getZ();
  goal.pose.orientation.w = temp.getW();
  goal.header.stamp = ros::Time::now();
  goal.header.frame_id = "/map";
  goalReceived = false;
  atGoal = false;
  goalPub.publish(goal);
  sleep(1);

  while(nh.ok()){
    if(atGoal)
      break;
    ros::spinOnce();
  }
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
ros::Subscriber moveBaseStatus_sub;
ros::ServiceClient clearClient;
ros::NodeHandle nh;

tf::TransformListener tfListener;
};

int main(int argc, char** argv){
  ros::init(argc, argv, "demo_3d_nav");
  Demo3DNav demo;

  ROS_INFO("Go to table");
  demo.goToPose(demo.table2_goal);
  ROS_INFO("Go to table");
  demo.goToPose(demo.table3_goal);
  ROS_INFO("Go to table");
  demo.goToPose(demo.table2_goal);
  ROS_INFO("Go to table");
  demo.goToPose(demo.table1_goal);
}

