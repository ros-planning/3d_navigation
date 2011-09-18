#include <3d_nav_executive/arm_motion_recorder.h>

ArmMotionRecorder::ArmMotionRecorder(std::string arm_name) : arm_name_(arm_name)
{
  left_record_motion_ = false;
  right_record_motion_ = false;
  reference_frame_ = "base_link";

  joint_names_.push_back("l_shoulder_pan_joint");
  joint_names_.push_back("l_shoulder_lift_joint");
  joint_names_.push_back("l_upper_arm_roll_joint");
  joint_names_.push_back("l_elbow_flex_joint");
  joint_names_.push_back("l_forearm_roll_joint");
  joint_names_.push_back("l_wrist_flex_joint");
  joint_names_.push_back("l_wrist_roll_joint");
  left_arm_state_subscriber_ = root_handle_.subscribe("l_arm_controller/state_throttle", 1, &ArmMotionRecorder::leftArmControllerCallback,this);
  joint_names_.push_back("r_shoulder_pan_joint");
  joint_names_.push_back("r_shoulder_lift_joint");
  joint_names_.push_back("r_upper_arm_roll_joint");
  joint_names_.push_back("r_elbow_flex_joint");
  joint_names_.push_back("r_forearm_roll_joint");
  joint_names_.push_back("r_wrist_flex_joint");
  joint_names_.push_back("r_wrist_roll_joint");
  right_arm_state_subscriber_ = root_handle_.subscribe("r_arm_controller/state_throttle", 1, &ArmMotionRecorder::rightArmControllerCallback,this);

  joy_subscriber_ = root_handle_.subscribe("joy_throttle", 1, &ArmMotionRecorder::joyCallback, this);

  
  node_handle_.param<std::string>("left_filename", lfilename_, "/tmp/lmotion.txt");
  node_handle_.param<std::string>("right_filename", rfilename_, "/tmp/rmotion.txt");

  lfile_ = fopen(lfilename_.c_str(), "w");
  rfile_ = fopen(rfilename_.c_str(), "w");

  right_stop_button_ = 13;
  right_start_button_ = 15;

  left_stop_button_ = 5;
  left_start_button_ = 7;
}

ArmMotionRecorder::~ArmMotionRecorder()
{
  fclose(lfile_);
  fclose(rfile_);
}

int ArmMotionRecorder::run()
{
  ros::spin();
  return 0;
}

void ArmMotionRecorder::joyCallback(const joy::JoyConstPtr &joy)
{
  printf("I got the callback!\n");
  if(joy->buttons.size() > left_stop_button_ && joy->buttons[left_stop_button_])
  {
    ROS_INFO("STOP RECORDING LEFT ARM");
    left_record_motion_ = false;
    printMotion(std::string("left_arm"));
    left_motion_.clear();
  }
  else if(joy->buttons.size() > left_start_button_ && joy->buttons[left_start_button_])
  {
    ROS_INFO("START RECORDING LEFT ARM");
    left_record_motion_ = true;
  }
  else if(joy->buttons.size() > right_start_button_ && joy->buttons[right_start_button_])
  {
    ROS_INFO("START RECORDING RIGHT ARM");
    right_record_motion_ = true;
  }
  else if(joy->buttons.size() > right_stop_button_ && joy->buttons[right_stop_button_])
  {
    ROS_INFO("STOP RECORDING RIGHT ARM");
    right_record_motion_ = false;
    printMotion(std::string("right_arm"));
    right_motion_.clear();
  }
}

void ArmMotionRecorder::leftArmControllerCallback(const pr2_controllers_msgs::JointTrajectoryControllerStateConstPtr &state)
{
  if(left_record_motion_)
  {
    printf("recording joint positions\n");
    left_motion_.push_back(state->actual.positions);
    for(size_t i = 0; i < state->actual.positions.size(); ++i)
      fprintf(lfile_, "%0.4f ", state->actual.positions[i]);
    fprintf(lfile_, "\n");
  }
}

void ArmMotionRecorder::rightArmControllerCallback(const pr2_controllers_msgs::JointTrajectoryControllerStateConstPtr &state)
{
  if(right_record_motion_)
  {
    printf("recording joint positions\n");
    right_motion_.push_back(state->actual.positions);
    for(size_t i = 0; i < state->actual.positions.size(); ++i)
      fprintf(rfile_, "%0.4f ", state->actual.positions[i]);
    fprintf(rfile_, "\n");
  }
}

/*
ArmMotionRecorder::recordMotion(int rate)
{
  record_motion_ = true;

//  ROS_INFO("press ENTER to stop recording");

  ros::Rate r(rate);
  while (record_motion_)  
  {
    ros::spinOnce();
    r.sleep();
  }

}
*/

void ArmMotionRecorder::printMotion(std::string arm_name)
{
  if(arm_name.compare("left_arm")  == 0)
  {
    for(size_t i = 0; i < left_motion_.size(); ++i)
    {
      printf("[%d] ", int(i));
      for(size_t j = 0; j < left_motion_[i].size(); ++j)
      {
        printf("%0.3f ", left_motion_[i][j]);
      }
      printf("\n");
    }
  }
  else
  {
    for(size_t i = 0; i < right_motion_.size(); ++i)
    {
      printf("[%d] ", int(i));
      for(size_t j = 0; j < right_motion_[i].size(); ++j)
      {
        printf("%0.3f ", right_motion_[i][j]);
      }
      printf("\n");
    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "record_arm");
  ArmMotionRecorder arm(std::string("right_arm"));

  return arm.run();
}
  
