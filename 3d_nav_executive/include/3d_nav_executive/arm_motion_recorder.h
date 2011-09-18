#include <ros/ros.h>
#include <signal.h>
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>
#include <joy/Joy.h>
#include <iostream>


class ArmMotionRecorder
{
  public:

    ArmMotionRecorder(std::string arm_name);
    ~ArmMotionRecorder();

    int run();

    void printMotion(std::string arm_name);

  private:

    bool left_record_motion_;
    bool right_record_motion_;

    unsigned int right_stop_button_;
    unsigned int right_start_button_;
    unsigned int left_stop_button_;
    unsigned int left_start_button_;


    std::string lfilename_;
    std::string rfilename_;
    std::string arm_name_;
    FILE* lfile_;
    FILE* rfile_;

    ros::NodeHandle root_handle_;
    ros::NodeHandle node_handle_;
    ros::Subscriber left_arm_state_subscriber_;
    ros::Subscriber right_arm_state_subscriber_;
    ros::Subscriber joy_subscriber_;

    std::string reference_frame_;
    std::vector<std::string> joint_names_;

    std::vector<std::vector<double> >  left_motion_;
    std::vector<std::vector<double> >  right_motion_;

    void leftArmControllerCallback(const pr2_controllers_msgs::JointTrajectoryControllerStateConstPtr &state);
    void rightArmControllerCallback(const pr2_controllers_msgs::JointTrajectoryControllerStateConstPtr &state);

    void joyCallback(const joy::JoyConstPtr &joy);
};

