#include <iostream>
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/Pose.h>
#include <tf/tf.h>
#include <geometry_msgs/Quaternion.h>
#include <kinematics_msgs/GetPositionIK.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>

#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>
#include <kinematics_msgs/GetKinematicSolverInfo.h>
#include <kinematics_msgs/GetPositionFK.h>


using namespace std;

typedef actionlib::SimpleActionClient< pr2_controllers_msgs::JointTrajectoryAction > TrajClient;

class Arm
{
  public: 
    
    Arm(std::string arm_name);
    ~Arm();

    void stopArm();

    void sendArmToPose(double pose[], double move_time);
    void sendArmToPoseQuaternion(double pose[], double move_time);
    void sendArmToPoses(std::vector<std::vector<double> > &poses, std::vector<double> move_times);

    void sendArmToConfiguration(double configuration[], double move_time);
    void sendArmToConfigurations(std::vector<std::vector<double> > &configurations, std::vector<double> move_times);
    void sendArmToConfiguration(std::vector<float> configuration, double move_time);

    bool computeIK(const geometry_msgs::Pose &pose, std::vector<double> jnt_pos, std::vector<double> &solution);
    bool performFK(const std::vector<double> jnt_pos, std::vector<double> &cart_pose);

    void getCurrentArmConfiguration(vector<double>& current_angles);
    void getCurrentArmPose(vector<double>& cpose);

  private:
    ros::NodeHandle nh_;
    //ros::Subscriber joint_states_subscriber_;
  
    std::string arm_name_;
    std::string ik_service_name_;
    std::string fk_service_name_;
    std::vector<std::string> joint_names_;
    std::string reference_frame_;

    std::string controller_state_name_;

    TrajClient* traj_client_;
};


