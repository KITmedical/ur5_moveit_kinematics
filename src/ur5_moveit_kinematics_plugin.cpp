#include "ur5_moveit_kinematics_plugin.hpp"

// system includes

// library includes
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <urdf/model.h>

// custom includes
#include <ahbstring.h>

PLUGINLIB_EXPORT_CLASS(ur5_moveit_kinematics::UR5MoveItKinematicsPlugin, kinematics::KinematicsBase);

namespace ur5_moveit_kinematics {

/*---------------------------------- public: -----------------------------{{{-*/
bool UR5MoveItKinematicsPlugin::getPositionIK(const geometry_msgs::Pose &ik_pose,
                   const std::vector<double> &ik_seed_state,
                   std::vector<double> &solution,
                   moveit_msgs::MoveItErrorCodes &error_code,
                   const kinematics::KinematicsQueryOptions &options) const
{

  error_code.val = error_code.SUCCESS;
  //solution = 

  return true;
}

bool UR5MoveItKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                      const std::vector<double> &ik_seed_state,
                      double timeout,
                      std::vector<double> &solution,
                      moveit_msgs::MoveItErrorCodes &error_code,
                      const kinematics::KinematicsQueryOptions &options) const
{
  return getPositionIK(ik_pose, ik_seed_state, solution, error_code, options);
}

bool UR5MoveItKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                      const std::vector<double> &ik_seed_state,
                      double timeout,
                      const std::vector<double> &consistency_limits,
                      std::vector<double> &solution,
                      moveit_msgs::MoveItErrorCodes &error_code,
                      const kinematics::KinematicsQueryOptions &options) const
{
  return getPositionIK(ik_pose, ik_seed_state, solution, error_code, options);
}

bool UR5MoveItKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                      const std::vector<double> &ik_seed_state,
                      double timeout,
                      std::vector<double> &solution,
                      const IKCallbackFn &solution_callback,
                      moveit_msgs::MoveItErrorCodes &error_code,
                      const kinematics::KinematicsQueryOptions &options) const
{
  return getPositionIK(ik_pose, ik_seed_state, solution, error_code, options);
}

bool UR5MoveItKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                      const std::vector<double> &ik_seed_state,
                      double timeout,
                      const std::vector<double> &consistency_limits,
                      std::vector<double> &solution,
                      const IKCallbackFn &solution_callback,
                      moveit_msgs::MoveItErrorCodes &error_code,
                      const kinematics::KinematicsQueryOptions &options) const
{
  return getPositionIK(ik_pose, ik_seed_state, solution, error_code, options);
}

bool UR5MoveItKinematicsPlugin::getPositionFK(const std::vector<std::string> &link_names,
                   const std::vector<double> &joint_angles,
                   std::vector<geometry_msgs::Pose> &poses) const
{
  ROS_INFO_STREAM("link_names=" << ahb::string::toString(link_names)
                  << " joint_angles=" << ahb::string::toString(joint_angles));

  // TODO ERROR if link_names != m_linkNames


  //geometry_msgs::Pose cartesianPose;
  //poses.push_back(cartesianPose);

  return true;
}

bool UR5MoveItKinematicsPlugin::initialize(const std::string& robot_description,
                const std::string& group_name,
                const std::string& base_name,
                const std::string& tip_name,
                double search_discretization)
{
  ROS_INFO_STREAM("robot_description=" << robot_description
                  << " group_name=" << group_name
                  << " base_name=" << base_name
                  << " tip_name=" << tip_name
                  << " search_discretization=" << search_discretization);
  setValues(robot_description, group_name, base_name, tip_name, search_discretization);

  ROS_INFO_STREAM("Reading robot_description from parameter server: " << robot_description);
  std::string robotUrdf;
  ros::NodeHandle nodeHandle("~/" + group_name);
  while (!nodeHandle.getParam(robot_description, robotUrdf)) {
    ROS_ERROR("Could not read robot_description from parameter server");
    ros::Duration(0.5).sleep();
  }
  urdf::Model robotModel;
  robotModel.initString(robotUrdf);
  boost::shared_ptr<const urdf::Link> link = robotModel.getRoot();
  while (true) {
    if (link->name == base_name) {
      ROS_INFO_STREAM("base_name found. Clearing previous links and joints.");
      m_linkNames.clear();
      m_jointNames.clear();
    }

    ROS_INFO_STREAM("link=" << link->name);
    m_linkNames.push_back(link->name);
    if (link->child_joints.empty()) {
      break;
    }
    boost::shared_ptr<const urdf::Joint> joint = link->child_joints[0];
    ROS_INFO_STREAM("joint=" << joint->name);
    m_jointNames.push_back(joint->name);

    link = robotModel.getLink(joint->child_link_name);
  }


  ROS_INFO_STREAM("robotModel: links=" << ahb::string::toString(m_linkNames) << " joints=" << ahb::string::toString(m_jointNames));

  return true;
}

const std::vector<std::string>& UR5MoveItKinematicsPlugin::getJointNames() const
{
  return m_jointNames;
}

const std::vector<std::string>& UR5MoveItKinematicsPlugin::getLinkNames() const
{
  return m_linkNames;
}
/*------------------------------------------------------------------------}}}-*/

/*--------------------------------- protected: ---------------------------{{{-*/
/*------------------------------------------------------------------------}}}-*/

/*---------------------------------- private: ----------------------------{{{-*/
/*------------------------------------------------------------------------}}}-*/

}
