#include "ur5_moveit_kinematics_plugin.hpp"

// system includes

// library includes
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <urdf/model.h>
#include <tf/transform_datatypes.h>

// custom includes
#include <ahbstring.h>
#include <ur_kinematics/ur_kin.h>

PLUGINLIB_EXPORT_CLASS(ur5_moveit_kinematics::UR5MoveItKinematicsPlugin, kinematics::KinematicsBase);

namespace ur5_moveit_kinematics {

/*---------------------------------- public: -----------------------------{{{-*/
#define UR5_JOINTS 6
int
rowmajoridx(int row, int col, int numcols)
{
  return row * numcols + col;
}

void
jsolprint(const double* joint_solutions, unsigned joint_solutions_count)
{
  printf("joint_solutions:\n");
  for (unsigned solutionIdx = 0; solutionIdx < joint_solutions_count; solutionIdx++) {
    printf("%d: ", solutionIdx);
    for (unsigned jointIdx = 0; jointIdx < UR5_JOINTS; jointIdx++) {
      printf("%2.3lf ", joint_solutions[rowmajoridx(solutionIdx,jointIdx,UR5_JOINTS)]);
    }
    printf("\n");
  }
  printf("\n");
}

bool UR5MoveItKinematicsPlugin::getPositionIK(const geometry_msgs::Pose &ik_pose,
                   const std::vector<double> &ik_seed_state,
                   std::vector<double> &solution,
                   moveit_msgs::MoveItErrorCodes &error_code,
                   const kinematics::KinematicsQueryOptions &options) const
{
  ROS_INFO_STREAM("ik_seed_state=" << ahb::string::toString(ik_seed_state));

  // C&P from ur5_safe_cartesian/src/UR5SafeCartesian.cpp
  tf::Pose tfpose;
  tf::poseMsgToTF(ik_pose, tfpose);

  double T[4*4];
  for (unsigned col = 0; col < 3; col++) {
    T[rowmajoridx(3,col,4)] = 0;
  }
  T[rowmajoridx(3,3,4)] = 1;
  tf::Matrix3x3 rotationMatrix = tfpose.getBasis();
  for (unsigned row = 0; row < 3; row++) {
    for (unsigned col = 0; col < 3; col++) {
       T[rowmajoridx(row,col,4)] = rotationMatrix[row][col];
    }
  }
  tf::Vector3 translationVector = tfpose.getOrigin();
  for (unsigned row = 0; row < 3; row++) {
    T[rowmajoridx(row,3,4)] = translationVector[row];
  }

  double joint_solutions[8*6];
  unsigned joint_solutions_count = ur_kinematics::inverse(T, joint_solutions);
  printf("raw solutions:\n");
  jsolprint(joint_solutions, joint_solutions_count);
  if (joint_solutions_count == 0) {
    ROS_ERROR_STREAM("ur_kinematics::inverse(): No solution found");
    error_code.val = error_code.NO_IK_SOLUTION;
    return false;
  }

  // use solution closest to current pos (in joint space)
  int minDistSolutionIdx = -1;
  double minDistSolution = 10e6;
  for (unsigned solutionIdx = 0; solutionIdx < joint_solutions_count; solutionIdx++) {
    bool validSolution = true;
    // ur_kinematics::inverse returns angles in [0,2*PI) since all axis are +-2*PI, we want to use value of interval we are in
    printf("joint_optimal_interval: ");
    for (unsigned jointIdx = 0; jointIdx < UR5_JOINTS; jointIdx++) {
      if (isnan(joint_solutions[rowmajoridx(solutionIdx,jointIdx,UR5_JOINTS)])) {
        printf("NaN!");
        validSolution = false;
        break;
      }
      double distpos = abs(ik_seed_state[jointIdx] - joint_solutions[rowmajoridx(solutionIdx,jointIdx,UR5_JOINTS)]); 
      double distneg = abs(ik_seed_state[jointIdx] - (joint_solutions[rowmajoridx(solutionIdx,jointIdx,UR5_JOINTS)] - 2*M_PI)); 
      double ival;
      if (distneg < distpos) {
        ival = -2 * M_PI;
      } else {
        ival = 0;
      }
      joint_solutions[rowmajoridx(solutionIdx,jointIdx,UR5_JOINTS)] += ival;
      printf("%2.3lf ", ival);
    }
    printf("\n");
    if (!validSolution) {
      continue;
    }

    double dist = 0;
    for (unsigned jointIdx = 0; jointIdx < UR5_JOINTS; jointIdx++) {
      double d = joint_solutions[rowmajoridx(solutionIdx,jointIdx,UR5_JOINTS)] - ik_seed_state[jointIdx];
      while (d > M_PI) {
        d -= 2*M_PI;
      }
      while (d < -M_PI) {
        d += 2*M_PI;
      }
      dist += d * d;
    }
    printf("Solution %d dist=%lf\n", solutionIdx, dist);
    if (dist < minDistSolution) {
      minDistSolution = dist;
      minDistSolutionIdx = solutionIdx;
    }
  }
  if (minDistSolutionIdx == -1) {
    ROS_ERROR_STREAM("ur_kinematics::inverse(): No minDistSolution found");
    error_code.val = error_code.NO_IK_SOLUTION;
    return false;
  }
  printf("optimal interval solutions:\n");
  jsolprint(joint_solutions, joint_solutions_count);
  printf("Using solution %d\n", minDistSolutionIdx);

  error_code.val = error_code.SUCCESS;
  for (unsigned jointIdx = 0; jointIdx < UR5_JOINTS; jointIdx++) {
    solution.push_back(joint_solutions[rowmajoridx(minDistSolutionIdx,jointIdx,UR5_JOINTS)]);
  }
  ROS_INFO_STREAM("solution=" << ahb::string::toString(solution));

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

  double T[4*4];
  ur_kinematics::forward(&joint_angles[0], T);
  //tprint(T);
  tf::Transform cartesianPose;
  tf::Matrix3x3 rotationMatrix;
  for (unsigned row = 0; row < 3; row++) {
    for (unsigned col = 0; col < 3; col++) {
       rotationMatrix[row][col] = T[rowmajoridx(row,col,4)];
    }
  }
  tf::Vector3 translationVector;
  for (unsigned row = 0; row < 3; row++) {
    translationVector[row] = T[rowmajoridx(row,3,4)];
  }
  cartesianPose.setBasis(rotationMatrix);
  cartesianPose.setOrigin(translationVector);

  geometry_msgs::Pose cartesianPoseMsg;
  tf::poseTFToMsg(cartesianPose, cartesianPoseMsg);
  poses.push_back(cartesianPoseMsg);

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
