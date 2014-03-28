#ifndef _UR5_MOVEIT_KINEMATICS_PLUGIN_H_
#define _UR5_MOVEIT_KINEMATICS_PLUGIN_H_

// system includes

// library includes
#include <moveit/kinematics_base/kinematics_base.h>

// custom includes


// forward declarations

namespace ur5_moveit_kinematics {

class UR5MoveItKinematicsPlugin
  : public kinematics::KinematicsBase
{
  public:
    // enums

    // typedefs

    // const static member variables
 
    // static utility functions


    // constructors

    // overwritten methods
    virtual bool getPositionIK(const geometry_msgs::Pose &ik_pose,
                               const std::vector<double> &ik_seed_state,
                               std::vector<double> &solution,
                               moveit_msgs::MoveItErrorCodes &error_code,
                               const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;
    virtual bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                  const std::vector<double> &ik_seed_state,
                                  double timeout,
                                  std::vector<double> &solution,
                                  moveit_msgs::MoveItErrorCodes &error_code,
                                  const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;
    virtual bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                  const std::vector<double> &ik_seed_state,
                                  double timeout,
                                  const std::vector<double> &consistency_limits,
                                  std::vector<double> &solution,
                                  moveit_msgs::MoveItErrorCodes &error_code,
                                  const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;
    virtual bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                  const std::vector<double> &ik_seed_state,
                                  double timeout,
                                  std::vector<double> &solution,
                                  const IKCallbackFn &solution_callback,
                                  moveit_msgs::MoveItErrorCodes &error_code,
                                  const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;
    virtual bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                  const std::vector<double> &ik_seed_state,
                                  double timeout,
                                  const std::vector<double> &consistency_limits,
                                  std::vector<double> &solution,
                                  const IKCallbackFn &solution_callback,
                                  moveit_msgs::MoveItErrorCodes &error_code,
                                  const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;
    virtual bool getPositionFK(const std::vector<std::string> &link_names,
                               const std::vector<double> &joint_angles,
                               std::vector<geometry_msgs::Pose> &poses) const;
    virtual bool initialize(const std::string& robot_description,
                            const std::string& group_name,
                            const std::string& base_name,
                            const std::string& tip_name,
                            double search_discretization);
    virtual const std::vector<std::string>& getJointNames() const;
    virtual const std::vector<std::string>& getLinkNames() const;

    // methods

    // variables


  protected:
    // methods

    // variables


  private:
    // methods

    // variables
    std::vector<std::string> m_jointNames;
    std::vector<std::string> m_linkNames;


};

}

#endif // _UR5_MOVEIT_KINEMATICS_PLUGIN_H_
