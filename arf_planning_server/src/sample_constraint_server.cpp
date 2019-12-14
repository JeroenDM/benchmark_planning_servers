#include <ros/ros.h>
#include <eigen_conversions/eigen_msg.h>
#include <eigen3/Eigen/Geometry>
#include <geometry_msgs/Pose.h>

#include <arf_moveit_wrapper/moveit_wrapper.h>
#include <arf_trajectory/trajectory.h>
#include <nexon_msgs/SampleConstraint.h>
#include <nexon_msgs/JointPose.h>
#include <nexon_msgs/PoseConstraint.h>

// I put some code in header files instead of separate libraries #quickanddirty
#include "arf_planning_server/visual_tools_wrapper.h"

bool DEBUG = false;

/* \Brief Find collision free inverse kinematic solutions for a given toleranced trajectory point. */
std::vector<std::vector<double>> calculateValidJointPoses(arf::Robot& robot, arf::TrajectoryPoint& tp, arf::Rviz& rviz)
{
  std::vector<std::vector<double>> joint_poses;
  for (auto pose : tp.getGridSamples())
  {
    if (DEBUG){
      rviz.plotPose(pose);
    }
    for (auto q_sol : robot.ik(pose))
    {
      if (!robot.isInCollision(q_sol))
      {
        joint_poses.push_back(q_sol);
        if (DEBUG){
          robot.plot(rviz.visual_tools_, q_sol);
        }
      }
    }
  }
  if (joint_poses.size() == 0)
  {
    std::cout << "No valid joint poses found within constraints." << std::endl;
  }
  return joint_poses;
}

/* \Brief Create (Toleranced)Numbers depending on the upper and lower bounds for the values. */
std::vector<arf::Number> parseConstraint(Eigen::Vector3d vals, std::vector<double>& mins, std::vector<double>& maxs)
{
  std::vector<arf::Number> numbers;
  if (mins.size() == 3 && maxs.size() == 3)
  {
    for (std::size_t i=0; i<3; ++i)
    {
      if (mins[i] == maxs[i])
      {
        numbers.push_back(arf::Number(vals[i]));
      }
      else
      {
        numbers.push_back(arf::TolerancedNumber(vals[i], mins[i], maxs[i]));
      }
    }
  }
  else if (mins.size() == 0 && maxs.size() == 0)
  {
    // Fixed position
    for (std::size_t i=0; i<3; ++i)
    {
      numbers.push_back(arf::Number(vals[i]));
    }
  }
  else
  {
    ROS_ERROR_STREAM("Invalid PoseConstraint position bounds.");
  }
  return numbers;

}

/* \Brief Create Toleranced Trajectory Point from pose constraints in request. */
arf::TrajectoryPoint poseConstraintsToTP(geometry_msgs::Pose& pose, nexon_msgs::PoseConstraint& con)
{
  Eigen::Affine3d transform;
  tf::poseMsgToEigen(pose, transform);

  Eigen::Vector3d p = transform.translation();
  Eigen::Vector3d rpy = transform.rotation().eulerAngles(0, 1, 2);

  auto position = parseConstraint(p, con.xyz_min, con.xyz_max);
  auto orientation = parseConstraint(rpy, con.rpy_min, con.rpy_max);
  

  arf::Number rx(rpy[0]), ry(rpy[1]), rz(rpy[2]);
  arf::TrajectoryPoint tp(position[0], position[1], position[2], orientation[0], orientation[1], orientation[2]);

  return tp;
}

class SampleServer
{
  ros::NodeHandle nh_;
  ros::ServiceServer cart_plannig_server_;
  arf::Robot robot_;
  arf::Rviz rviz_;
public:
  SampleServer()
  {
    cart_plannig_server_ = nh_.advertiseService("arf_sample_constraint", &SampleServer::executeSampleRequest, this);
    ROS_INFO_STREAM("Ready to receive sample consraint request.");
  }

  bool executeSampleRequest(nexon_msgs::SampleConstraintRequest& req, nexon_msgs::SampleConstraintResponse& resp)
  {
    ROS_INFO_STREAM("Received sample request.");
    ROS_INFO_STREAM(req);

    if (!req.constraint.relative)
    {
      ROS_ERROR_STREAM("Absolute (relative=False) constraints not implemented yet. Assuming relative=True.");
    }

    auto tp = poseConstraintsToTP(req.pose, req.constraint);

    for (auto joint_pose : calculateValidJointPoses(robot_, tp, rviz_))
    {
      nexon_msgs::JointPose jp;
      jp.positions = joint_pose;
      resp.joint_poses.push_back(jp);
    }

    ROS_INFO_STREAM("Found " << resp.joint_poses.size() << " valid joint poses.");

    return true;
  }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "arf_sample_constraint_server");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(3);
  spinner.start();

  SampleServer server;

  ros::waitForShutdown();
  return 0;
}