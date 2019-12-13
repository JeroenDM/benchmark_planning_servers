#include <ros/ros.h>
#include <nexon_msgs/SampleConstraint.h>
#include <eigen_conversions/eigen_msg.h>
#include <eigen3/Eigen/Geometry>

#include "arf_moveit_wrapper/moveit_wrapper.h"
#include "arf_trajectory/trajectory.h"

// I put some code in header files instead of separate libraries #quickanddirty
#include "arf_planning_server/visual_tools_wrapper.h"

bool DEBUG = false;

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
    // Create trajectory point from pose and pose constraint
    Eigen::Affine3d pose;
    tf::poseMsgToEigen(req.pose, pose);

    Eigen::Vector3d p = pose.translation();
    Eigen::Vector3d rpy = pose.rotation().eulerAngles(0, 1, 2);

    
    arf::Number x(p[0]), y(p[1]), z(p[2]);
    arf::Number rx(rpy[0]), ry(rpy[1]), rz(rpy[2]);
    arf::TrajectoryPoint tp(x, y, z, rx, ry, rz);

    // Now we can do the actual sampling
    // for (auto joint_pose : calculateValidJointPoses(robot_, tp, rviz_))
    // {
    //   resp.joint_poses.push_back(joint_pose);
    // }
    // std::vector<double> jv = {1.0, 2.0, 3.0};
    // std::vector<std::vector<double>> jv = {{1, 2}, {3, 4}};
    // resp.joint_poses = jv;

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