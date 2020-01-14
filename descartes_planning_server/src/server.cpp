#include <string>

#include <ros/ros.h>

#include <eigen_conversions/eigen_msg.h>

// #include <descartes_moveit/moveit_state_adapter.h>
// #include <descartes_moveit/ikfast_moveit_state_adapter.h>
#include <descartes_opw_model/descartes_opw_model.h>
#include <descartes_planner/dense_planner.h>
#include <descartes_trajectory/joint_trajectory_pt.h>
#include <descartes_trajectory/axial_symmetric_pt.h>
#include <descartes_trajectory/cart_trajectory_pt.h>
#include <descartes_utilities/ros_conversions.h>

#include <nexon_msgs/LINPlanning.h>
#include <eigen3/Eigen/Dense>
#include <opw_kinematics/opw_parameters.h>
#include "descartes_planning_server/interpolate.h"

// using Transform = Eigen::Isometry3d;

template <typename T>
const opw_kinematics::Parameters<T> makeKukaKr5()
{
  opw_kinematics::Parameters<T> p;
  p.a1 = T(0.180);
  p.a2 = T(-0.120);
  p.b = T(0.000);
  p.c1 = T(0.400);
  p.c2 = T(0.600);
  p.c3 = T(0.620);
  p.c4 = T(0.115);

  p.offsets[1] = -M_PI / 2.0;
  p.sign_corrections[0] = -1;
  p.sign_corrections[3] = -1;
  p.sign_corrections[5] = -1;

  return p;
}

static const std::string PLANNING_GROUP_NAME = "manipulator";

std::vector<descartes_core::TrajectoryPtPtr> makePath(std::vector<double>& start_config, Eigen::Isometry3d& start,
                                                      Eigen::Isometry3d& goal, const nexon_msgs::PoseConstraint& con);
descartes_core::TrajectoryPtPtr makeTolerancedPt(const Eigen::Isometry3d& pose, const nexon_msgs::PoseConstraint& con,
                                                 double dt, double num_intervals);

void printPose(const Eigen::Isometry3d& pose)
{
  ROS_INFO_STREAM("Translation " << pose.translation());
  Eigen::Quaterniond q(pose.rotation());
  ROS_INFO_STREAM("Orientation: " << q.x() << " " << q.y() << " " << q.z() << " " << q.w());
}

class PlanningServer
{
public:
  PlanningServer()
  {
    planning_request_server_ = nh_.advertiseService("desc_lin_planning", &PlanningServer::executePlanningRequest, this);
    model.reset(new descartes_opw_model::OPWMoveitStateAdapter(makeKukaKr5<double>(), "base_link", "tool0"));

    if (!model->initialize(robot_description, group_name, world_frame, tcp_frame))
    {
      ROS_INFO("Could not initialize robot model");
    }
    model->setCheckCollisions(true);
  }

  void tellThemImReady()
  {
    ROS_INFO_STREAM("Ready to receive planning requests now!");
  }

  bool executePlanningRequest(nexon_msgs::LINPlanning::Request& req, nexon_msgs::LINPlanning::Response& resp)
  {
    ROS_INFO_STREAM("Received a planning request\n" << req);

    Eigen::Isometry3d start, goal;
    model->getFK(req.start_config, start);

    ROS_INFO_STREAM("Solved fk to find start pose");
    printPose(start);

    tf::poseMsgToEigen(req.pose_goal, goal);
    ROS_INFO_STREAM("Goal pose: ");
    printPose(goal);

    auto path = makePath(req.start_config, start, goal, req.pose_constraint);

    descartes_planner::DensePlanner planner;
    if (!planner.initialize(model))
    {
      ROS_ERROR("Failed to initialize planner");
      resp.success = false;
      return true;
    }

    if (!planner.planPath(path))
    {
      ROS_ERROR("Could not solve for a valid path");
      resp.success = false;
      return true;
    }

    std::vector<descartes_core::TrajectoryPtPtr> result;
    if (!planner.getPath(result))
    {
      ROS_ERROR("Could not retrieve path");
      resp.success = false;
      return true;
    }

    std::vector<std::string> names;
    nh_.getParam("controller_joint_names", names);
    trajectory_msgs::JointTrajectory joint_solution;
    joint_solution.joint_names = names;
    const static double default_joint_vel = 0.5;  // rad/s
    if (!descartes_utilities::toRosJointPoints(*model, result, default_joint_vel, joint_solution.points))
    {
      ROS_ERROR("Unable to convert Descartes trajectory to joint points");
      return -5;
    }

    resp.joint_path = joint_solution.points;
    resp.success = true;

    ROS_INFO_STREAM("Service call finished.");
    return true;
  }

private:
  ros::NodeHandle nh_;
  ros::ServiceServer planning_request_server_;
  descartes_core::RobotModelPtr model;
  const std::string robot_description = "robot_description";
  const std::string group_name = "manipulator";
  const std::string world_frame = "world";
  const std::string tcp_frame = "tool_tip";
};  // end class

std::vector<descartes_core::TrajectoryPtPtr> makePath(std::vector<double>& start_config, Eigen::Isometry3d& start,
                                                      Eigen::Isometry3d& goal, const nexon_msgs::PoseConstraint& con)
{
  std::vector<descartes_core::TrajectoryPtPtr> result;

  auto poses = interpolate(start, goal, 0.02, 0.03);
  // const int num_pts = 10;

  // // only do position interpolation for now
  // Eigen::Vector3d diff = goal.translation() - start.translation();
  // double step_size = diff.norm() / (num_pts - 1);

  // EigenSTL::vector_Isometry3d poses;
  // for (int i = 0; i < num_pts; ++i)
  // {
  //   Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  //   pose *= goal.rotation();

  //   pose.translation() =
  //       start.translation() + static_cast<double>(i) * step_size * diff;  // static_cast<double>(i) * step_size

  //   // ROS_INFO_STREAM("Pose " << i);
  //   // printPose(pose);

  //   poses.push_back(pose);
  // }

  double time = 0;
  double time_step = 0.5;

  descartes_core::TrajectoryPtPtr first_pt(new descartes_trajectory::JointTrajectoryPt(start_config));
  result.push_back(first_pt);
  for (auto p : poses)
  {
    time += time_step;
    result.push_back(makeTolerancedPt(p, con, time, 1000));
  }

  return result;
}

descartes_core::TrajectoryPtPtr makeTolerancedPt(const Eigen::Isometry3d& pose, const nexon_msgs::PoseConstraint& con,
                                                 double dt, double num_intervals)
{
  // if (!con.relative)
  // {
  //   ROS_ERROR_STREAM("Absolution tolerance not supported yet.");
  // }
  // if (con.symmetric)
  // {
  //   ROS_INFO_STREAM("Creating a symmetric tolerance point.");
  // }
  // else
  // {
  //   ROS_INFO_STREAM("Creating tolerance point with different upper and lower bounds.");
  // }

  using namespace descartes_core;
  using namespace descartes_trajectory;

  Eigen::Vector3d t = pose.translation();
  Eigen::Matrix3d m = pose.rotation();
  Eigen::Vector3d rxyz = m.eulerAngles(0, 1, 2);

  PositionTolerance pos_tol = ToleranceBase::zeroTolerance<PositionTolerance>(t(0), t(1), t(2));

  double z_tolerance = 2 * M_PI;
  OrientationTolerance orient_tol =
      ToleranceBase::createSymmetric<OrientationTolerance>(t(0), t(1), t(2), 0.0, 0.0, z_tolerance);

  return TrajectoryPtPtr(new CartTrajectoryPt(TolerancedFrame(pose, pos_tol, orient_tol), 0,
                                              z_tolerance / num_intervals, TimingConstraint(dt)));
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "descartes_planning_server");
  ROS_INFO_STREAM("Launching descartes linear planning server.");

  // Use more than one thread (3) !
  // the move group stuff needs it's own thread
  ros::AsyncSpinner spinner(3);
  spinner.start();

  PlanningServer planning_server;
  planning_server.tellThemImReady();

  ros::waitForShutdown();
  ROS_INFO_STREAM("Shutting down.");

  return 0;
}
