#include <ros/ros.h>
#include <nexon_msgs/LINPlanning.h>
#include <eigen_conversions/eigen_msg.h>
#include <eigen3/Eigen/Geometry>

#include "arf_moveit_wrapper/moveit_wrapper.h"
#include "arf_trajectory/trajectory.h"

// I put some code in header files instead of separate libraries #quickanddirty
#include "arf_planning_server/visual_tools_wrapper.h"
#include "arf_planning_server/planner.h"

struct ArfPlannerSettings
{
  double max_translation = 0.01;
  double max_rotation = 0.06;
  double pos_tol_resolution = 0.01;
  double rot_tol_resolution = 0.15;
};

arf::Trajectory createTrajectory()
{
  arf::Trajectory ee_trajectory_;
  for (int i = 0; i < 10; ++i)
  {
    arf::Number x(0.98);
    arf::Number y(-0.5 + static_cast<double>(i) / 9);
    arf::Number z(0.02);
    arf::Number rx(0.0), ry(135.0 * M_PI / 180.0);  //, ry(-M_PI);
    // TolerancedNumber ry(-M_PI, -M_PI - 1.0, -M_PI + 1.0, 5);
    arf::TolerancedNumber rz(0, -M_PI, M_PI, 10);
    arf::TrajectoryPoint tp(x, y, z, rx, ry, rz);
    ee_trajectory_.push_back(tp);
  }
  return ee_trajectory_;
}

arf::Trajectory createTrajectory(Eigen::Affine3d& start, Eigen::Affine3d& goal, ArfPlannerSettings& settings)
{
  // const int num_pts = 10;
  // const double max_translation = 0.01;
  // const double max_rotation = 0.03;
  const double MIN_STEPS = 10;
  // limit the maximum number of points
  // as the planning can take a long time
  // which is inconvenient for testing
  const double MAX_STEPS = 50;

  Eigen::Quaterniond start_quat(start.rotation());
  Eigen::Quaterniond goal_quat(goal.rotation());

  // decide how many steps we will need for this trajectory
  // copied from MoveIt's computeCartesianPath function
  double translation_distance = (goal.translation() - start.translation()).norm();
  double rotation_distance = start_quat.angularDistance(goal_quat);

  std::size_t translation_steps = 0;
  translation_steps = floor(translation_distance / settings.max_translation);

  std::size_t rotation_steps = 0;
  rotation_steps = floor(rotation_distance / settings.max_rotation);

  std::size_t steps = std::max(translation_steps, rotation_steps) + 1;
  if (steps < MIN_STEPS)
    steps = MIN_STEPS;
  if (steps > MAX_STEPS)
    steps = MAX_STEPS;

  ROS_INFO_STREAM("Selected " << steps);
  ROS_INFO_STREAM("Translation: " << translation_steps << " Rotation: " << rotation_steps);

  // Interpolation
  Eigen::Vector3d diff = goal.translation() - start.translation();
  double step_size = diff.norm() / (steps - 1);
  Eigen::Vector3d diff_unit = diff.normalized();

  std::vector<Eigen::Affine3d> poses;
  double fraction = 0.0;
  for (int i = 0; i < steps; ++i)
  {
    Eigen::Affine3d pose = Eigen::Affine3d::Identity();
    fraction = static_cast<double>(i) / (steps - 1);

    // orientation interpolation
    pose *= start_quat.slerp(fraction, goal_quat);

    // position interpolation
    pose.translation() = start.translation() + static_cast<double>(i) * step_size * diff_unit;

    ROS_INFO_STREAM("Pose " << i);
    ROS_INFO_STREAM("Fraction " << fraction);
    // ROS_INFO_STREAM(pose.translation() << pose.rotation());

    poses.push_back(pose);
  }

  arf::Trajectory ee_trajectory_;
  for (int i = 0; i < steps; ++i)
  {
    Eigen::Vector3d position = poses[i].translation();
    arf::Number x(position[0]);
    arf::Number y(position[1]);
    arf::Number z(position[2]);

    Eigen::Vector3d rpy_angles = poses[i].rotation().eulerAngles(0, 1, 2);
    // ROS_INFO_STREAM("EUler angles: " << rpy_angles << "\n");
    arf::Number rx(rpy_angles[0]), ry(rpy_angles[1]);
    // arf::TolerancedNumber ry(rpy_angles[1], -0.5 + rpy_angles[1], rpy_angles[1] + 0.5, 5);
    int num_rot_points = floor(2 * M_PI / settings.rot_tol_resolution) + 1;
    arf::TolerancedNumber rz(rpy_angles[2], -M_PI, M_PI, num_rot_points);
    // Number rx(0.0), ry(135.0 * M_PI / 180.0);
    // TolerancedNumber rz(0, -M_PI, M_PI, 10);

    arf::TrajectoryPoint tp(x, y, z, rx, ry, rz);
    ee_trajectory_.push_back(tp);
  }
  return ee_trajectory_;
}

std::vector<trajectory_msgs::JointTrajectoryPoint> jointPathToMsg(arf::JointPath& jp)
{
  double time = 0;
  double dt = 0.1;

  std::vector<trajectory_msgs::JointTrajectoryPoint> ros_traj;

  for (auto q : jp)
  {
    trajectory_msgs::JointTrajectoryPoint pt;
    pt.positions = q;
    pt.velocities.resize(q.size(), 0.0);
    pt.accelerations.resize(q.size(), 0.0);
    pt.time_from_start = ros::Duration(time);
    time += dt;

    // ROS_INFO_STREAM("Created ros traj pt: " << pt);
    ros_traj.push_back(pt);
  }
  return ros_traj;
}

class PlanningServer
{
  ros::NodeHandle nh_;
  ros::ServiceServer cart_plannig_server_;
  arf::Robot robot_;
  arf::Rviz rviz_;
  ArfPlannerSettings settings_;

public:
  PlanningServer()
  {
    cart_plannig_server_ = nh_.advertiseService("arf_cart_planning", &PlanningServer::executePlanningRequest, this);
    // start with default settings, read them from parameter server when planning
    settings_ = ArfPlannerSettings();
    ROS_INFO_STREAM("Ready to receive Cartesian planning requests.");
  }

  ~PlanningServer() = default;

  void testServer()
  {
    robot_.printCurrentJointValues();
  }

  void readSettigsFromParameterServer()
  {
    if (ros::param::has("/cart_config"))
    {
      double max_translation, max_rotation, pos_tol_resolution, rot_tol_resolution;
      if (ros::param::get("/cart_config/max_translation", max_translation))
        settings_.max_translation = max_translation;

      if (ros::param::get("/cart_config/max_rotation", max_rotation))
        settings_.max_rotation = max_rotation;

      if (ros::param::get("/cart_config/pos_tol_resolution", pos_tol_resolution))
        settings_.pos_tol_resolution = pos_tol_resolution;

      if (ros::param::get("/cart_config/max_translation", rot_tol_resolution))
        settings_.rot_tol_resolution = rot_tol_resolution;
    }
    else
    {
      ROS_INFO_STREAM("Could not find a planner configuration /cart_config, using defaults.");
    }
  }

  bool executePlanningRequest(nexon_msgs::LINPlanning::Request& req, nexon_msgs::LINPlanning::Response& resp)
  {
    // do stuff
    ROS_INFO_STREAM("ARF Cartesian planning server received planning request.");
    ROS_INFO_STREAM(req);
    readSettigsFromParameterServer();

    arf::JointPose start_config = req.start_config;
    auto start_pose = robot_.fk(start_config);
    Eigen::Affine3d goal_pose;
    tf::poseMsgToEigen(req.pose_goal, goal_pose);

    // ROS_INFO_STREAM("Start pose:\n" << start_pose.translation());

    auto traj = createTrajectory(start_pose, goal_pose, settings_);
    auto gd = arf::calculateValidJointPoses(robot_, traj, rviz_);

    // slow but easy operation
    std::vector<arf::JointPose> first_tp = { start_config };
    gd.insert(gd.begin(), first_tp);

    // std::cout << "Created graph data.\n";
    // std::cout << gd << std::endl;

    auto jp = arf::calculateShortestPath(robot_, gd);

    // rviz_.plotPath(robot_, jp);

    auto ros_jp = jointPathToMsg(jp);

    resp.success = true;
    resp.joint_path = ros_jp;
    ROS_INFO_STREAM("Arf planning request finished.");
    return true;
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "arf_cart_planning_server");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(3);
  spinner.start();

  PlanningServer server;

  ros::waitForShutdown();
  return 0;
}