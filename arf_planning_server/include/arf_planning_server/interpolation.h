#ifndef _ARF_DEMO_INTERPOLATION_H_
#define _ARF_DEMO_INTERPOLATION_H_

#include <ros/ros.h>
#include <eigen3/Eigen/Geometry>

namespace arf
{
/* \Brief Linear interpolation between two Affine3d poses.

    Using quaternion slerp to interpolate orientation
 */
std::vector<Eigen::Affine3d> interpolate(Eigen::Affine3d& start, Eigen::Affine3d& goal, double max_pos, double max_rot)
{
  const double MIN_STEPS = 10;
  // limit the maximum number of points
  // as the planning can take a long time
  // which is inconvenient for testing
  const double MAX_STEPS = 50;

  std::vector<Eigen::Affine3d> poses;

  Eigen::Quaterniond start_quat(start.rotation());
  Eigen::Quaterniond goal_quat(goal.rotation());

  // decide how many steps we will need for this trajectory
  // copied from MoveIt's computeCartesianPath function
  double translation_distance = (goal.translation() - start.translation()).norm();
  double rotation_distance = start_quat.angularDistance(goal_quat);

  std::size_t translation_steps = 0;
  translation_steps = floor(translation_distance / max_pos);

  std::size_t rotation_steps = 0;
  rotation_steps = floor(rotation_distance / max_rot);

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

  return poses;
}

}  // namespace arf

#endif