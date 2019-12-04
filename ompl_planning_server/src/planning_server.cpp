#include <string>

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <nexon_msgs/PTPPlanning.h>

static const std::string PLANNING_GROUP_NAME = "manipulator";

class PlanningServer
{
public:
  PlanningServer()
  {
    mg_.reset(new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP_NAME));
    jmg_ = mg_->getCurrentState()->getJointModelGroup(PLANNING_GROUP_NAME);

    planning_request_server_ = nh_.advertiseService("planning_request", &PlanningServer::executePlanningRequest, this);
  }

  void tellThemImReady()
  {
    ROS_INFO_STREAM("Ready to receive planning requests now!");
  }

  bool executePlanningRequest(nexon_msgs::PTPPlanning::Request& req, nexon_msgs::PTPPlanning::Response& resp)
  {
    ROS_INFO_STREAM("Received a planning request\n" << req);

    // Check if we need to set a specific start state (joint values)
    robot_state::RobotState start_state(*mg_->getCurrentState());
    if (req.start_config.size() == 0)
    {
      ROS_INFO_STREAM("Planning from current state.");
      mg_->setStartState(start_state);
    }
    else
    {
      ROS_INFO_STREAM("Planning from a given start state.");
      start_state.setJointGroupPositions(jmg_, req.start_config);
      mg_->setStartState(start_state);
    }

    // Check if we plan to a joint values or end-effector pose
    if (req.joint_goal.size() == 0)
    {
      ROS_INFO_STREAM("Received a pose goal.");
      mg_->setPoseTarget(req.pose_goal);
    }
    else
    {
      ROS_INFO_STREAM("Received a joint goal:\n");
      mg_->setJointValueTarget(req.joint_goal);
    }

    // Plan and return joint path if successful
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (mg_->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    resp.success = success;
    if (success)
    {
      ROS_INFO_STREAM("Found plan with length: " << plan.trajectory_.joint_trajectory.points.size());
      resp.joint_path = plan.trajectory_.joint_trajectory.points;
    }
    else
    {
      ROS_INFO_STREAM("Failed to find plan.");
    }

    ROS_INFO_STREAM("Service call finished.");
    return true;
  }

private:
  ros::NodeHandle nh_;
  moveit::planning_interface::MoveGroupInterfacePtr mg_;
  const moveit::core::JointModelGroup* jmg_;
  ros::ServiceServer planning_request_server_;
};  // end class

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ompl_planning_server");
  ROS_INFO_STREAM("Launching ompl planning server.");

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