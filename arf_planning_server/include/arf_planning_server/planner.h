#ifndef _ARF_DEMO_PLANNER_H_
#define _ARF_DEMO_PLANNER_H_

#include <vector>
#include <ros/ros.h>

#include "arf_graph/graph.h"
#include "arf_graph/util.h"  // enable std::cout << graph|node
#include "arf_trajectory/trajectory.h"
#include "arf_planning_server/visual_tools_wrapper.h"
#include "arf_moveit_wrapper/redundant_robot.h"

namespace arf
{
bool DEBUG = false;
// ros::param::get<bool>("/arf_debug_flag", DEBUG, false);

struct PlannerSettings
{
  double max_translation = 0.01;
  double max_rotation = 0.06;
  double pos_tol_resolution = 0.01;
  double rot_tol_resolution = 0.15;
};

// TODO move this to arf_trajectory
using Trajectory = std::vector<TrajectoryPoint>;

using GraphData = std::vector<std::vector<std::vector<double>>>;
using JointPose = std::vector<double>;
using JointPath = std::vector<JointPose>;

GraphData calculateValidJointPoses(Robot& robot, Trajectory& traj, Rviz& rviz)
{
  GraphData graph_data;

  for (auto tp : traj)
  {
    std::vector<std::vector<double>> new_data;
    for (auto pose : tp.getGridSamples())
    {
      if (DEBUG)
      {
        rviz.plotPose(pose);
      }
      for (auto q_sol : robot.ik(pose))
      {
        if (!robot.isInCollision(q_sol))
        {
          new_data.push_back(q_sol);
          if (DEBUG)
          {
            robot.plot(rviz.visual_tools_, q_sol);
          }
        }
      }
    }
    if (new_data.size() == 0)
    {
      throw std::runtime_error("No valid joint poses found for some pose.");
    }
    graph_data.push_back(new_data);
  }
  return graph_data;
}

GraphData calculateValidJointPoses(RedundantRobot& robot, Trajectory& traj, Rviz& rviz)
{
  GraphData graph_data;

  for (auto tp : traj)
  {
    std::vector<std::vector<double>> new_data;
    for (auto pose : tp.getGridSamples())
    {
      if (DEBUG)
      {
        rviz.plotPose(pose);
      }
      for (auto q_sol : robot.ikGridSamples(pose))
      {
        if (!robot.isInCollision(q_sol))
        {
          new_data.push_back(q_sol);
          if (DEBUG)
          {
            robot.plot(rviz.visual_tools_, q_sol);
          }
        }
      }
    }
    if (new_data.size() == 0)
    {
      throw std::runtime_error("No valid joint poses found for some pose.");
    }
    graph_data.push_back(new_data);
  }
  return graph_data;
}

JointPath calculateShortestPath(Robot& robot, GraphData& gd)
{
  std::cout << "Calculation shortest path..." << std::endl;

  JointPath solution;
  Graph demo_graph(gd);
  demo_graph.runMultiSourceDijkstra();
  std::vector<Node*> sp = demo_graph.getShortestPath();

  for (auto node : sp)
  {
    std::cout << (*node) << std::endl;
    solution.push_back(*(*node).jv);
  }

  return solution;
}

JointPath calculateShortestPath(RedundantRobot& robot, GraphData& gd)
{
  std::cout << "Calculation shortest path..." << std::endl;

  JointPath solution;
  Graph demo_graph(gd);
  demo_graph.runMultiSourceDijkstra();
  std::vector<Node*> sp = demo_graph.getShortestPath();

  for (auto node : sp)
  {
    std::cout << (*node) << std::endl;
    solution.push_back(*(*node).jv);
  }

  return solution;
}

}  // namespace arf

#endif