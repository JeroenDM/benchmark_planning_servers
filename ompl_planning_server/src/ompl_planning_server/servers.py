""" OMPL planning server implementation.

This module implements a point-to-point and cartesian planning server
using the move group API through a simplified wrapper in the
ompl_planning_server.robot module.
"""
import rospy
import time
from moveit_msgs.msg import RobotTrajectory
from rospy_message_converter import message_converter

from nexon_msgs.srv import PTPPlanning, PTPPlanningResponse
from nexon_msgs.srv import LINPlanning, LINPlanningResponse


def log_request(request_handler):
    """ Decorator to log all the info of a planning request.
    This decorator is made for class methods.

    TODO do we log the service call, or the specific planning time
    without setup time, request parsing, ... ?
    """

    def wrapper(self, req):

        start_time = time.time()
        response = request_handler(self, req)
        runtime = time.time() - start_time

        # check if a run id was published by the benchmark runner node
        if not rospy.has_param("/benchmark_run_id"):
            raise Exception(
                "No /benchmark_run_id was found on the ROS parameter server.")

        # get the current run id from the parameter server
        run_id = rospy.get_param("/benchmark_run_id")

        log_data = {
            "runtime": runtime,
            "run_id": run_id,
            "request": message_converter.convert_ros_message_to_dictionary(
                req
            ),
            "response": message_converter.convert_ros_message_to_dictionary(
                response
            )
        }

        self.db.add_data(log_data)

        return response
    return wrapper


class PTPServer:
    """ Point-to-point planning ROS service. """

    def __init__(self, server_name, robot, db):
        self.robot = robot
        self.server = rospy.Service(
            'ompl_ptp_planning',
            PTPPlanning,
            lambda x: self.handle_request(x)
        )
        self.db = db

    @log_request
    def handle_request(self, req):
        print("Received ptp planning request")
        # print(req)
        config = rospy.get_param("/ptp_config")
        self.robot.set_ompl_planner_params(config)

        plan = RobotTrajectory()

        # set start configuration if given
        if len(req.start_config) > 0:
            state = self.robot.mc.get_current_state()
            state.joint_state.position = req.start_config
            self.robot.mg.set_start_state(state)

        # plan to joint goal if given
        if len(req.joint_goal) > 0:
            plan = self.robot.movej(req.joint_goal)

        # otherwise plan to pose goal
        else:
            plan = self.robot.movep(req.pose_goal)

        # check if the plan has joint positions in it
        if len(plan.joint_trajectory.points) > 0:
            success = True
        else:
            success = False

        if success:
            return PTPPlanningResponse(success, plan.joint_trajectory.points)
        else:
            return PTPPlanningResponse(False, [])


class CartServer:
    """ Cartesian space planning ROS service. """

    def __init__(self, server_name, robot, db):
        self.robot = robot
        self.server = rospy.Service(
            'ompl_lin_planning',
            LINPlanning,
            lambda x: self.handle_request(x)
        )
        self.db = db

    @log_request
    def handle_request(self, req):
        print("Received linear planning request")
        # print(req)
        config = rospy.get_param("/cart_config")
        self.robot.set_cart_planner_params(config)

        resp = LINPlanningResponse()

        if req.has_constraints:
            print("This planner does not support constraints.")
            resp.success = False
            return resp

        # set start configuration if given
        if len(req.start_config) > 0:
            state = self.robot.mc.get_current_state()
            state.joint_state.position = req.start_config
            self.robot.mg.set_start_state(state)
            p_start = self.robot.forward_kinematics(req.start_config)
        else:
            p_start = self.robot.forward_kinematics(
                self.robot.mg.get_current_joint_values())

        plan = self.robot.movel(p_start, req.pose_goal)

        return LINPlanningResponse(
            success=True,
            joint_path=plan.joint_trajectory.points
        )


# TODO import data logging
# from rospy_message_converter import message_converter
# from nexon.io import LogDB
# DO_LOGGING = False

# if DO_LOGGING:
#     DB = LogDB()

#     def log_planning_request(func):
#         """Write planning requests to a log database.
#         This function is used as a decorator on the functions that handle planning requests
#         like this:

#         @log_planning_requests
#         def handle_request(request, robot):
#             # do stuff
#             return response

#         """
#         def wrapper(req, robot):
#             # Log input
#             logdata = {}
#             logdata["planning_request"] = message_converter.convert_ros_message_to_dictionary(
#                 req)

#             try:
#                 logdata["planner_config"] = rospy.get_param(
#                     "/move_group/planner_configs/" + req.planner)
#             except:
#                 logdata["planner_config"] = "NaN"

#             # Execute the actual
#             response = func(req, robot)

#             # Log output
#             #logdata["planning_response"] = response
#             print(logdata)
#             DB.add_data(logdata)

#             return response

#         return wrapper
# else:
#     def log_planning_request(func):
#         def wrapper(req, robot):
#             return func(req, robot)
#         return wrapper
