#!/usr/bin/env python
import math

import rospy
import geometry_msgs.msg
from tf.transformations import quaternion_from_matrix

import nexon_msgs.srv
import nexon_msgs.msg
from nexon.io import parse_rotation


if __name__ == "__main__":
    rospy.init_node('ompl_ptp_planning_server')

    rospy.wait_for_service("arf_sample_constraint")
    service = rospy.ServiceProxy(
        "arf_sample_constraint", nexon_msgs.srv.SampleConstraint)

    # fixed pose for setup_1, l_profile task
    R = parse_rotation([0, 135, 90])
    q = quaternion_from_matrix(R)
    pose = geometry_msgs.msg.Pose()
    pose.position.x = 0.92
    pose.position.y = -0.5
    pose.position.z = 0.02
    pose.orientation.x = q[0]
    pose.orientation.y = q[1]
    pose.orientation.z = q[2]
    pose.orientation.w = q[3]

    # Free rotation around z-axis
    constraint = nexon_msgs.msg.PoseConstraint()
    constraint.relative = True
    constraint.rpy_min = [0, 0, -math.pi]
    constraint.rpy_max = [0, 0, math.pi]

    request = nexon_msgs.srv.SampleConstraintRequest()
    request.pose = pose
    request.constraint = constraint

    response = service(request)
    print(response)
