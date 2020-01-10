import copy
import rospy
import moveit_commander
import moveit_msgs.srv


class Robot:
    """ Wrapper for move_group interface.

    Setup the move group interface,
    a robot commander (to access the robot state)
    and the forward kinematics service
    """
    GROUP_NAME = "manipulator"

    def __init__(self, argv=[]):
        moveit_commander.roscpp_initialize(argv)
        self.mc = moveit_commander.RobotCommander()
        self.mg = self.mc.get_group(self.GROUP_NAME)
        self.scene = moveit_commander.PlanningSceneInterface()

        rospy.wait_for_service("/compute_fk")
        self.fk_service = rospy.ServiceProxy(
            "/compute_fk", moveit_msgs.srv.GetPositionFK)
        rospy.wait_for_service("/get_planner_params")
        self.set_planner_params = rospy.ServiceProxy(
            "/set_planner_params", moveit_msgs.srv.SetPlannerParams)

        self.ee_link = self.mg.get_end_effector_link()
        # self.rospack = rospkg.RosPack()
        # self.mesh_path = self.rospack.get_path(
        #     'setup_2_support') + '/meshes/small_wobj.stl'
        # TODO also change this in plot_pose, but not working yet
        # self.print_info()

        # default cart_config
        self.eef_step = 0.01
        self.jump_threshold = 0.0

    def set_cart_planner_params(self, config):
        self.eef_step = config["eef_step"]
        self.jump_threshold = config["jump_threshold"]

    def set_planner(self, planner_id):
        self.mg.set_planner_id(planner_id)

    def set_ompl_planner_params(self, planner_config):
        """ Set parameters using the 'SetPlannerParams' service.

        Planner name and planning time are special it seems.
        Use a dict with with structure:
        {
            "planner_id": <planner_name>,
            "planning_time": <planning_time>,
            "planner_params": {
                "longest_valid_segment_fractoin": <number>,
                ...
            }
        }

        """

        req = moveit_msgs.srv.SetPlannerParamsRequest()
        req.planner_config = planner_config["planner_id"]
        req.group = self.GROUP_NAME
        req.params = moveit_msgs.msg.PlannerParams(
            keys=["longest_valid_segment_fraction"],
            values=["0.001"]
        )
        self.set_planner_params(req)
        self.mg.set_planning_time(planner_config["planning_time"])
        self.mg.set_planner_id(planner_config["planner_id"])

    def print_info(self):
        print("============ Printing robot state")
        print(self.mc.get_current_state())
        print("")
        print(self.ee_link)
        print(self.mc.get_link_names(group=self.GROUP_NAME))

    def forward_kinematics(self, joint_values):
        """ There is no forward kinematics function in the move group interface.
        But there is a ros service available called "/compute_fk" that we can call.
        """
        request = moveit_msgs.srv.GetPositionFKRequest()
        request.header.frame_id = "world"
        request.fk_link_names = [self.ee_link]
        request.robot_state = self.mc.get_current_state()

        request.robot_state.joint_state.position = joint_values

        response = self.fk_service(request)
        # print("======== request =============")
        # print(request)
        # print("======== response ============")
        # print(response)

        if response.error_code.val == moveit_msgs.msg.MoveItErrorCodes.SUCCESS:
            return response.pose_stamped[0].pose
        else:
            print(response)
            raise Exception("Forward kinematics failed")

    def movep(self, pose_msg):
        """ Move to and end-effector pose without path constraints."""
        self.mg.set_pose_target(pose_msg)
        return self.mg.plan()

    def movej(self, config):
        """ Move to a joint configuration without path constraints."""
        self.mg.set_joint_value_target(config)
        return self.mg.plan()

    def movel(self, p_start, p_stop):
        """ Move along a line in cartesian space to a given end-effector pose."""
        waypoints = []
        waypoints.append(copy.deepcopy(p_start))
        waypoints.append(copy.deepcopy(p_stop))
        (plan, fraction) = self.mg.compute_cartesian_path(
            waypoints,
            self.eef_step,
            self.jump_threshold)

        if fraction == 1.0:
            return plan
        else:
            # return plan
            raise Exception(
                "Failed to plan cartesian plan, fraction: {}".format(fraction))

    def move_to_pose(self, goal_pose):
        self.mg.set_pose_target(goal_pose)
        plan = self.mg.plan()
        success = self.mg.go(wait=True)
        self.mg.stop()
        self.mg.clear_pose_targets()
        return success, plan

    def move_to_joint_pos(self, joint_pos):
        self.mg.set_joint_value_target(joint_pos)
        plan = self.mg.plan()
        success = self.mg.go(wait=True)
        self.mg.stop()
        return success, plan

    def move_to_named_target(self, name):
        self.mg.set_named_target(name)
        plan = self.mg.plan()
        success = self.mg.go(wait=True)
        self.mg.stop()
        return success, plan

    # def create_pick_object(self):
    #     box_pose = geometry_msgs.msg.PoseStamped()
    #     box_pose.header.frame_id = self.ee_link
    #     box_pose.pose.orientation.w = 1.0
    #     box_pose.pose.position.z = 0.0  # end effector offset
    #     self.box_name = "pick_object"
    #     # self.scene.add_box(self.box_name, box_pose, size=(0.16, 0.16, 0.06))
    #     self.scene.add_mesh(self.box_name, box_pose, self.mesh_path)
    #     return self.wait_for_state_update(box_is_known=True, box_is_attached=False)

    def attach_pick_object(self):
        touch_links = ['tool_tip', 'magnet', 'magnet_holder']
        self.scene.attach_box(self.ee_link, self.box_name,
                              touch_links=touch_links)
        return self.wait_for_state_update(box_is_known=True, box_is_attached=True)

    def drop_pick_object(self):
        self.scene.remove_attached_object(self.ee_link, name=self.box_name)
        return self.wait_for_state_update(box_is_known=True, box_is_attached=False)

    def remove_pick_object(self):
        self.scene.remove_world_object(self.box_name)
        return self.wait_for_state_update(box_is_known=False, box_is_attached=False)

    def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
        """ Adding a new collision object to the planning scene can take a while.
        This functions checks when a collision object is added and/or attached or
        a timeout is reached.
        """
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            attached_objects = self.scene.get_attached_objects([self.box_name])
            is_attached = len(attached_objects.keys()) > 0
            is_known = self.box_name in self.scene.get_known_object_names()
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True
            rospy.sleep(0.1)
            seconds = rospy.get_time()
        return False
