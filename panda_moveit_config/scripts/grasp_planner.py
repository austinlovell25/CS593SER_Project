#!/usr/bin/env python3
"""
Lab 02: Grasp Planning with MoveIt 2

Sequences arm motions via /move_action and gripper commands via the
gripper trajectory controller to perform a top-down pick-and-place.

Students must implement the TODO sections.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion, Vector3
from moveit_msgs.msg import (
    Constraints, PositionConstraint, OrientationConstraint,
    MoveItErrorCodes, CollisionObject, PlanningScene,
    AttachedCollisionObject, WorkspaceParameters, RobotState,
)
from moveit_msgs.action import MoveGroup
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, ColorRGBA
from shape_msgs.msg import SolidPrimitive
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration

import copy
import random


class GraspPlanner(Node):
    """Plans and executes pick/place operations using MoveIt 2."""

    # Gripper joint names for Panda
    GRIPPER_JOINTS = ['panda_finger_joint1', 'panda_finger_joint2']
    GRIPPER_OPEN = [0.04, 0.04]
    GRIPPER_CLOSED = [0.018, 0.018]

    # ---- TODO 1: Top-down orientation ----
    # The panda_hand frame has its z-axis along the fingers.
    # A 180° rotation about the x-axis flips z from +up to -down,
    # giving us the top-down gripper orientation.
    # Hint: a rotation of angle θ about axis (ux,uy,uz) is the quaternion
    #   (x,y,z,w) = (ux·sin(θ/2), uy·sin(θ/2), uz·sin(θ/2), cos(θ/2))
    TOP_DOWN_ORIENTATION = None  # TODO: replace with the correct Quaternion

    def __init__(self):
        super().__init__('grasp_planner')

        # MoveGroup action client  (plans + executes arm motions)
        self.move_client = ActionClient(self, MoveGroup, '/move_action')

        # Gripper trajectory action client
        self.gripper_client = ActionClient(
            self, FollowJointTrajectory,
            '/gripper_trajectory_controller/follow_joint_trajectory'
        )

        # Planning-scene publisher (attach / detach objects)
        self.scene_pub = self.create_publisher(PlanningScene, '/planning_scene', 10)

        # Listen for detected collision objects from the object detector
        self.detected_objects = {}
        self._manipulating = False  # ignore collision updates during pick-and-place
        self.collision_sub = self.create_subscription(
            CollisionObject, '/collision_object',
            self.collision_object_callback, 10
        )

        # Pause/unpause the object detector during manipulation
        pause_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pause_pub = self.create_publisher(Bool, '/pause_detection', qos_profile=pause_qos)

        # Marker publisher for visualizing grasp poses
        self.marker_pub = self.create_publisher(MarkerArray, '/grasp_markers', 10)
        self._marker_id = 0

        # Cache latest joint states so we can check readiness
        self.latest_joint_state = None
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states',
            lambda msg: setattr(self, 'latest_joint_state', msg), 10
        )

        self.get_logger().info('Waiting for /move_action server...')
        self.move_client.wait_for_server()
        self.get_logger().info('Waiting for gripper controller...')
        self.gripper_client.wait_for_server()
        self.get_logger().info('Grasp planner ready')

    # ------------------------------ callbacks ---------------------------------

    def collision_object_callback(self, msg: CollisionObject):
        """Cache detected collision objects and forward to MoveIt's planning scene."""
        self.get_logger().info(f"Manipulating: {self._manipulating}, received collision object: {msg.id} with operation {msg.operation}")
        if self._manipulating:
            return
        if msg.operation == CollisionObject.ADD and msg.primitive_poses:
            self.detected_objects[msg.id] = msg.primitive_poses[0]
            self.get_logger().info(
                f'Detected "{msg.id}" at '
                f'({msg.primitive_poses[0].position.x:.3f}, '
                f'{msg.primitive_poses[0].position.y:.3f}, '
                f'{msg.primitive_poses[0].position.z:.3f})'
            )

        # Forward to /planning_scene so MoveIt sees it
        scene = PlanningScene(is_diff=True)
        scene.world.collision_objects.append(msg)
        self.scene_pub.publish(scene)

    # ----------------------------- visualization --------------------------------

    def publish_pose_axes(self, pose: Pose, label: str, scale: float = 0.1):
        """Publish an axis marker (like a TF frame) at the given pose in RViz."""
        ma = MarkerArray()

        # Three arrows for x (red), y (green), z (blue)
        import tf_transformations
        import numpy as np

        q = [pose.orientation.x, pose.orientation.y,
             pose.orientation.z, pose.orientation.w]
        R = tf_transformations.quaternion_matrix(q)[:3, :3]

        colors = [
            ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0),  # x = red
            ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0),  # y = green
            ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0),  # z = blue
        ]

        origin = pose.position
        for axis_idx, color in enumerate(colors):
            m = Marker()
            m.header.frame_id = 'world'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = label
            m.id = self._marker_id
            self._marker_id += 1
            m.type = Marker.ARROW
            m.action = Marker.ADD
            m.scale = Vector3(x=0.008, y=0.015, z=0.0)  # shaft, head
            m.color = color
            m.lifetime = Duration(sec=30)

            direction = R[:, axis_idx] * scale
            m.points = [
                Point(x=origin.x, y=origin.y, z=origin.z),
                Point(
                    x=origin.x + direction[0],
                    y=origin.y + direction[1],
                    z=origin.z + direction[2],
                ),
            ]
            ma.markers.append(m)

        # Text label
        m = Marker()
        m.header.frame_id = 'world'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = label
        m.id = self._marker_id
        self._marker_id += 1
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.pose = copy.deepcopy(pose)
        m.pose.position.z += scale + 0.02
        m.scale.z = 0.03
        m.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        m.text = label
        m.lifetime = Duration(sec=30)
        ma.markers.append(m)

        self.marker_pub.publish(ma)

    # Initial joint configuration (matches initial_joint_positions.yaml)
    # Arm is upright and out of the camera's view
    INITIAL_JOINTS = {
        'panda_joint1': 0.0,
        'panda_joint2': -0.2,
        'panda_joint3': 0.0,
        'panda_joint4': -1.0,
        'panda_joint5': 0.0,
        'panda_joint6': 1.0,
        'panda_joint7': 0.0,
    }

    # ----------------------------- arm helpers --------------------------------

    def _build_move_goal(self) -> MoveGroup.Goal:
        """Create a MoveGroup goal with common settings."""
        goal = MoveGroup.Goal()
        goal.request.group_name = 'arm'
        goal.request.planner_id = 'RRTConnectkConfigDefault'
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 15.0
        goal.request.max_velocity_scaling_factor = 0.5
        goal.request.max_acceleration_scaling_factor = 0.5

        ws = WorkspaceParameters()
        ws.header.frame_id = 'world'
        ws.min_corner = Vector3(x=-2.0, y=-2.0, z=-2.0)
        ws.max_corner = Vector3(x=2.0, y=2.0, z=2.0)
        goal.request.workspace_parameters = ws

        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 3
        return goal

    def _send_move_goal(self, goal: MoveGroup.Goal) -> bool:
        """Send a MoveGroup goal and wait for the result."""
        future = self.move_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Arm goal rejected')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        wrapped = result_future.result()
        status = wrapped.status
        result = wrapped.result
        if result.error_code.val == MoveItErrorCodes.SUCCESS:
            self.get_logger().info('Arm motion succeeded')
            return True
        else:
            self.get_logger().error(
                f'Arm motion failed (code {result.error_code.val}, '
                f'action status {status})')
            self._dump_planning_scene()
            return False

    def _dump_planning_scene(self):
        """Query and log the current planning scene for debugging."""
        from moveit_msgs.srv import GetPlanningScene
        client = self.create_client(GetPlanningScene, '/get_planning_scene')
        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('GetPlanningScene service not available')
            return

        req = GetPlanningScene.Request()
        # Request world collision objects only (bit 1)
        req.components.components = req.components.WORLD_OBJECT_GEOMETRY
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        scene = future.result().scene
        for co in scene.world.collision_objects:
            dims = list(co.primitives[0].dimensions) if co.primitives else '?'
            op = co.pose.position
            pp = co.primitive_poses[0].position if co.primitive_poses else None
            pp_str = f'({pp.x:.3f}, {pp.y:.3f}, {pp.z:.3f})' if pp else 'none'
            self.get_logger().warn(
                f'  "{co.id}": dims={dims}, '
                f'frame_pose=({op.x:.3f}, {op.y:.3f}, {op.z:.3f}), '
                f'prim_pose={pp_str}'
            )

    def move_arm_to_joints(self, joint_positions: dict) -> bool:
        """Move the arm to a joint-space goal (reliable, no IK needed)."""
        from moveit_msgs.msg import JointConstraint
        goal = self._build_move_goal()

        c = Constraints()
        for name, value in joint_positions.items():
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = value
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            c.joint_constraints.append(jc)
        goal.request.goal_constraints.append(c)

        return self._send_move_goal(goal)

    def move_arm_to_pose(self, pose: Pose, acm_object_id: str = None,
                         position_tolerance: float = 0.05,
                         orientation_tolerance: float = 0.2) -> bool:
        """Plan and execute an arm motion to a Cartesian goal pose.

        If acm_object_id is given, allow gripper-object collisions
        directly in the goal's planning_scene_diff (atomic with planning).
        """
        goal = self._build_move_goal()
        goal.request.goal_constraints.append(
            self._pose_to_constraints(pose, 'world',
                                      position_tolerance=position_tolerance,
                                      orientation_tolerance=orientation_tolerance)
        )
        if acm_object_id:
            goal.planning_options.planning_scene_diff = self._build_acm_scene(acm_object_id)
        return self._send_move_goal(goal)

    @staticmethod
    def _pose_to_constraints(pose: Pose, frame_id: str,
                             position_tolerance: float = 0.05,
                             orientation_tolerance: float = 0.2) -> Constraints:
        """Convert a Pose into MoveIt Constraints (position + orientation)."""
        c = Constraints()

        # Position constraint: a tiny sphere around the target point
        pc = PositionConstraint()
        pc.header.frame_id = frame_id
        pc.link_name = 'panda_hand_tcp'
        pc.target_point_offset = Vector3(x=0.0, y=0.0, z=0.0)

        # Bounding region – small sphere
        region = SolidPrimitive()
        region.type = SolidPrimitive.SPHERE
        region.dimensions = [position_tolerance]
        pc.constraint_region.primitives.append(region)

        region_pose = Pose()
        region_pose.position = copy.deepcopy(pose.position)
        region_pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        pc.constraint_region.primitive_poses.append(region_pose)
        pc.weight = 1.0
        c.position_constraints.append(pc)

        # Orientation constraint
        oc = OrientationConstraint()
        oc.header.frame_id = frame_id
        oc.link_name = 'panda_hand_tcp'
        oc.orientation = copy.deepcopy(pose.orientation)
        oc.absolute_x_axis_tolerance = orientation_tolerance
        oc.absolute_y_axis_tolerance = orientation_tolerance
        oc.absolute_z_axis_tolerance = orientation_tolerance
        oc.weight = 1.0
        c.orientation_constraints.append(oc)

        return c

    # --------------------------- gripper helpers ------------------------------

    def move_gripper(self, open: bool) -> bool:
        """Open or close the gripper via the trajectory controller."""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = self.GRIPPER_JOINTS

        pt = JointTrajectoryPoint()
        pt.positions = self.GRIPPER_OPEN if open else self.GRIPPER_CLOSED
        pt.time_from_start = Duration(sec=1, nanosec=0)
        goal.trajectory.points.append(pt)

        future = self.gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Gripper goal rejected')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        self.get_logger().info(f'Gripper {"opened" if open else "closed"}')
        return True

    # ----------------------- planning-scene helpers ---------------------------

    def attach_object(self, object_id: str):
        """Attach object to the gripper in the planning scene."""
        aco = AttachedCollisionObject()
        aco.link_name = 'panda_hand'
        aco.object.id = object_id
        aco.object.header.frame_id = 'world'
        aco.object.operation = CollisionObject.ADD
        aco.touch_links = self.GRIPPER_LINKS

        scene = PlanningScene(is_diff=True)
        scene.robot_state.attached_collision_objects.append(aco)
        # Remove from world (it's now attached)
        remove = CollisionObject()
        remove.id = object_id
        remove.header.frame_id = 'world'
        remove.operation = CollisionObject.REMOVE
        scene.world.collision_objects.append(remove)
        self.scene_pub.publish(scene)
        self.get_logger().info(f'Attached "{object_id}" to panda_hand')

    def detach_object(self, object_id: str):
        """Detach object from the gripper back into the world."""
        aco = AttachedCollisionObject()
        aco.link_name = 'panda_hand'
        aco.object.id = object_id
        aco.object.operation = CollisionObject.REMOVE

        scene = PlanningScene(is_diff=True)
        scene.robot_state.attached_collision_objects.append(aco)
        scene.robot_state.is_diff = True
        self.scene_pub.publish(scene)
        self.get_logger().info(f'Detached "{object_id}" from panda_hand')

    # --------------------- high-level pick / place ----------------------------

    def pick(self, object_id: str, object_pose: Pose) -> bool:
        """
        Top-down pick sequence:

        ---- TODO 2: Pre-grasp approach ----
        1. Open gripper
        2. Move to a pose directly above the object (+0.1 m z offset)

        ---- TODO 3: Grasp ----
        3. Descend to the grasp pose (at the object centre height)
        4. Close gripper
        5. Attach object to the gripper in the planning scene

        ---- Post-grasp retreat ----
        6. Retreat upward (+0.1 m)
        """
        self.get_logger().info(f'Picking "{object_id}" ...')

        # 1. Open gripper
        if not self.move_gripper(open=True):
            return False

        # ---- TODO 2: Pre-grasp pose ----
        # Construct `pre_grasp` (Pose): position the gripper above the object
        # using TOP_DOWN_ORIENTATION, then call move_arm_to_pose().
        # Hint: study place() below for the same pattern.
        raise NotImplementedError('TODO 2: implement pre-grasp approach')

        # ---- TODO 3: Descend to grasp ----
        # Construct `grasp_pose` (Pose): descend closer to the object so the
        # fingers can close around it.  Pass acm_object_id=object_id to
        # move_arm_to_pose() to allow gripper-object collisions during planning.
        raise NotImplementedError('TODO 3: implement grasp descent')

        # 4. Close gripper
        if not self.move_gripper(open=False):
            return False

        # 5. Attach object in planning scene
        self.attach_object(object_id)

        # 6. Retreat upward
        retreat = copy.deepcopy(grasp_pose)
        retreat.position.z += 0.1
        if not self.move_arm_to_pose(retreat):
            return False

        self.get_logger().info(f'Pick of "{object_id}" complete')
        return True

    def place(self, object_id: str, place_pose: Pose) -> bool:
        """
        Top-down place sequence (provided as reference):
        1. Move above the placement location (+0.1 m z offset)
        2. Descend to the place height
        3. Open gripper
        4. Detach object from the planning scene
        5. Retreat upward (+0.1 m)
        """
        self.get_logger().info(f'Placing "{object_id}" ...')

        # Pre-place: position above the target
        pre_place = Pose()
        pre_place.position = Point(
            x=place_pose.position.x,
            y=place_pose.position.y,
            z=place_pose.position.z + 0.1
        )
        pre_place.orientation = self.TOP_DOWN_ORIENTATION
        self.publish_pose_axes(pre_place, 'pre_place')
        if not self.move_arm_to_pose(pre_place):
            return False

        # Descend to place height
        place_down = copy.deepcopy(pre_place)
        place_down.position.z = place_pose.position.z
        if not self.move_arm_to_pose(place_down):
            return False

        # Open gripper to release
        if not self.move_gripper(open=True):
            return False

        # Detach from planning scene
        self.detach_object(object_id)

        # Retreat upward
        retreat = copy.deepcopy(place_down)
        retreat.position.z += 0.1
        if not self.move_arm_to_pose(retreat):
            return False

        self.get_logger().info(f'Place of "{object_id}" complete')
        return True

    GRIPPER_LINKS = ['panda_link8', 'panda_hand', 'panda_leftfinger', 'panda_rightfinger']

    def _build_acm_scene(self, object_id: str) -> PlanningScene:
        """Build a PlanningScene diff that allows object to collide with ALL links."""
        scene = PlanningScene(is_diff=True)
        acm = scene.allowed_collision_matrix
        acm.default_entry_names = [object_id]
        acm.default_entry_values = [True]
        return scene

    def allow_gripper_object_collision(self, object_id: str, allow: bool):
        """Allow or disallow collisions between gripper links and the target object."""
        scene = self._build_acm_scene(object_id)
        if not allow:
            # Flip all True entries to False
            for entry in scene.allowed_collision_matrix.entry_values:
                entry.enabled = [False] * len(entry.enabled)
        self.scene_pub.publish(scene)
        state = "Allowing" if allow else "Disallowing"
        self.get_logger().info(f'{state} gripper-object collision for "{object_id}"')

    def retreat_to_initial(self):
        """Move the arm back to the initial pose so it's out of the camera view."""
        self.get_logger().info('Retreating to initial joint configuration...')
        if not self.move_arm_to_joints(self.INITIAL_JOINTS):
            self.get_logger().error('Failed to retreat to initial pose!')

    def pick_and_place(self, object_id: str, object_pose: Pose, place_pose: Pose) -> bool:
        # Pause perception so the robot doesn't become part of the collision object
        self.pause_pub.publish(Bool(data=True))
        self._manipulating = True
        self.get_logger().info('Paused object detection')

        try:
            if not self.pick(object_id, object_pose):
                self.get_logger().error('Pick failed, retreating to initial pose')
                self.retreat_to_initial()
                return False
            if not self.place(object_id, place_pose):
                self.get_logger().error('Place failed, retreating to initial pose')
                self.retreat_to_initial()
                return False
            self.retreat_to_initial()
            return True
        finally:
            self._manipulating = False
            self.pause_pub.publish(Bool(data=False))
            self.get_logger().info('Resumed object detection')


def main(args=None):
    rclpy.init(args=args)
    node = GraspPlanner()

    # ---- TODO 7: Main pick-and-place logic ----
    # Wait for the object detector to publish a collision object,
    # then pick it and place it at a new location.

    # Wait for joint states so MoveIt has a valid start state
    node.get_logger().info('Waiting for /joint_states ...')
    while node.latest_joint_state is None:
        rclpy.spin_once(node, timeout_sec=0.5)
    js = node.latest_joint_state
    node.get_logger().info(f'Got joint states:')
    for name, pos in zip(js.name, js.position):
        node.get_logger().info(f'  {name}: {pos:.4f}')

    node.get_logger().info('Waiting for detected objects on /collision_object ...')
    while not node.detected_objects:
        rclpy.spin_once(node, timeout_sec=0.5)

    # Select the first detected object
    object_id, object_pose = next(iter(node.detected_objects.items()))
    node.get_logger().info(f'Targeting "{object_id}" for pick-and-place')

    # Place the object at a random nearby spot on the table,
    # biased toward the robot (negative x direction).
    dx = random.uniform(-0.15, -0.05)
    dy = random.uniform(0.05, 0.15) * random.choice([-1, 1])
    place_pose = Pose()
    place_pose.position = Point(
        x=object_pose.position.x + dx,
        y=object_pose.position.y + dy,
        z=object_pose.position.z + 0.08,
    )
    place_pose.orientation = GraspPlanner.TOP_DOWN_ORIENTATION
    node.get_logger().info(
        f'Place target: ({place_pose.position.x:.3f}, '
        f'{place_pose.position.y:.3f}, {place_pose.position.z:.3f})'
    )

    success = node.pick_and_place(object_id, object_pose, place_pose)
    if success:
        node.get_logger().info('Pick-and-place complete!')
    else:
        node.get_logger().error('Pick-and-place failed.')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
