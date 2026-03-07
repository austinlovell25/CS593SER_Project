#!/usr/bin/env python3
"""
grasp_planner_gpd.py  –  GPD-based grasp planner for a user-specified cuboid.

How it works
------------
1. The user specifies the object as a cuboid via ROS parameters
   (or lets it be detected automatically from /collision_object).
2. A dense surface point cloud is sampled for the cuboid and written to a
   temporary PCD file.
3. GPD (Grasp Pose Detection) is called through its ctypes Python interface
   (libgpd_python.so, built from gpd/src/detect_grasps_python.cpp) to score
   and rank 6-DOF grasp candidates.
4. The top-ranked feasible grasp is converted to a Panda TCP Pose and executed
   with MoveIt 2.

Prerequisites
-------------
Build the GPD Python shared library first:

    cd /path/to/workspace/gpd
    mkdir -p build && cd build
    cmake .. && make -j$(nproc) gpd_python

Then run this node with:

    ros2 run panda_moveit_config grasp_planner_gpd.py --ros-args \\
      -p object_width:=0.05  \\
      -p object_depth:=0.04  \\
      -p object_height:=0.07 \\
      -p gpd_config:=/path/to/gpd/cfg/eigen_params.cfg \\
      -p gpd_lib:=/path/to/gpd/build/libgpd_python.so

If gpd_lib is empty or the library is not found, the node falls back to a
simple top-down analytical grasp (same as grasp_planner.py).

GPD frame → Panda TCP frame convention
---------------------------------------
GPD orientation matrix columns (from gpd/candidate/hand_set.cpp):
    col 0  =  approach direction (toward the object surface)
    col 1  =  binormal
    col 2  =  hand axis (finger closing direction)

Panda panda_hand_tcp axes:
    x  =  finger closing axis  →  GPD col 2
    y  =  -binormal            →  -GPD col 1   (sign flip for det = +1)
    z  =  approach             →  GPD col 0

So:  R_panda = [ col2 | -col1 | col0 ]
"""

import copy
import ctypes
import os
import random
import struct
import tempfile

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import (Point, Pose, PoseStamped, Quaternion, Vector3)
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    AttachedCollisionObject, CollisionObject, Constraints, MoveItErrorCodes,
    OrientationConstraint, PlanningScene, PositionConstraint,
    RobotState, WorkspaceParameters,
)
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Bool, ColorRGBA
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import Marker, MarkerArray

import tf_transformations


# ---------------------------------------------------------------------------
# GPD ctypes interface
# ---------------------------------------------------------------------------

class _GraspStruct(ctypes.Structure):
    """Mirrors the C Grasp struct in gpd/src/detect_grasps_python.cpp."""
    _fields_ = [
        ('pos',    ctypes.POINTER(ctypes.c_double)),  # [3]  grasp position
        ('orient', ctypes.POINTER(ctypes.c_double)),  # [4]  quaternion x,y,z,w
        ('sample', ctypes.POINTER(ctypes.c_double)),  # [3]  sample point
        ('score',  ctypes.c_double),
        ('label',  ctypes.c_bool),                    # is_full_antipodal
        ('image',  ctypes.POINTER(ctypes.c_int)),
    ]


class GPDInterface:
    """
    Thin wrapper around libgpd_python.so.

    Loads the shared library and exposes detectGraspsInFile() which takes a
    PCD file path and a GPD config file and returns a list of grasp dicts:
        {'pos': [x,y,z], 'orient': [qx,qy,qz,qw], 'score': float, 'antipodal': bool}
    """

    def __init__(self, lib_path: str):
        self._lib = ctypes.CDLL(lib_path)

        # int detectGraspsInFile(config, pcd, normals, view_points,
        #                        num_view_points, Grasp**)
        fn = self._lib.detectGraspsInFile
        fn.restype = ctypes.c_int
        fn.argtypes = [
            ctypes.c_char_p,
            ctypes.c_char_p,
            ctypes.c_char_p,
            ctypes.POINTER(ctypes.c_float),
            ctypes.c_int,
            ctypes.POINTER(ctypes.POINTER(_GraspStruct)),
        ]
        self._detect = fn

        free_fn = self._lib.freeMemoryGrasps
        free_fn.restype = ctypes.c_int
        free_fn.argtypes = [ctypes.POINTER(_GraspStruct)]
        self._free = free_fn

    def detect(self, config: str, pcd: str, camera_pos=(0.0, 0.0, 1.0)):
        """
        Run GPD on *pcd* with *config* and return sorted grasp candidates.

        Parameters
        ----------
        config : str     path to GPD *.cfg file
        pcd    : str     path to ASCII/binary PCD file
        camera_pos : (x,y,z)  position of the camera viewpoint in world frame

        Returns
        -------
        list of dict  sorted by score (descending)
            {'pos': np.ndarray[3], 'R': np.ndarray[3,3], 'score': float,
             'antipodal': bool}
        """
        vp = (ctypes.c_float * 3)(*camera_pos)
        grasps_ptr = ctypes.POINTER(_GraspStruct)()

        n = self._detect(
            config.encode(),
            pcd.encode(),
            None,          # no normals file
            vp, 1,
            ctypes.byref(grasps_ptr),
        )

        results = []
        for i in range(n):
            g = grasps_ptr[i]
            pos = np.array([g.pos[0], g.pos[1], g.pos[2]])
            # Eigen::Quaterniond stores coefficients as [x, y, z, w]
            qx, qy, qz, qw = g.orient[0], g.orient[1], g.orient[2], g.orient[3]
            # Reconstruct rotation matrix from quaternion
            R = tf_transformations.quaternion_matrix([qx, qy, qz, qw])[:3, :3]
            results.append({
                'pos':       pos,
                'R':         R,
                'score':     g.score,
                'antipodal': bool(g.label),
            })

        if n > 0:
            self._free(grasps_ptr)

        results.sort(key=lambda g: g['score'], reverse=True)
        return results


def _gpd_R_to_panda_pose(pos: np.ndarray, R_gpd: np.ndarray,
                         depth_offset: float = 0.03) -> Pose:
    """
    Convert a GPD grasp (world-frame position + orientation matrix) to a
    Panda TCP Pose.

    Mapping:  R_panda = [ col2 | -col1 | col0 ]
    (see module docstring for derivation)

    depth_offset : shift the TCP forward along the approach axis so the
                   fingers actually wrap around the object rather than
                   stopping at the contact surface (GPD's pos).
    """
    col0 = R_gpd[:, 0]  # approach
    col1 = R_gpd[:, 1]  # binormal
    col2 = R_gpd[:, 2]  # hand axis

    R_panda = np.column_stack([col2, -col1, col0])

    # Shift the TCP forward along the approach direction so the fingers
    # reach proper depth (GPD pos is the surface contact point).
    tcp_pos = pos + depth_offset * col0

    # Embed into 4×4 for tf_transformations
    T = np.eye(4)
    T[:3, :3] = R_panda
    T[:3, 3] = tcp_pos

    qx, qy, qz, qw = tf_transformations.quaternion_from_matrix(T)

    p = Pose()
    p.position = Point(x=float(tcp_pos[0]), y=float(tcp_pos[1]), z=float(tcp_pos[2]))
    p.orientation = Quaternion(x=float(qx), y=float(qy),
                               z=float(qz), w=float(qw))
    return p


# ---------------------------------------------------------------------------
# PCD utilities
# ---------------------------------------------------------------------------

def sample_cuboid_surface(center, dims, n_points: int = 2000) -> np.ndarray:
    """
    Sample *n_points* uniformly on the surface of a cuboid.

    Parameters
    ----------
    center : (x, y, z)   world-frame centre of the cuboid
    dims   : (dx, dy, dz) edge lengths in metres
    """
    cx, cy, cz = center
    dx, dy, dz = dims
    hx, hy, hz = dx / 2.0, dy / 2.0, dz / 2.0

    Axy = dx * dy   # top + bottom
    Axz = dx * dz   # front + back
    Ayz = dy * dz   # left + right
    A_total = 2.0 * (Axy + Axz + Ayz)

    parts = []
    for (face_dims, fixed_axis, fixed_val) in [
        ((hx, hy), 2,  hz),  # top    (+z)
        ((hx, hy), 2, -hz),  # bottom (-z)
        ((hx, hz), 1,  hy),  # front  (+y)
        ((hx, hz), 1, -hy),  # back   (-y)
        ((hy, hz), 0,  hx),  # right  (+x)
        ((hy, hz), 0, -hx),  # left   (-x)
    ]:
        face_A = face_dims[0] * face_dims[1] * 4.0   # full half-extents → area
        n = max(4, int(n_points * face_A / A_total))
        u = np.random.uniform(-face_dims[0], face_dims[0], n)
        v = np.random.uniform(-face_dims[1], face_dims[1], n)
        face_pts = np.zeros((n, 3))
        if fixed_axis == 2:
            face_pts[:, 0] = u;  face_pts[:, 1] = v;  face_pts[:, 2] = fixed_val
        elif fixed_axis == 1:
            face_pts[:, 0] = u;  face_pts[:, 1] = fixed_val;  face_pts[:, 2] = v
        else:
            face_pts[:, 0] = fixed_val;  face_pts[:, 1] = u;  face_pts[:, 2] = v
        parts.append(face_pts)

    pts = np.vstack(parts).astype(np.float32)
    pts[:, 0] += cx
    pts[:, 1] += cy
    pts[:, 2] += cz
    return pts


def write_pcd_ascii(points: np.ndarray, path: str):
    """Write an (N,3) float32 array as an ASCII PCD file."""
    n = len(points)
    with open(path, 'w') as f:
        f.write('# .PCD v0.7 - Point Cloud Data file format\n')
        f.write('VERSION 0.7\n')
        f.write('FIELDS x y z\n')
        f.write('SIZE 4 4 4\n')
        f.write('TYPE F F F\n')
        f.write('COUNT 1 1 1\n')
        f.write(f'WIDTH {n}\n')
        f.write('HEIGHT 1\n')
        f.write('VIEWPOINT 0 0 0 1 0 0 0\n')
        f.write(f'POINTS {n}\n')
        f.write('DATA ascii\n')
        for p in points:
            f.write(f'{p[0]:.6f} {p[1]:.6f} {p[2]:.6f}\n')


# ---------------------------------------------------------------------------
# Main node
# ---------------------------------------------------------------------------

class GraspPlannerGPD(Node):
    """
    Grasp planner that uses GPD to propose 6-DOF grasps on a cuboid object.

    The object can be:
    - detected automatically from /collision_object (same as grasp_planner.py)
    - overridden via ROS parameters 'object_pose_x/y/z' and
      'object_width/depth/height'.
    """

    GRIPPER_JOINTS = ['panda_finger_joint1', 'panda_finger_joint2']
    GRIPPER_OPEN   = [0.04, 0.04]
    GRIPPER_CLOSED = [0.0, 0.0]   # command fully closed; object stops fingers
    GRIPPER_LINKS  = ['panda_link8', 'panda_hand', 'panda_leftfinger', 'panda_rightfinger']

    # Fallback top-down orientation (180° about x) used when GPD is unavailable
    TOP_DOWN_ORIENTATION = Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)

    INITIAL_JOINTS = {
        'panda_joint1': 0.0,
        'panda_joint2': -0.2,
        'panda_joint3': 0.0,
        'panda_joint4': -1.0,
        'panda_joint5': 0.0,
        'panda_joint6':  1.0,
        'panda_joint7': 0.0,
    }

    # Arm pose that points the wrist camera down at the tabletop workspace.
    # joint4 deeply negative bends the elbow so the hand faces the table;
    # joint6 high pitches the wrist so the camera looks straight down.
    # Tune these values if the camera view needs adjustment.
    LOOK_JOINTS = {
        'panda_joint1': 0.0,
        'panda_joint2': -0.2,
        'panda_joint3': 0.0,
        'panda_joint4': -2.6,
        'panda_joint5': 0.0,
        'panda_joint6':  2.4,
        'panda_joint7': 0.785,
    }

    def __init__(self):
        super().__init__('grasp_planner_gpd')

        # ---- ROS parameters ------------------------------------------------
        self.declare_parameter('gpd_config', '')
        self.declare_parameter('gpd_lib',    '')
        self.declare_parameter('grasp_depth_offset', 0.03)  # metres forward along approach
        # Camera viewpoint for GPD normal estimation (world frame)
        self.declare_parameter('camera_x', 0.6)
        self.declare_parameter('camera_y', 0.0)
        self.declare_parameter('camera_z', 1.05)
        # Optional manual object override (leave 0 to use detected pose)
        self.declare_parameter('object_pose_x', 0.0)
        self.declare_parameter('object_pose_y', 0.0)
        self.declare_parameter('object_pose_z', 0.0)
        # Cuboid dimensions (metres)
        self.declare_parameter('object_width',  0.05)   # x extent
        self.declare_parameter('object_depth',  0.04)   # y extent
        self.declare_parameter('object_height', 0.07)   # z extent

        # ---- GPD interface --------------------------------------------------
        self._gpd: GPDInterface | None = None
        lib_path = self.get_parameter('gpd_lib').value
        if lib_path and os.path.isfile(lib_path):
            try:
                self._gpd = GPDInterface(lib_path)
                self.get_logger().info(f'Loaded GPD library from {lib_path}')
            except Exception as e:
                self.get_logger().error(f'Failed to load GPD lib: {e}')
        else:
            self.get_logger().warn(
                'gpd_lib not set or file not found – falling back to '
                'top-down analytical grasp.  Build libgpd_python.so first.'
            )

        # ---- Action clients ------------------------------------------------
        self.move_client = ActionClient(self, MoveGroup, '/move_action')
        self.gripper_client = ActionClient(
            self, FollowJointTrajectory,
            '/gripper_trajectory_controller/follow_joint_trajectory',
        )

        # ---- Publishers / subscribers -------------------------------------
        self.scene_pub = self.create_publisher(PlanningScene, '/planning_scene', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/grasp_markers', 10)
        self._marker_id = 0

        self.detected_objects: dict[str, tuple[Pose, list]] = {}
        self._manipulating = False

        self.collision_sub = self.create_subscription(
            CollisionObject, '/collision_object',
            self._collision_cb, 10,
        )

        pause_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pause_pub = self.create_publisher(Bool, '/pause_detection',
                                               qos_profile=pause_qos)

        self.latest_joint_state = None
        self.create_subscription(
            JointState, '/joint_states',
            lambda msg: setattr(self, 'latest_joint_state', msg), 10,
        )

        self.get_logger().info('Waiting for /move_action ...')
        self.move_client.wait_for_server()
        self.get_logger().info('Waiting for gripper controller ...')
        self.gripper_client.wait_for_server()
        self.get_logger().info('GraspPlannerGPD ready')

    # ------------------------------------------------------------------
    # Collision-object callback
    # ------------------------------------------------------------------

    def _collision_cb(self, msg: CollisionObject):
        if self._manipulating:
            return
        if msg.operation == CollisionObject.ADD and msg.primitive_poses:
            pose = msg.primitive_poses[0]
            dims = list(msg.primitives[0].dimensions) if msg.primitives else [0.05, 0.05, 0.05]
            self.detected_objects[msg.id] = (pose, dims)
            self.get_logger().info(
                f'Detected "{msg.id}" at '
                f'({pose.position.x:.3f}, {pose.position.y:.3f}, {pose.position.z:.3f}) '
                f'dims={[f"{d:.3f}" for d in dims]}'
            )
        scene = PlanningScene(is_diff=True)
        scene.world.collision_objects.append(msg)
        self.scene_pub.publish(scene)

    # ------------------------------------------------------------------
    # Visualization helpers
    # ------------------------------------------------------------------

    def _publish_pose_axes(self, pose: Pose, label: str, scale: float = 0.1):
        """Draw RGB axis arrows and a text label at *pose* in RViz."""
        ma = MarkerArray()
        q = [pose.orientation.x, pose.orientation.y,
             pose.orientation.z, pose.orientation.w]
        R = tf_transformations.quaternion_matrix(q)[:3, :3]
        origin = pose.position
        colors = [
            ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0),
            ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0),
            ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0),
        ]
        for axis_idx, color in enumerate(colors):
            m = Marker()
            m.header.frame_id = 'world'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = label;  m.id = self._marker_id;  self._marker_id += 1
            m.type = Marker.ARROW;  m.action = Marker.ADD
            m.scale = Vector3(x=0.008, y=0.015, z=0.0)
            m.color = color;  m.lifetime = Duration(sec=60)
            d = R[:, axis_idx] * scale
            m.points = [
                Point(x=origin.x, y=origin.y, z=origin.z),
                Point(x=origin.x + d[0], y=origin.y + d[1], z=origin.z + d[2]),
            ]
            ma.markers.append(m)
        # text
        m = Marker()
        m.header.frame_id = 'world';  m.header.stamp = self.get_clock().now().to_msg()
        m.ns = label;  m.id = self._marker_id;  self._marker_id += 1
        m.type = Marker.TEXT_VIEW_FACING;  m.action = Marker.ADD
        m.pose = copy.deepcopy(pose);  m.pose.position.z += scale + 0.02
        m.scale.z = 0.03;  m.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        m.text = label;  m.lifetime = Duration(sec=60)
        ma.markers.append(m)
        self.marker_pub.publish(ma)

    def _publish_grasp_candidates(self, candidates: list):
        """Visualize all GPD grasp candidates as green arrows in RViz."""
        depth_offset = self.get_parameter('grasp_depth_offset').value
        ma = MarkerArray()
        for i, g in enumerate(candidates):
            pose = _gpd_R_to_panda_pose(g['pos'], g['R'], depth_offset)
            q = [pose.orientation.x, pose.orientation.y,
                 pose.orientation.z, pose.orientation.w]
            R = tf_transformations.quaternion_matrix(q)[:3, :3]
            # Draw approach arrow (z-axis, blue intensity ∝ score)
            score_norm = min(1.0, max(0.0, float(g['score'])))
            m = Marker()
            m.header.frame_id = 'world'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'gpd_candidates';  m.id = i
            m.type = Marker.ARROW;  m.action = Marker.ADD
            m.scale = Vector3(x=0.008, y=0.015, z=0.0)
            m.color = ColorRGBA(r=0.0, g=score_norm, b=1.0 - score_norm, a=0.8)
            m.lifetime = Duration(sec=60)
            origin = pose.position
            d = R[:, 2] * 0.08
            m.points = [
                Point(x=origin.x, y=origin.y, z=origin.z),
                Point(x=origin.x + d[0], y=origin.y + d[1], z=origin.z + d[2]),
            ]
            ma.markers.append(m)
        self.marker_pub.publish(ma)
        self.get_logger().info(f'Published {len(candidates)} GPD candidate markers')

    # ------------------------------------------------------------------
    # Arm helpers (same as grasp_planner.py)
    # ------------------------------------------------------------------

    def _build_move_goal(self) -> MoveGroup.Goal:
        goal = MoveGroup.Goal()
        goal.request.group_name = 'arm'
        goal.request.planner_id = 'RRTConnectkConfigDefault'
        goal.request.num_planning_attempts = 5
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = 1.0
        goal.request.max_acceleration_scaling_factor = 1.0
        ws = WorkspaceParameters()
        ws.header.frame_id = 'world'
        ws.min_corner = Vector3(x=-2.0, y=-2.0, z=-2.0)
        ws.max_corner = Vector3(x=2.0,  y=2.0,  z=2.0)
        goal.request.workspace_parameters = ws
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 3
        return goal

    def _send_move_goal(self, goal: MoveGroup.Goal) -> bool:
        future = self.move_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Arm goal rejected')
            return False
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        if result.error_code.val == MoveItErrorCodes.SUCCESS:
            self.get_logger().info('Arm motion succeeded')
            return True
        self.get_logger().error(f'Arm motion failed (code {result.error_code.val})')
        return False

    def move_arm_to_joints(self, joint_positions: dict) -> bool:
        from moveit_msgs.msg import JointConstraint
        goal = self._build_move_goal()
        c = Constraints()
        for name, value in joint_positions.items():
            jc = JointConstraint()
            jc.joint_name = name;  jc.position = value
            jc.tolerance_above = 0.01;  jc.tolerance_below = 0.01;  jc.weight = 1.0
            c.joint_constraints.append(jc)
        goal.request.goal_constraints.append(c)
        return self._send_move_goal(goal)

    def move_arm_to_pose(self, pose: Pose, acm_object_id: str = None,
                         position_tolerance: float = 0.05,
                         orientation_tolerance: float = 0.2,
                         planning_time: float = None) -> bool:
        goal = self._build_move_goal()
        if planning_time is not None:
            goal.request.allowed_planning_time = planning_time
        goal.request.goal_constraints.append(
            self._pose_to_constraints(pose, 'world', position_tolerance,
                                      orientation_tolerance)
        )
        if acm_object_id:
            goal.planning_options.planning_scene_diff = self._build_acm_scene(acm_object_id)
        return self._send_move_goal(goal)

    @staticmethod
    def _pose_to_constraints(pose: Pose, frame_id: str,
                             position_tolerance: float = 0.05,
                             orientation_tolerance: float = 0.2) -> Constraints:
        c = Constraints()
        pc = PositionConstraint()
        pc.header.frame_id = frame_id;  pc.link_name = 'panda_hand_tcp'
        pc.target_point_offset = Vector3(x=0.0, y=0.0, z=0.0)
        region = SolidPrimitive()
        region.type = SolidPrimitive.SPHERE;  region.dimensions = [position_tolerance]
        pc.constraint_region.primitives.append(region)
        rp = Pose()
        rp.position = copy.deepcopy(pose.position)
        rp.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        pc.constraint_region.primitive_poses.append(rp);  pc.weight = 1.0
        c.position_constraints.append(pc)
        oc = OrientationConstraint()
        oc.header.frame_id = frame_id;  oc.link_name = 'panda_hand_tcp'
        oc.orientation = copy.deepcopy(pose.orientation)
        oc.absolute_x_axis_tolerance = orientation_tolerance
        oc.absolute_y_axis_tolerance = orientation_tolerance
        oc.absolute_z_axis_tolerance = orientation_tolerance
        oc.weight = 1.0
        c.orientation_constraints.append(oc)
        return c

    def _build_acm_scene(self, object_id: str) -> PlanningScene:
        scene = PlanningScene(is_diff=True)
        acm = scene.allowed_collision_matrix
        acm.default_entry_names = [object_id]
        acm.default_entry_values = [True]
        return scene

    # ------------------------------------------------------------------
    # Gripper helpers
    # ------------------------------------------------------------------

    def move_gripper(self, open: bool) -> bool:
        from control_msgs.msg import JointTolerance
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = self.GRIPPER_JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = self.GRIPPER_OPEN if open else self.GRIPPER_CLOSED
        pt.time_from_start = Duration(sec=2, nanosec=0)
        goal.trajectory.points.append(pt)
        # Large goal tolerance so the controller doesn't fault when an object
        # blocks the fingers from reaching fully-closed position.
        if not open:
            for name in self.GRIPPER_JOINTS:
                tol = JointTolerance()
                tol.name = name
                tol.position = 0.05   # allow up to 5 cm error (object in the way)
                goal.goal_tolerance.append(tol)
        future = self.gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Gripper goal rejected');  return False
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        self.get_logger().info(f'Gripper {"opened" if open else "closed"}')
        return True

    # ------------------------------------------------------------------
    # Planning-scene helpers
    # ------------------------------------------------------------------

    def remove_object_from_scene(self, object_id: str):
        """Remove a collision object from the planning scene (before grasp descent)."""
        co = CollisionObject()
        co.id = object_id;  co.header.frame_id = 'world'
        co.operation = CollisionObject.REMOVE
        scene = PlanningScene(is_diff=True)
        scene.world.collision_objects.append(co)
        self.scene_pub.publish(scene)
        self.get_logger().info(f'Removed "{object_id}" from planning scene')

    def attach_object(self, object_id: str):
        aco = AttachedCollisionObject()
        aco.link_name = 'panda_hand';  aco.object.id = object_id
        aco.object.header.frame_id = 'world';  aco.object.operation = CollisionObject.ADD
        aco.touch_links = self.GRIPPER_LINKS
        scene = PlanningScene(is_diff=True)
        scene.robot_state.attached_collision_objects.append(aco)
        remove = CollisionObject()
        remove.id = object_id;  remove.header.frame_id = 'world'
        remove.operation = CollisionObject.REMOVE
        scene.world.collision_objects.append(remove)
        self.scene_pub.publish(scene)
        self.get_logger().info(f'Attached "{object_id}" to panda_hand')

    def detach_object(self, object_id: str):
        aco = AttachedCollisionObject()
        aco.link_name = 'panda_hand';  aco.object.id = object_id
        aco.object.operation = CollisionObject.REMOVE
        scene = PlanningScene(is_diff=True)
        scene.robot_state.attached_collision_objects.append(aco)
        scene.robot_state.is_diff = True
        self.scene_pub.publish(scene)
        self.get_logger().info(f'Detached "{object_id}" from panda_hand')

    def retreat_to_initial(self):
        self.get_logger().info('Retreating to initial configuration ...')
        self.move_gripper(open=True)
        self.move_arm_to_joints(self.INITIAL_JOINTS)

    def look(self) -> bool:
        """Move to the observation pose so the wrist camera sees the workspace."""
        self.get_logger().info('Moving to observation pose ...')
        return self.move_arm_to_joints(self.LOOK_JOINTS)

    # ------------------------------------------------------------------
    # GPD grasp selection
    # ------------------------------------------------------------------

    def _run_gpd(self, object_pose: Pose, dims: list) -> list:
        """
        Generate surface point cloud for the cuboid, run GPD, and return
        ranked grasp candidates.  Returns [] if GPD is unavailable.
        """
        if self._gpd is None:
            return []

        config = self.get_parameter('gpd_config').value
        if not config or not os.path.isfile(config):
            self.get_logger().error(
                f'gpd_config not set or file not found: "{config}"'
            )
            return []

        cam_x = self.get_parameter('camera_x').value
        cam_y = self.get_parameter('camera_y').value
        cam_z = self.get_parameter('camera_z').value

        center = (object_pose.position.x,
                  object_pose.position.y,
                  object_pose.position.z)

        self.get_logger().info(
            f'Sampling cuboid point cloud: center={center}, dims={dims}'
        )
        pts = sample_cuboid_surface(center, dims, n_points=3000)

        with tempfile.NamedTemporaryFile(suffix='.pcd', delete=False) as f:
            pcd_path = f.name
        try:
            write_pcd_ascii(pts, pcd_path)
            self.get_logger().info(
                f'Running GPD on {len(pts)} points (PCD: {pcd_path}) ...'
            )
            candidates = self._gpd.detect(config, pcd_path,
                                           camera_pos=(cam_x, cam_y, cam_z))
            self.get_logger().info(f'GPD returned {len(candidates)} candidates')
        finally:
            os.unlink(pcd_path)

        return candidates

    # ------------------------------------------------------------------
    # High-level pick / place
    # ------------------------------------------------------------------

    def pick(self, object_id: str, object_pose: Pose, dims: list) -> bool:
        """
        GPD-assisted pick sequence:

        1. Run GPD to get ranked grasp candidates.
        2. For each candidate (best first):
           a. Open gripper.
           b. Move to pre-grasp (10 cm back along approach).
           c. Move to grasp pose.
           d. Close gripper.
           e. Attach object and retreat.
        3. Fall back to top-down analytical grasp if GPD fails / unavailable.
        """
        self.get_logger().info(f'Picking "{object_id}" with GPD ...')

        candidates = self._run_gpd(object_pose, dims)

        if candidates:
            self._publish_grasp_candidates(candidates)

        # Build list of (pose, label) to try.  GPD candidates first, then fallback.
        grasp_poses = []
        depth_offset = self.get_parameter('grasp_depth_offset').value
        for i, g in enumerate(candidates):
            # Pre-filter: reject grasps approaching from below the table or
            # with TCP below the object (approach z-component pointing up).
            approach = g['R'][:, 0]          # GPD col 0 = approach direction
            if approach[2] > 0.3:            # approaching from below — skip
                continue
            # Reject grasps where TCP would be below the table surface (~0.76 m)
            tcp_z = g['pos'][2] + depth_offset * approach[2]
            if tcp_z < 0.20:
                continue
            grasp_pose = _gpd_R_to_panda_pose(g['pos'], g['R'], depth_offset)
            grasp_poses.append((grasp_pose, f'gpd_{i}', g['score']))

        # Analytical top-down fallback
        fallback = Pose()
        fallback.position = Point(
            x=object_pose.position.x,
            y=object_pose.position.y,
            z=object_pose.position.z + 0.08,
        )
        fallback.orientation = self.TOP_DOWN_ORIENTATION
        grasp_poses.append((fallback, 'top_down_fallback', 0.0))

        for grasp_pose, label, score in grasp_poses:
            self.get_logger().info(
                f'Trying grasp "{label}" (score={score:.3f}) at '
                f'({grasp_pose.position.x:.3f}, '
                f'{grasp_pose.position.y:.3f}, '
                f'{grasp_pose.position.z:.3f})'
            )
            self._publish_pose_axes(grasp_pose, label)

            # Pre-grasp: retreat 10 cm along the approach axis (−z of TCP)
            q = [grasp_pose.orientation.x, grasp_pose.orientation.y,
                 grasp_pose.orientation.z, grasp_pose.orientation.w]
            R = tf_transformations.quaternion_matrix(q)[:3, :3]
            approach_vec = R[:, 2]  # TCP z-axis = approach direction

            pre_grasp = copy.deepcopy(grasp_pose)
            pre_grasp.position.x -= float(approach_vec[0]) * 0.10
            pre_grasp.position.y -= float(approach_vec[1]) * 0.10
            pre_grasp.position.z -= float(approach_vec[2]) * 0.10
            self._publish_pose_axes(pre_grasp, f'{label}_pre')

            if not self.move_gripper(open=True):
                continue
            if not self.move_arm_to_pose(pre_grasp):
                self.get_logger().warn(f'Pre-grasp failed for {label}, trying next')
                self.move_arm_to_joints(self.INITIAL_JOINTS)
                continue
            if not self.move_arm_to_pose(grasp_pose,
                                          acm_object_id=object_id,
                                          position_tolerance=0.005,
                                          orientation_tolerance=0.05,
                                          planning_time=2.0):
                self.get_logger().warn(f'Grasp descent failed for {label}, trying next')
                self.move_arm_to_joints(self.INITIAL_JOINTS)
                continue
            if not self.move_gripper(open=False):
                self.move_arm_to_joints(self.INITIAL_JOINTS)
                continue

            self.attach_object(object_id)

            # Step 1: retreat along the approach axis to clear the object
            retreat = copy.deepcopy(grasp_pose)
            retreat.position.x -= float(approach_vec[0]) * 0.10
            retreat.position.y -= float(approach_vec[1]) * 0.10
            retreat.position.z -= float(approach_vec[2]) * 0.10
            self.move_arm_to_pose(retreat)

            # Step 2: lift straight up so we don't drag the object sideways
            lift = copy.deepcopy(retreat)
            lift.position.z += 0.15
            self.move_arm_to_pose(lift)

            self.get_logger().info(f'Pick succeeded with grasp "{label}"')
            return True

        self.get_logger().error('All grasp candidates failed')
        return False

    def place(self, object_id: str, place_pose: Pose, object_half_height: float = 0.10) -> bool:
        """Top-down place sequence: move above target, lower to surface, release."""
        self.get_logger().info(f'Placing "{object_id}" ...')

        # TCP set-down height = table surface + half object height
        # place_pose.position.z is the table surface at the drop location
        set_down_z = place_pose.position.z + object_half_height

        pre_place = Pose()
        pre_place.position = Point(
            x=place_pose.position.x,
            y=place_pose.position.y,
            z=set_down_z + 0.12,
        )
        pre_place.orientation = self.TOP_DOWN_ORIENTATION
        self._publish_pose_axes(pre_place, 'pre_place')
        if not self.move_arm_to_pose(pre_place):
            return False

        place_down = copy.deepcopy(pre_place)
        place_down.position.z = set_down_z
        if not self.move_arm_to_pose(place_down):
            return False

        if not self.move_gripper(open=True):
            return False

        self.detach_object(object_id)

        retreat = copy.deepcopy(place_down)
        retreat.position.z += 0.15
        self.move_arm_to_pose(retreat)

        self.get_logger().info(f'Place of "{object_id}" complete')
        return True

    def pick_and_place(self, object_id: str, object_pose: Pose,
                       dims: list, place_pose: Pose) -> bool:
        self.pause_pub.publish(Bool(data=True))
        self._manipulating = True
        self.get_logger().info('Paused object detection')
        try:
            if not self.pick(object_id, object_pose, dims):
                self.get_logger().error('Pick failed, retreating')
                self.retreat_to_initial()
                return False
            if not self.place(object_id, place_pose, object_half_height=dims[2] / 2.0):
                self.get_logger().error('Place failed, retreating')
                self.detach_object(object_id)
                self.move_gripper(open=True)
                self.retreat_to_initial()
                return False
            self.retreat_to_initial()
            return True
        finally:
            self._manipulating = False
            self.pause_pub.publish(Bool(data=False))
            self.get_logger().info('Resumed object detection')


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = GraspPlannerGPD()

    # Wait for joint states
    node.get_logger().info('Waiting for /joint_states ...')
    while node.latest_joint_state is None:
        rclpy.spin_once(node, timeout_sec=0.5)

    # ---- Determine object pose and dims -----------------------------------
    # Check if the user manually specified a pose via parameters
    px = node.get_parameter('object_pose_x').value
    py = node.get_parameter('object_pose_y').value
    pz = node.get_parameter('object_pose_z').value
    manual_pose = (px != 0.0 or py != 0.0 or pz != 0.0)

    if manual_pose:
        object_id = 'user_object'
        object_pose = Pose()
        object_pose.position = Point(x=px, y=py, z=pz)
        object_pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        dims = [
            node.get_parameter('object_width').value,
            node.get_parameter('object_depth').value,
            node.get_parameter('object_height').value,
        ]
        node.get_logger().info(
            f'Using manual object pose ({px:.3f}, {py:.3f}, {pz:.3f}), '
            f'dims={dims}'
        )
        # Add a collision box for the manually specified object
        co = CollisionObject()
        co.id = object_id
        co.header.frame_id = 'world'
        co.operation = CollisionObject.ADD
        box = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=dims)
        co.primitives.append(box)
        co.primitive_poses.append(object_pose)
        scene = PlanningScene(is_diff=True)
        scene.world.collision_objects.append(co)
        node.scene_pub.publish(scene)
    else:
        # Move to observation pose so the wrist camera can see the workspace,
        # then wait for the object detector to publish a detection.
        node.look()
        node.get_logger().info('Waiting for detected objects on /collision_object ...')
        while not node.detected_objects:
            rclpy.spin_once(node, timeout_sec=0.5)
        object_id, (object_pose, dims) = next(iter(node.detected_objects.items()))
        node.get_logger().info(f'Targeting detected object "{object_id}"')

    # ---- Placement target (random nearby spot) ----------------------------
    dx = random.uniform(-0.15, -0.05)
    dy = random.uniform(0.05, 0.15) * random.choice([-1, 1])
    table_z = object_pose.position.z - dims[2] / 2.0   # bottom of object = table surface
    place_pose = Pose()
    place_pose.position = Point(
        x=object_pose.position.x + dx,
        y=object_pose.position.y + dy,
        z=table_z,
    )
    place_pose.orientation = GraspPlannerGPD.TOP_DOWN_ORIENTATION
    node.get_logger().info(
        f'Place target: ({place_pose.position.x:.3f}, '
        f'{place_pose.position.y:.3f}, {place_pose.position.z:.3f})'
    )

    # ---- Execute ----------------------------------------------------------
    success = node.pick_and_place(object_id, object_pose, dims, place_pose)
    if success:
        node.get_logger().info('Pick-and-place complete!')
    else:
        node.get_logger().error('Pick-and-place failed.')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
