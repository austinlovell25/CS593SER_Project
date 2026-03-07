#!/usr/bin/env python3
"""
panda_moveit_config/scripts/object_detector.py
"""

import rclpy
from rclpy.node import Node
import numpy as np

from std_msgs.msg import Header, Bool
from sensor_msgs.msg import PointCloud2
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import Pose, Point, Quaternion
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import CollisionObject, PlanningScene

# Point-cloud helper
from sensor_msgs_py import point_cloud2 as pc2
from typing import Tuple

# TF utilities
import tf2_ros
import tf_transformations


class ObjectDetector(Node):
    """ROS 2 node that detects one tabletop object and publishes it as a box."""

    # ----------------------------- initialisation -----------------------------

    def __init__(self):
        super().__init__('object_detector')

        # TF listener (world  ←  camera)
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscribe to bridged point-cloud
        self.pc_sub = self.create_subscription(
            PointCloud2, '/point_cloud',
            self.point_cloud_callback, 10
        )

        self.filtered_pc_pub = self.create_publisher(PointCloud2, '/filtered_points', 10)

        # Pause detection during manipulation
        self.paused = False
        pause_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pause_sub = self.create_subscription(
            Bool, '/pause_detection',
            lambda msg: setattr(self, 'paused', msg.data),
            qos_profile=pause_qos
        )

        self.get_logger().info('ObjectDetector ready')

    # ---------------------------- deliverable --------------------------------

    @staticmethod
    def transform_points(points: np.ndarray, T: np.ndarray):
        raise NotImplementedError('Point transformation not implemented yet')

    @staticmethod
    def filter_points(points, ):
        raise NotImplementedError('Point filtering not implemented yet')

    @staticmethod
    def compute_bounding_box(filtered_points: np.ndarray) -> Tuple[Pose, np.ndarray]:
        """Axis-aligned box → (Pose, [dx, dy, dz])."""
        raise NotImplementedError('Bounding-box computation not implemented yet')

    @staticmethod
    def transform_points(points: np.ndarray, T: np.ndarray):
        """
        Transform a point-cloud from camera to world coordinates.

        Parameters
        ----------
        points : np.ndarray
            Shape (N, 3) array in the *camera* frame.
        T : np.ndarray
            4 × 4 homogeneous transform that maps camera → world
            (usually built from TF lookup:  R|t  obtained via
            `tf_transformations.quaternion_matrix` and the translation vector).

        Returns
        -------
        np.ndarray
            Shape (N, 3) array of the same points in the *world* frame.
        """
        raise NotImplementedError('Point transformation not implemented yet')

    @staticmethod
    def filter_points(points: np.ndarray):
        """
        Remove table and floor points, keep only object points.

        Strategy
        --------
        • The table plane is assumed to be the *minimum* z-value in the world
          frame.  
        • Retain points whose z > (table_z + 0.02 m).

        Parameters
        ----------
        points : np.ndarray
            Shape (N, 3) array in the world frame.

        Returns
        -------
        np.ndarray
            Subset of `points` representing the object.
        """
        raise NotImplementedError('Point filtering not implemented yet')

    @staticmethod
    def compute_bounding_box(filtered_points: np.ndarray) -> Tuple[Pose, np.ndarray]:
        """
        Compute an axis-aligned bounding box for the object.

        Parameters
        ----------
        filtered_points : np.ndarray
            Object points after filtering (world frame).

        Returns
        -------
        (Pose, np.ndarray)
            • Pose – centre of the box, orientation = identity  
            • ndarray – `[dx, dy, dz]` edge lengths (metres)
        """
        raise NotImplementedError('Bounding-box computation not implemented yet')

    def point_cloud_callback(self, msg: PointCloud2):
        """
        Full pipeline:  PointCloud2 → NumPy → transform → filter → box → publish.

        Steps
        -----
        1. Convert the incoming cloud to `pts_cam`.
        2. Look up the camera pose (translation *p*, quaternion *q*) in TF,
           build `T = [R|t]`, and transform points to world coordinates.
        3. Identify the table plane as the minimum world-frame z; keep points
           ≥ 2 cm above it.
        4. Compute the axis-aligned bounding box of the remaining points and
           publish it via `publish_objects()`.
        """
        pts_cam = self.pointcloud2_to_xyz(msg)
        if pts_cam is None or len(pts_cam) == 0:
            return

        # 1 · summary statistics in camera frame
        self.get_logger().info(f"Received point cloud in frame '{msg.header.frame_id}'")
        self.get_logger().info(f'Received point cloud with {pts_cam.shape[0]} points')

        # 2 · transform to world frame
        (p, q) = self.get_world_transform(msg)          # translation, quaternion
        # TODO: build T from p, q and transform points to world frame
        #       note you may use `tf_transformations.quaternion_matrix` to get the rotation matrix from q
        pts_world = self.transform_points(pts_cam, T)

        # 3 · height-based filtering
        obj_pts = self.filter_points(pts_world)

        # publish filtered cloud for visualization / debugging
        pc2_msg = self.make_pointcloud2_from_numpy(obj_pts, frame_id='world', max_points=15000)
        if pc2_msg is not None:
            self.filtered_pc_pub.publish(pc2_msg)
            self.get_logger().debug(f'Published filtered point cloud with {obj_pts.shape[0]} points (decimated to {len(pc2_msg.data)//pc2_msg.point_step if pc2_msg.point_step else "?"})')

        # 4 · bounding-box computation and publication
        detected_object = self.compute_bounding_box(obj_pts)
        self.publish_objects([detected_object])

    # ---------------------------- !ANSWER! --------------------------------

    @staticmethod
    def transform_points(points: np.ndarray, T: np.ndarray) -> np.ndarray:
        """
        Apply a 4×4 homogeneous transform to an (N, 3) point cloud.

        Parameters
        ----------
        points : np.ndarray
            Camera–frame points, shape (N, 3).
        T : np.ndarray
            4 × 4 matrix that maps camera → world.

        Returns
        -------
        np.ndarray
            World–frame points, shape (N, 3).
        """
        ones = np.ones((points.shape[0], 1), dtype=points.dtype)
        pts_h = np.hstack((points, ones))        # (N, 4)
        pts_world = (T @ pts_h.T).T              # homogeneous → world
        return pts_world[:, :3]

    @staticmethod
    def filter_points(points: np.ndarray) -> np.ndarray:
        """
        Keep only points that are ≥ 2 cm above the table plane.

        The table plane is assumed to be the minimum z-value in the cloud.

        Parameters
        ----------
        points : np.ndarray
            World–frame points, shape (N, 3).

        Returns
        -------
        np.ndarray
            Filtered point cloud containing the object only.
        """
        table_z = np.min(points[:, 2])
        mask = points[:, 2] > (table_z + 0.02)   # 2 cm clearance
        return points[mask]

    @staticmethod
    def compute_bounding_box(filtered_points: np.ndarray) -> Tuple[Pose, np.ndarray]:
        """
        Compute an axis-aligned bounding box for the object.

        Returns
        -------
        Pose
            Centre of the box (world frame), orientation = identity.
        np.ndarray
            Edge lengths [dx, dy, dz] in metres.
        """
        mins = filtered_points.min(axis=0)
        maxs = filtered_points.max(axis=0)

        centre = (mins + maxs) / 2.0
        dims = (maxs - mins).astype(np.float32)

        pose = Pose(
            position=Point(x=float(centre[0]),
                           y=float(centre[1]),
                           z=float(centre[2])),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        )
        return pose, dims

    # ───────────────────────── main callback ───────────────────────────────────
    def point_cloud_callback(self, msg: PointCloud2):
        """
        Pipeline: convert → transform → filter → bounding box → publish.
        """
        if self.paused:
            self.get_logger().debug('Skipping point cloud (paused)')
            return

        # 1  Camera-frame NumPy cloud
        pts_cam = self.pointcloud2_to_xyz(msg)
        if pts_cam is None or len(pts_cam) == 0:
            return

        self.get_logger().info(f"Received point cloud in frame '{msg.header.frame_id}' with {pts_cam.shape[0]} points")

        # 2  Look up camera pose in TF and build T
        result = self.get_world_transform(msg)
        if result is None:
            return
        p, q = result                                              # translation, quaternion
        T = tf_transformations.quaternion_matrix([q.x, q.y, q.z, q.w])
        T[0:3, 3] = [p.x, p.y, p.z]

        pts_world = self.transform_points(pts_cam, T)

        # 3  Height-based filtering
        obj_pts = self.filter_points(pts_world)
        if obj_pts.size == 0:
            self.get_logger().warn('No object points after filtering')
            return

        # Publish filtered cloud for visualization
        pc2_msg = self.make_pointcloud2_from_numpy(obj_pts, frame_id='world')
        if pc2_msg is not None:
            self.filtered_pc_pub.publish(pc2_msg)

        # 4  Bounding box and publish
        detected_object = self.compute_bounding_box(obj_pts)
        self.publish_objects([detected_object])


    # --------------------------- helper functions -----------------------------
    
    def make_pointcloud2_from_numpy(self, points: np.ndarray, frame_id: str = 'world',
                                    max_points: int = 20000) -> PointCloud2:
        """
        Convert an (N,3) float32 NumPy array into a sensor_msgs/PointCloud2.

        - Decimates to at most max_points (random sampling) to avoid huge messages.
        - Uses sensor_msgs_py.point_cloud2.create_cloud_xyz32 helper.
        """
        if points is None or points.size == 0:
            return None

        # ensure proper shape
        pts = np.asarray(points, dtype=np.float32).reshape(-1, 3)

        # decimate if too many points
        n = pts.shape[0]
        if n > max_points:
            idx = np.random.choice(n, max_points, replace=False)
            pts = pts[idx]

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = frame_id

        # create_cloud_xyz32 expects an iterable of (x,y,z) tuples or list
        pc2_msg = pc2.create_cloud_xyz32(header, pts.tolist())
        return pc2_msg

    def get_world_transform(self, msg):
        try:
            tf_msg = self.tf_buffer.lookup_transform(
                'world', msg.header.frame_id, rclpy.time.Time())
            return tf_msg.transform.translation, tf_msg.transform.rotation
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            self.get_logger().warn('[get_world_frame failed]: skipping frame')
            return

    @staticmethod
    def pointcloud2_to_xyz(msg: PointCloud2) -> np.ndarray:
        """
        Convert a PointCloud2 message into an (N, 3) float32 NumPy array.

        This function copes with multiple return formats from sensor_msgs_py.read_points:
        - generator of (x,y,z) tuples
        - list of tuples
        - numpy structured array with named fields ('x','y','z')
        """
        try:
            data = pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        except Exception as e:
            Node.get_logger(rclpy.logging.get_logger('object_detector')) \
                .error(f'read_points call failed: {e}')
            return None

        # If data is a generator or list of tuples -> try straightforward conversion
        try:
            pts_list = list(data)
        except TypeError:
            # If read_points returned a numpy array-like supporting __array__
            try:
                arr = np.asarray(data)
            except Exception as e:
                Node.get_logger(rclpy.logging.get_logger('object_detector')) \
                    .error(f'Could not convert point data to array: {e}')
                return None
        else:
            # pts_list is a list (maybe empty)
            if len(pts_list) == 0:
                return None
            # If the list elements are tuples/lists of scalars, this will succeed
            try:
                pts = np.array(pts_list, dtype=np.float32)
                # ensure shape is (N,3)
                if pts.ndim == 1 and pts.size == 3:
                    pts = pts.reshape(1, 3)
                if pts.shape[1] != 3:
                    # fallback to structured-array-like handling below
                    raise ValueError('Unexpected shape from list->array')
                return pts
            except Exception:
                # fall through to structured-array handling
                arr = np.asarray(pts_list)

        # At this point `arr` is a numpy array; handle structured dtype
        if arr.dtype.names is not None:
            # structured array with fields, pick x,y,z
            try:
                x = np.asarray(arr['x'], dtype=np.float32)
                y = np.asarray(arr['y'], dtype=np.float32)
                z = np.asarray(arr['z'], dtype=np.float32)
                pts = np.vstack((x, y, z)).T
                return pts
            except Exception as e:
                Node.get_logger(rclpy.logging.get_logger('object_detector')) \
                    .error(f'Failed to extract named fields from structured array: {e}')
                return None
        else:
            # arr is plain numeric array but maybe flattened; try reshape
            try:
                pts = arr.astype(np.float32).reshape(-1, 3)
                return pts
            except Exception as e:
                Node.get_logger(rclpy.logging.get_logger('object_detector')) \
                    .error(f'Could not reshape array to (N,3): {e}')
                return None

    # ---------------------------- scene updates ------------------------------

    def publish_objects(self, objects):
        """Add the supplied axis-aligned boxes to MoveIt’s planning scene and
        additionally publish a wireframe MarkerArray for RViz and individual
        CollisionObject messages for downstream consumers.

        Parameters
        ----------
        objects : list of dict
            Each dict must contain:
            - 'pose'       : geometry_msgs/Pose (box center, world frame)
            - 'dimensions' : [dx, dy, dz] edge lengths in metres

        Behavior
        --------
        * Publishes a PlanningScene diff on the existing `self.scene_pub`.
        * Publishes a MarkerArray on `/detected_markers` (LINE_LIST wireframe).
        The publisher is created lazily and stored as `self.marker_pub`.
        * Publishes each CollisionObject on `/collision_object` (one message per
        object) via a lazily-created `self.collision_pub`.
        """
        # 1) Lazy-create marker publisher and collision publisher if needed
        if not hasattr(self, 'marker_pub'):
            from visualization_msgs.msg import MarkerArray, Marker
            self.marker_pub = self.create_publisher(MarkerArray, '/detected_markers', 10)
            # store Marker class for local use
            self._Marker = Marker
            self._MarkerArray = MarkerArray
        if not hasattr(self, 'collision_pub'):
            self.collision_pub = self.create_publisher(CollisionObject, '/collision_object', 10)

        # 3) Build and publish MarkerArray (wireframe boxes)
        marker_array = self._MarkerArray()
        for idx, (pose, dims) in enumerate(objects):
            cx, cy, cz = pose.position.x, pose.position.y, pose.position.z
            dx, dy, dz = dims
            hx, hy, hz = dx / 2.0, dy / 2.0, dz / 2.0

            # 8 corners in world coords
            corners = [
                (cx - hx, cy - hy, cz - hz),
                (cx + hx, cy - hy, cz - hz),
                (cx + hx, cy + hy, cz - hz),
                (cx - hx, cy + hy, cz - hz),
                (cx - hx, cy - hy, cz + hz),
                (cx + hx, cy - hy, cz + hz),
                (cx + hx, cy + hy, cz + hz),
                (cx - hx, cy + hy, cz + hz),
            ]

            # edges as pairs of indices (each pair -> two points in LINE_LIST)
            edges = [
                (0, 1), (1, 2), (2, 3), (3, 0),  # bottom
                (4, 5), (5, 6), (6, 7), (7, 4),  # top
                (0, 4), (1, 5), (2, 6), (3, 7)   # verticals
            ]

            # create marker
            m = self._Marker()
            m.header.frame_id = 'world'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'detected_wireframes'
            m.id = idx
            m.type = m.LINE_LIST
            m.action = m.ADD
            m.scale.x = 0.005  # line width (meters)
            # color (green)
            m.color.r = 0.0
            m.color.g = 1.0
            m.color.b = 0.0
            m.color.a = 1.0
            # points are in absolute world coords, so pose must be identity
            m.pose = Pose(position=Point(x=0.0, y=0.0, z=0.0),
                          orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))

            # append points for each edge (two Point entries per edge)
            for (i, j) in edges:
                p1 = Point(x=corners[i][0], y=corners[i][1], z=corners[i][2])
                p2 = Point(x=corners[j][0], y=corners[j][1], z=corners[j][2])
                m.points.append(p1)
                m.points.append(p2)

            marker_array.markers.append(m)

        # publish markers
        self.marker_pub.publish(marker_array)
        self.get_logger().info(f'Published {len(marker_array.markers)} wireframe marker(s)')

        # 4) Publish individual CollisionObject messages for downstream nodes
        # (some systems prefer /collision_object single messages rather than full PlanningScene diffs)
        for idx, (pose, dims) in enumerate(objects):
            co_msg = CollisionObject()
            co_msg.id = f'detected_object_{idx}'
            co_msg.header.frame_id = 'world'
            co_msg.header.stamp = self.get_clock().now().to_msg()
            co_msg.operation = CollisionObject.ADD

            box = SolidPrimitive(type=SolidPrimitive.BOX,
                                dimensions=dims)
            co_msg.primitives.append(box)
            co_msg.primitive_poses.append(pose)

            self.collision_pub.publish(co_msg)

        self.get_logger().info(f'Published {len(objects)} CollisionObject message(s) on /collision_object')



    # ------------------------------ accessors --------------------------------


# -----------------------------------------------------------------------------
# entry-point
# -----------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
