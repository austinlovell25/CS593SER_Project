#!/usr/bin/env python3
"""
Publishes the static table collision object to the MoveIt planning scene.
Dynamic objects (e.g. cylinders, boxes) are added via the perception pipeline.
"""

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose, Point, Quaternion
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import CollisionObject, PlanningScene


class ScenePublisher(Node):
    """Node that publishes the table to the MoveIt planning scene."""

    def __init__(self):
        super().__init__('scene_publisher')

        self.declare_parameter('frame_id', 'world')
        self.frame_id = self.get_parameter('frame_id').value

        self.scene_pub = self.create_publisher(
            PlanningScene,
            '/planning_scene',
            10
        )

        # Add static objects after delay
        self.static_timer = self.create_timer(3.0, self.add_static_objects)
        self.static_added = False

        self.get_logger().info('Scene publisher initialized')

    def add_static_objects(self):
        """Add static collision objects (table with legs)."""
        if self.static_added:
            return

        scene = PlanningScene()
        scene.is_diff = True

        table = CollisionObject()
        table.header.frame_id = self.frame_id
        table.header.stamp = self.get_clock().now().to_msg()
        table.id = 'table'
        table.operation = CollisionObject.ADD

        # Table top
        table_top = SolidPrimitive()
        table_top.type = SolidPrimitive.BOX
        table_top.dimensions = [0.8, 1.0, 0.04]
        table_top_pose = Pose()
        table_top_pose.position = Point(x=0.6, y=0.0, z=0.25)
        table_top_pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        table.primitives.append(table_top)
        table.primitive_poses.append(table_top_pose)

        # Table legs
        for x, y, z in [(0.95, 0.45, 0.115), (0.95, -0.45, 0.115),
                        (0.25, 0.45, 0.115), (0.25, -0.45, 0.115)]:
            leg = SolidPrimitive()
            leg.type = SolidPrimitive.BOX
            leg.dimensions = [0.04, 0.04, 0.23]
            leg_pose = Pose()
            leg_pose.position = Point(x=x, y=y, z=z)
            leg_pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            table.primitives.append(leg)
            table.primitive_poses.append(leg_pose)

        scene.world.collision_objects.append(table)
        self.scene_pub.publish(scene)

        self.static_added = True
        self.get_logger().info('Added table to planning scene')


def main(args=None):
    rclpy.init(args=args)
    node = ScenePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
