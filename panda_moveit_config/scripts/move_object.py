#!/usr/bin/env python3
"""
Helper script to move objects in Gazebo and update the MoveIt planning scene.
Usage: ros2 run panda_moveit_config move_object.py <object_name> <x> <y> <z>

Example:
  ros2 run panda_moveit_config move_object.py red_cube 0.5 0.2 0.445
"""

import sys
import subprocess
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import CollisionObject


OBJECTS = {
    'red_box': ('box', [0.15, 0.15, 0.15]),
    'blue_cylinder': ('cylinder', [0.21, 0.075]),  # height, radius
}


def move_in_gazebo(name: str, x: float, y: float, z: float) -> bool:
    """Move object in Gazebo using gz service."""
    try:
        cmd = [
            'gz', 'service', '-s', '/world/tabletop_world/set_pose',
            '--reqtype', 'gz.msgs.Pose',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '2000',
            '--req', f'name: "{name}", position: {{x: {x}, y: {y}, z: {z}}}'
        ]
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=5)
        return result.returncode == 0
    except Exception as e:
        print(f'Failed to move in Gazebo: {e}')
        return False


def update_planning_scene(node: Node, name: str, x: float, y: float, z: float):
    """Update the object in MoveIt planning scene."""
    if name not in OBJECTS:
        print(f'Unknown object: {name}')
        return

    obj_type, dims = OBJECTS[name]

    pub = node.create_publisher(CollisionObject, '/collision_object', 10)

    obj = CollisionObject()
    obj.header.frame_id = 'world'
    obj.header.stamp = node.get_clock().now().to_msg()
    obj.id = name
    obj.operation = CollisionObject.ADD

    primitive = SolidPrimitive()
    if obj_type == 'box':
        primitive.type = SolidPrimitive.BOX
    elif obj_type == 'cylinder':
        primitive.type = SolidPrimitive.CYLINDER
    elif obj_type == 'sphere':
        primitive.type = SolidPrimitive.SPHERE
    primitive.dimensions = list(dims)

    pose = Pose()
    pose.position = Point(x=x, y=y, z=z)
    pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

    obj.primitives.append(primitive)
    obj.primitive_poses.append(pose)

    # Publish a few times to ensure delivery
    for _ in range(3):
        pub.publish(obj)
        rclpy.spin_once(node, timeout_sec=0.1)

    print(f'Updated {name} in planning scene to ({x}, {y}, {z})')


def main():
    if len(sys.argv) != 5:
        print('Usage: move_object.py <object_name> <x> <y> <z>')
        print('Objects:', list(OBJECTS.keys()))
        sys.exit(1)

    name = sys.argv[1]
    x, y, z = float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4])

    if name not in OBJECTS:
        print(f'Unknown object: {name}')
        print('Available objects:', list(OBJECTS.keys()))
        sys.exit(1)

    rclpy.init()
    node = Node('move_object')

    print(f'Moving {name} to ({x}, {y}, {z})...')

    # Move in Gazebo
    if move_in_gazebo(name, x, y, z):
        print(f'Moved {name} in Gazebo')
    else:
        print('Note: Gazebo move may have failed, but updating planning scene anyway')

    # Update planning scene
    update_planning_scene(node, name, x, y, z)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
