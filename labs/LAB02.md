# Lab 02: Perception and Grasping

## Overview

In this lab, you will extend the Panda robot simulation with perception and grasping capabilities. You will:

1. Add an RGBD camera sensor to the Gazebo simulation
2. Configure the ROS-Gazebo bridge to publish point cloud data
3. Implement object detection from point cloud data
4. Implement grasp planning using MoveIt's pick/place functionality
5. Integrate GPD for learned 6-DoF grasp candidate ranking

## Prerequisites

Complete Lab 01 and ensure your environment is working. Use the same Docker container.

> **Tip:** The container includes the following aliases:
>
> | Alias | Description |
> |-------|-------------|
> | `launch_ctrl` | Launch Gazebo + MoveIt 2 + RViz |
> | `launch_detector` | Run the tabletop object detector |
> | `launch_planner` | Run the grasp planner |
> | `build` | Rebuild the ROS workspace |

## Submission Requirements

Submit a PDF document containing:

1. Screenshots of each deliverable as specified
2. Written answers to all questions

And a ZIP file containing:

3. Your completed `object_detector.py` code
4. Your completed `grasp_planner.py` code (including GPD integration for Part 4)

**Point breakdown:** Part 1 (15) + Part 2 (30) + Part 3 (30) + Part 4 (25) = **100 points**

---

## Part 1: Adding the Camera Sensor (15 points)

### Step 1.1: Add an Overhead Camera

Open `panda_description/worlds/tabletop.sdf` and add the following camera model **before** the closing `</world>` tag:

```xml
<!-- Overhead RGBD Camera -->
<model name="overhead_camera">
  <static>true</static>
  <pose>0.5 0 1.5 0 1.5708 0</pose>
  <link name="camera_link">
    <visual name="visual">
      <geometry>
        <box>
          <size>0.05 0.1 0.05</size>
        </box>
      </geometry>
      <material>
        <ambient>0.2 0.2 0.2 1</ambient>
        <diffuse>0.2 0.2 0.2 1</diffuse>
      </material>
    </visual>
    <sensor name="rgbd_camera" type="rgbd_camera">
      <update_rate>10</update_rate>
      <camera>
        <horizontal_fov>1.047</horizontal_fov>
        <image>
          <width>640</width>
          <height>480</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.1</near>
          <far>10.0</far>
        </clip>
        <depth_camera>
          <clip>
            <near>0.1</near>
            <far>10.0</far>
          </clip>
        </depth_camera>
      </camera>
      <always_on>true</always_on>
      <visualize>true</visualize>
      <topic>overhead_camera</topic>
      <gz_frame_id>overhead_camera_link</gz_frame_id>
    </sensor>
  </link>
</model>
```

Launch the simulation and list Gazebo topics:

```shell
ros2 launch panda_moveit_config ex_gz_control.launch.py
```

In another terminal:

```shell
gz topic -l | grep overhead
```

**Deliverable 1.1:** What Gazebo topics does the camera publish? List all topics containing "overhead".

### Step 1.2: The Camera Model

The camera model consists of a 6-DoF pose `P = (x, y, z, e_x, e_y, e_z)` where `(x, y, z)` is the camera location and `(e_x, e_y, e_z)` is the camera rotation, and a 4x4 camera intrinsic matrix `K`. To convert depth images into point clouds, Gazebo inverts the projection from 3D camera coordinates to the image plane.

**Homogeneous Transformation**

`P` is often represented as a 4x4 homogeneous matrix:

```
T = [[R     t]
     [0 0 0 1]]
```

where `t` is the 3D camera position and `R` must be a valid (orthogonal) 3x3 rotation matrix.
The Rodriguez formula for XYZ-ordered Euler angles is as follows:

```
R =
[[ cos(e_z)cos(e_y),
   cos(e_z)sin(e_y)sin(e_x) - sin(e_z)cos(e_x),
   cos(e_z)sin(e_y)cos(e_x) + sin(e_z)sin(e_x) ],

 [ sin(e_z)cos(e_y),
   sin(e_z)sin(e_y)sin(e_x) + cos(e_z)cos(e_x),
   sin(e_z)sin(e_y)cos(e_x) - cos(e_z)sin(e_x) ],

 [ -sin(e_y),
   cos(e_y)sin(e_x),
   cos(e_y)cos(e_x) ]]
```

**Deliverable 1.2.1:** What is the 4x4 homogeneous transform given the pose of the camera model from 1.1?

**Intrinsic Matrix**

The intrinsic matrix, sometimes called the _projection_, is a 3x3 matrix specific to a camera. It includes information like focal length `f` and optical center `(c_x, c_y)`. It is expressed as:

```
K = [[f, 0, c_x],
     [0, f, c_y],
     [0, 0, 1  ]]
```

The focal length can be computed from the camera width and horizontal FoV using trigonometry:

```
f = w/(2*tan(theta/2))
```

We assume `f_x = f_y`, but in general if both vertical and horizontal FoV's are known, their focal lengths may differ. Finally, assume the optical center is the center pixel of the image.

**Deliverable 1.2.2:** What is the camera intrinsic matrix of the camera model from 1.1?

### Step 1.3: Bridging Points and Publishing Transforms

Open `panda_moveit_config/config/gz_bridge.yaml` and add an entry to bridge the point cloud topic.

The bridge entry format is:

```yaml
- ros_topic_name: <ros_tf2_name>
  gz_topic_name: <gazebo_topic_name>
  ros_type_name: sensor_msgs/msg/PointCloud2
  gz_type_name: gz.msgs.PointCloudPacked
  direction: GZ_TO_ROS
```

Where `<gazebo_topic_name>` is the name from part 1.1. Pick a reasonable `<ros_topic_name>`, e.g. `/point_cloud`. Relaunch the simulation and verify the topic appears in ROS 2:

```shell
ros2 topic list | grep -i <ros_topic_name>
```

Try adding the point cloud to the display. You will see an error:

"Could not transform from [overhead_camera_link] to [panda_link0]"

We published the point cloud, but it could not be transformed to the correct world coordinate system. The problem is that though `<gazebo_topic_name>` is visible internally to Gazebo, it does not publish automatically to the global transform tree [TF2](https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Tf2.html).

**Deliverable 1.3.1**

Launch a new node camera_tf that publishes the static transform from the world frame to the camera frame.
You can launch nodes programmatically using the following syntax.

```python
camera_tf = Node(
  package    ='tf2_ros',
  executable = '<fill-in>',          # discover with:  `ros2 pkg executables tf2_ros`
  arguments  = [
    '0.5', '0.0', '1.5',             # x y z   (from the SDF pose)
    '0',   '1.5708', '0',            # roll pitch yaw
    'world',                         # parent frame
    'overhead_camera_link'           # child frame
  ]
)
```

Place `camera_tf` inside `panda_moveit_config/launch/ex_gz_control.launch.py`, under the `generate_launch_description` function, and add it to the returned `LaunchDescription([camera_tf] + other_nodes...)`.

**Deliverable 1.3.2** 

Launch the system and take a screenshot of RViz showing the point cloud visualization with the table and objects visible. Verify the frame was added to the TF2 tree: run `ros2 run tf2_tools view_frames` and take a screenshot that clearly shows the edge `world -> overhead_camera_link` and paste it in your PDF.

## Part 2: Implementing Object Detection (30 points)

In this part you will write the core logic of the **`ObjectDetector`** node so that a single object on the tabletop is detected and added to the MoveIt planning scene as a collision box. Open

```shell
panda_moveit_config/scripts/object_detector.py
```  

This file contains:

* A subscriber on `/point_cloud`
* Utility functions to convert point clouds, build bounding boxes, and **publish them**.

**Deliverable 2**

Your job is to complete the following functions, marked with numbered TODOs in the file:

| TODO | Function | Description |
|------|----------|-------------|
| 2.1 | `build_transform` | Build a 4×4 homogeneous transform from a TF translation and quaternion |
| 2.2 | `transform_points` | Apply the transform to convert camera-frame points to world frame |
| 2.3 | `filter_points` | Remove table and floor points, keeping only object points |
| 2.4 | `compute_bounding_box` | Compute an axis-aligned bounding box from the filtered object points |

Complete them in order, then run your script.

## Part 3: Implementing Grasp Planning (30 points)

### Step 3.1: Understand the Grasp Planner

Open `panda_moveit_config/scripts/grasp_planner.py`. This node uses MoveIt's `MoveGroup` action to plan and execute arm motions, and a `FollowJointTrajectory` action for gripper control. The pick-and-place sequence is built from simple Cartesian pose targets with z-offsets for approach and retreat.

The skeleton has 3 TODOs:

| TODO | Description |
|------|-------------|
| 1 | Define the top-down gripper orientation as a quaternion |
| 2 | Construct the pre-grasp pose (above the object) |
| 3 | Construct the grasp pose (descend to the object) |

The `place()` method is provided as a reference implementation — read it carefully, as its structure is a hint for how to implement `pick()`.

### Step 3.2: Top-Down Orientation (TODO 1)

The `panda_hand` frame has its z-axis pointing along the fingers. To point the gripper straight down, you need a 180-degree rotation about the x-axis. Express this as a unit quaternion and assign it to `TOP_DOWN_ORIENTATION`.

**Hint:** Recall that a rotation of angle `theta` about axis `(ux, uy, uz)` has the quaternion representation `(x, y, z, w) = (ux*sin(theta/2), uy*sin(theta/2), uz*sin(theta/2), cos(theta/2))`.

### Step 3.3: Implement the Pick Sequence (TODOs 2, 3)

Inside the `pick()` method, you need to construct two `Pose` targets:

1. **Pre-grasp pose (TODO 2):** Position the gripper some distance above the object using the top-down orientation. Use `move_arm_to_pose()` to move there.

2. **Grasp pose (TODO 3):** Descend closer to the object so the fingers can close around it. Use the `acm_object_id` parameter of `move_arm_to_pose()` to allow gripper-object collisions during this motion.

The rest of the pick (close gripper, attach object, retreat) is provided.

**Hint:** Think about what z-offsets make sense. Too high and the gripper won't reach the object; too low and you'll collide with the table. Study `place()` to see what offsets work for a similar sequence.

### Step 3.5: Test Pick and Place

The `main()` function is provided: it waits for the object detector to publish a collision object, then calls `pick_and_place()` with the detected pose and a fixed placement location. You do not need to modify `main()`.

Run the grasp planner:

```shell
ros2 run panda_moveit_config grasp_planner.py
```

**Deliverable 3.1:** Take a sequence of screenshots showing:
1. Initial state with detected objects
2. Robot approaching an object
3. Robot grasping the object
4. Robot placing the object at a new location

**Deliverable 3.2:** Include your completed `grasp_planner.py` in your submission.

**Deliverable 3.3:** After you get the pick-and-place planner working at least once, execute the grasp planner multiple times in sequence and record the maximum number of attempts you see succeed. What modeling assumption broke that explains the failure?

**Deliverable 3.4:** Answer these questions:
1. Why is the gripper orientation important for successful grasping?
2. What happens if the approach/retreat z-offset is too small? Too large?
3. How would you modify the grasp strategy for a cylindrical object vs a box?


---

## Part 4: 6-DoF Grasp Planning with GPD (25 points)

Parts 1–3 use a hand-crafted top-down grasp that works only for upright objects.
**Grasp Pose Detection (GPD)** replaces that heuristic with a learned approach: it
scores grasp candidates from a point cloud with a CNN and returns ranked 6-DoF poses.

In this part you will extend your `grasp_planner.py` to call GPD and attempt its
candidates before falling back to the top-down grasp you already implemented.

### How GPD works

GPD operates in three stages:

1. **Candidate sampling** — random antipodal surface point pairs define candidate gripper poses.
2. **Image encoding** — each candidate is projected into a 15-channel descriptor image encoding local geometry and normals.
3. **CNN scoring** — a LeNet variant scores candidates; the top-ranked poses are returned.

GPD's dependencies are pre-installed in the container. Build the shared library with:

```shell
cmake --build /root/ws/src/panda_gz_moveit2/deps/gpd/build --target gpd_python -j$(nproc)
```

You should see several targets quickly build. Two helper utilities are provided in `grasp_planner_gpd.py` that you can import:

```python
from grasp_planner_gpd import GPDInterface, sample_cuboid_surface, write_pcd_ascii
```

- **`GPDInterface(lib_path)`** — loads the library and exposes `detect(config, pcd, camera_pos)`, which returns a list of candidate dicts sorted by score (descending):
  ```python
  # Each candidate:
  {'pos': np.ndarray[3],   # grasp position in world frame
   'R':   np.ndarray[3,3], # orientation matrix (see frame convention below)
   'score': float,
   'antipodal': bool}
  ```
- **`sample_cuboid_surface(center, dims, n_points)`** — samples a synthetic point cloud from the bounding box of the detected object and returns an `(N, 3)` numpy array.
- **`write_pcd_ascii(points, path)`** — writes the array to a PCD file for GPD to consume.

Modify the `pick()` method so that before attempting the top-down grasp it:

1. Loads `GPDInterface` from the path in a ROS parameter `gpd_lib` (default `''`).
2. Samples a point cloud from the object bounding box using `sample_cuboid_surface`.
3. Calls `GPDInterface.detect()` to get ranked candidates.
4. Iterates through candidates, converting each with `gpd_grasp_to_pose` and attempting a pre-grasp → grasp descent sequence.
5. Falls back to the top-down grasp if no candidate succeeds or if `gpd_lib` is empty.

The GPD config file is pre-installed; pass its path via a `gpd_config` ROS parameter.

Run with:

```shell
ros2 run panda_moveit_config grasp_planner.py --ros-args \
  -p gpd_config:=/root/ws/src/panda_gz_moveit2/deps/gpd/cfg/eigen_params.cfg \
  -p gpd_lib:=/root/ws/src/panda_gz_moveit2/deps/gpd/build/libgpd_python.so
```

**Deliverable 4:** Report the scores of the top 5 GPD candidates. Describe what you observe when the arm attempts the top-ranked candidate — does it succeed or fail, and why?

---

## Troubleshooting

- The container may take a while to build (several minutes). Instead of rebuilding the container from scratch, run `build`, which calls this alias:

```shell
colcon build --merge-install --symlink-install --cmake-args "-DCMAKE_BUILD_TYPE=Release"
```

you don't have to do this for Python files, you do usually have to do it for configuration changes.

### Point cloud not appearing
- Verify `gz_bridge.yaml` syntax
- Check Gazebo topic name matches exactly
- Ensure sensor is publishing: `gz topic -e -t /overhead_camera/points`

### Points in wrong frame
- Camera frame has origin at camera position
- Transform to world frame if needed, or adjust thresholds

### Pick/Place fails
- Check that object is in planning scene
- Verify grasp pose is reachable
- Try adjusting z-offsets
- Check gripper orientation quaternion

---

## Resources

- [MoveIt Pick and Place Tutorial](https://moveit.picknik.ai/main/doc/examples/pick_place/pick_place_tutorial.html)
- [sensor_msgs_py](https://docs.ros.org/en/rolling/p/sensor_msgs_py/)
