# Lab 01: MoveIt 2 Motion Planning with the Panda Robot

## Overview

In this assignment, you will use MoveIt 2 to plan and execute motions for a simulated Franka Emika Panda robot arm. You will learn how to:

1. Set up and run a ROS 2 simulation environment using Docker
2. Use MoveIt 2's RViz interface for interactive motion planning
3. Understand frames and transforms, inspecting telemetry topics, and experimenting with different planner behaviors
4. Execute planned motions on a simulated robot

## Submission Requirements

Submit a PDF document containing:

1. All screenshots as specified in the deliverables.
2. Written answers to all questions.

### For NVIDIA GPU Users (Optional but Recommended)

If you have an NVIDIA GPU, install the NVIDIA Container Toolkit for hardware-accelerated rendering:

```bash
# Add the repository
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
  sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
  sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

# Install
sudo apt-get update && sudo apt-get install -y nvidia-container-toolkit
sudo systemctl restart docker
```

## Part 1: Environment Setup (10 points)

### Step 1.1: Clone and Build

Clone this repository and build the Docker image:

```bash
git clone https://github.com/CoMMALab/CS593-SER-LAB-01 lab01
cd lab01
.docker/build.bash
```

### Step 1.2: Run the Container

Start the Docker container:

```bash
.docker/run.bash
```

You should now be inside the container at `/root/ws`.

> **Tip:** The container includes the following aliases:
> - `launch_ctrl` — `ros2 launch panda_moveit_config ex_gz_control.launch.py`
> - `build` — `colcon build --merge-install --symlink-install --cmake-args "-DCMAKE_BUILD_TYPE=Release"`

### Step 1.3: Build the Workspace

Build the ROS 2 workspace inside the container:

```bash
colcon build --merge-install --symlink-install --cmake-args "-DCMAKE_BUILD_TYPE=Release"
source install/setup.bash
```

**Deliverable 1.3:** Based on the output of `colcon build`, how many packages are built as part of the project?

## Part 2: Running the Motion Planning Demo (20 points)

### Step 2.1: Launch the Planning Interface

First, start a tmux session so you can run multiple commands:

```shell
tmux
```

You can create new tabs with `Ctrl+b` then `c`, and switch between them with `Ctrl+b` then `p` (previous) or `n` (next).

Launch the fake controller demo (no physics simulation, instant motion execution):

```shell
ros2 launch panda_moveit_config ex_fake_control.launch.py
```

This will open RViz with the MoveIt Motion Planning plugin.

### Step 2.2: Explore the Interface

In RViz, locate the "MotionPlanning" panel on the left side. You should see:

- **Planning Group:** Currently set to "arm" (the 7-DOF manipulator)
- **Start State:** The current robot configuration (shown in green)
- **Goal State:** The target configuration (shown in orange)

### Step 2.3: Understanding Frames and Transforms

In RViz, click **Add** in the Displays panel, select **TF**, and click OK to visualize the robot's coordinate frames.

In another tmux tab, generate a TF tree diagram:

```shell
ros2 run tf2_tools view_frames --output frames
```

This creates a `frames.pdf` file. To view it, open a terminal **outside** the container and copy it out:

```shell
docker cp $(docker ps -qf "ancestor=panda_gz_moveit2"):/root/ws/frames.pdf .
```

Then open `frames.pdf` on your host machine.

**Deliverable 2.3:** Answer the following questions:
- What is the child frame of `panda_link8` (i.e. the end-effector frame)?
- What is the parent frame of `panda_link4`? What about `panda_link0`?
- How many links (not including endpoints) are in the kinematic chain between `panda_link0` and `panda_rightfinger`?

## Part 3: Interactive Motion Planning (35 points)

### Step 3.1: Set a Goal Using the Interactive Marker

1. In RViz, you should see an interactive marker (colored rings and arrows) at the robot's end-effector
2. Drag the marker to move the goal pose
3. The orange "ghost" robot shows where the arm will move to
4. With a goal pose set, click the **"Plan"** button in the MotionPlanning panel
5. Observe the planned trajectory animation. The trajectory shows how the robot will move from start to goal

**Deliverable 3.1:**
- Plan a motion that moves the end-effector of the robot.
- Take a screenshot showing the planned trajectory (the animated path).
- Record the planning time displayed in the panel.

### Step 3.2: Execute the Motion

1. Click **"Execute"** to run the planned trajectory
2. The robot (green) will move to match the goal (orange)
3. Alternatively, use **"Plan & Execute"** to do both steps at once

**Deliverable 3.2:** Take a screenshot showing the robot in its new position after execution.

### Step 3.3: Use Predefined Poses

The SRDF defines several named poses:

- **ready** - A neutral starting pose
- **extended** - Arm fully extended
- **transport** - Compact pose for transport

1. In the MotionPlanning panel, find the "Goal State" dropdown
2. Select one of the predefined poses
3. Plan and execute the motion

**Deliverable 3.3:**
- Execute motions to visit a sequence of all three predefined poses and a custom pose (ready → extended → transport → a custom pose → ready)
- For each transition, record:
  - Planning time
  - Whether the plan succeeded on the first attempt

## Part 4: Understanding Motion Planning (35 points)

### Step 4.1: Examine the Planning Pipeline

Now launch the Gazebo simulation (close the fake control demo first, or use a new tmux tab):

```shell
ros2 launch panda_moveit_config ex_gz_control.launch.py
```

The motion planner uses OMPL (Open Motion Planning Library) with the RRTConnect algorithm by default.

**Deliverable 4.1:** After launching the simulation, answer the following questions using `ros2 topic list` and `ros2 topic info`:

1. Run `ros2 topic list` in the other `tmux` session. What topic is published to so that MoveIt can query the planning scene? (Hint: you can cross-check `panda_moveit_config/scripts/scene_publisher.py`)
2. Run `gz topic -e -t /world/tabletop_world/dynamic_pose/info`. What is the ID of the red box? What is its initial position and orientation? (Hint: try piping the output of `gz` to `grep`)
3. What action server does MoveIt use for trajectory execution? (Hint: use `ros2 action list`)

### Step 4.2: Collision Awareness

1. In Gazebo, drag one of the tabletop objects (red box or blue cylinder) to a new position
2. Observe that RViz's planning scene updates automatically
3. Position an object between the robot and a goal pose
4. Attempt to plan a motion through/around the obstacle

**Deliverable 4.2:**
- Take two screenshots showing a planned trajectory that avoids the obstacle, before and after executing.
- What happens if you place the obstacle directly on the goal pose?

### Step 4.3: Comparing Planners

1. In the MotionPlanning panel, find the **Context** tab
2. Use the planner dropdown to select different OMPL planners (e.g., RRTConnect, RRT*, PRM, EST)
3. Find another goal pose that has an obstacle between it and the robot
4. For each planner, run `plan` for the same motion 5 times

**Deliverable 4.3:** Compare at least 3 different planners and create a table with:
- Planner name
- Average planning time (out of 5)
- Path length (qualitative: short/medium/long)
- Success rate (out of 5)

Which planner would you choose for a time-critical application? Which for path quality?

## Troubleshooting

### Docker/GPU issues
- Without NVIDIA toolkit, Gazebo will use software rendering (slower)
- If GUI doesn't appear, ensure X11 forwarding is working: `xhost +local:docker`

## Resources

- [MoveIt 2 Documentation](https://moveit.picknik.ai/)
- [OMPL Planners](https://ompl.kavrakilab.org/planners.html)
