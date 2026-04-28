# CS593_SER_Project

ROS 2 Jazzy project for a Franka Panda arm in Gazebo/MoveIt, with a chessboard scene and robot chess-move scripts.

## Docker Setup

Build the project image from the host machine:

```bash
.docker/build.bash
```

Start the container from the repository root:

```bash
.docker/run.bash
```

Inside the container, rebuild the workspace after changing code:

```bash
build
```

## Main Simulation

Launch Gazebo, MoveIt, RViz, and the chess tabletop world:

```bash
ros2 launch panda_moveit_config ex_gz_chess_control.launch.py
```

Useful tmux shortcuts while the launch is running:

```text
ctrl+b, c    new terminal
ctrl+b, p    previous terminal
```

## Run The Chess Scripts

Run a short built-in chess move sequence:

```bash
ros2 run panda_moveit_config chess_dummy_moves.py
```

Pick a specific dummy scenario:

```bash
ros2 run panda_moveit_config chess_dummy_moves.py --scenario capture
ros2 run panda_moveit_config chess_dummy_moves.py --scenario castle
```

Run a Stockfish-vs-Stockfish chess sequence:

```bash
ros2 run panda_moveit_config chess_sequence.py
```





## less usefull past here.
Do a quick chess logic check without moving the robot:

```bash
ros2 run panda_moveit_config chess_sequence.py --dry-run --max-plies 6 --print-board
```

## Other Useful Commands

Launch MoveIt with fake controllers only:

```bash
ros2 launch panda_moveit_config ex_fake_control.launch.py
```

View the robot description in RViz:

```bash
ros2 launch panda_description view.launch.py
```

View the robot model in Gazebo:

```bash
ros2 launch panda_description view_gz.launch.py
```

Move a known object in Gazebo and update the MoveIt planning scene:

```bash
ros2 run panda_moveit_config move_object.py red_box 0.5 0.2 0.445
ros2 run panda_moveit_config move_object.py blue_cylinder 0.5 -0.2 0.445
```

Regenerate robot description files after editing xacro files:

```bash
panda_description/scripts/xacro2urdf.bash
panda_description/scripts/xacro2sdf.bash
panda_moveit_config/scripts/xacro2srdf.bash
```

## Testing And Checks

Run the available colcon test target:

```bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

Run a fast sanity check for the chess move generator:

```bash
ros2 run panda_moveit_config chess_sequence.py --dry-run --max-plies 4
```

Install and run the formatting/pre-commit checks:

```bash
.git_hooks/setup.bash
pre-commit run --all-files
```
