# CS593_SER_Project

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
