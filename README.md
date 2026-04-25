# CS593_SER_Project

Right now this is mostly just Lab2 with the chessboard from this repo (https://github.com/ZeinBarhoum/chess_manipulator) wired in.

Run instructions:

# from wsl terminal
.docker/build.bash
.docker/run.bash
# inside docker
tmux
build
ros2 launch panda_moveit_config ex_gz_chess_control.launch.py

ctrl+b, c 	: open term
ctrl+b, p	: previous

## runs chess game
ros2 run panda_moveit_config chess_sequence.py


