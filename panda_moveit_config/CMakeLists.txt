cmake_minimum_required(VERSION 3.5)
project(panda_moveit_config)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(panda_description REQUIRED)

# Install directories
install(DIRECTORY config launch rviz srdf scripts DESTINATION share/${PROJECT_NAME})

# Install Python scripts
install(PROGRAMS
  scripts/scene_publisher.py
  scripts/move_object.py
  scripts/grasp_planner.py
  scripts/grasp_planner_gpd.py
  DESTINATION lib/${PROJECT_NAME}
)

# Setup the project
ament_package()

