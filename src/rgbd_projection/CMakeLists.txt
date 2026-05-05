cmake_minimum_required(VERSION 3.8)
project(rgbd_projection)

find_package(ament_cmake REQUIRED)

# Install directories so ros2 launch / ament_index can find them
install(DIRECTORY
  launch
  worlds
  rviz
  stl
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/save_rgbd_snapshot.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
