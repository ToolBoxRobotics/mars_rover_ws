cmake_minimum_required(VERSION 3.8)
project(rover_navigation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch rviz maps DESTINATION share/${PROJECT_NAME})

# gps_goal.py is the executable entry point (ros2 run rover_navigation
# gps_goal.py); gps_coordinate_validation.py is installed alongside it
# (not as a PROGRAM - it's imported, never run directly) so the
# same-directory import in gps_goal.py resolves correctly post-install.
install(PROGRAMS scripts/gps_goal.py DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/gps_coordinate_validation.py DESTINATION lib/${PROJECT_NAME})

ament_package()
