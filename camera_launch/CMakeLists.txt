cmake_minimum_required(VERSION 3.0.2)
project(camera_launch)

find_package(catkin REQUIRED COMPONENTS rospy topic_tools)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/mux_select.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

