cmake_minimum_required(VERSION 3.0.2)
project(robotac_flight)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  nav_msgs
  rospy
  std_msgs
  std_srvs
  tf
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/check_px4_vision_config.py
  scripts/fastlio_vision_bridge.py
  scripts/local_flight_preflight.py
  scripts/local_waypoint_flight.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
