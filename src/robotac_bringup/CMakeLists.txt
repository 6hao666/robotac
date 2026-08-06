cmake_minimum_required(VERSION 3.0.2)
project(robotac_bringup)

find_package(catkin REQUIRED)

catkin_package()

install(PROGRAMS
  scripts/check_geographiclib.sh
  scripts/check_hardware_config.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
