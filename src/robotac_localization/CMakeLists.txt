cmake_minimum_required(VERSION 3.0.2)
project(robotac_localization)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  rospy
  std_msgs
)

catkin_python_setup()
catkin_package()

catkin_install_python(PROGRAMS
  scripts/vision_pose_bridge.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_validation.py)
endif()
