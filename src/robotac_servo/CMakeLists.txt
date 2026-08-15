cmake_minimum_required(VERSION 3.0.2)
project(robotac_servo)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  std_srvs
)

catkin_python_setup()
catkin_package()

catkin_install_python(PROGRAMS
  scripts/servo_node.py
  scripts/servo_angle_tuner.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_servo_protocol.py)
endif()
