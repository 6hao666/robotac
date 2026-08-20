cmake_minimum_required(VERSION 3.0.2)
project(robotac_mission)

find_package(catkin REQUIRED COMPONENTS
  apriltag_ros
  geometry_msgs
  mavros_msgs
  rospy
  std_msgs
  std_srvs
)

catkin_python_setup()
catkin_package()

catkin_install_python(PROGRAMS
  scripts/mission_node.py
  test/fake_vision_state.py
  test/mission_sim_test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config test
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_nosetests(test/test_state_machine.py)
  catkin_add_nosetests(test/test_guards.py)
  catkin_add_nosetests(test/test_config.py)
  add_rostest(launch/mission_sim.test)
endif()
