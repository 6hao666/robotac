cmake_minimum_required(VERSION 3.0.2)
project(robotac_examples)

find_package(catkin REQUIRED COMPONENTS
  apriltag_ros
  geometry_msgs
  mavros_msgs
  rospy
  std_msgs
  std_srvs
  tf2_geometry_msgs
  tf2_ros
)

catkin_python_setup()
catkin_package()

catkin_install_python(PROGRAMS
  scripts/01_fcu_state.py
  scripts/02_local_pose.py
  scripts/03_apriltag_detection.py
  scripts/04_apriltag_local_pose.py
  scripts/05_setpoint_preview.py
  scripts/06_hover.py
  scripts/07_move_relative.py
  scripts/08_waypoints.py
  scripts/09_tag_centering_preview.py
  scripts/10_tag_centering_flight.py
  scripts/11_payload_release.py
  scripts/12_move_cardinal.py
  test/fake_fcu.py
  test/fake_tag.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config test
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_nosetests(test/test_geometry.py)
  catkin_add_nosetests(test/test_route.py)
  catkin_add_nosetests(test/test_tag.py)
  add_rostest(test/examples_sim.test)
endif()
