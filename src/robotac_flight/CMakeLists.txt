cmake_minimum_required(VERSION 3.0.2)
project(robotac_flight)

find_package(catkin REQUIRED COMPONENTS
  apriltag_ros
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
  scripts/active_flight_observer.py
  scripts/audit_local_mission.py
  scripts/check_px4_vision_config.py
  scripts/create_route_file.py
  scripts/ev_acceptance_observer.py
  scripts/fastlio_frame_alignment_observer.py
  scripts/fastlio_vision_bridge.py
  scripts/local_odom_to_vision_pose.py
  scripts/local_flight_readiness.py
  scripts/local_flight_preflight.py
  scripts/local_waypoint_flight.py
  scripts/preview_local_route.py
  scripts/publish_waypoints.py
  scripts/tag_payload_mission.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
