cmake_minimum_required(VERSION 3.0.2)
project(robotac_bringup)

find_package(catkin REQUIRED)

find_path(LIVOX_LIDAR_SDK_INCLUDE_DIR
  NAMES livox_lidar_api.h livox_lidar_def.h
  PATHS /usr/local/include
  REQUIRED
)
find_library(LIVOX_LIDAR_SDK_LIBRARY
  NAMES livox_lidar_sdk_shared
  PATHS /usr/local/lib
  REQUIRED
)

catkin_package()

add_executable(robotac_lidar_discover
  tools/livox_lidar_discover.cpp
)
target_include_directories(robotac_lidar_discover PRIVATE
  ${LIVOX_LIDAR_SDK_INCLUDE_DIR}
)
target_link_libraries(robotac_lidar_discover
  ${LIVOX_LIDAR_SDK_LIBRARY}
  pthread
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS robotac_lidar_discover
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
