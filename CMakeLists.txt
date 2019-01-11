cmake_minimum_required(VERSION 2.8.3)
project(uwb_kalman)

add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(yaml-cpp REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  
  nav_msgs 
  sensor_msgs 
  message_filters
)

IF(NOT ROS_BUILD_TYPE)
  SET(ROS_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${ROS_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES uwb_test
 CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
)

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(rcm_net
  src/rcm.cpp
  src/rcmIf.cpp
  src/rn.cpp
  src/utils.cpp
  src/KalmanFilter.cpp
  
  src/uwblocalization/uwblocalization.cpp
  src/kf_localization/KalmanLocalization.cpp
)

add_executable(uwb_data
  src/uwblocalization/uwb_data.cpp
  src/uwblocalization/uwblocalization.cpp
  
  src/rcm.cpp
  src/rcmIf.cpp
  src/rn.cpp
  src/utils.cpp
  src/KalmanFilter.cpp
)

target_link_libraries(uwb_data
  rcm_net
  ${catkin_LIBRARIES}
)

add_executable(uwb_driver_a
  src/uwblocalization/uwb_driver_a.cpp
  src/rcm.cpp
  src/rcmIf.cpp
  src/rn.cpp
)
target_link_libraries(uwb_driver_a
  rcm_net
  ${catkin_LIBRARIES}
)

add_executable(uwb_driver_b
  src/uwblocalization/uwb_driver_b.cpp
  src/rcm.cpp
  src/rcmIf.cpp
  src/rn.cpp
)
target_link_libraries(uwb_driver_b
  rcm_net
  ${catkin_LIBRARIES}
)

add_executable(left_main
  src/kf_localization/left_main.cpp
  src/kf_localization/KalmanLocalization.cpp
  src/KalmanFilter.cpp
)
target_link_libraries(left_main
  rcm_net
  yaml-cpp
  ${catkin_LIBRARIES}
)

add_executable(right_main
  src/kf_localization/left_main.cpp
  src/kf_localization/KalmanLocalization.cpp
  src/KalmanFilter.cpp
)
target_link_libraries(right_main
  rcm_net
  yaml-cpp
  ${catkin_LIBRARIES}
)

add_executable(main
  src/kf_localization/main.cpp
)
target_link_libraries(main
  ${catkin_LIBRARIES}
)
