cmake_minimum_required(VERSION 2.8.3)
project(odom_estimator)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  geometry_msgs
  message_generation
  std_msgs
  nav_msgs
)

#add_message_files(
#  FILES
#)

#generate_messages(
#  DEPENDENCIES
#)

catkin_package(
#  CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  "include"
)

add_executable(odom_node src/helper_functions.cpp src/odom_estimator.cpp src/odom_estimator_main.cpp)

add_dependencies(odom_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(odom_node
  ${catkin_LIBRARIES}
)
