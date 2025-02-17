cmake_minimum_required(VERSION 3.0.2)
project(igus_arm_driver)

file(GLOB_RECURSE PROGRAM_SOURCES RELATIVE ${PROJECT_SOURCE_DIR}
  "src/${PROJECT_NAME}/.../*.cpp"
  "src/${PROJECT_NAME}/*.cpp"
)
file(GLOB_RECURSE PROGRAM_HEADERS RELATIVE ${PROJECT_SOURCE_DIR}
  "include/${PROJECT_NAME}/.../*.h"
  "include/${PROJECT_NAME}/*.h"
)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  kdl_parser
  trac_ik_lib
  actionlib
  actionlib_msgs
)

find_package(Eigen3 3.3 REQUIRED)

find_package(Boost REQUIRED COMPONENTS system)

add_action_files(
  DIRECTORY action
  FILES IgusCommand.action
)

generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs  # Or other packages containing msgs
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES igus_arm_driver
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs trac_ik_lib actionlib_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_sim_node
  ${PROGRAM_SOURCES}
  ${PROGRAM_HEADERS}
  src/igus_arm_driver_sim_node.cpp
)

add_dependencies(${PROJECT_NAME}_sim_node 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
  ${actionlib_tutorials_EXPORTED_TARGETS}
)

# Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_sim_node
  ${catkin_LIBRARIES}
  Eigen3::Eigen
)

add_executable(${PROJECT_NAME}_hard_node
  ${PROGRAM_SOURCES}
  ${PROGRAM_HEADERS}
  src/igus_arm_driver_hard_node.cpp
)

add_dependencies(${PROJECT_NAME}_hard_node 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
  ${actionlib_tutorials_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}_hard_node
  ${catkin_LIBRARIES}
  Eigen3::Eigen
)

add_executable(test_trac_lib_node
  ${PROGRAM_SOURCES}
  ${PROGRAM_HEADERS}
  tests/test_trac_lib_node.cpp
)

add_dependencies(test_trac_lib_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(test_trac_lib_node
  ${catkin_LIBRARIES}
  Eigen3::Eigen
)