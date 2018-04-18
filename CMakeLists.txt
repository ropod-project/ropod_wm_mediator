cmake_minimum_required(VERSION 2.8.6)

project(ropod_wm_mediator)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/ropod_wm_mediator/cmake")

set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/bin)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR})

set(INSTALL_DIR /usr/lib CACHE PATH "Installation directory for mediator")

find_package(yaml-cpp REQUIRED)

########################################################################
# ROS dependency
########################################################################
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  ropod_ros_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(CATKIN_DEPENDS
  roscpp
  ropod_ros_msgs
)


########################################################################
# Mediator
########################################################################

add_executable(test_wm_mediator ${PROJECT_SOURCE_DIR}/src/test.cpp ${HEADER_FILES})
add_dependencies(test_wm_mediator ropod_ros_msgs_gencpp)
target_link_libraries(test_wm_mediator ${LIBS} ${catkin_LIBRARIES})

add_executable(ropod_wm_mediator_demo ${PROJECT_SOURCE_DIR}/src/wm_mediator.cpp)
add_dependencies(ropod_wm_mediator_demo ropod_ros_msgs_gencpp)
target_link_libraries(ropod_wm_mediator_demo ${LIBS}  ${catkin_LIBRARIES} yaml-cpp)
