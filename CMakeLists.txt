cmake_minimum_required(VERSION 2.8.6)

project(ropod_wm_mediator)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/ropod_wm_mediator/cmake")

set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/bin)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR})

set(INSTALL_DIR /usr/lib CACHE PATH "Installation directory for mediator")

include(/opt/ropod/ropod_common/ropodcpp/ftsm/CMakeLists.txt)
find_package(yaml-cpp REQUIRED)

########################################################################
# ROS dependency
########################################################################
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  ropod_ros_msgs
  osm_bridge_ros_wrapper
  genmsg
  actionlib_msgs
  actionlib
)

include_directories(
    include
    /opt/ropod/ftsm/cppftsm/include
    #/opt/ropod/ropod_common/ropodcpp/ftsm/include
    ${catkin_INCLUDE_DIRS}
)

catkin_package(CATKIN_DEPENDS
  roscpp
  ropod_ros_msgs
  osm_bridge_ros_wrapper
  actionlib_msgs
)


########################################################################
# Mediator
########################################################################

add_library(osm ${PROJECT_SOURCE_DIR}/src/osm.cpp)
add_dependencies(osm ropod_ros_msgs_generate_messages_cpp)
target_link_libraries(osm ${LIBS} ${catkin_LIBRARIES})

add_executable(ropod_wm_mediator 
    ${PROJECT_SOURCE_DIR}/src/wm_mediator.cpp
    ${HEADER_FILES}
    /opt/ropod/ropod_common/ropodcpp/ftsm/src/ftsm_base.cpp
    /opt/ropod/ftsm/cppftsm/src/ftsm.cpp
)
add_dependencies(ropod_wm_mediator ropod_ros_msgs_generate_messages_cpp ${ropod_wm_mediator_EXPORTED_TARGETS})
target_link_libraries(ropod_wm_mediator ${LIBS}  ${catkin_LIBRARIES} ${FTSM_LIBRARIES} yaml-cpp osm)





