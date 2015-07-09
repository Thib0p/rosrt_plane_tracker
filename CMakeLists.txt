cmake_minimum_required(VERSION 2.8.3)
project(rosrt_plane_tracker)

# this command is required for autocomplete in vim
SET(CMAKE_EXPORT_COMPILE_COMMANDS 1)

# this sets all the required paths 
include("${CMAKE_SOURCE_DIR}/cmake/cmake_global/LAB.cmake")
## find SL dependencies using catkin
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosrt
  std_msgs
  utilities
  lwpr
  sl
  apollo
)

message ("catkin_librairies content :\n"${catkin_LIBRARIES} )
find_package(xenomai_ros QUIET)

if(Xenomai_FOUND)
  message("COMPILING FOR XENOMAI !")
  add_xenomai_flags()
  include_directories( ${Xenomai_INCLUDE_DIR} )
endif()

include_directories(${catkin_INCLUDE_DIRS})

add_library(rosrt_plane_tracker src/rosrt_plane_tracker_clean.cpp src/GeometryUtils.cpp src/rosInterface.cpp src/myGeometryUtils.cpp)
target_link_libraries(rosrt_plane_tracker ${catkin_LIBRARIES} rosrt apollo_task apollo utility ${LAB_STD_LIBS} )
install(TARGETS rosrt_plane_tracker
  ARCHIVE DESTINATION ${LAB_LIBDIR}
  LIBRARY DESTINATION ${LAB_LIBDIR})


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES rosrt_plane_tracker
  CATKIN_DEPENDS utilities utilities lwpr sl apollo roscpp rosrt std_msgs
)

