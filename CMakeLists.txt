cmake_minimum_required(VERSION 2.8.3)
project(labphantoms)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
#list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

find_package( Boost REQUIRED )
if ( NOT Boost_FOUND )
  message(WARNING "This project requires the Boost library, and will not be compiled.")
  return()  
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
  tf2_msgs
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES assignment_6_1
  CATKIN_DEPENDS roscpp std_msgs tf tf2_msgs message_runtime
#  DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

include_directories(${catkin_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

include_directories(include)

add_library(labphantoms SHARED ./src/labphantoms.cpp)
target_link_libraries(labphantoms ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

