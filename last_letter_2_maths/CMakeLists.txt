cmake_minimum_required(VERSION 3.10.2)
project(last_letter_2_maths)


find_package(catkin REQUIRED 
  COMPONENTS
  roscpp
  rosgraph_msgs
  std_srvs
)
find_package(Boost REQUIRED)

catkin_package(DEPENDS
  CATKIN_DEPENDS 
  LIBRARIES math
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR})

add_library(math src/mathlib.cpp)

target_link_libraries(math ${catkin_LIBRARIES})