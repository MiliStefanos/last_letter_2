cmake_minimum_required(VERSION 3.10.2)
project(last_letter_2)

find_package(catkin REQUIRED 
  COMPONENTS
  roscpp
  geometry_msgs
  rosgraph_msgs
  sensor_msgs
  std_msgs
  last_letter_2_msgs
)
find_package(Boost REQUIRED)

catkin_package(DEPENDS
  CATKIN_DEPENDS 
  std_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${Boost_INCLUDE_DIR})

add_executable(joy2chan src/joy2chan.cpp) 
add_executable(chan2basics src/chan2basics.cpp) 
add_executable(core src/core.cpp)
add_executable(controller_mediator src/controller_mediator.cpp)
add_executable(environment src/environment.cpp)
add_executable(controller src/controller.cpp)


target_link_libraries(joy2chan ${catkin_LIBRARIES})
target_link_libraries(chan2basics ${catkin_LIBRARIES})
target_link_libraries(core ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
target_link_libraries(controller_mediator ${catkin_LIBRARIES})
target_link_libraries(environment ${catkin_LIBRARIES})
target_link_libraries(controller ${catkin_LIBRARIES})



