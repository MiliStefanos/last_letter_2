cmake_minimum_required(VERSION 2.8.3)
project(plane_simple)


find_package(catkin REQUIRED COMPONENTS
	roscpp
	geometry_msgs
	sensor_msgs
	message_generation
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})

#generate_dynamic_reconfigure_options(
#  cfg/controller.cfg
#)

#add_library(box_lib src/box_plugin.cpp)
#target_link_libraries(box_lib ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES})

add_service_files(
  FILES
  apply_wrench.srv
)

add_message_files(
  FILES
  joystick_input.msg
  InputSignals.msg
  planeForces.msg
  model_states.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

add_library(motor_plugin src/gazebo/motor_plugin.cpp)
add_library(world_plugin SHARED src/gazebo/world_plugin.cpp)

catkin_package(DEPENDS
	 CATKIN_DEPENDS message_runtime
   std_msgs
)


add_executable(joy2chan src/ros/joy2chan.cpp) 
add_executable(chan2basics src/ros/chan2basics.cpp) 
add_executable(basics2forces src/ros/basics2forces.cpp)


target_link_libraries(joy2chan ${catkin_LIBRARIES})
target_link_libraries(chan2basics ${catkin_LIBRARIES})
target_link_libraries(basics2forces ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
target_link_libraries(motor_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
target_link_libraries(world_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_dependencies(basics2forces plane_simple_gencpp)





