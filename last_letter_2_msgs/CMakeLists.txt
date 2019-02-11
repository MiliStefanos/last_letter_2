cmake_minimum_required(VERSION 3.10.2)
project(last_letter_2_msgs)


find_package(catkin REQUIRED 
  COMPONENTS
  roscpp
  std_msgs
  message_generation
)

add_message_files(
  FILES
  joystick_input.msg
  input_signals.msg
  model_wrenches.msg
  model_states.msg
  air_data.msg
  control_signals.msg
  aero_wrenches.msg
  prop_wrenches.msg
)

add_service_files(
  FILES
  apply_wrench_srv.srv
  get_model_states_srv.srv
  get_control_signals_srv.srv
  airdata_srv.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package(DEPENDS
  CATKIN_DEPENDS  message_runtime
  std_msgs
)
