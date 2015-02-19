cmake_minimum_required(VERSION 2.8.3)
project(rose_joystick)

find_package(catkin REQUIRED COMPONENTS
  joy 
  message_generation
)

add_message_files(
    FILES
        available_modes.msg
)

add_service_files(
    FILES
        switch_joystick_mode.srv
)

generate_messages(
  DEPENDENCIES
    std_msgs  
)

catkin_package(
  CATKIN_DEPENDS 
    joy 
    std_msgs 
    message_runtime
  DEPENDS
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

