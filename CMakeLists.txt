cmake_minimum_required(VERSION 2.8.3)
project(beginner_tutorials)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin and my catkin packages 
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation 
  genmsg
  tf
)

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   change_output_string.srv
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES beginner_tutorials
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories( include ${catkin_INCLUDE_DIRS})


## Declare a C++ executable
add_executable(talker src/talker.cpp )
add_executable(listener src/listener.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(talker ${catkin_LIBRARIES})
target_link_libraries(listener ${catkin_LIBRARIES})

add_dependencies(talker beginner_tutorials_generate_messages_cpp)
add_dependencies(listener beginner_tutorials_generate_messages_cpp)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(
    test_talker
    test/test.launch
    test/main.cpp
    test/test.cpp
  )
  target_link_libraries( test_talker ${catkin_LIBRARIES})
  
add_dependencies(
    test_talker
    talker
    listener
    ${catkin_EXPORTED_TARGETS}
  )
endif()
