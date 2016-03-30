# %Tag(FULLTEXT)%
cmake_minimum_required(VERSION 2.8.3)
project(intercom2)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
MESSAGE("FALGS: ${CMAKE_CXX_FLAGS}")

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy roslib std_msgs)

## Declare ROS messages and services
#add_message_files(FILES Num.msg)
#add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
#generate_messages(DEPENDENCIES)

## Declare a catkin package
catkin_package(  INCLUDE_DIRS include)

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_library(topic_tools src/shape_shifter.cpp src/parse.cpp)
target_link_libraries(topic_tools ${catkin_LIBRARIES})

add_executable(intercom2 src/intercom2.cpp src/relay_topic.cpp)
target_link_libraries(intercom2 topic_tools ${catkin_LIBRARIES})
#add_dependencies(subscriber beginner_tutorials_generate_messages_cpp)


add_executable(talker2 src/talker2.cpp)
target_link_libraries(talker2 ${catkin_LIBRARIES})
add_dependencies(talker2 intercom2_generate_messages_cpp)
#add_executable(relay_topic src/relay_topic.cpp)
#target_link_libraries(relay_topic intercom2 ${catkin_LIBRARIES})


# %EndTag(FULLTEXT)%
