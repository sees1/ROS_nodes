cmake_minimum_required(VERSION 2.8.3)
project(graph_planner)


set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
MESSAGE("FALGS: ${CMAKE_CXX_FLAGS}")


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  interactive_markers
  roscpp
  rospy
  std_msgs
  roslib
  message_generation
  cmake_modules
)


FIND_PACKAGE(Eigen3 REQUIRED )

# Build Viewer
FIND_PACKAGE( Qt4 REQUIRED )
SET( QT_USE_QTXML 1 )
SET( QT_USE_QTOPENGL 1 )

INCLUDE(${QT_USE_FILE})

INCLUDE_DIRECTORIES(
   ${CMAKE_SOURCE_DIR}/src
   ${CMAKE_CURRENT_BINARY_DIR}

)
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   Point.msg
   PointCmd.msg
   Edge.msg
   EdgeCmd.msg
   GraphStructure.msg
   Plan.msg
 )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
   graph_planner
 )

###################################
## catkin specific configuration ##
###################################

catkin_package(
CATKIN_DEPENDS message_runtime
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_executable(creator
    src/creator_app.cpp
    src/creator_wrapper.cpp
    src/creator.cpp
  
)

TARGET_LINK_LIBRARIES(creator
    ${catkin_LIBRARIES} )

#############
## Install ##
#############

#add gui
QT4_WRAP_UI(
  UI_HDRS
  src/graph_window.ui
)

QT4_WRAP_CPP(
    GUI_MOC
   src/GraphWindowApp.h
)



add_executable(graph_test_gui
    src/GraphWindowApp.cpp
    ${UI_HDRS}
    ${GUI_MOC}
)

TARGET_LINK_LIBRARIES(graph_test_gui
    ${QT_LIBRARIES}  ${catkin_LIBRARIES} 
)

