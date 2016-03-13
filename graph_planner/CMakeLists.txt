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
  genmsg
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


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

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
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES learning_tf
#  CATKIN_DEPENDS roscpp rospy tf turtlesim
#  DEPENDS system_lib
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


add_dependencies(creator graph_planner_generate_messages_cpp)

## Declare a cpp library
# add_library(learning_tf
#   src/${PROJECT_NAME}/learning_tf.cpp
# )

## Declare a cpp executable
# add_executable(learning_tf_node src/learning_tf_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(learning_tf_node learning_tf_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(learning_tf_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS learning_tf learning_tf_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )




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







#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_learning_tf.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
#add_executable(tf_listener src/listener.cpp)
#target_link_libraries(tf_listener ${catkin_LIBRARIES})

#add_executable(turtle_tf_listener src/turtle_tf_listener.cpp)
#target_link_libraries(turtle_tf_listener ${catkin_LIBRARIES})
