cmake_minimum_required(VERSION 2.8.3)
project(wpr1_tutorials)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp rospy std_msgs
  pcl_conversions pcl_ros 
  cv_bridge image_transport tf 
  sensor_msgs geometry_msgs std_msgs visualization_msgs 
  image_geometry message_generation
  move_base_msgs actionlib 
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

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
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
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
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
add_service_files(DIRECTORY srv
                  FILES
                  Follow.srv
                  Grab.srv)
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
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

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
 INCLUDE_DIRS include
#  LIBRARIES wpr1_apps
 CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})

## Declare a C++ library
# add_library(wpr1_apps
#   src/${PROJECT_NAME}/wpr1_apps.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(wpr1_apps ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(wpr1_apps_node src/wpr1_apps_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(wpr1_apps_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(wpr1_apps_node
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
# install(TARGETS wpr1_apps wpr1_apps_node
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

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_wpr1_apps.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)


## Declare a C++ executable
add_executable(wpr1_face_detect
  src/wpr1_face_detect.cpp
)
add_dependencies(wpr1_face_detect ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(wpr1_face_detect
  ${catkin_LIBRARIES}
)

add_executable(wpr1_joint_control
  src/wpr1_joint_control.cpp
)
add_dependencies(wpr1_joint_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(wpr1_joint_control
  ${catkin_LIBRARIES}
)

add_executable(wpr1_velocity_control
  src/wpr1_velocity_control.cpp
)
add_dependencies(wpr1_velocity_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(wpr1_velocity_control
  ${catkin_LIBRARIES}
)

add_executable(wpr1_imu_demo
  src/wpr1_imu_demo.cpp
)
add_dependencies(wpr1_imu_demo ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(wpr1_imu_demo
  ${catkin_LIBRARIES}
)

add_executable(wpr1_simple_goal
  src/wpr1_simple_goal.cpp
)
add_dependencies(wpr1_simple_goal ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(wpr1_simple_goal
  ${catkin_LIBRARIES}
)

add_executable(wpr1_cruise
  src/wpr1_cruise.cpp
)
add_dependencies(wpr1_cruise ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(wpr1_cruise
  ${catkin_LIBRARIES}
)

add_executable(wpr1_follow_client
  src/wpr1_follow_client.cpp
)
add_dependencies(wpr1_follow_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(wpr1_follow_client
  ${catkin_LIBRARIES}
)

add_executable(wpr1_grab_client
  src/wpr1_grab_client.cpp
)
add_dependencies(wpr1_grab_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(wpr1_grab_client
  ${catkin_LIBRARIES}
)

add_executable(wpr1_pass_client
  src/wpr1_pass_client.cpp
)
add_dependencies(wpr1_pass_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(wpr1_pass_client
  ${catkin_LIBRARIES}
)

add_executable(wpr1_dock_client
  src/wpr1_dock_client.cpp
)
add_dependencies(wpr1_dock_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(wpr1_dock_client
  ${catkin_LIBRARIES}
)

add_executable(wpr1_speak
  src/wpr1_speak.cpp
)
add_dependencies(wpr1_speak ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(wpr1_speak
  ${catkin_LIBRARIES}
)

add_executable(home_test
  src/home_test.cpp
  src/action_manager/action_manager.cpp
  src/scripts/10_home_script.cpp
)
add_dependencies(home_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(home_test
  ${catkin_LIBRARIES}
)

add_executable(gpsr
  src/gpsr.cpp
  src/action_manager/action_manager.cpp
)
add_dependencies(gpsr ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(gpsr
  ${catkin_LIBRARIES}
)

add_executable(free_test
  src/free_test.cpp
  src/action_manager/action_manager.cpp
  src/scripts/6_free_script.cpp
)
add_dependencies(free_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(free_test
  ${catkin_LIBRARIES}
)

add_executable(recreational_test
  src/recreational_test.cpp
  src/action_manager/action_manager.cpp
  src/scripts/7_recreational_script.cpp
)
add_dependencies(recreational_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(recreational_test
  ${catkin_LIBRARIES}
)

add_executable(innovation_test
  src/innovation_test.cpp
  src/action_manager/action_manager.cpp
  src/scripts/8_innovation_script.cpp
)
add_dependencies(innovation_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(innovation_test
  ${catkin_LIBRARIES}
)

add_executable(campus_test
  src/campus_test.cpp
  src/action_manager/action_manager.cpp
  src/scripts/9_campus_script.cpp
)
add_dependencies(campus_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(campus_test
  ${catkin_LIBRARIES}
)

add_executable(entrance_detect
  src/entrance_detect.cpp
)
add_dependencies(entrance_detect ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(entrance_detect
  ${catkin_LIBRARIES}
)

add_executable(who_is_who
  src/who_is_who.cpp
)
add_dependencies(who_is_who ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(who_is_who
  ${catkin_LIBRARIES}
)

add_executable(shopping
  src/shopping.cpp
)
add_dependencies(shopping ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(shopping
  ${catkin_LIBRARIES}
)

add_executable(object_recognition
  src/object_recognition.cpp
)
add_dependencies(object_recognition ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(object_recognition
  ${catkin_LIBRARIES}
)

add_executable(person_recognition
  src/person_recognition.cpp
)
add_dependencies(person_recognition ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(person_recognition
  ${catkin_LIBRARIES}
)

add_executable(speech_recognition
  src/speech_recognition.cpp
)
add_dependencies(speech_recognition ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(speech_recognition
  ${catkin_LIBRARIES}
)

add_executable(sr_cn
  src/sr_cn.cpp
)
add_dependencies(sr_cn ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(sr_cn
  ${catkin_LIBRARIES}
)

add_executable(wpr1_image_node
  src/wpr1_image_node.cpp
)
add_dependencies(wpr1_image_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(wpr1_image_node
  ${catkin_LIBRARIES}
)

add_executable(wpr1_wp_grab
  src/wpr1_wp_grab.cpp
)
add_dependencies(wpr1_wp_grab ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(wpr1_wp_grab
  ${catkin_LIBRARIES}
)

add_executable(wpr1_sr_follow
  src/wpr1_sr_follow.cpp
)
add_dependencies(wpr1_sr_follow ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(wpr1_sr_follow
  ${catkin_LIBRARIES}
)
