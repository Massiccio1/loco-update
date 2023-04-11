cmake_minimum_required(VERSION 3.0.2)
project(test_cpp)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  my_vision_messages
)
find_package(Eigen3 REQUIRED)

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
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##  catkin_package(CATKIN_DEPENDS message_runtime)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
#add_message_files(
#  FILES
#
#)
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
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
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
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES test_cpp
  CATKIN_DEPENDS roscpp rospy std_msgs my_vision_messages
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS} #aggiutno
)

## Declare a C++ library

#SET(KIN "../include/tl/x1.h" "../include/tl/x2.h")

# add_library(
#     ${PROJECT_NAME} ${HELP}
#     ${CMAKE_CURRENT_SOURCE_DIR}/include/Helper.o
#     ${CMAKE_CURRENT_SOURCE_DIR}/include/Kin.o
#  )

 ## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_executable(${PROJECT_NAME}_node ${CMAKE_CURRENT_SOURCE_DIR}/src/test1.cpp)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
#set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
#add_dependencies(${PROJECT_NAME}_node ${CMAKE_CURRENT_SOURCE_DIR}/include/Helper.o ${CMAKE_CURRENT_SOURCE_DIR}/include/Kin.o)

## Specify libraries to link a library or executable target against

# add_custom_command(

#     OUTPUT  ${CMAKE_CURRENT_SOURCE_DIR}/include/Helper.o
#     SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/Helper.cpp 
#     DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/Helper.h ${CMAKE_CURRENT_SOURCE_DIR}/src/Helper.cpp
#     COMMAND g++ -c ${CMAKE_CURRENT_SOURCE_DIR}/src/Helper.cpp -o ${CMAKE_CURRENT_SOURCE_DIR}/include/Helper.o
#     COMMAND echo costruito ${CMAKE_CURRENT_SOURCE_DIR}/include/Helper.o
#     COMMAND echo ${CMAKE_CURRENT_SOURCE_DIR}
# )

# add_custom_command(
#     OUTPUT  ${CMAKE_CURRENT_SOURCE_DIR}/include/Kin.o
#     SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/Kin.cpp
#     DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/Kin.cpp ${CMAKE_CURRENT_SOURCE_DIR}/src/Kin.h
#     COMMAND  g++ -c ${CMAKE_CURRENT_SOURCE_DIR}/src/Kin.cpp -o ${CMAKE_CURRENT_SOURCE_DIR}/include/Kin.o
#     COMMAND echo costruito ${CMAKE_CURRENT_SOURCE_DIR}/include/Kin.o
# )


# add_custom_command(
#     OUTPUT 
#        # This is the file your custom command will create
#        ${CMAKE_CURRENT_SOURCE_DIR}/include/Helper.o

#     DEPENDS
#        # These are all the files you need to run your custom command
#        ${ch}

#     COMMAND echo "from command">>${CMAKE_CURRENT_SOURCE_DIR}/out.txt
#     COMMENT "output foobar, depends on fooout"
# )
  
# add_custom_target(
#     dummy ALL
#     # DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/test1.cpp
#     COMMAND make
#     COMMENT "in target"
# )

# execute_process(
#   COMMAND "make -f Makefile"
# )

add_custom_target(
    myCustomTarget
    #COMMAND cd ${CMAKE_CURRENT_SOURCE_DIR}
    COMMAND cd ${CMAKE_CURRENT_SOURCE_DIR}/src && make -f ${CMAKE_CURRENT_SOURCE_DIR}/Makefile.make FROM_PATH=${CMAKE_CURRENT_SOURCE_DIR}/src CPATH=~/ros_ws/install/include
    #COMMAND make -f ${CMAKE_CURRENT_SOURCE_DIR}/Makefile2.make FROM_PATH=${CMAKE_CURRENT_SOURCE_DIR}/include
    #COMMAND make -f ${CMAKE_CURRENT_SOURCE_DIR}/Makefile.make
    #COMMAND make -f ${CMAKE_CURRENT_SOURCE_DIR}/Makefile.make test
    COMMENT "building with make"
)


# add_custom_target(
#     my_test
#     OUTPUT  ${CMAKE_CURRENT_SOURCE_DIR}/out.txt
#     SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/in.txt
#     DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/in.txt
#     COMMAND  echo "from cmake" > ${CMAKE_CURRENT_SOURCE_DIR}/out.txt
#     COMMAND cat ${CMAKE_CURRENT_SOURCE_DIR}/out.txt
# )

#add_library(Helper_o ${CMAKE_CURRENT_SOURCE_DIR}/include/Helper.o)
#set_target_properties(Helper_o PROPERTIES LINKER_LANGUAGE CXX)

add_dependencies(${PROJECT_NAME}_node myCustomTarget)

link_directories(${Eigen_INCLUDE_DIRS})


target_link_libraries(
  ${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${cEigen_LIBRARIES}
  ${CMAKE_CURRENT_SOURCE_DIR}/include/Helper.o
  ${CMAKE_CURRENT_SOURCE_DIR}/include/Kin.o
  ${CMAKE_CURRENT_SOURCE_DIR}/include/robot.o
  #${CMAKE_CURRENT_SOURCE_DIR}/include/obj.o
  Eigen3::Eigen
  ${Eigen_LIBRARIES}
)


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_test_cpp.cpp)
# if(TARGET ${PROJECT_NAME}-test)
  #target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
