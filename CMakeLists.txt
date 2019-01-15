cmake_minimum_required(VERSION 2.8.3)
project(bobble_description)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS)

catkin_package()

set(GAZEBO_MODEL_PATH $ENV{HOME}/.gazebo/models)

add_custom_target(COMMAND cmake -E make_directory ${GAZEBO_MODEL_PATH}/${PROJECT_NAME})

#############
## Install ##
#############

install(DIRECTORY launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY meshes
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# copy my_robot meshes to gazebo models folder
install(DIRECTORY meshes
  DESTINATION ${GAZEBO_MODEL_PATH}/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.dae"
)

install(DIRECTORY xacro
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_bobble_description.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
