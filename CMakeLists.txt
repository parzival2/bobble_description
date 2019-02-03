cmake_minimum_required(VERSION 2.8.3)
project(bobble_description)

find_package(catkin REQUIRED COMPONENTS
   hector_gazebo_plugins
)

catkin_package(
   CATKIN_DEPENDS
   hector_gazebo_plugins
)

install(DIRECTORY launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(FILES
        meshes/chassis.dae
        meshes/wheel.dae
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes)

install(DIRECTORY xacro
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
