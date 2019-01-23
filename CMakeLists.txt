cmake_minimum_required(VERSION 2.8.3)
project(bobble_description)

find_package(catkin REQUIRED COMPONENTS)

catkin_package()

install(DIRECTORY launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(FILES
        meshes/chassis.dae
        meshes/wheel.dae
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes)

install(DIRECTORY xacro
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
