cmake_minimum_required(VERSION 2.8.3)
project(phantomx_reactor_arm_description)
find_package(catkin REQUIRED COMPONENTS urdf xacro)
catkin_package(
  CATKIN_DEPENDS urdf xacro
)

install(DIRECTORY meshes
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY robots
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


