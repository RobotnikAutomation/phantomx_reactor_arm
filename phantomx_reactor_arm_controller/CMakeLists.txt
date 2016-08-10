cmake_minimum_required(VERSION 2.8.3)
project(phantomx_reactor_arm_controller)

find_package(catkin REQUIRED COMPONENTS rospy arbotix_python)

catkin_python_setup()
catkin_package()

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS src/phantomx_reactor.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

