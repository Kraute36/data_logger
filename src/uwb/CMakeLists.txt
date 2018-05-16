cmake_minimum_required(VERSION 2.8.3)
project(uwb)

find_package(catkin REQUIRED COMPONENTS dynamic_reconfigure)

catkin_package(DEPENDS CATKIN DEPENDS dynamic_reconfigure)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS 
	nodes/uwb_node.py
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/nodes
)
