cmake_minimum_required(VERSION 2.8.3)
project(rqt_camera_is1500)

find_package(catkin REQUIRED COMPONENTS
  rospy
  rqt_gui
  rqt_gui_py
)

catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package()

###########
## Build ##
###########


#############
## Install ##
#############
install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY resources
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
