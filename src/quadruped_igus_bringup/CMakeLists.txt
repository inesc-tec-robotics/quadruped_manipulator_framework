cmake_minimum_required(VERSION 3.0.2)
project(quadruped_igus_bringup)
find_package(catkin REQUIRED)

file(GLOB PYTHON_SCRIPTS scripts/*.py)

catkin_package(

)

catkin_install_python(PROGRAMS ${PYTHON_SCRIPTS}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

