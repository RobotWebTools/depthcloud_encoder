cmake_minimum_required(VERSION 2.8)
project(depthcloud_www)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY html/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
