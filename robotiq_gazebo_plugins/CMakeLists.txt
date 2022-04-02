cmake_minimum_required(VERSION 3.0.2)
project(robotiq_gazebo_plugins)

# add_compile_options(-std=c++17)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  gazebo_msgs
  roscpp
  )


###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES robotiq_gazebo_mimic_joint_plugin
  CATKIN_DEPENDS roscpp gazebo_ros gazebo_msgs
  DEPENDS 
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${GAZEBO_INCLUDE_DIRS}
)


add_library(robotiq_gazebo_mimic_joint_plugin src/mimic_joint_plugin.cpp)
target_link_libraries(robotiq_gazebo_mimic_joint_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(robotiq_gazebo_mimic_joint_plugin ${catkin_EXPORTED_TARGETS})



install(TARGETS
  robotiq_gazebo_mimic_joint_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)


