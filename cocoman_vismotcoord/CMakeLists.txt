cmake_minimum_required(VERSION 2.8.3)
project(cocoman_vismotcoord)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
  std_msgs
  sensor_msgs
  roslib
  tf
  message_generation
)

## CUDA
set(CUDA_HOST_COMPILER /usr/bin/gcc-4.8)
FIND_PACKAGE(CUDA REQUIRED)
INCLUDE(FindCUDA)
CUDA_ADD_LIBRARY (cuda_particlefilter
                src/${PROJECT_NAME}/particlescoregpu.cu
                src/${PROJECT_NAME}/particlescoregpu.h
                src/${PROJECT_NAME}/define.h
)

## PCL 1.8
find_package(PCL 1.8 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
if (NOT PCL_FOUND)
    MESSAGE(FATAL_ERROR "PCL not found.\n")
endif (NOT PCL_FOUND)

find_package(Eigen3 REQUIRED)

## Generate services in the 'srv' folder
add_service_files(
   FILES
   command.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs  # and/or other packages containing depended messages
)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES cocoman_vismotcoord
  CATKIN_DEPENDS roscpp rospy std_msgs roslib pcl_ros
  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(cocoman_vismotcoord
  src/${PROJECT_NAME}/vismotcoord.cpp
  src/${PROJECT_NAME}/bodypart.cpp
  src/${PROJECT_NAME}/mesh.cpp
  src/${PROJECT_NAME}/scene.cpp
  src/${PROJECT_NAME}/model.cpp
  src/${PROJECT_NAME}/posest.cpp
  src/${PROJECT_NAME}/common.h
  src/${PROJECT_NAME}/define.h
)

add_executable(cocoman_vismotcoord_node src/cocoman_vismotcoord_node.cpp)
add_dependencies( cocoman_vismotcoord_node ${PROJECT_NAME}_generate_messages_cpp )

target_link_libraries(cocoman_vismotcoord
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)

target_link_libraries(cocoman_vismotcoord_node
   ${catkin_LIBRARIES}
   cocoman_vismotcoord
   assimp
   cuda_particlefilter
   ${PCL_LIBRARIES}
)

SET_TARGET_PROPERTIES(cocoman_vismotcoord_node PROPERTIES LINK_FLAGS -L${PCL_LIBRARY_DIRS})
