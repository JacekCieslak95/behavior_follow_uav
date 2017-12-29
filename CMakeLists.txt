cmake_minimum_required(VERSION 2.8.3)
project(behavior_follow_uav)

cmake_minimum_required(VERSION 2.8.3)
project(behavior_follow_uav)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

add_definitions(-std=c++11)

# Directories definition
set(BEHAVIOR_FOLLOW_UAV_SOURCE_DIR
  src/source
)

set(BEHAVIOR_FOLLOW_UAV_INCLUDE_DIR
  src/include
)

# Files declaration
set(BEHAVIOR_FOLLOW_UAV_SOURCE_FILES
    ${BEHAVIOR_FOLLOW_UAV_SOURCE_DIR}/behavior_follow_uav.cpp
  ${BEHAVIOR_FOLLOW_UAV_SOURCE_DIR}/behavior_follow_uav.cpp
)

set(BEHAVIOR_FOLLOW_UAV_HEADER_FILES
  ${BEHAVIOR_FOLLOW_UAV_INCLUDE_DIR}/behavior_follow_uav.h
)

### Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
  COMPONENTS
  roscpp
  std_msgs
  behavior_process
  droneMsgsROS
)

###################################
## catkin specific configuration ##
###################################
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    INCLUDE_DIRS ${BEHAVIOR_FOLLOW_UAV_INCLUDE_DIR}
    CATKIN_DEPENDS
    roscpp
    std_msgs
    drone_process
    behavior_process
    droneMsgsROS
    DEPENDS yaml-cpp
)

###########
## Build ##
###########
include_directories(
    ${BEHAVIOR_FOLLOW_UAV_INCLUDE_DIR}
    ${BEHAVIOR_FOLLOW_UAV_SOURCE_DIR}
)
include_directories(
    ${catkin_INCLUDE_DIRS}
)

add_library(behavior_follow_uav_lib ${BEHAVIOR_FOLLOW_UAV_SOURCE_FILES} ${BEHAVIOR_FOLLOW_UAV_HEADER_FILES})
add_dependencies(behavior_follow_uav_lib ${catkin_EXPORTED_TARGETS})
target_link_libraries(behavior_follow_uav_lib ${catkin_LIBRARIES})
target_link_libraries(behavior_follow_uav_lib yaml-cpp)


add_executable(behavior_follow_uav ${BEHAVIOR_FOLLOW_UAV_SOURCE_DIR}/behavior_follow_uav_main.cpp)
add_dependencies(behavior_follow_uav ${catkin_EXPORTED_TARGETS})
target_link_libraries(behavior_follow_uav behavior_follow_uav_lib)
target_link_libraries(behavior_follow_uav ${catkin_LIBRARIES})

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_behavior_go_to_point_angle.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
