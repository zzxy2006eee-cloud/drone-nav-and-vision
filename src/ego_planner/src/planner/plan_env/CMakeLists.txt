cmake_minimum_required(VERSION 2.8.3)
project(plan_env)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++17 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(OpenCV REQUIRED)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  mavros_msgs
  rospy
  std_msgs
  geometry_msgs
  visualization_msgs
  cv_bridge
  message_filters
)

add_message_files(FILES VoxelUpdate.msg)
add_service_files(FILES MapArchive.srv GetVoxelSnapshot.srv)
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES plan_env
 CATKIN_DEPENDS roscpp std_msgs geometry_msgs mavros_msgs message_runtime
#  DEPENDS system_lib
)

include_directories(
    SYSTEM
    include
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

add_library( plan_env
    src/grid_map.cpp
    src/raycast.cpp
    )
add_dependencies(plan_env ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries( plan_env
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    )

# Explicit offline regression target; not part of normal simulator startup.
add_executable(map_message_offline_test EXCLUDE_FROM_ALL tests/map_message_offline_test.cpp)
target_link_libraries(map_message_offline_test ${catkin_LIBRARIES} ${PCL_LIBRARIES})
