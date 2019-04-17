cmake_minimum_required(VERSION 2.8.3)
project(opt_view)

add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE Debug)

set(EIGEN3_INCLUDE_DIR /usr/include/eigen3)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    message_generation
    eigen_conversions)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)


include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${EIGEN3_INCLUDE_DIR}
)

add_message_files(
	FILES
	MultiagentPose.msg
	Formation.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package (CATKIN_DEPENDS message_runtime)

add_library (pd_lib SHARED src/pd_rosnode.cpp)
target_link_libraries (pd_lib ${catkin_LIBRARIES})
add_dependencies(pd_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable (multiagent_command src/multiagent_command.cpp)
target_link_libraries (multiagent_command ${catkin_LIBRARIES} pd_lib)
add_dependencies(multiagent_command ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable (self_positioning src/self_positioning_node.cpp src/self_positioning.cpp)
target_link_libraries (self_positioning ${catkin_LIBRARIES} pd_lib)
add_dependencies(self_positioning ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})



