cmake_minimum_required(VERSION 3.1.3)
project(abb_new_moveit_config)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    moveit_core
    moveit_ros_planning_interface
    moveit_ros_move_group
    moveit_ros_visualization
    moveit_visual_tools
    rviz_visual_tools
    trajectory_msgs
    tf
    tf_conversions
    eigen_conversions
    )
#/\
#||
#||
#add this for descartes

#descartes_core
#descartes_moveit
#descartes_trajectory
#descartes_planner
#descartes_utilities


catkin_package()

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#add executable for descartes and moveit
include_directories(
    ${catkin_INCLUDE_DIRS}
    )
add_executable(
    ${PROJECT_NAME}_node src/move_group/move_group_test.cpp
    )

target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
    )
