cmake_minimum_required(VERSION 2.8.3)
project(cpp_prac)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

add_message_files(
  FILES
)

add_service_files(
  FILES 
)

generate_messages(
  DEPENDENCIES
  std_msgs
 )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ros_essentials_cpp
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(turtlesim_move_node src/turtlesim/move.cpp)
target_link_libraries (turtlesim_move_node ${catkin_LIBRARIES})

add_executable(turtlesim_rotate_node src/turtlesim/rotate.cpp)
target_link_libraries(turtlesim_rotate_node ${catkin_LIBRARIES})

add_executable(turtlesim_desired_rotate_node src/turtlesim/desired_rotate.cpp)
target_link_libraries(turtlesim_desired_rotate_node ${catkin_LIBRARIES})

add_executable(go_to_goal_node src/turtlesim/go_to_goal.cpp)
target_link_libraries(go_to_goal_node ${catkin_LIBRARIES})

add_executable(spiral_node src/turtlesim/spiral.cpp)
target_link_libraries(spiral_node ${catkin_LIBRARIES})

add_executable(clean_grid_node src/turtlesim/clean_grid.cpp)
target_link_libraries(clean_grid_node ${catkin_LIBRARIES})
