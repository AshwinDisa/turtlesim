cmake_minimum_required(VERSION 2.8.3)
project(turtlesim_motion)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(turtlesim_move_node src/move.cpp)
target_link_libraries (turtlesim_move_node ${catkin_LIBRARIES})

add_executable(turtlesim_rotate_node src/rotate.cpp)
target_link_libraries(turtlesim_rotate_node ${catkin_LIBRARIES})

add_executable(turtlesim_desired_rotate_node src/desired_rotate.cpp)
target_link_libraries(turtlesim_desired_rotate_node ${catkin_LIBRARIES})

add_executable(go_to_goal_node src/go_to_goal.cpp)
target_link_libraries(go_to_goal_node ${catkin_LIBRARIES})

add_executable(spiral_node src/spiral.cpp)
target_link_libraries(spiral_node ${catkin_LIBRARIES})

add_executable(clean_grid_node src/clean_grid.cpp)
target_link_libraries(clean_grid_node ${catkin_LIBRARIES})
