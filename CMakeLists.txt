cmake_minimum_required(VERSION 2.8.3)
project(collision_free_traj)

find_package(catkin REQUIRED COMPONENTS 
  moveit_ros_planning_interface 
  moveit_ros_planning 
  moveit_planners_ompl
  trajectory_msgs
  roscpp
  rospy
  std_msgs
  tf
  #ompl
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fPIC")


#set(CMAKE_MODULE_PATH    "/home/ghanim/ws_moveit/src/collision_free_traj/")


find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(Boost REQUIRED COMPONENTS serialization system filesystem program_options thread)
include_directories(SYSTEM ${Boost_INCLUDE_DIR})

find_package(OpenCV REQUIRED COMPONENTS features2d nonfree)
find_package(PCL 1.7 REQUIRED)
find_package(VTK REQUIRED)

include_directories(include ${PCL_INCLUDE_DIRS} /usr/local/include)
link_directories(${PCL_LIBRARY_DIRS} /usr/local/lib)

include_directories(
  ${catkin_INCLUDE_DIRS} 
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES baxter_python_cpp
#  CATKIN_DEPENDS moveit_ros_planning_interface roscpp rospy std_msgs tf
#  DEPENDS system_lib
)

add_executable(generate_baxter_database_moveit src/generate_baxter_database_moveit.cpp)
target_link_libraries(generate_baxter_database_moveit ${catkin_LIBRARIES}  )

add_executable(test_baxter_database src/test_baxter_database.cpp)
target_link_libraries(test_baxter_database ${catkin_LIBRARIES}  ${Boost_SERIALIZATION_LIBRARY}  
image_processing ${PCL_LIBRARIES} 
${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} ${Boost_PROGRAM_OPTIONS_LIBRARY})

add_executable(gripper_control src/gripper_control.cpp)
target_link_libraries(gripper_control ${catkin_LIBRARIES})

add_executable(collision_with_moveit src/collision_with_moveit.cpp)
target_link_libraries(collision_with_moveit ${catkin_LIBRARIES} )

add_executable(test_ikfast src/test_ikfast.cpp)
target_link_libraries(test_ikfast ${catkin_LIBRARIES} )

#add_executable(refined_baxter_collision_free src/refined_baxter_collision_free.cpp)
#target_link_libraries(refined_baxter_collision_free ${catkin_LIBRARIES})

#add_executable(naive_try src/naive_try.cpp)
#target_link_libraries(naive_try ${catkin_LIBRARIES})

#add_executable(obstacle_avoidance_traj src/obstacle_avoidance_traj.cpp)
#target_link_libraries(obstacle_avoidance_traj ${catkin_LIBRARIES})

#add_executable(test_joint_action_left src/test_joint_action_left.cpp)
#target_link_libraries(test_joint_action_left ${catkin_LIBRARIES})

#add_executable(complete_algorithm src/complete_algorithm.cpp)
#target_link_libraries(complete_algorithm ${catkin_LIBRARIES})

#add_executable(test_moveit_baxter src/test_moveit_baxter.cpp)
#target_link_libraries(test_moveit_baxter ${catkin_LIBRARIES})
